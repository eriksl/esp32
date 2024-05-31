#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "string.h"
#include "cli-command.h"
#include "wlan.h"
#include "config.h"
#include "log.h"
#include "util.h"

#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_netif.h>

static bool inited = false;

static void wlan_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	switch(event_id)
	{
		case(WIFI_EVENT_HOME_CHANNEL_CHANGE):
		{
			wifi_event_home_channel_change_t *event = (wifi_event_home_channel_change_t *)event_data;
			log_format("wlan event: home channel change: old: %u, new: %u", event->old_chan, event->new_chan);
			break;
		}
		case(WIFI_EVENT_STA_START):
		{
			log("wlan event: start");
			util_warn_on_esp_err("esp_wifi_connect", esp_wifi_connect());
			break;
		}
		case(WIFI_EVENT_STA_CONNECTED):
		{
			wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;

			log_format("wlan event: associated: channel: %u, ssid: %.*s, bssid: %02x:%02x:%02x:%02x:%02x:%02x", event->channel, event->ssid_len, event->ssid,
					event->bssid[0], event->bssid[1], event->bssid[2], event->bssid[3], event->bssid[4], event->bssid[5]);
			break;
		}
		case(WIFI_EVENT_STA_DISCONNECTED):
		{
			wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
			log_format("wlan event: disconnected: reason: %u, try to reconnect", event->reason);
			util_warn_on_esp_err("esp_wifi_connect", esp_wifi_connect());
			break;
		}

		default:
		{
			log_format("wlan event: unknown event: %ld", event_id);
			break;
		}
	}
}

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	switch(event_id)
	{
		case(IP_EVENT_STA_GOT_IP):
		{
			string_t ip = string_new(32);
			string_t netmask = string_new(32);
			string_t gw = string_new(32);

			ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

			util_esp_ipv4_addr_to_string(ip, &event->ip_info.ip);
			util_esp_ipv4_addr_to_string(netmask, &event->ip_info.netmask);
			util_esp_ipv4_addr_to_string(gw, &event->ip_info.gw);

			log_format("ip event: got ipv4: changed: %d, ip: %s, netmask: %s, gw: %s",
					event->ip_changed, string_cstr(ip), string_cstr(netmask), string_cstr(gw));

			break;
		}

		case(IP_EVENT_GOT_IP6):
		{
			string_t ip = string_new(128);

			ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
			util_esp_ipv6_addr_to_string(ip, &event->ip6_info.ip);
			log_format("ip event: got ipv6: ip: %s", string_cstr(ip));

			break;
		}

		case(IP_EVENT_STA_LOST_IP):
		{
			log("ip event: lost ipv4");
			break;
		}

		default:
		{
			log_format("ip event: unknown event: %ld", event_id);
			break;
		}
	}
}

void command_wlan_client_config(cli_command_call_t *call)
{
	string_auto(value, 64);
	string_auto_init(key_ssid, "wlan-ssid");
	string_auto_init(key_passwd, "wlan-passwd");

	assert(inited);
	assert(call->parameter_count  < 3);

	if(call->parameter_count > 1)
		config_set_string(key_passwd, call->parameters[1].string);

	if(call->parameter_count > 0)
		config_set_string(key_ssid, call->parameters[0].string);

	string_append_cstr(call->result, "client ssid: ");

	if(config_get_string(key_ssid, value))
		string_append_string(call->result, value);
	else
		string_append_cstr(call->result, "<unset>");

	string_append_cstr(call->result, "\nclient password: ");

	if(config_get_string(key_passwd, value))
		string_append_string(call->result, value);
	else
		string_append_cstr(call->result, "<unset>");

	if(call->parameter_count > 1)
	{
		esp_err_t rv;
		wifi_config_t config;

		string_to_cstr(call->parameters[0].string, sizeof(config.sta.ssid), (char *)config.sta.ssid);
		string_to_cstr(call->parameters[1].string, sizeof(config.sta.password), (char *)config.sta.password);
		config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
		config.sta.bssid_set = 0;
		config.sta.channel = 0;
		config.sta.listen_interval = 3;
		config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
		config.sta.threshold.rssi = -80;
		config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
		config.sta.failure_retry_cnt = 3;
		config.sta.pmf_cfg.capable = false;
		config.sta.pmf_cfg.required = false;

		rv = esp_wifi_set_config(WIFI_IF_STA, &config);

		util_warn_on_esp_err("esp_wifi_set_config", rv);

		log_format("associate: %s/%s", config.sta.ssid, config.sta.password);

		if(rv)
		{
			string_append_cstr(call->result, "\nesp_wifi_set_config returns error");
			return;
		}

		rv = esp_wifi_connect();

		util_warn_on_esp_err("esp_wifi_connect", rv);

		if(rv)
		{
			string_append_cstr(call->result, "\nesp_wifi_connect returns error");
			return;
		}
	}
}

void wlan_init(void)
{
	wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
	esp_netif_t *netif;

	init_config.ampdu_rx_enable = 1;
	init_config.ampdu_tx_enable = 1;
	init_config.amsdu_tx_enable = 1;
	init_config.nvs_enable = 1;
	init_config.wifi_task_core_id = 0;

	assert(!inited);

	util_abort_on_esp_err("esp_event_loop_create_default", esp_event_loop_create_default());
	util_abort_on_esp_err("esp_event_handler_instance_register 1",
			esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wlan_event_handler, (void *)0, (esp_event_handler_instance_t *)0));
	util_abort_on_esp_err("esp_event_handler_instance_register 2",
			esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, ip_event_handler, (void *)0, (esp_event_handler_instance_t *)0));

	util_abort_on_esp_err("esp_netif_init", esp_netif_init());
	netif = esp_netif_create_default_wifi_sta();

	util_abort_on_esp_err("esp_wifi_init", esp_wifi_init(&init_config));
	util_abort_on_esp_err("esp_wifi_set_mode", esp_wifi_set_mode(WIFI_MODE_STA));
	util_abort_on_esp_err("esp_wifi_config_11b_rate", esp_wifi_config_11b_rate(WIFI_IF_STA, true));
	util_abort_on_esp_err("esp_wifi_start", esp_wifi_start());

	inited = true;

}