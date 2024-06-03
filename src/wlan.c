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
#include <esp_netif_sntp.h>
#include <esp_timer.h>

typedef enum
{
	ws_invalid,
	ws_init,
	ws_associating,
	ws_associated,
	ws_ipv4_address_acquired,
	ws_ipv6_link_local_address_acquired,
	ws_ipv6_slaac_address_acquired,
} wlan_state_t;

static bool inited = false;
static esp_netif_t *netif;
static uint64_t wlan_state_since = 0;
static wlan_state_t wlan_state = ws_invalid;

static const char *wlan_state_to_string(wlan_state_t state)
{
	const char *rv = (const char *)0;

	switch(state)
	{
		case(ws_init): { rv = "init"; break; }
		case(ws_associating): { rv = "associating"; break; }
		case(ws_associated): { rv = "associated"; break; }
		case(ws_ipv4_address_acquired): { rv = "ipv4 address acquired"; break; }
		case(ws_ipv6_link_local_address_acquired): { rv = "ipv6 link local address acquired"; break; }
		case(ws_ipv6_slaac_address_acquired): { rv = "ipv6 autoconfig address acquired"; break; }
		default: { rv = "unknown state"; break; }
	}

	return(rv);
}

static void wlan_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	unsigned int rv;

	assert(inited);

	switch(event_id)
	{
		case(WIFI_EVENT_STA_START): /* 2 */
		{
#if 0
			log("wlan event: start");
#endif
			wlan_state = ws_associating;
			wlan_state_since = esp_timer_get_time();
			util_warn_on_esp_err("esp_wifi_connect", esp_wifi_connect());
			break;
		}
		case(WIFI_EVENT_STA_STOP): /* 3 */
		{
#if 0
			log("wlan event: stop");
#endif
			wlan_state = ws_init;
			wlan_state_since = esp_timer_get_time();
			break;
		}
		case(WIFI_EVENT_STA_CONNECTED): /* 4 */
		{
#if 0
			wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;

			log_format("wlan event: associated: channel: %u, ssid: %.*s",
					event->channel, event->ssid_len, event->ssid);
#endif
			wlan_state_since = esp_timer_get_time();
			wlan_state = ws_associated;

			break;
		}
		case(WIFI_EVENT_STA_DISCONNECTED): /* 5 */
		{
			wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
			log_format("wlan event: disconnected: reason: %u, try to reconnect", event->reason);
			wlan_state_since = esp_timer_get_time();
			wlan_state = ws_associating;

			rv = esp_wifi_connect();
			if(rv != ESP_ERR_WIFI_NOT_STARTED)
				util_warn_on_esp_err("esp_wifi_connect", esp_wifi_connect());
			break;
		}
		case(WIFI_EVENT_HOME_CHANNEL_CHANGE): /* 40 */
		{
#if 0
			wifi_event_home_channel_change_t *event = (wifi_event_home_channel_change_t *)event_data;
			log_format("wlan event: home channel change: old: %u, new: %u", event->old_chan, event->new_chan);
#endif
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
	assert(inited);

	switch(event_id)
	{
		case(IP_EVENT_STA_GOT_IP):
		{
#if 0
			string_auto(ip, 32);
			string_auto(netmask, 32);
			string_auto(gw, 32);

			ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

			util_esp_ipv4_addr_to_string(ip, &event->ip_info.ip);
			util_esp_ipv4_addr_to_string(netmask, &event->ip_info.netmask);
			util_esp_ipv4_addr_to_string(gw, &event->ip_info.gw);

			log_format("ip event: got ipv4: changed: %d, ip: %s, netmask: %s, gw: %s",
					event->ip_changed, string_cstr(ip), string_cstr(netmask), string_cstr(gw));
#endif
			wlan_state_since = esp_timer_get_time();
			wlan_state = ws_ipv4_address_acquired;

			util_abort_on_esp_err("esp_netif_create_ip6_linklocal", esp_netif_create_ip6_linklocal(netif));
			util_abort_on_esp_err("esp_netif_sntp_start", esp_netif_sntp_start());

			break;
		}

		case(IP_EVENT_GOT_IP6):
		{
			esp_ip6_addr_t ip6;
#if 0
			string_auto(ip, 128);

			ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
			util_esp_ipv6_addr_to_string(ip, &event->ip6_info.ip);
			log_format("ip event: got ipv6: ip: %s", string_cstr(ip));
#endif
			wlan_state_since = esp_timer_get_time();

			if(esp_netif_get_ip6_global(netif, &ip6))
				wlan_state = ws_ipv6_link_local_address_acquired;
			else
				wlan_state = ws_ipv6_slaac_address_acquired;

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
	string_auto_init(hostname_key, "hostname");
	string_auto(hostname, 16);
	wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
	esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(0, {});

	if(!config_get_string(hostname_key, hostname))
		string_assign_cstr(hostname, "esp32s3");

	init_config.ampdu_rx_enable = 1;
	init_config.ampdu_tx_enable = 1;
	init_config.amsdu_tx_enable = 1;
	init_config.nvs_enable = 1;
	init_config.wifi_task_core_id = 0;

	sntp_config.start = false;
	sntp_config.server_from_dhcp = true;

	assert(!inited);

	util_abort_on_esp_err("esp_event_loop_create_default", esp_event_loop_create_default());
	util_abort_on_esp_err("esp_event_handler_instance_register 1",
			esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wlan_event_handler, (void *)0, (esp_event_handler_instance_t *)0));
	util_abort_on_esp_err("esp_event_handler_instance_register 2",
			esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, ip_event_handler, (void *)0, (esp_event_handler_instance_t *)0));

	util_abort_on_esp_err("esp_netif_init", esp_netif_init());
	netif = esp_netif_create_default_wifi_sta();
	util_abort_on_esp_err("esp_netif_sntp_init", esp_netif_sntp_init(&sntp_config));
	util_abort_on_esp_err("esp_wifi_init", esp_wifi_init(&init_config));
	util_abort_on_esp_err("esp_wifi_set_mode", esp_wifi_set_mode(WIFI_MODE_STA));
	util_abort_on_esp_err("esp_wifi_config_11b_rate", esp_wifi_config_11b_rate(WIFI_IF_STA, true));
	util_abort_on_esp_err("esp_wifi_start", esp_wifi_start());

	wlan_state_since = esp_timer_get_time();
	wlan_state = ws_init;
	inited = true;

	util_abort_on_esp_err("esp_netif_set_hostname", esp_netif_set_hostname(netif, string_cstr(hostname)));
}
