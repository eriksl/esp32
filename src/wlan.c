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

void wlan_command_client_config(cli_command_call_t *call)
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

void wlan_command_info(cli_command_call_t *call)
{
	esp_netif_t *default_netif;
	esp_netif_ip_info_t ip_info;
	esp_netif_flags_t if_flags;
	wifi_ap_record_t ap_info;
	const char *hostname, *key;
	uint8_t mac[6];
	string_auto(mac_str, 32);
	string_auto(ipv4_str, 32);
	string_auto(ipv6_str, 64);
	char ifname[16];
	esp_ip6_addr_t esp_ip6_addr[8];
	wifi_mode_t wlan_mode;
	int ix, rv;
	wifi_ps_type_t ps_type;
	uint8_t protocol_bitmap;
	uint16_t timeout;
	wifi_bandwidth_t wbw;
	wifi_phy_mode_t mode;

	assert(inited);
	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "WLAN INFO");

	default_netif = esp_netif_get_default_netif();

	string_format_append(call->result, "\ncurrent state: %s, since %llu seconds",
			wlan_state_to_string(wlan_state), (esp_timer_get_time() - wlan_state_since) / 1000000);

	if((rv = esp_wifi_get_mode(&wlan_mode)))
	{
		util_warn_on_esp_err("esp_wifi_get_mode", rv);
		key = "<invalid>";
	}
	else
	{
		switch(wlan_mode)
		{
			case(WIFI_MODE_NULL): { key = "null"; break; }
			case(WIFI_MODE_STA): { key = "station"; break; }
			case(WIFI_MODE_AP): { key = "access point"; break; }
			case(WIFI_MODE_APSTA): { key = "station and access point"; break; }
			case(WIFI_MODE_NAN): { key = "NAN mode"; break; }
			default: { key = "<unknown>"; break; }
		}
	}

	string_format_append(call->result, "\noperating mode: %s", key);
	string_append_cstr(call->result, "\nwlan status:");

	if((rv = esp_wifi_sta_get_ap_info(&ap_info)))
	{
		util_warn_on_esp_err("esp_wifi_sta_get_ap_info", rv);
		string_append_cstr(call->result, " <no info>");
	}
	else
	{
		util_mac_addr_to_string(mac_str, ap_info.bssid, false);
		string_format_append(call->result, "\n- access point: %s", string_cstr(mac_str));
		string_format_append(call->result, "\n- SSID: %s", ap_info.ssid);
		string_append_cstr(call->result, "\n- channel: ");

		if(ap_info.second == WIFI_SECOND_CHAN_ABOVE)
			string_format_append(call->result, "%u+%u", ap_info.primary, ap_info.primary + 1);
		else
			if(ap_info.second == WIFI_SECOND_CHAN_BELOW)
				string_format_append(call->result, "%u+%u", ap_info.primary, ap_info.primary - 1);
			else
				string_format_append(call->result, "%u", ap_info.primary);

		string_format_append(call->result, "\n- rssi: %d", ap_info.rssi);

		switch(ap_info.authmode)
		{
			case(WIFI_AUTH_OPEN): key = "open"; break;
			case(WIFI_AUTH_WEP): key = "wep"; break;
			case(WIFI_AUTH_WPA_PSK): key = "wpa psk"; break;
			case(WIFI_AUTH_WPA2_PSK): key = "wpa2 psk"; break;
			case(WIFI_AUTH_WPA_WPA2_PSK): key = "wpa+wpa2 psk"; break;
			case(WIFI_AUTH_ENTERPRISE): key = "wpa+wpa2 802.1x"; break;
			case(WIFI_AUTH_WPA3_PSK): key = "wpa3 psk"; break;
			case(WIFI_AUTH_WPA2_WPA3_PSK): key = "wpa2+wpa3 psk"; break;
			case(WIFI_AUTH_WAPI_PSK): key = "wapi psk"; break;
			case(WIFI_AUTH_OWE): key = "owe"; break;
			case(WIFI_AUTH_WPA3_ENT_192): key = "wpa3 802.1x 192 bits"; break;
			case(WIFI_AUTH_WPA3_EXT_PSK): key = "wpa3 802.1x psk"; break;
			case(WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE): key = "wpa3 802.1x psk mixed"; break;
			default: key = "<invalid>"; break;
		}

		string_format_append(call->result, "\n- authentication mode: %s", key);

		switch(ap_info.pairwise_cipher)
		{
			case(WIFI_CIPHER_TYPE_NONE): key = "none"; break;
			case(WIFI_CIPHER_TYPE_WEP40): key = "wep 40"; break;
			case(WIFI_CIPHER_TYPE_WEP104): key = "wep 104"; break;
			case(WIFI_CIPHER_TYPE_TKIP): key = "tkip"; break;
			case(WIFI_CIPHER_TYPE_CCMP): key = "ccmp"; break;
			case(WIFI_CIPHER_TYPE_TKIP_CCMP): key = "tkip+ccmp"; break;
			case(WIFI_CIPHER_TYPE_AES_CMAC128): key = "aes cmac128"; break;
			case(WIFI_CIPHER_TYPE_SMS4): key = "sms 4"; break;
			case(WIFI_CIPHER_TYPE_GCMP): key = "gcmp"; break;
			case(WIFI_CIPHER_TYPE_GCMP256): key = "gcmp 256"; break;
			case(WIFI_CIPHER_TYPE_AES_GMAC128): key = "aes gmac 128"; break;
			case(WIFI_CIPHER_TYPE_AES_GMAC256): key = "aes gmac 256"; break;
			default: key = "<invalid>"; break;
		}

		string_format_append(call->result, "\n- pairwise cipher: %s", key);

		switch(ap_info.group_cipher)
		{
			case(WIFI_CIPHER_TYPE_NONE): key = "none"; break;
			case(WIFI_CIPHER_TYPE_WEP40): key = "wep 40"; break;
			case(WIFI_CIPHER_TYPE_WEP104): key = "wep 104"; break;
			case(WIFI_CIPHER_TYPE_TKIP): key = "tkip"; break;
			case(WIFI_CIPHER_TYPE_CCMP): key = "ccmp"; break;
			case(WIFI_CIPHER_TYPE_TKIP_CCMP): key = "tkip+ccmp"; break;
			case(WIFI_CIPHER_TYPE_AES_CMAC128): key = "aes cmac128"; break;
			case(WIFI_CIPHER_TYPE_SMS4): key = "sms 4"; break;
			case(WIFI_CIPHER_TYPE_GCMP): key = "gcmp"; break;
			case(WIFI_CIPHER_TYPE_GCMP256): key = "gcmp 256"; break;
			case(WIFI_CIPHER_TYPE_AES_GMAC128): key = "aes gmac 128"; break;
			case(WIFI_CIPHER_TYPE_AES_GMAC256): key = "aes gmac 256"; break;
			default: key = "<invalid>"; break;
		}

		string_format_append(call->result, "\n- group cipher: %s", key);
		string_format_append(call->result, "\n- country: %.2s [%u-%u], max power: %d dB",
				ap_info.country.cc, ap_info.country.schan, ap_info.country.nchan - ap_info.country.schan + 1, ap_info.country.max_tx_power);
	}

	string_append_cstr(call->result, "\n- protocols:");

	if((rv = esp_wifi_get_protocol(WIFI_IF_STA, &protocol_bitmap)))
	{
		util_warn_on_esp_err("esp_wifi_get_protocol", rv);
		string_append_cstr(call->result, " <invalid>");
	}
	else
	{
		if(protocol_bitmap & WIFI_PROTOCOL_11B)
			string_append_cstr(call->result, " 802.11b");

		if(protocol_bitmap & WIFI_PROTOCOL_11G)
			string_append_cstr(call->result, " 802.11g");

		if(protocol_bitmap & WIFI_PROTOCOL_11N)
			string_append_cstr(call->result, " 802.11n");

		if(protocol_bitmap & WIFI_PROTOCOL_11AX)
			string_append_cstr(call->result, " 802.11ax");
	}

	string_append_cstr(call->result, ", bandwidth: ");

	if((rv = esp_wifi_get_bandwidth(WIFI_IF_STA, &wbw)))
	{
		util_warn_on_esp_err("esp_wifi_get_bandwidth", rv);
		string_append_cstr(call->result, "<invalid>");
	}
	else
		string_append_cstr(call->result, (wbw == WIFI_BW_HT40) ? "ht40" : "ht20");

	if((rv = esp_wifi_sta_get_negotiated_phymode(&mode)))
	{
		util_warn_on_esp_err("esp_wifi_sta_get_negotiated_phymode", rv);
		key = "<invalid>";
	}
	else
	{
		switch(mode)
		{
			case(WIFI_PHY_MODE_LR): key =	"low rate"; break;
			case(WIFI_PHY_MODE_11B): key =	"802.11b"; break;
			case(WIFI_PHY_MODE_11G): key =	"802.11g"; break;
			case(WIFI_PHY_MODE_HT20): key =	"802.11n ht20"; break;
			case(WIFI_PHY_MODE_HT40): key =	"802.11n ht40"; break;
			case(WIFI_PHY_MODE_HE20): key =	"802.11ax he20"; break;
			default: key =					"<invalid>"; break;
		}
	}

	string_format_append(call->result, "\n- phy mode: %s", key);

	string_format_append(call->result, "\n- TSF timestamp: %llu", esp_wifi_get_tsf_time(WIFI_IF_STA));

	string_append_cstr(call->result, "\n- configured inactive time: ");

	if((rv = esp_wifi_get_inactive_time(WIFI_IF_STA, &timeout)))
	{
		util_warn_on_esp_err("esp_wifi_get_inactive_time", rv);
		string_append_cstr(call->result, "<invalid>");
	}
	else
		string_format_append(call->result, "%u", timeout);

	if((rv = esp_wifi_get_ps(&ps_type)))
	{
		util_warn_on_esp_err("esp_wifi_get_ps", rv);
		key = "<invalid>";
	}
	else
	{
		switch(ps_type)
		{
			case(WIFI_PS_NONE): key = "none"; break;
			case(WIFI_PS_MIN_MODEM): key = "minimal"; break;
			case(WIFI_PS_MAX_MODEM): key = "maximal"; break;
			default: key = "<unknown>"; break;
		}
	}

	string_format_append(call->result, "\n- power saving: %s", key);

	if(!default_netif)
	{
		string_append_cstr(call->result, "\n* netif not ready *");
		return;
	}

	string_append_cstr(call->result, "\nhostname: ");

	if((rv = esp_netif_get_hostname(default_netif, &hostname)))
	{
		util_warn_on_esp_err("esp_netif_get_hostname", rv);
		string_append_cstr(call->result, "<unknown>");
	}
	else
		string_append_cstr(call->result, hostname);

	string_append_cstr(call->result, "\ninterface:");
	string_format_append(call->result, "\n- number of interfaces: %u", esp_netif_get_nr_of_ifs());
	string_format_append(call->result, "\n- index: %u", esp_netif_get_netif_impl_index(default_netif));

	util_abort_on_esp_err("esp_netif_get_netif_impl_name", esp_netif_get_netif_impl_name(default_netif, ifname));

	string_format_append(call->result, "\n- name: %s", ifname);

	key = esp_netif_get_ifkey(default_netif);

	string_format_append(call->result, "\n- key: %s", key ? key : "<invalid>");

	key = esp_netif_get_desc(default_netif);

	string_format_append(call->result, "\n- description: %s", key ? key : "<invalid>");

	string_append_cstr(call->result, "\n- flags:");

	if_flags = esp_netif_get_flags(default_netif);

	if(if_flags & ESP_NETIF_DHCP_CLIENT)
		string_append_cstr(call->result, " dhcp-client");

	if(if_flags & ESP_NETIF_DHCP_SERVER)
		string_append_cstr(call->result, " dhcp-server");

	if(if_flags & ESP_NETIF_FLAG_AUTOUP)
		string_append_cstr(call->result, " auto-up");

	if(if_flags & ESP_NETIF_FLAG_GARP)
		string_append_cstr(call->result, " garp");

	if(if_flags & ESP_NETIF_FLAG_EVENT_IP_MODIFIED)
		string_append_cstr(call->result, " event-ip-modified");

	if(if_flags & ESP_NETIF_FLAG_MLDV6_REPORT)
		string_append_cstr(call->result, " mldv6-report");

	string_append_cstr(call->result, "\nmac:\n- address:");

	if((rv = esp_netif_get_mac(default_netif, mac)))
	{
		util_warn_on_esp_err("esp_netif_get_mac", rv);
		string_append_cstr(call->result, "<unknown>");
	}
	else
	{
		util_mac_addr_to_string(mac_str, mac, false);
		string_append_string(call->result, mac_str);
	}

	string_append_cstr(call->result, "\nipv4:");

	if((rv = esp_netif_get_ip_info(default_netif, &ip_info)))
	{
		util_warn_on_esp_err("esp_netif_get_ip_info", rv);
		string_append_cstr(call->result, "\n- interface address: <unknown>");
		string_append_cstr(call->result, "\n- gateway address: <unknown>");
		string_append_cstr(call->result, "\n- netmask: <unknown>");
	}
	else
	{
		util_esp_ipv4_addr_to_string(ipv4_str, &ip_info.ip);
		string_append_cstr(call->result, "\n- interface address: ");
		string_append_string(call->result, ipv4_str);
		string_append_cstr(call->result, "\n- gateway address: ");
		util_esp_ipv4_addr_to_string(ipv4_str, &ip_info.gw);
		string_append_string(call->result, ipv4_str);
		string_append_cstr(call->result, "\n- netmask: ");
		util_esp_ipv4_addr_to_string(ipv4_str, &ip_info.netmask);
		string_append_string(call->result, ipv4_str);
	}

	string_append_cstr(call->result, "\nipv6:");

	rv = esp_netif_get_all_ip6(default_netif, esp_ip6_addr);

	for(ix = 0; ix < rv; ix++)
	{
		util_esp_ipv6_addr_to_string(ipv6_str, &esp_ip6_addr[ix]);
		string_format_append(call->result, "\n- address %u: %s", ix, string_cstr(ipv6_str));
	}
}

#if 0
	if((rv = esp_wifi_config_80211_tx_rate(WIFI_IF_STA, &rate)))
	{
		util_warn_on_esp_err("esp_wifi_config_80211_tx_rate", rv);
		key = "<invalid>";
	}
	else
	{
		switch(rate)
		{
			case(WIFI_PHY_RATE_1M_L): key =			"802.11b 1 Mbps long preamble"; break;
			case(WIFI_PHY_RATE_2M_L): key =			"802.11b 2 Mbps long preamble"; break;
			case(WIFI_PHY_RATE_5M_L): key =			"802.11b 5 Mbps long preamble"; break;
			case(WIFI_PHY_RATE_11M_L): key =		"802.11b 11 Mbps long preamble"; break;
			case(WIFI_PHY_RATE_2M_S): key =			"802.11b 2 Mbps short preamble"; break;
			case(WIFI_PHY_RATE_5M_S): key =			"802.11b 5 Mbps short preamble"; break;
			case(WIFI_PHY_RATE_11M_S): key =		"802.11b 11 Mbps short preamble"; break;
			case(WIFI_PHY_RATE_48M): key =			"802.11g 48 Mbps"; break;
			case(WIFI_PHY_RATE_24M): key =			"802.11g 24 Mbps"; break;
			case(WIFI_PHY_RATE_12M): key =			"802.11g 12 Mbps"; break;
			case(WIFI_PHY_RATE_6M): key =			"802.11g 6 Mbps"; break;
			case(WIFI_PHY_RATE_54M): key =			"802.11g 54 Mbps"; break;
			case(WIFI_PHY_RATE_36M): key =			"802.11g 36 Mbps"; break;
			case(WIFI_PHY_RATE_18M): key =			"802.11g 18 Mbps"; break;
			case(WIFI_PHY_RATE_9M): key =			"802.11g 9 Mbps"; break;
			case(WIFI_PHY_RATE_MCS0_LGI): key =		"802.11n mcs0 large guard interval"; break;
			case(WIFI_PHY_RATE_MCS1_LGI): key =		"802.11n mcs1 large guard interval"; break;
			case(WIFI_PHY_RATE_MCS2_LGI): key =		"802.11n mcs2 large guard interval"; break;
			case(WIFI_PHY_RATE_MCS3_LGI): key =		"802.11n mcs3 large guard interval"; break;
			case(WIFI_PHY_RATE_MCS4_LGI): key =		"802.11n mcs4 large guard interval"; break;
			case(WIFI_PHY_RATE_MCS5_LGI): key =		"802.11n mcs5 large guard interval"; break;
			case(WIFI_PHY_RATE_MCS6_LGI): key =		"802.11n mcs6 large guard interval"; break;
			case(WIFI_PHY_RATE_MCS7_LGI): key =		"802.11n mcs7 large guard interval"; break;
			case(WIFI_PHY_RATE_MCS0_SGI): key =		"802.11n mcs0 short guard interval"; break;
			case(WIFI_PHY_RATE_MCS1_SGI): key =		"802.11n mcs1 short guard interval"; break;
			case(WIFI_PHY_RATE_MCS2_SGI): key =		"802.11n mcs2 short guard interval"; break;
			case(WIFI_PHY_RATE_MCS3_SGI): key =		"802.11n mcs3 short guard interval"; break;
			case(WIFI_PHY_RATE_MCS4_SGI): key =		"802.11n mcs4 short guard interval"; break;
			case(WIFI_PHY_RATE_MCS5_SGI): key =		"802.11n mcs5 short guard interval"; break;
			case(WIFI_PHY_RATE_MCS6_SGI): key =		"802.11n mcs6 short guard interval"; break;
			case(WIFI_PHY_RATE_MCS7_SGI): key =		"802.11n mcs7 short guard interval"; break;
			case(WIFI_PHY_RATE_LORA_250K): key =	"lora 250 kbps"; break;
			case(WIFI_PHY_RATE_LORA_500K): key =	"lora 500 kbps"; break;
			default: key =							"<invalid>"; break;
		}
	}

	string_format_append(call->result, "\ntx rate: %s", key);
#endif
