#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <lwip/netif.h>

#include "wlan.h"

#include "cli.h"
#include "config.h"
#include "log.h"
#include "util.h"
#include "packet.h"
#include "cli-command.h"
#include "notify.h"
#include "exception.h"

#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_netif_net_stack.h>
#include <esp_netif_sntp.h>
#include <esp_timer.h>

#include <string>
#include <boost/format.hpp>

typedef enum
{
	ws_invalid,
	ws_init,
	ws_associating,
	ws_associated,
	ws_ipv4_address_acquired,
	ws_ipv6_link_local_address_acquired,
	ws_ipv6_slaac_address_acquired,
	ws_ipv6_static_address_active,
	ws_rescue_ap_mode_init,
	ws_rescue_ap_mode_idle,
	ws_rescue_ap_mode_associated,
	ws_size,
} wlan_state_t;

static const char *key_ipv6_static_address = "ipv6-address";

static bool inited = false;
static esp_netif_t *netif_sta;
static esp_netif_t *netif_ap;
static wlan_state_t state = ws_invalid;
static unsigned int state_time;
static TimerHandle_t state_timer;

static bool static_ipv6_address_set;
static esp_ip6_addr_t static_ipv6_address;

typedef struct
{
	uint32_t valid_transitions;
	const char *name;
	Notify::Notification notification;
} state_info_t;

static const state_info_t state_info[ws_size] =
{
	[ws_invalid] =							{ (1 << ws_init),
											"invalid", Notify::Notification::none },
	[ws_init] =								{ (1 << ws_init) | (1 << ws_associating),
											"init", Notify::Notification::none },
	[ws_associating] =						{ (1 << ws_init) | (1 << ws_associating) | (1 << ws_associated)  | (1 << ws_rescue_ap_mode_idle) | (1 << ws_rescue_ap_mode_associated),
											"associating", Notify::Notification::net_associating },
	[ws_associated] =						{ (1 << ws_init) | (1 << ws_associating) | (1 << ws_ipv4_address_acquired) | (1 << ws_ipv6_link_local_address_acquired),
											"associated", Notify::Notification::net_associating_finished },
	[ws_ipv4_address_acquired] =			{ (1 << ws_init) | (1 << ws_associating) | (1 << ws_ipv6_link_local_address_acquired),
											"ipv4 address acquired", Notify::Notification::net_ipv4_acquired },
	[ws_ipv6_link_local_address_acquired] =	{ (1 << ws_init) | (1 << ws_associating) | (1 << ws_ipv4_address_acquired) | (1 << ws_ipv6_slaac_address_acquired) | (1 << ws_ipv6_static_address_active),
											"ipv6 link local address acquired", Notify::Notification::net_ipv6_ll_active },
	[ws_ipv6_slaac_address_acquired] =		{ (1 << ws_init) | (1 << ws_associating) | (1 << ws_ipv6_static_address_active),
											"ipv6 autoconfig address acquired", Notify::Notification::net_ipv6_slaac_acquired },
	[ws_ipv6_static_address_active] =		{ (1 << ws_init) | (1 << ws_associating) | (1 << ws_ipv6_slaac_address_acquired) | (1 << ws_ipv6_static_address_active),
											"ipv6 static address set", Notify::Notification::net_ipv6_static_active },
	[ws_rescue_ap_mode_init] =				{ (1 << ws_init) | (1 << ws_associating) | (1 << ws_rescue_ap_mode_idle) | (1 << ws_rescue_ap_mode_associated),
											"rescue access point mode init", Notify::Notification::net_ap_mode_init },
	[ws_rescue_ap_mode_idle] =				{ (1 << ws_rescue_ap_mode_init) | (1 << ws_rescue_ap_mode_associated),
											"rescue access point mode idle", Notify::Notification::net_ap_mode_idle },
	[ws_rescue_ap_mode_associated] =		{ (1 << ws_rescue_ap_mode_idle),
											"rescue access point mode associated", Notify::Notification::net_ap_mode_associated },
};

static const char *wlan_state_to_cstr(wlan_state_t state_in)
{
	assert((state >= 0) && (state < ws_size));
	assert((state_in >= 0) && (state_in < ws_size));

	return(state_info[state_in].name);
}

static void set_state(wlan_state_t state_new)
{
	const char *state_string;
	const char *state_new_string;
	wifi_mode_t wlan_mode;

	assert((state >= 0) && (state < ws_size));
	assert((state_new >= 0) && (state_new < ws_size));

	state_string = wlan_state_to_cstr(state);
	state_new_string = wlan_state_to_cstr(state_new);

#if 0
	log_format("wlan: state from %s to %s", state_string, state_new_string);
#endif

	if((state_new == ws_associating) || (!(state_info[state].valid_transitions & (1 << state_new))))
	{
		if(!(state_info[state].valid_transitions & (1 << state_new)))
			log_format("wlan: invalid state transition from %s (%d) to %s (%d), %x, reassociating", state_string, state, state_new_string, state_new, (unsigned int)state_info[state].valid_transitions);
		else
			if((state != ws_init) && (state != ws_associating))
				log_format("wlan: reassociate, switch from %s to %s", state_string, state_new_string);

		wlan_mode = WIFI_MODE_STA;
		util_warn_on_esp_err("esp_wifi_get_mode", esp_wifi_get_mode(&wlan_mode));

		if(wlan_mode == WIFI_MODE_AP)
		{
			log("wlan: switch from AP mode to STA mode");
			util_warn_on_esp_err("esp_wifi_deauth_sta", esp_wifi_deauth_sta(0));
			util_warn_on_esp_err("esp_wifi_stop", esp_wifi_stop());
			util_warn_on_esp_err("esp_wifi_set_mode", esp_wifi_set_mode(WIFI_MODE_STA));
			util_warn_on_esp_err("esp_wifi_start", esp_wifi_start());
		}
		else
			if((state != ws_init) && (state != ws_associating))
			{
				log("wlan: start disconnect");
				util_warn_on_esp_err("esp_wifi_disconnect", esp_wifi_disconnect());
			}

		util_warn_on_esp_err("esp_wifi_connect", esp_wifi_connect());

		state_new = ws_associating;
	}

	if(state != state_new)
		state_time = 0;

	state = state_new;

	Notify::get().notify(state_info[state].notification);
}

static void state_callback(TimerHandle_t handle)
{
	static wifi_config_t config_ap =
	{
		.ap =
		{
			.ssid = {},
			.password = {},
			.ssid_len = 0,
			.channel = 11,
			.authmode = WIFI_AUTH_WPA2_PSK,
			.ssid_hidden = 0,
			.max_connection = 1,
			.beacon_interval = 100,
			.csa_count = 3,
			.dtim_period = 1,
			.pairwise_cipher = WIFI_CIPHER_TYPE_CCMP,
			.ftm_responder = 0,
			.pmf_cfg =
			{
				.capable = false,
				.required = false,
			},
			.sae_pwe_h2e = {},
			.transition_disable = 0,
			.sae_ext = 0,
			.bss_max_idle_cfg = {},
			.gtk_rekey_interval = 0,
		},
	};

	uint8_t mac_address[6];

	state_time++;

	if(((state == ws_associating) || (state == ws_associated)) && (state_time > 30))
	{
		util_warn_on_esp_err("esp_wifi_get_mac", esp_wifi_get_mac(WIFI_IF_AP, mac_address));
		snprintf((char *)config_ap.ap.ssid, sizeof(config_ap.ap.ssid), "esp32-%02x:%02x:%02x:%02x:%02x:%02x",
				mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
		snprintf((char *)config_ap.ap.password, sizeof(config_ap.ap.password), "rescue-%02x:%02x:%02x:%02x:%02x:%02x",
				mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]);

		log_format("wlan: switching to rescue access point mode (ssid: %s, password: %s)", config_ap.ap.ssid, config_ap.ap.password);
		log_format("wlan: after %u seconds of disassociation", state_time);

		util_warn_on_esp_err("esp_wifi_disconnect", esp_wifi_disconnect());
		util_warn_on_esp_err("esp_wifi_stop", esp_wifi_stop());
		util_warn_on_esp_err("esp_wifi_set_mode", esp_wifi_set_mode(WIFI_MODE_AP));
		util_warn_on_esp_err("esp_wifi_set_config", esp_wifi_set_config(WIFI_IF_AP, &config_ap));

		state = ws_rescue_ap_mode_init;
		state_time = 0;

		util_warn_on_esp_err("esp_wifi_start", esp_wifi_start());
	}

	if(((state == ws_rescue_ap_mode_init) || (state == ws_rescue_ap_mode_idle) || (state == ws_rescue_ap_mode_associated)) && (state_time > 300))
	{
		log_format("wlan: resetting after %u seconds in rescue mode", state_time);
		esp_restart();
	}
}

static void wlan_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	assert(inited);

	switch(event_id)
	{
		case(WIFI_EVENT_STA_START): /* 2 */
		{
			log("wlan: associating");
			set_state(ws_associating);
			break;
		}
		case(WIFI_EVENT_STA_STOP): /* 3 */
		{
			log("wlan: stop");
			set_state(ws_init);
			break;
		}
		case(WIFI_EVENT_STA_CONNECTED): /* 4 */
		{
			util_warn_on_esp_err("esp_netif_create_ip6_linklocal", esp_netif_create_ip6_linklocal(netif_sta));

			if(static_ipv6_address_set) // ugly workaround, skip SLAAC because it also adds it's address as "preferred"
			{
				struct netif *lwip_netif;
				lwip_netif = (struct netif *)esp_netif_get_netif_impl(netif_sta);
				lwip_netif->ip6_autoconfig_enabled = 0;
			}

			log("wlan: associated");

			set_state(ws_associated);
			break;
		}
		case(WIFI_EVENT_STA_DISCONNECTED): /* 5 */
		{
			wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
			log_format("wlan: disconnected: reason: %x", event->reason);
			set_state(ws_associating);
			break;
		}
		case(WIFI_EVENT_AP_START):
		{
			log("wlan: start access point");
			set_state(ws_rescue_ap_mode_idle);
			break;
		}
		case(WIFI_EVENT_AP_STOP):
		{
			log("wlan: stop access point");
			set_state(ws_rescue_ap_mode_init);
			break;
		}
		case(WIFI_EVENT_AP_STACONNECTED):
		{
			log("wlan: access point associated");
			set_state(ws_rescue_ap_mode_associated);
			break;
		}
		case(WIFI_EVENT_AP_STADISCONNECTED):
		{
			log("wlan: access point deassociated");
			set_state(ws_rescue_ap_mode_idle);
			break;
		}
		case(WIFI_EVENT_AP_PROBEREQRECVED):
		{
			log("wlan: ap probe received");
			break;
		}
		case(WIFI_EVENT_STA_BEACON_TIMEOUT): /* 21 */
		{
			log("wlan: beacon timeout");
			break;
		}
		case(WIFI_EVENT_HOME_CHANNEL_CHANGE): /* 40 */
		{
			log("wlan: home channel change");
			break;
		}
		default:
		{
			log_format("wlan: unknown event: %ld", event_id);
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
			const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;

			util_abort_on_esp_err("esp_netif_sntp_start", esp_netif_sntp_start());

			log_format("wlan: ipv4: " IPSTR " (mask: " IPSTR ", gw: " IPSTR ")",
					IP2STR(&event->ip_info.ip), IP2STR(&event->ip_info.netmask), IP2STR(&event->ip_info.gw));

			set_state(ws_ipv4_address_acquired);

			break;
		}

		case(IP_EVENT_GOT_IP6):
		{
			const ip_event_got_ip6_t *event = (const ip_event_got_ip6_t *)event_data;
			const char *address_type;

			switch(util_ipv6_address_type(reinterpret_cast<const uint8_t *>(&event->ip6_info.ip.addr)))
			{
				case(ipv6_address_link_local):
				{
					address_type = "link-local";

					set_state(ws_ipv6_link_local_address_acquired);

					if(static_ipv6_address_set)
						util_warn_on_esp_err("esp_netif_add_ip6_address", esp_netif_add_ip6_address(netif_sta, static_ipv6_address, true));

					break;
				}
				case(ipv6_address_global_slaac):
				{
					address_type = "SLAAC";

					set_state(ws_ipv6_slaac_address_acquired);

					break;
				}
				case(ipv6_address_global_static):
				{
					address_type = "static";

					set_state(ws_ipv6_static_address_active);

					break;
				}
				default:
				{
					address_type = "invalid";

					log("wlan: invalid IPv6 address received");

					break;
				}
			}

			log_format("wlan: %s ipv6: %s", address_type, util_ipv6_addr_to_string(reinterpret_cast<const uint8_t *>(&event->ip6_info.ip.addr[0])).c_str());

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
	std::string value;

	assert(inited);
	assert(call->parameter_count  < 3);

	if(call->parameter_count > 1)
		Config::get().set_string("wlan-passwd", call->parameters[1].str);

	if(call->parameter_count > 0)
		Config::get().set_string("wlan-ssid", call->parameters[0].str);

	call->result = "client ssid: ";

	try
	{
		call->result += Config::get().get_string("wlan-ssid");
	}
	catch(transient_exception &)
	{
		call->result += "<unset>";
	}

	call->result += "\nclient password: ";

	try
	{
		call->result += Config::get().get_string("wlan-passwd");
	}
	catch(transient_exception &)
	{
		call->result += "<unset>";
	}

	if(call->parameter_count > 1)
	{
		static wifi_config_t config_sta =
		{
			.sta =
			{
				.ssid = {},
				.password = {},
				.scan_method = WIFI_ALL_CHANNEL_SCAN,
				.bssid_set = 0,
				.bssid = {},
				.channel = 0,
				.listen_interval = 3,
				.sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
				.threshold = {},
				.pmf_cfg =
				{
					.capable = false,
					.required = false,
				},
				.rm_enabled = 0,
				.btm_enabled = 0,
				.mbo_enabled = 0,
				.ft_enabled = 0,
				.owe_enabled = 0,
				.transition_disable = 0,
				.reserved1 = 0,
				.sae_pwe_h2e = {},
				.sae_pk_mode = {},
				.failure_retry_cnt = 1,
  				.he_dcm_set = 0,
    			.he_dcm_max_constellation_tx = 0,
    			.he_dcm_max_constellation_rx = 0,
    			.he_mcs9_enabled = 0,
    			.he_su_beamformee_disabled = 0,
    			.he_trig_su_bmforming_feedback_disabled = 0,
    			.he_trig_mu_bmforming_partial_feedback_disabled = 0,
    			.he_trig_cqi_feedback_disabled = 0,
    			.vht_su_beamformee_disabled = 0,
    			.vht_mu_beamformee_disabled = 0,
    			.vht_mcs8_enabled = 0,
    			.reserved2 = 0,
    			.sae_h2e_identifier = {},
			},
		};

		esp_err_t rv;
		wifi_mode_t wlan_mode;

		wlan_mode = WIFI_MODE_STA;
		util_warn_on_esp_err("esp_wifi_get_mode", esp_wifi_get_mode(&wlan_mode));

		if(wlan_mode == WIFI_MODE_AP)
		{
			util_warn_on_esp_err("esp_wifi_deauth_sta", esp_wifi_deauth_sta(0));
			util_warn_on_esp_err("esp_wifi_stop", esp_wifi_stop());
			util_warn_on_esp_err("esp_wifi_set_mode", esp_wifi_set_mode(WIFI_MODE_STA));
			util_warn_on_esp_err("esp_wifi_start", esp_wifi_start());
		}

		call->parameters[0].str.copy(reinterpret_cast<char *>(config_sta.sta.ssid), sizeof(config_sta.sta.ssid));
		call->parameters[1].str.copy(reinterpret_cast<char *>(config_sta.sta.password), sizeof(config_sta.sta.password));

		rv = esp_wifi_set_config(WIFI_IF_STA, &config_sta);

		util_warn_on_esp_err("esp_wifi_set_config", rv);

		if(rv)
		{
			call->result = "\nesp_wifi_set_config returns error";
			return;
		}

		set_state(ws_associating);
	}
}

void wlan_command_ipv6_static(cli_command_call_t *call)
{
	std::string ipv6_address_string;
	esp_ip6_addr_t ipv6_address;

	assert(inited);
	assert(call->parameter_count < 2);

	if(call->parameter_count > 0)
	{
		if(esp_netif_str_to_ip6(call->parameters[0].str.c_str(), &ipv6_address))
		{
			call->result = "invalid ipv6 address";
			return;
		}

		ipv6_address_string = ip6addr_ntoa((const ip6_addr_t *)&ipv6_address);

		for(auto &c : ipv6_address_string)
			c = std::tolower(c);

		Config::get().set_string(key_ipv6_static_address, ipv6_address_string);

		static_ipv6_address = ipv6_address;
		static_ipv6_address_set = true;
	}

	call->result = "ipv6 static address: ";

	try
	{
		call->result += Config::get().get_string(key_ipv6_static_address);
	}
	catch(transient_exception &)
	{
		call->result += "<unset>";
	}
}

void wlan_init(void)
{
	std::string hostname;
	std::string ipv6_address_string;
	wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
	static esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(0, {});

	assert(!inited);

	init_config.ampdu_rx_enable = 1;
	init_config.ampdu_tx_enable = 1;
	init_config.amsdu_tx_enable = 0;
	init_config.nvs_enable = 1;
	init_config.wifi_task_core_id = 0;

	try
	{
		hostname = Config::get().get_string("hostname");
	}
	catch(transient_exception &e)
	{
		hostname = "esp32s3";
	}

	sntp_config.start = false;
	sntp_config.server_from_dhcp = true;

	state_timer = xTimerCreate("wlan-state", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, state_callback);
	assert(state_timer);

	set_state(ws_init);

	try
	{
		ipv6_address_string = Config::get().get_string(key_ipv6_static_address);
		static_ipv6_address_set = esp_netif_str_to_ip6(ipv6_address_string.c_str(), &static_ipv6_address) == ESP_OK;
	}
	catch(transient_exception &)
	{
		static_ipv6_address_set = false;
	}

	inited = true;

	util_abort_on_esp_err("esp_event_loop_create_default", esp_event_loop_create_default());

	util_abort_on_esp_err("esp_netif_init", esp_netif_init());
	netif_sta = esp_netif_create_default_wifi_sta();
	netif_ap = esp_netif_create_default_wifi_ap();

	util_abort_on_esp_err("esp_event_handler_instance_register 1",
			esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wlan_event_handler, (void *)0, (esp_event_handler_instance_t *)0));
	util_abort_on_esp_err("esp_event_handler_instance_register 2",
			esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, ip_event_handler, (void *)0, (esp_event_handler_instance_t *)0));

	util_abort_on_esp_err("esp_netif_sntp_init", esp_netif_sntp_init(&sntp_config));
	util_abort_on_esp_err("esp_wifi_init", esp_wifi_init(&init_config));
	util_abort_on_esp_err("esp_wifi_set_mode", esp_wifi_set_mode(WIFI_MODE_STA));
	util_abort_on_esp_err("esp_wifi_config_11b_rate", esp_wifi_config_11b_rate(WIFI_IF_STA, true)); // FIXME
	util_abort_on_esp_err("esp_wifi_config_11b_rate", esp_wifi_config_11b_rate(WIFI_IF_AP, true)); // FIXME
	util_abort_on_esp_err("esp_wifi_start", esp_wifi_start());

	util_abort_on_esp_err("esp_netif_set_hostname", esp_netif_set_hostname(netif_sta, hostname.c_str()));
	util_abort_on_esp_err("esp_netif_set_hostname", esp_netif_set_hostname(netif_ap, hostname.c_str()));

	xTimerStart(state_timer, portMAX_DELAY);
}

void wlan_command_info(cli_command_call_t *call)
{
	esp_netif_t *netif;
	esp_netif_ip_info_t ip_info;
	esp_netif_flags_t if_flags;
	wifi_ap_record_t ap_info;
	const char *hostname, *key;
	uint8_t mac[6];
	char ifname[16];
	esp_ip6_addr_t esp_ip6_addr[8];
	wifi_mode_t wlan_mode;
	wifi_ps_type_t ps_type;
	unsigned int ix, rv;

	assert(inited);
	assert(call->parameter_count == 0);

	call->result = "WLAN INFO";
	call->result += (boost::format("\ncurrent state: %s, since %u seconds ago") % wlan_state_to_cstr(state) % state_time).str();

	if((rv = esp_wifi_get_mode(&wlan_mode)))
	{
		util_warn_on_esp_err("esp_wifi_get_mode", rv);
		call->result += "no information";
		return;
	}

	call->result += "\noperating mode: ";

	if(wlan_mode == WIFI_MODE_AP)
	{
		call->result += "access point";
		netif = netif_ap;
	}
	else
	{
		call->result += "station";
		netif = netif_sta;
	}

	call->result += "\ninterface:";
	call->result += (boost::format("\n- number of interfaces: %u") % esp_netif_get_nr_of_ifs()).str();
	call->result += (boost::format("\n- index: %d") % esp_netif_get_netif_impl_index(netif)).str();

	util_abort_on_esp_err("esp_netif_get_netif_impl_name", esp_netif_get_netif_impl_name(netif, ifname));

	call->result += (boost::format("\n- name: %s") % ifname).str();

	key = esp_netif_get_ifkey(netif);

	call->result += (boost::format("\n- key: %s") % (key ? key : "<invalid>")).str();

	key = esp_netif_get_desc(netif);

	call->result += (boost::format("\n- description: %s") % (key ? key : "<invalid>")).str();
	call->result += "\n- flags:";

	if_flags = esp_netif_get_flags(netif);

	if(if_flags & ESP_NETIF_DHCP_CLIENT)
		call->result += " dhcp-client";

	if(if_flags & ESP_NETIF_DHCP_SERVER)
		call->result += " dhcp-server";

	if(if_flags & ESP_NETIF_FLAG_AUTOUP)
		call->result += " auto-up";

	if(if_flags & ESP_NETIF_FLAG_GARP)
		call->result += " garp";

	if(if_flags & ESP_NETIF_FLAG_EVENT_IP_MODIFIED)
		call->result += " event-ip-modified";

	if(if_flags & ESP_NETIF_FLAG_MLDV6_REPORT)
		call->result += " mldv6-report";

	call->result += "\nmac:\n- address:";

	if((rv = esp_netif_get_mac(netif, mac)))
	{
		util_warn_on_esp_err("esp_netif_get_mac", rv);
		call->result += "<unknown>";
	}
	else
		call->result += util_mac_addr_to_string(mac, false);

	call->result += "\nipv4:";

	if((rv = esp_netif_get_ip_info(netif, &ip_info)))
	{
		util_warn_on_esp_err("esp_netif_get_ip_info", rv);
		call->result += "\n- interface address: <unknown>";
		call->result += "\n- gateway address: <unknown>";
		call->result += "\n- netmask: <unknown>";
	}
	else
	{
		call->result += "\n- interface address: ";
		call->result += util_ipv4_addr_to_string(&ip_info.ip.addr);
		call->result += "\n- gateway address: ";
		call->result += util_ipv4_addr_to_string(&ip_info.gw.addr);
		call->result += "\n- netmask: ";
		call->result += util_ipv4_addr_to_string(&ip_info.netmask.addr);
	}

	call->result += "\nipv6:";

	rv = esp_netif_get_all_ip6(netif, esp_ip6_addr);

	for(ix = 0; ix < rv; ix++)
		call->result += std::format("\n- address {:d}: {} ({})", ix,
				util_ipv6_addr_to_string(reinterpret_cast<const uint8_t *>(&esp_ip6_addr[ix].addr)),
				util_ipv6_address_type_string(reinterpret_cast<const uint8_t *>(&esp_ip6_addr[ix].addr)));

	call->result += "\nhostname: ";

	if((rv = esp_netif_get_hostname(netif, &hostname)))
	{
		util_warn_on_esp_err("esp_netif_get_hostname", rv);
		call->result =+ "<unknown>";
	}
	else
		call->result += hostname;

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

	call->result += (boost::format("\n- power saving: %s") % key).str();

	if(wlan_mode == WIFI_MODE_STA)
	{
		uint8_t protocol_bitmap;
		uint16_t timeout;
		wifi_bandwidth_t wbw;
		wifi_phy_mode_t mode;

		call->result += "\nwlan STA status:";

		if((rv = esp_wifi_sta_get_ap_info(&ap_info)))
		{
			util_warn_on_esp_err("esp_wifi_sta_get_ap_info", rv);
			call->result += " <no info>";
		}
		else
		{
			call->result += (boost::format("\n- access point: %s") % util_mac_addr_to_string(ap_info.bssid, false)).str();
			call->result += (boost::format("\n- SSID: %s") % ap_info.ssid).str();
			call->result += "\n- ";

			if(ap_info.second == WIFI_SECOND_CHAN_ABOVE)
				call->result += (boost::format("channels: %u+%u") % static_cast<unsigned int>(ap_info.primary) % static_cast<unsigned int>(ap_info.primary + 1)).str();
			else
				if(ap_info.second == WIFI_SECOND_CHAN_BELOW)
					call->result += (boost::format("channels: %u+%u") % static_cast<unsigned int>(ap_info.primary) % static_cast<unsigned int>(ap_info.primary - 1)).str();
				else
					call->result += (boost::format("channel: %u") % static_cast<unsigned int>(ap_info.primary)).str();

			call->result += (boost::format("\n- rssi: %d") % static_cast<int>(ap_info.rssi)).str();

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

			call->result += (boost::format("\n- authentication mode: %s") % key).str();

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

			call->result += (boost::format("\n- pairwise cipher: %s") % key).str();

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

			call->result += (boost::format( "\n- group cipher: %s") % key).str();
			call->result += (boost::format( "\n- country: %.2s [%u-%u], max power: %d dB") %
					ap_info.country.cc %
					ap_info.country.schan %
					(ap_info.country.nchan - ap_info.country.schan + 1) %
					ap_info.country.max_tx_power).str();
		}

		call->result += "\n- protocols:";

		if((rv = esp_wifi_get_protocol(WIFI_IF_STA, &protocol_bitmap)))
		{
			util_warn_on_esp_err("esp_wifi_get_protocol", rv);
			call->result += " <invalid>";
		}
		else
		{
			if(protocol_bitmap & WIFI_PROTOCOL_11B)
				call->result += " 802.11b";

			if(protocol_bitmap & WIFI_PROTOCOL_11G)
				call->result += " 802.11g";

			if(protocol_bitmap & WIFI_PROTOCOL_11N)
				call->result += " 802.11n";

			if(protocol_bitmap & WIFI_PROTOCOL_11AX)
				call->result += " 802.11ax";
		}

		call->result += ", bandwidth: ";

		if((rv = esp_wifi_get_bandwidth(WIFI_IF_STA, &wbw)))
		{
			util_warn_on_esp_err("esp_wifi_get_bandwidth", rv);
			call->result += "<invalid>";
		}
		else
			call->result += (wbw == WIFI_BW_HT40) ? "ht40" : "ht20";

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

		call->result += (boost::format("\n- phy mode: %s") % key).str();
		call->result += (boost::format( "\n- TSF timestamp: %llu") % esp_wifi_get_tsf_time(WIFI_IF_STA)).str();
		call->result += "\n- configured inactive time: ";

		if((rv = esp_wifi_get_inactive_time(WIFI_IF_STA, &timeout)))
		{
			util_warn_on_esp_err("esp_wifi_get_inactive_time", rv);
			call->result += "<invalid>";
		}
		else
			call->result += (boost::format("%u") % timeout).str();
	}
	else
	{
		uint8_t channel;
		wifi_second_chan_t secondary;
		wifi_country_t country;

		call->result += "\nwlan AP status:";

		if((rv = esp_wifi_get_channel(&channel, &secondary)))
		{
			util_warn_on_esp_err("esp_wifi_get_channel", rv);
			channel = 0;
		}

		call->result += (boost::format("\n- channel: %u") % channel).str();
		call->result += "\n- country: ";

		if((rv = esp_wifi_get_country(&country)))
		{
			util_warn_on_esp_err("esp_wifi_get_country", rv);
			call->result += "<invalid>";
		}
		else
			call->result += (boost::format("%.2s") % country.cc).str();
	}
}
