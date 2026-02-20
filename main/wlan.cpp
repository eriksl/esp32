#include "wlan.h"

#include "config.h"
#include "log.h"
#include "notify.h"
#include "system.h"
#include "exception.h"

#include <lwip/netif.h>

#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_netif_net_stack.h>
#include <esp_netif_sntp.h>

#include <freertos/FreeRTOS.h>

#include <format>

#include "magic_enum/magic_enum.hpp"

WLAN *WLAN::singleton = nullptr;

const WLAN::state_info_map_t WLAN::state_info
{
	{ WLAN::state_t::invalid,							{
															(1 << magic_enum::enum_integer(WLAN::state_t::init)),
															"invalid", Notify::Notification::none
														}},
	{ WLAN::state_t::init,								{
															(1 << magic_enum::enum_integer(WLAN::state_t::init)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::associating)),
															"init", Notify::Notification::none
														}},
	{ WLAN::state_t::associating,						{
															(1 << magic_enum::enum_integer(WLAN::state_t::init)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::associating)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::associated)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::rescue_ap_mode_idle)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::rescue_ap_mode_associated)),
															"associating", Notify::Notification::net_associating
														}},
	{ WLAN::state_t::associated,						{
															(1 << magic_enum::enum_integer(WLAN::state_t::init)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::associating)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::ipv4_address_acquired)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::ipv6_link_local_address_acquired)),
															"associated", Notify::Notification::net_associating_finished
														}},
	{ WLAN::state_t::ipv4_address_acquired,				{
															(1 << magic_enum::enum_integer(WLAN::state_t::init)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::associating)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::ipv6_link_local_address_acquired)),
															"ipv4 address acquired", Notify::Notification::net_ipv4_acquired
														}},
	{ WLAN::state_t::ipv6_link_local_address_acquired,	{
															(1 << magic_enum::enum_integer(WLAN::state_t::init)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::associating)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::ipv4_address_acquired)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::ipv6_slaac_address_acquired)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::ipv6_static_address_active)),
															"ipv6 link local address acquired", Notify::Notification::net_ipv6_ll_active
														}},
	{ WLAN::state_t::ipv6_slaac_address_acquired,		{
															(1 << magic_enum::enum_integer(WLAN::state_t::init)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::associating)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::ipv6_static_address_active)),
															"ipv6 autoconfig address acquired", Notify::Notification::net_ipv6_slaac_acquired
														}},
	{ WLAN::state_t::ipv6_static_address_active,		{
															(1 << magic_enum::enum_integer(WLAN::state_t::init)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::associating)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::ipv6_slaac_address_acquired)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::ipv6_static_address_active)),
															"ipv6 static address set", Notify::Notification::net_ipv6_static_active
														}},
	{ WLAN::state_t::rescue_ap_mode_init,				{
															(1 << magic_enum::enum_integer(WLAN::state_t::init)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::associating)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::rescue_ap_mode_idle)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::rescue_ap_mode_associated)),
															"rescue access point mode init", Notify::Notification::net_ap_mode_init
														}},
	{ WLAN::state_t::rescue_ap_mode_idle,				{
															(1 << magic_enum::enum_integer(WLAN::state_t::rescue_ap_mode_init)) |
															(1 << magic_enum::enum_integer(WLAN::state_t::rescue_ap_mode_associated)),
															"rescue access point mode idle", Notify::Notification::net_ap_mode_idle
														}},
	{ WLAN::state_t::rescue_ap_mode_associated,			{
															(1 << magic_enum::enum_integer(WLAN::state_t::rescue_ap_mode_idle)),
															"rescue access point mode associated", Notify::Notification::net_ap_mode_associated
														}},
};

const WLAN::wifi_auth_mode_map_t WLAN::wifi_auth_mode_map
{
	{ WIFI_AUTH_OPEN,						"open" },
	{ WIFI_AUTH_WEP,						"wep" },
	{ WIFI_AUTH_WPA_PSK,					"wpa psk" },
	{ WIFI_AUTH_WPA2_PSK,					"wpa2 psk" },
	{ WIFI_AUTH_WPA_WPA2_PSK,				"wpa or wpa2 psk" },
	{ WIFI_AUTH_ENTERPRISE,					"wpa enterprise" },
	{ WIFI_AUTH_WPA2_ENTERPRISE,			"wpa 2 enterprise" },
	{ WIFI_AUTH_WPA3_PSK,					"wpa3 psk" },
	{ WIFI_AUTH_WPA2_WPA3_PSK,				"wpa2 or wpa3 psk" },
	{ WIFI_AUTH_WAPI_PSK,					"wapi psk" },
	{ WIFI_AUTH_OWE,						"owe" },
	{ WIFI_AUTH_WPA3_ENT_192,				"wpa3 enterprise 192 bit" },
	{ WIFI_AUTH_WPA3_EXT_PSK,				"wpa3 extended psk" },
	{ WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE,	"wpa3 extended psk mixed mode", },
	{ WIFI_AUTH_DPP,						"dpp" },
	{ WIFI_AUTH_WPA3_ENTERPRISE,			"wpa 3 enterprise" },
	{ WIFI_AUTH_WPA2_WPA3_ENTERPRISE,		"wpa2 or wpa enterprise" },
	{ WIFI_AUTH_WPA_ENTERPRISE,				"wpa enterprise" },
};

const WLAN::wifi_cipher_type_map_t WLAN::wifi_cipher_type_map
{
	{ WIFI_CIPHER_TYPE_NONE,		"none" },
	{ WIFI_CIPHER_TYPE_WEP40,		"wep 40" },
	{ WIFI_CIPHER_TYPE_WEP104,		"wep 104" },
	{ WIFI_CIPHER_TYPE_TKIP,		"tkip" },
	{ WIFI_CIPHER_TYPE_CCMP,		"ccmp" },
	{ WIFI_CIPHER_TYPE_TKIP_CCMP,	"tkip or ccmp" },
	{ WIFI_CIPHER_TYPE_AES_CMAC128,	"aes cmac 128" },
	{ WIFI_CIPHER_TYPE_SMS4,		"sms 4" },
	{ WIFI_CIPHER_TYPE_GCMP,		"gcmp" },
	{ WIFI_CIPHER_TYPE_GCMP256,		"gcmp 256" },
	{ WIFI_CIPHER_TYPE_AES_GMAC128,	"aes gmac 128" },
	{ WIFI_CIPHER_TYPE_AES_GMAC256,	"aes gmac 256" },
};

const WLAN::wifi_phy_mode_map_t WLAN::wifi_phy_mode_map
{
	{ WIFI_PHY_MODE_LR,		"low rate"	},
	{ WIFI_PHY_MODE_11B,	"11b"		},
	{ WIFI_PHY_MODE_11G,	"11g"		},
	{ WIFI_PHY_MODE_11A,	"11a"		},
	{ WIFI_PHY_MODE_HT20,	"ht20"		},
	{ WIFI_PHY_MODE_HT40,	"ht40"		},
	{ WIFI_PHY_MODE_HE20,	"he20"		},
	{ WIFI_PHY_MODE_VHT20,	"vht20"		},
};

WLAN::WLAN(Log &log_in, Config &config_in, Notify &notify_in, System & system_in) :
		log(log_in), config(config_in), notify(notify_in), system(system_in)
{
	static esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(0, {});

	esp_err_t rv;
	std::string ipv6_address_string;
	wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();

	init_config.ampdu_rx_enable = 1;
	init_config.ampdu_tx_enable = 1;
	init_config.amsdu_tx_enable = 0;
	init_config.nvs_enable = 1;
	init_config.wifi_task_core_id = 0;

	this->hostname = "esp32s3";

	try
	{
		this->hostname = this->config.get_string("hostname");
	}
	catch(transient_exception &e)
	{
	}

	this->static_ipv6_address = "";

	try
	{
		this->static_ipv6_address = this->config.get_string(this->key_ipv6_static_address);
	}
	catch(transient_exception &)
	{
	}

	sntp_config.start = false;
	sntp_config.server_from_dhcp = true;

	this->state = state_t::init;

	if(!(this->state_timer = xTimerCreate("wlan-state", pdMS_TO_TICKS(1000), pdTRUE, nullptr, this->state_callback_wrapper)))
		throw(hard_exception("WLAN: timer couldn't be created"));

	if((rv = esp_event_loop_create_default()) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "WLAN: esp_event_loop_create_default")));

	if((rv = esp_netif_init()) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "WLAN: esp_netif_init")));

	if(!(this->netif_sta = esp_netif_create_default_wifi_sta()))
		throw(hard_exception("WLAN: cannot create netif_sta"));

	if(!(this->netif_ap = esp_netif_create_default_wifi_ap()))
		throw(hard_exception("WLAN: cannot create netif_ap"));

	if((rv = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, this->wlan_event_handler_wrapper, nullptr, nullptr)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "WLAN: esp_event_handler_instance_register wifi")));

	if((rv = esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, this->ip_event_handler_wrapper, nullptr, nullptr)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "WLAN: esp_event_handler_instance_register ip")));

	if((rv = esp_netif_sntp_init(&sntp_config)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "WLAN: esp_netif_sntp_init")));

	if((rv = esp_wifi_init(&init_config)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "WLAN: esp_wifi_init")));

	if((rv = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "WLAN: esp_wifi_set_mode")));

	if((rv = esp_wifi_config_11b_rate(WIFI_IF_STA, true)) != ESP_OK) // disable 11b rate
		throw(hard_exception(this->log.esp_string_error(rv, "WLAN: esp_wifi_config_11b_rate sta")));

	if((rv = esp_wifi_config_11b_rate(WIFI_IF_AP, true)) != ESP_OK) // disable 11b rate
		throw(hard_exception(this->log.esp_string_error(rv, "WLAN: esp_wifi_config_11b_rate ap")));

	if((rv = esp_netif_set_hostname(this->netif_sta, this->hostname.c_str())) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "WLAN: esp_netif_set_hostname sta")));

	if((rv = esp_netif_set_hostname(this->netif_ap, this->hostname.c_str())) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "WLAN: esp_netif_set_hostname ap")));

	this->singleton = this;
}

void WLAN::run()
{
	esp_err_t rv;

	if((rv = esp_wifi_start()) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "WLAN::run: esp_wifi_start")));

	xTimerStart(this->state_timer, portMAX_DELAY);
}

std::string WLAN::state_to_string(state_t state_in)
{
	if(!magic_enum::enum_contains<state_t>(state_in))
		throw(hard_exception(std::format("WLAN::state_to_string: invalid argument: {:d}", magic_enum::enum_integer(state_in))));

	return(this->state_info.at(state_in).name);
}

void WLAN::set_state(state_t state_new)
{
	std::string state_string;
	std::string state_new_string;
	wifi_mode_t wlan_mode;
	esp_err_t rv;

	if(!magic_enum::enum_contains<state_t>(state_new))
		throw(hard_exception("WLAN::set_state: invalid argument"));

	state_string = this->state_to_string(this->state);
	state_new_string = this->state_to_string(state_new);

#if 0
	this->log << std::format("wlan: state from {} to {}", state_string, state_new_string);
#endif

	if((state_new == state_t::associating) || (!(this->state_info.at(this->state).valid_transitions & (1 << magic_enum::enum_integer(state_new)))))
	{
		if(!(this->state_info.at(this->state).valid_transitions & (1 << magic_enum::enum_integer(state_new))))
			this->log << std::format("wlan: invalid state transition from {} ({:d}) to {} ({:d}), {:#x}, reassociating",
					state_string, static_cast<int>(this->state),
					state_new_string, static_cast<int>(state_new),
					this->state_info.at(this->state).valid_transitions);
		else
			if((this->state != state_t::init) && (this->state != state_t::associating))
				this->log << std::format("wlan: reassociate, switch from {} to {}", state_string, state_new_string);

		wlan_mode = WIFI_MODE_STA;

		if((rv = esp_wifi_get_mode(&wlan_mode)) != ESP_OK)
			throw(transient_exception(this->log.esp_string_error(rv, "WLAN::set_state: esp_wifi_get_mode")));

		if(wlan_mode == WIFI_MODE_AP)
		{
			this->log << "wlan: switch from AP mode to STA mode";

			this->log.warn_on_esp_err("esp_wifi_deauth_sta", esp_wifi_deauth_sta(0));

			if((rv = esp_wifi_stop()) != ESP_OK)
				throw(transient_exception(this->log.esp_string_error(rv, "WLAN::set_state: esp_wifi_stop")));

			if((rv = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK)
				throw(transient_exception(this->log.esp_string_error(rv, "WLAN::set_state: esp_wifi_set_mode")));

			if((rv = esp_wifi_start()) != ESP_OK)
				throw(transient_exception(this->log.esp_string_error(rv, "WLAN::set_state: esp_wifi_start")));
		}
		else
		{
			if((this->state != state_t::init) && (this->state != state_t::associating))
			{
				this->log << "wlan: start disconnect";
				this->log.warn_on_esp_err("esp_wifi_disconnect", esp_wifi_disconnect());
			}
		}

		this->log.warn_on_esp_err("esp_wifi_connect", esp_wifi_connect());

		state_new = state_t::associating;
	}

	if(this->state != state_new)
		this->state_time = 0;

	this->state = state_new;

	this->notify.notify(this->state_info.at(this->state).notification);
}

void WLAN::state_callback_wrapper(TimerHandle_t handle)
{
	if(!WLAN::singleton)
	{
		Console::emergency_wall("WLAN::state_callback_wrapper: not active");
		abort();
	}

	WLAN::singleton->state_callback(handle);
}

void WLAN::state_callback(TimerHandle_t handle)
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

	std::string ssid;
	std::string passwd;
	esp_err_t rv;

	uint8_t mac_address[6];

	this->state_time++;

	if(((this->state == state_t::associating) || (this->state == state_t::associated)) && (this->state_time > 30))
	{
		if((rv = esp_wifi_get_mac(WIFI_IF_AP, mac_address)) != ESP_OK)
			throw(transient_exception(this->log.esp_string_error(rv, "WLAN::state_callback: esp_wifi_get_mac")));

		passwd = ssid = this->system.mac_addr_to_string(std::string_view(reinterpret_cast<const char *>(mac_address), sizeof(mac_address)), false);

		ssid = std::format("esp32-{}", ssid);
		passwd = std::format("rescue-{}", passwd);

		ssid.copy(reinterpret_cast<char *>(config_ap.ap.ssid), sizeof(config_ap.ap.ssid), 0);
		config_ap.ap.ssid[sizeof(config_ap.ap.ssid) - 1] = '\0';

		passwd.copy(reinterpret_cast<char *>(config_ap.ap.password), sizeof(config_ap.ap.password), 0);
		config_ap.ap.password[sizeof(config_ap.ap.password) - 1] = '\0';

		this->log << std::format("wlan: switching to rescue access point mode (ssid: {}, password: {})",
				reinterpret_cast<const char *>(config_ap.ap.ssid), reinterpret_cast<const char *>(config_ap.ap.password));
		this->log << std::format("wlan: after {:d} seconds of disassociation", this->state_time);

		this->log.warn_on_esp_err("esp_wifi_disconnect", esp_wifi_disconnect());
		this->log.warn_on_esp_err("esp_wifi_stop", esp_wifi_stop());
		this->log.warn_on_esp_err("esp_wifi_set_mode", esp_wifi_set_mode(WIFI_MODE_AP));
		this->log.warn_on_esp_err("esp_wifi_set_config", esp_wifi_set_config(WIFI_IF_AP, &config_ap));

		this->state = state_t::rescue_ap_mode_init;
		this->state_time = 0;

		this->log.warn_on_esp_err("esp_wifi_start", esp_wifi_start());
	}

	if(((this->state == state_t::rescue_ap_mode_init) || (this->state == state_t::rescue_ap_mode_idle) || (this->state == state_t::rescue_ap_mode_associated)) && (this->state_time > 300))
	{
		log << std::format("wlan: resetting after {} seconds in rescue mode", this->state_time);
		esp_restart();
	}
}

void WLAN::wlan_event_handler_wrapper(void *arg, esp_event_base_t event_base, std::int32_t event_id, void *event_data)
{
	if(!WLAN::singleton)
	{
		Console::emergency_wall("WLAN::event_handler_wrapper: not active");
		abort();
	}

	WLAN::singleton->wlan_event_handler(arg, event_base, event_id, event_data);
}

void WLAN::wlan_event_handler(void *arg, esp_event_base_t event_base, std::int32_t event_id, void *event_data)
{
	switch(event_id)
	{
		case(WIFI_EVENT_STA_START): /* 2 */
		{
			this->log << "wlan: associating";
			this->set_state(state_t::associating);
			break;
		}
		case(WIFI_EVENT_STA_STOP): /* 3 */
		{
			this->log << "wlan: stop";
			this->set_state(state_t::init);
			break;
		}
		case(WIFI_EVENT_STA_CONNECTED): /* 4 */
		{
			this->log.warn_on_esp_err("esp_netif_create_ip6_linklocal", esp_netif_create_ip6_linklocal(this->netif_sta));

			if(!this->static_ipv6_address.empty()) // ugly workaround, skip SLAAC because it also adds it's address as "preferred"
			{
				struct netif *lwip_netif;
				lwip_netif = static_cast<struct netif *>(esp_netif_get_netif_impl(this->netif_sta));
				lwip_netif->ip6_autoconfig_enabled = 0;
			}

			this->log << "wlan: associated";

			this->set_state(state_t::associated);
			break;
		}
		case(WIFI_EVENT_STA_DISCONNECTED): /* 5 */
		{
			wifi_event_sta_disconnected_t *event = reinterpret_cast<wifi_event_sta_disconnected_t *>(event_data);
			this->log << std::format("wlan: disconnected: reason: {:#x}", event->reason);
			this->set_state(state_t::associating);
			break;
		}
		case(WIFI_EVENT_AP_START):
		{
			this->log << "wlan: start access point";
			this->set_state(state_t::rescue_ap_mode_idle);
			break;
		}
		case(WIFI_EVENT_AP_STOP):
		{
			this->log << "wlan: stop access point";
			this->set_state(state_t::rescue_ap_mode_init);
			break;
		}
		case(WIFI_EVENT_AP_STACONNECTED):
		{
			this->log << "wlan: access point associated";
			this->set_state(state_t::rescue_ap_mode_associated);
			break;
		}
		case(WIFI_EVENT_AP_STADISCONNECTED):
		{
			this->log << "wlan: access point deassociated";
			this->set_state(state_t::rescue_ap_mode_idle);
			break;
		}
		case(WIFI_EVENT_AP_PROBEREQRECVED):
		{
			this->log << "wlan: ap probe received";
			break;
		}
		case(WIFI_EVENT_STA_BEACON_TIMEOUT): /* 21 */
		{
			this->log << "wlan: beacon timeout";
			break;
		}
		case(WIFI_EVENT_HOME_CHANNEL_CHANGE): /* 40 */
		{
			this->log << "wlan: home channel change";
			break;
		}
		default:
		{
			this->log << std::format("wlan: unknown event: {:#x}", event_id);
			break;
		}
	}
}

void WLAN::ip_event_handler_wrapper(void *arg, esp_event_base_t event_base, std::int32_t event_id, void *event_data)
{
	if(!WLAN::singleton)
	{
		Console::emergency_wall("WLAN::ip_event_handler_wrapper: not active");
		abort();
	}

	WLAN::singleton->ip_event_handler(arg, event_base, event_id, event_data);
}

void WLAN::ip_event_handler(void *arg, esp_event_base_t event_base, std::int32_t event_id, void *event_data)
{
	esp_err_t rv;

	switch(event_id)
	{
		case(IP_EVENT_STA_GOT_IP):
		{
			const auto *event = reinterpret_cast<const ip_event_got_ip_t *>(event_data);

			if((rv = esp_netif_sntp_start()) != ESP_OK)
				throw(transient_exception(this->log.esp_string_error(rv, "WLAN::ip_event_handler: esp_netif_sntp_start")));

			this->log << std::format("wlan: ipv4: {} (mask: {}, gw: {})",
					this->system.ipv4_addr_to_string(&event->ip_info.ip.addr),
					this->system.ipv4_addr_to_string(&event->ip_info.gw.addr),
					this->system.ipv4_addr_to_string(&event->ip_info.netmask.addr));

			this->set_state(state_t::ipv4_address_acquired);

			break;
		}

		case(IP_EVENT_GOT_IP6):
		{
			const auto *event = reinterpret_cast<const ip_event_got_ip6_t *>(event_data);
			std::string address_type;

			switch(this->system.ipv6_address_type(reinterpret_cast<const uint8_t *>(&event->ip6_info.ip.addr)))
			{
				case(System::IPV6AddressType::link_local):
				{
					address_type = "link-local";

					this->set_state(state_t::ipv6_link_local_address_acquired);

					if(!this->static_ipv6_address.empty())
					{
						std::uint8_t ipv6_address[16];
						esp_ip6_addr_t esp_ipv6_address;

						this->system.string_to_ipv6_addr(this->static_ipv6_address, ipv6_address);

						memcpy(&esp_ipv6_address.addr[0], ipv6_address, sizeof(esp_ipv6_address.addr));

						this->log.warn_on_esp_err("esp_netif_add_ip6_address", esp_netif_add_ip6_address(this->netif_sta, esp_ipv6_address, true));
					}

					break;
				}
				case(System::IPV6AddressType::global_slaac):
				{
					address_type = "SLAAC";

					this->set_state(state_t::ipv6_slaac_address_acquired);

					break;
				}
				case(System::IPV6AddressType::global_static):
				{
					address_type = "static";

					this->set_state(state_t::ipv6_static_address_active);

					break;
				}
				default:
				{
					address_type = "invalid";

					this->log << "wlan: invalid IPv6 address received";

					break;
				}
			}

			this->log << std::format("wlan: {} ipv6: {}", address_type, this->system.ipv6_addr_to_string(&event->ip6_info.ip.addr[0]));

			break;
		}

		case(IP_EVENT_STA_LOST_IP):
		{
			this->log << "ip event: lost ipv4";
			break;
		}

		default:
		{
			this->log << std::format("ip event: unknown event: {:#x}", event_id);
			break;
		}
	}
}

void WLAN::set(const std::string &ssid, const std::string &passwd)
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

	this->config.set_string("wlan-passwd", ssid);
	this->config.set_string("wlan-ssid", passwd);

	wlan_mode = WIFI_MODE_STA;

	if((rv = esp_wifi_get_mode(&wlan_mode)) != ESP_OK)
		throw(transient_exception(this->log.esp_string_error(rv, "WLAN::set: esp_wifi_get_mode")));

	if(wlan_mode == WIFI_MODE_AP)
	{
		if((rv = esp_wifi_deauth_sta(0)) != ESP_OK)
			throw(transient_exception(this->log.esp_string_error(rv, "WLAN::set: esp_wifi_deauth_sta")));

		if((rv = esp_wifi_stop()) != ESP_OK)
			throw(transient_exception(this->log.esp_string_error(rv, "WLAN::set: esp_wifi_stop")));

		if((rv = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK)
			throw(transient_exception(this->log.esp_string_error(rv, "WLAN::set: esp_wifi_set_mode")));

		if((rv = esp_wifi_start()) != ESP_OK)
			throw(transient_exception(this->log.esp_string_error(rv, "WLAN::set: esp_wifi_start")));
	}

	ssid.copy(reinterpret_cast<char *>(config_sta.sta.ssid), sizeof(config_sta.sta.ssid));
	passwd.copy(reinterpret_cast<char *>(config_sta.sta.password), sizeof(config_sta.sta.password));

	if((rv = esp_wifi_set_config(WIFI_IF_STA, &config_sta)) != ESP_OK)
		throw(transient_exception(this->log.esp_string_error(rv, "WLAN::set: esp_wifi_set_config")));

	this->set_state(state_t::associating);
}

void WLAN::get(std::string &ssid, std::string &passwd)
{
	ssid = "<unset>";

	try
	{
		ssid = this->config.get_string("wlan-passwd");
	}
	catch(const transient_exception &)
	{
	}

	passwd = "<unset>";

	try
	{
		passwd = this->config.get_string("wlan-ssid");
	}
	catch(const transient_exception &)
	{
	}
}

void WLAN::set_ipv6_static(const std::string &input_address)
{
	uint8_t ipv6_address[16];
	std::string converted_address;

	if(input_address.empty())
	{
		this->static_ipv6_address = "";
		this->config.erase(this->key_ipv6_static_address);
	}
	else
	{
		this->system.string_to_ipv6_addr(input_address, ipv6_address /* sockaddr6_in->sin6_addr.in6_addr = uint8_t[16] */);
		converted_address = this->system.ipv6_addr_to_string(ipv6_address);

		for(auto &c : converted_address)
			c = std::tolower(c);

		this->config.set_string(this->key_ipv6_static_address, converted_address);
		this->static_ipv6_address = converted_address;
	}
}

void WLAN::get_ipv6_static(std::string &address)
{
	address = this->static_ipv6_address;

	if(address.empty())
		address = "<unset>";
}

void WLAN::info(std::string &out)
{
	esp_netif_t *netif;
	esp_netif_ip_info_t ip_info;
	esp_netif_flags_t if_flags;
	wifi_ap_record_t ap_info;
	const char *host, *key;
	uint8_t mac[6];
	char ifname[16];
	esp_ip6_addr_t esp_ip6_addr[8];
	wifi_mode_t wlan_mode;
	wifi_ps_type_t ps_type;
	int ix, count;
	std::string ip, gw, netmask;
	esp_err_t rv;

	out += std::format("\ncurrent state: {}, since {:d} seconds ago", this->state_to_string(this->state), this->state_time);
	out += "\noperating mode: ";

	if((rv = esp_wifi_get_mode(&wlan_mode)) != ESP_OK)
	{
		out += this->log.esp_string_error(rv, "no information: esp_wifi_get_mode");
		return;
	}

	if(wlan_mode == WIFI_MODE_AP)
	{
		out += "access point";
		netif = this->netif_ap;
	}
	else
	{
		out += "station";
		netif = this->netif_sta;
	}

	out += "\ninterface:";
	out += std::format("\n- number of interfaces: {:d}", esp_netif_get_nr_of_ifs());
	out += std::format("\n- index: {:d}", esp_netif_get_netif_impl_index(netif));
	out += "\n- name: ";

	if((rv = esp_netif_get_netif_impl_name(netif, ifname)) != ESP_OK)
		out += this->log.esp_string_error(rv, "no information: esp_netif_get_netif_impl_name");
	else
		out += ifname;

	key = esp_netif_get_ifkey(netif);

	out += std::format("\n- key: {}", key ? : "<invalid>");

	key = esp_netif_get_desc(netif);

	out += std::format("\n- description: {}", key ? : "<invalid>");

	if_flags = esp_netif_get_flags(netif);

	out += "\n- flags:";

	if(if_flags & ESP_NETIF_DHCP_CLIENT)
		out += " dhcp-client";

	if(if_flags & ESP_NETIF_DHCP_SERVER)
		out += " dhcp-server";

	if(if_flags & ESP_NETIF_FLAG_AUTOUP)
		out += " auto-up";

	if(if_flags & ESP_NETIF_FLAG_GARP)
		out += " garp";

	if(if_flags & ESP_NETIF_FLAG_EVENT_IP_MODIFIED)
		out += " event-ip-modified";

	if(if_flags & ESP_NETIF_FLAG_MLDV6_REPORT)
		out += " mldv6-report";

	out += "\nmac:\n- address:";

	if((rv = esp_netif_get_mac(netif, mac)) != ESP_OK)
		out += this->log.esp_string_error(rv, " no information: esp_netif_get_mac");
	else
		out += this->system.mac_addr_to_string(reinterpret_cast<const char *>(mac), false);

	out += "\nipv4:";

	if(esp_netif_get_ip_info(netif, &ip_info) != ESP_OK)
	{
		ip = "<unknown>";
		gw = "<unknown>";
		netmask = "<unknown>";
	}
	else
	{
		ip = this->system.ipv4_addr_to_string(&ip_info.ip.addr);
		gw = this->system.ipv4_addr_to_string(&ip_info.gw.addr);
		netmask = this->system.ipv4_addr_to_string(&ip_info.netmask.addr);
	}

	out += std::format("\n- interface address: {}", ip);
	out += std::format("\n- gateway address: {}", gw);
	out += std::format("\n- netmask: {}", netmask);

	out += "\nipv6:";

	if((count = esp_netif_get_all_ip6(netif, esp_ip6_addr)) <= 0)
		out += " <no addresses>";
	else
		for(ix = 0; ix < count; ix++)
			out += std::format("\n- address {:d}: {} ({})", ix,
					this->system.ipv6_addr_to_string(&esp_ip6_addr[ix].addr),
					this->system.ipv6_address_type_string(reinterpret_cast<const void *>(&esp_ip6_addr[ix].addr)));

	out += "\nhostname: ";

	if((rv = esp_netif_get_hostname(netif, &host)))
		out += this->log.esp_string_error(rv, "no information: esp_netif_get_hostname");
	else
		out += host;

	out += "\nsleep mode: ";

	if((rv = esp_wifi_get_ps(&ps_type)))
		out += this->log.esp_string_error(rv, "no information: esp_wifi_get_ps");
	else
	{
		switch(ps_type)
		{
			case(WIFI_PS_NONE): key = "none"; break;
			case(WIFI_PS_MIN_MODEM): key = "minimal"; break;
			case(WIFI_PS_MAX_MODEM): key = "maximal"; break;
			default: key = "<unknown>"; break;
		}

		out += key;
	}

	if(wlan_mode == WIFI_MODE_STA)
	{
		uint8_t protocol_bitmap;
		uint16_t timeout;
		wifi_bandwidth_t wbw;
		wifi_phy_mode_t mode;

		out += "\nwlan STA status:";

		if((rv = esp_wifi_sta_get_ap_info(&ap_info)))
			out += this->log.esp_string_error(rv, "no information: esp_wifi_sta_get_ap_info");
		else
		{
			out += std::format("\n- access point: {}", this->system.mac_addr_to_string(reinterpret_cast<const char *>(ap_info.bssid), false));
			out += std::format("\n- SSID: {}", reinterpret_cast<const char *>(ap_info.ssid));
			out += "\n- ";

			if(ap_info.second == WIFI_SECOND_CHAN_ABOVE)
				out += std::format("channels: {:d}+{:d}", static_cast<int>(ap_info.primary), static_cast<int>(ap_info.primary) + 1);
			else
				if(ap_info.second == WIFI_SECOND_CHAN_BELOW)
					out += std::format("channels: {:d}+{:d}", static_cast<int>(ap_info.primary), static_cast<int>(ap_info.primary) - 1);
				else
					out += std::format("channel: {:d}", static_cast<int>(ap_info.primary));

			out += std::format("\n- rssi: {:d}", static_cast<int>(ap_info.rssi));
			out += std::format("\n- authentication mode: ");

			{
				const auto it = wifi_auth_mode_map.find(ap_info.authmode);

				if(it == wifi_auth_mode_map.end())
					out += "<unknown>";
				else
					out += it->second;
			}

			out += "\n- pairwise cipher: ";

			{
				const auto it = wifi_cipher_type_map.find(ap_info.pairwise_cipher);

				if(it == wifi_cipher_type_map.end())
					out += "<unknown>";
				else
					out += it->second;
			}

			out += "\n- group cipher: ";

			{
				const auto it = wifi_cipher_type_map.find(ap_info.group_cipher);

				if(it == wifi_cipher_type_map.end())
					out += "<unknown>";
				else
					out += it->second;
			}

			std::string country_code;
			country_code.assign(ap_info.country.cc, 2);

			out += std::format( "\n- country: {:s} [{:d}-{:d}], max power: {:d} dB",
					country_code,
					ap_info.country.schan,
					ap_info.country.nchan - ap_info.country.schan + 1,
					ap_info.country.max_tx_power);
		}

		out += "\n- protocols:";

		if((rv = esp_wifi_get_protocol(WIFI_IF_STA, &protocol_bitmap)))
			out += this->log.esp_string_error(rv, " no information: esp_wifi_get_protocol");
		else
		{
			if(protocol_bitmap & WIFI_PROTOCOL_11B)
				out += " 802.11b";

			if(protocol_bitmap & WIFI_PROTOCOL_11G)
				out += " 802.11g";

			if(protocol_bitmap & WIFI_PROTOCOL_11N)
				out += " 802.11n";

			if(protocol_bitmap & WIFI_PROTOCOL_11AX)
				out += " 802.11ax";
		}

		out += ", bandwidth: ";

		if((rv = esp_wifi_get_bandwidth(WIFI_IF_STA, &wbw)))
			out += this->log.esp_string_error(rv, " no information: esp_wifi_get_bandwidth");
		else
			out += (wbw == WIFI_BW_HT40) ? "ht40" : "ht20";

		out+= "\n- phy mode: ";

		if((rv = esp_wifi_sta_get_negotiated_phymode(&mode)))
			out += this->log.esp_string_error(rv, " no information: esp_wifi_sta_get_negotiated_phymode");
		else
		{
			const auto it = wifi_phy_mode_map.find(mode);

			if(it == wifi_phy_mode_map.end())
				out += "<unknown>";
			else
				out += it->second;
		}

		out += std::format( "\n- TSF timestamp: {:d}", esp_wifi_get_tsf_time(WIFI_IF_STA));

		out += "\n- configured inactive time: ";

		if((rv = esp_wifi_get_inactive_time(WIFI_IF_STA, &timeout)))
			out += this->log.esp_string_error(rv, " no information: esp_wifi_get_inactive_time");
		else
			out += std::format("{:d}", timeout);
	}
	else
	{
		uint8_t channel;
		wifi_second_chan_t secondary;
		wifi_country_t country;

		out += "\nwlan AP status:";

		if((rv = esp_wifi_get_channel(&channel, &secondary)))
			out += this->log.esp_string_error(rv, " no information: esp_wifi_get_channel");
		else
			out += std::format("\n- channel: {}", channel);

		out += "\n- country: ";

		if((rv = esp_wifi_get_country(&country)))
			out += this->log.esp_string_error(rv, " no information: esp_wifi_get_country");
		else
		{
			std::string country_code;
			country_code.assign(country.cc, 2);
			out += country_code;
		}
	}
}
