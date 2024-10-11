#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>

#include "string.h"
#include "cli.h"
#include "wlan.h"
#include "config.h"
#include "log.h"
#include "util.h"
#include "packet.h"
#include "cli-command.h"
#include "notify.h"

#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_netif_sntp.h>
#include <esp_timer.h>

#include <assert.h>

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

static const char *key_ipv6_address = "ipv6-address";

static bool inited = false;
static esp_netif_t *netif_sta;
static esp_netif_t *netif_ap;
static wlan_state_t state = ws_invalid;
static unsigned int state_time;
static TimerHandle_t state_timer;
static int tcp_socket_fd = -1;
static int udp_socket_fd = -1;

static unsigned int tcp_send_bytes;
static unsigned int tcp_send_segments;
static unsigned int tcp_send_packets;
static unsigned int tcp_send_errors;
static unsigned int tcp_send_no_connection;
static unsigned int tcp_receive_bytes;
static unsigned int tcp_receive_packets;
static unsigned int tcp_receive_accepts;
static unsigned int tcp_receive_accept_errors;
static unsigned int tcp_receive_errors;
static unsigned int tcp_receive_incomplete_packets;

static unsigned int udp_send_bytes;
static unsigned int udp_send_segments;
static unsigned int udp_send_packets;
static unsigned int udp_send_errors;
static unsigned int udp_send_no_connection;
static unsigned int udp_receive_bytes;
static unsigned int udp_receive_packets;
static unsigned int udp_receive_errors;
static unsigned int udp_receive_incomplete_packets;

typedef struct
{
	uint32_t valid_transitions;
	const char *name;
	notify_t notification;
} state_info_t;

static const state_info_t state_info[ws_size] =
{
	[ws_invalid] =							{ (1 << ws_init),
											"invalid", notify_net_init },
	[ws_init] =								{ (1 << ws_init) | (1 << ws_associating),
											"init", notify_net_init },
	[ws_associating] =						{ (1 << ws_init) | (1 << ws_associating) | (1 << ws_associated)  | (1 << ws_rescue_ap_mode_idle) | (1 << ws_rescue_ap_mode_associated),
											"associating", notify_net_associating },
	[ws_associated] =						{ (1 << ws_init) | (1 << ws_associating) | (1 << ws_ipv4_address_acquired) | (1 << ws_ipv6_link_local_address_acquired),
											"associated", notify_net_associating_finished },
	[ws_ipv4_address_acquired] =			{ (1 << ws_init) | (1 << ws_associating) | (1 << ws_ipv6_link_local_address_acquired),
											"ipv4 address acquired", notify_net_ipv4_acquired },
	[ws_ipv6_link_local_address_acquired] =	{ (1 << ws_init) | (1 << ws_associating) | (1 << ws_ipv4_address_acquired) | (1 << ws_ipv6_slaac_address_acquired) | (1 << ws_ipv6_static_address_active),
											"ipv6 link local address acquired", notify_net_ipv6_ll_active },
	[ws_ipv6_slaac_address_acquired] =		{ (1 << ws_init) | (1 << ws_associating) | (1 << ws_ipv6_static_address_active),
											"ipv6 autoconfig address acquired", notify_net_ipv6_slaac_acquired },
	[ws_ipv6_static_address_active] =		{ (1 << ws_init) | (1 << ws_associating) | (1 << ws_ipv6_slaac_address_acquired) | (1 << ws_ipv6_static_address_active),
											"ipv6 static address set", notify_net_ipv6_static_active },
	[ws_rescue_ap_mode_init] =				{ (1 << ws_init) | (1 << ws_associating) | (1 << ws_rescue_ap_mode_idle) | (1 << ws_rescue_ap_mode_associated),
											"rescue access point mode init", notify_net_ap_mode_init },
	[ws_rescue_ap_mode_idle] =				{ (1 << ws_rescue_ap_mode_init) | (1 << ws_rescue_ap_mode_associated),
											"rescue access point mode idle", notify_net_ap_mode_idle },
	[ws_rescue_ap_mode_associated] =		{ (1 << ws_rescue_ap_mode_idle),
											"rescue access point mode associated", notify_net_ap_mode_associated },
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
			log_format("wlan: invalid state transition from %s (%u) to %s (%u), %x, reassociating", state_string, state, state_new_string, state_new, (unsigned int)state_info[state].valid_transitions);
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

	notify(state_info[state].notification);
}

static void state_callback(TimerHandle_t handle)
{
	static wifi_config_t config_ap =
	{
		.ap =
		{
			.channel = 11,
			.authmode = WIFI_AUTH_WPA2_PSK,
			.max_connection = 1,
			.beacon_interval = 100,
			.pairwise_cipher = WIFI_CIPHER_TYPE_CCMP,
			.ftm_responder = 0,
			.pmf_cfg =
			{
				.capable = false,
				.required = false,
			},
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
		case(WIFI_EVENT_HOME_CHANNEL_CHANGE): /* 40 */
		{
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
			string_auto(ipv6_address_string, 64);
			esp_ip6_addr_t ipv6_address;
			const char *address_type;

			switch(util_ipv6_address_type(&event->ip6_info.ip))
			{
				case(ipv6_address_link_local):
				{
					address_type = "link-local";

					set_state(ws_ipv6_link_local_address_acquired);

					if(config_get_string_cstr(key_ipv6_address, ipv6_address_string) && !esp_netif_str_to_ip6(string_cstr(ipv6_address_string), &ipv6_address))
						util_warn_on_esp_err("esp_netif_add_ip6_address", esp_netif_add_ip6_address(netif_sta, ipv6_address, true));

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

			util_esp_ipv6_addr_to_string(ipv6_address_string, &event->ip6_info.ip);
			log_format("wlan: %s ipv6: %s", address_type, string_cstr(ipv6_address_string));

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

static void run_tcp(void *)
{
	int accept_fd;
	struct sockaddr_in6 si6_addr;
	socklen_t si6_addr_length;
	int length;
	string_t receive_buffer;
	cli_buffer_t cli_buffer;

	assert(inited);
	assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr));
	assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr_in));
	assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr_in6));

	receive_buffer = string_new(4096 + 128);

	memset(&si6_addr, 0, sizeof(si6_addr));
	si6_addr.sin6_family = AF_INET6;
	si6_addr.sin6_port = htons(24);

	assert((accept_fd = socket(AF_INET6, SOCK_STREAM, 0)) >= 0);
	assert(bind(accept_fd, (const struct sockaddr *)&si6_addr, sizeof(si6_addr)) == 0);
	assert(listen(accept_fd, 0) == 0);

	for(;;)
	{
		si6_addr_length = sizeof(si6_addr);

		if((tcp_socket_fd = accept(accept_fd, (struct sockaddr *)&si6_addr, &si6_addr_length)) < 0)
		{
			tcp_receive_accept_errors++;
			continue;
		}

		tcp_receive_accepts++;

		for(;;)
		{
			string_clear(receive_buffer);

			assert(sizeof(si6_addr) >= si6_addr_length);
			assert(sizeof(cli_buffer.ip.address.sin6_addr) >= si6_addr_length);

			util_memcpy(&cli_buffer.ip.address.sin6_addr, &si6_addr, si6_addr_length);
			cli_buffer.ip.address.sin6_length = si6_addr_length;
			cli_buffer.mtu = 1024;
			cli_buffer.source = cli_source_wlan_tcp;

			length = string_recvfrom_fd(receive_buffer, tcp_socket_fd, (unsigned int *)0, (void *)0);

			log_format("$ recvfrom: %d", length);

			if(length <= 0)
			{
				tcp_receive_errors++;
				break;
			}

			tcp_receive_bytes += length;

			if(packet_valid(receive_buffer))
			{
				if(packet_complete(receive_buffer))
					cli_buffer.packetised = 1;
				else
				{
					tcp_receive_incomplete_packets++;
					continue;
				}
			}
			else
				cli_buffer.packetised = 0;

			tcp_receive_packets++;

			cli_buffer.data = string_new(string_length(receive_buffer));
			string_assign_string(cli_buffer.data, receive_buffer);

			cli_receive_queue_push(&cli_buffer);
		}

		close(tcp_socket_fd);
		tcp_socket_fd = -1;
	}

	string_free(&receive_buffer);
}

void wlan_tcp_send(const cli_buffer_t *src)
{
	int length, offset, chunk, sent;

	assert(inited);

	if(tcp_socket_fd < 0)
	{
		tcp_send_no_connection++;
		return;
	}

	tcp_send_packets++;

	offset = 0;
	length = string_length(src->data);

	for(;;)
	{
		chunk = length;

		sent = send(tcp_socket_fd, string_data(src->data) + offset, chunk, 0);

		tcp_send_segments++;

		if(sent <= 0)
		{
			tcp_send_errors++;
			break;
		}

		tcp_send_bytes += sent;

		length -= sent;
		offset += sent;

		assert(length >= 0);
		assert(offset <= string_length(src->data));

		if(length == 0)
			break;

		assert(offset < string_length(src->data));
	}
}

static void run_udp(void *)
{
	struct sockaddr_in6 si6_addr;
	unsigned int si6_addr_length;
	int length;
	string_t receive_buffer;
	cli_buffer_t cli_buffer;

	assert(inited);
	assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr));
	assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr_in));
	assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr_in6));

	receive_buffer = string_new(4096 + 128);

	memset(&si6_addr, 0, sizeof(si6_addr));
	si6_addr.sin6_family = AF_INET6;
	si6_addr.sin6_port = htons(24);

	assert((udp_socket_fd = socket(AF_INET6, SOCK_DGRAM, 0)) >= 0);
	assert(bind(udp_socket_fd, (const struct sockaddr *)&si6_addr, sizeof(si6_addr)) == 0);

	for(;;)
	{
		string_clear(receive_buffer);

		si6_addr_length = sizeof(si6_addr);

		length = string_recvfrom_fd(receive_buffer, udp_socket_fd, &si6_addr_length, &si6_addr);

		assert(sizeof(si6_addr) >= si6_addr_length);
		assert(sizeof(cli_buffer.ip.address.sin6_addr) >= si6_addr_length);

		util_memcpy(&cli_buffer.ip.address.sin6_addr, &si6_addr, si6_addr_length);
		cli_buffer.ip.address.sin6_length = si6_addr_length;

		if(length <= 0)
		{
			udp_receive_errors++;
			util_sleep(100);
			continue;
		}

		udp_receive_bytes += length;

		if(packet_valid(receive_buffer))
		{
			if(packet_complete(receive_buffer))
				cli_buffer.packetised = 1;
			else
			{
				udp_receive_incomplete_packets++;
				continue;
			}
		}
		else
			cli_buffer.packetised = 0;

		udp_receive_packets++;

		cli_buffer.source = cli_source_wlan_udp;
		cli_buffer.mtu = 4096;
		cli_buffer.data = string_new(string_length(receive_buffer));
		string_assign_string(cli_buffer.data, receive_buffer);

		cli_receive_queue_push(&cli_buffer);
	}

	close(udp_socket_fd);
	udp_socket_fd = -1;

	string_free(&receive_buffer);
}

void wlan_udp_send(const cli_buffer_t *src)
{
	int length, offset, chunk, sent;

	assert(inited);

	if(udp_socket_fd < 0)
	{
		udp_send_no_connection++;
		return;
	}

	udp_send_packets++;

	offset = 0;
	length = string_length(src->data);

	for(;;)
	{
		chunk = length;

		//if(chunk > 1024) // FIXME do proper segmentation on both sides
			//chunk = 1024;

		sent = sendto(udp_socket_fd, string_data(src->data) + offset, chunk, 0, (const struct sockaddr *)&src->ip.address.sin6_addr, src->ip.address.sin6_length);

		udp_send_segments++;

		if(sent <= 0)
		{
			udp_send_errors++;
			break;
		}

		udp_send_bytes += sent;

		length -= sent;
		offset += sent;

		assert(length >= 0);
		assert(offset <= string_length(src->data));

		if(length == 0)
			break;

		assert(offset < string_length(src->data));
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
		static wifi_config_t config_sta =
		{
			.sta =
			{
				.scan_method = WIFI_ALL_CHANNEL_SCAN,
				.bssid_set = 0,
				.channel = 0,
				.listen_interval = 3,
				.sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
				.pmf_cfg.capable = false,
				.pmf_cfg.required = false,
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

		string_to_cstr(call->parameters[0].string, sizeof(config_sta.sta.ssid), (char *)config_sta.sta.ssid);
		string_to_cstr(call->parameters[1].string, sizeof(config_sta.sta.password), (char *)config_sta.sta.password);

		rv = esp_wifi_set_config(WIFI_IF_STA, &config_sta);

		util_warn_on_esp_err("esp_wifi_set_config", rv);

		if(rv)
		{
			string_append_cstr(call->result, "\nesp_wifi_set_config returns error");
			return;
		}

		set_state(ws_associating);
	}
}

void wlan_command_ipv6_config(cli_command_call_t *call)
{
	string_auto(ipv6_address_string, 64);
	esp_ip6_addr_t ipv6_address;

	assert(inited);
	assert(call->parameter_count < 2);

	if(call->parameter_count > 0)
	{
		if(esp_netif_str_to_ip6(string_cstr(call->parameters[0].string), &ipv6_address))
		{
			string_assign_cstr(call->result, "invalid ipv6 address");
			return;
		}

		string_assign_cstr(ipv6_address_string, ip6addr_ntoa((const ip6_addr_t *)&ipv6_address));
		string_tolower(ipv6_address_string);
		config_set_string_cstr(key_ipv6_address, ipv6_address_string);
	}

	string_assign_cstr(call->result, "ipv6 static address: ");

	if(config_get_string_cstr(key_ipv6_address, ipv6_address_string))
		string_append_string(call->result, ipv6_address_string);
	else
		string_append_cstr(call->result, "<unset>");
}

void wlan_init(void)
{
	string_auto_init(hostname_key, "hostname");
	string_auto(hostname, 16);
	wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
	static esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(0, {});

	assert(!inited);

	init_config.ampdu_rx_enable = 1;
	init_config.ampdu_tx_enable = 1;
	init_config.amsdu_tx_enable = 1;
	init_config.nvs_enable = 1;
	init_config.wifi_task_core_id = 0;

	if(!config_get_string(hostname_key, hostname))
		string_assign_cstr(hostname, "esp32s3");

	sntp_config.start = false;
	sntp_config.server_from_dhcp = true;

	state_timer = xTimerCreate("wlan-state", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, state_callback);
	assert(state_timer);
	set_state(ws_init);

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

	util_abort_on_esp_err("esp_netif_set_hostname", esp_netif_set_hostname(netif_sta, string_cstr(hostname)));
	util_abort_on_esp_err("esp_netif_set_hostname", esp_netif_set_hostname(netif_ap, string_cstr(hostname)));

	if(xTaskCreatePinnedToCore(run_tcp, "wlan-tcp", 3 * 1024, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
        util_abort("wlan: xTaskCreatePinnedToNode run_tcp");

	if(xTaskCreatePinnedToCore(run_udp, "wlan-udp", 3 * 1024, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
        util_abort("wlan: xTaskCreatePinnedToNode run_udp");

	xTimerStart(state_timer, portMAX_DELAY);
}

void wlan_command_ip_info(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "IP INFO");
	string_append_cstr(call->result, "\ntcp send");
	string_format_append(call->result, "\n- sent bytes %u", tcp_send_bytes);
	string_format_append(call->result, "\n- sent segments %u", tcp_send_segments);
	string_format_append(call->result, "\n- sent packets: %u", tcp_send_packets);
	string_format_append(call->result, "\n- send errors: %u", tcp_send_errors);
	string_format_append(call->result, "\n- disconnected socket events: %u", tcp_send_no_connection);
	string_append_cstr(call->result, "\ntcp receive");
	string_format_append(call->result, "\n- received bytes: %u", tcp_receive_bytes);
	string_format_append(call->result, "\n- received packets: %u", tcp_receive_packets);
	string_format_append(call->result, "\n- received incomplete packets: %u", tcp_receive_incomplete_packets);
	string_format_append(call->result, "\n- receive errors: %u", tcp_receive_errors);
	string_format_append(call->result, "\n- accepted connections: %u", tcp_receive_accepts);
	string_format_append(call->result, "\n- accept errors: %u", tcp_receive_accept_errors);
	string_append_cstr(call->result, "\nudp send");
	string_format_append(call->result, "\n- sent bytes %u", udp_send_bytes);
	string_format_append(call->result, "\n- sent segments %u", udp_send_segments);
	string_format_append(call->result, "\n- sent packets: %u", udp_send_packets);
	string_format_append(call->result, "\n- send errors: %u", udp_send_errors);
	string_format_append(call->result, "\n- disconnected socket events: %u", udp_send_no_connection);
	string_append_cstr(call->result, "\nudp receive");
	string_format_append(call->result, "\n- received bytes: %u", udp_receive_bytes);
	string_format_append(call->result, "\n- received packets: %u", udp_receive_packets);
	string_format_append(call->result, "\n- received incomplete packets: %u", udp_receive_incomplete_packets);
	string_format_append(call->result, "\n- receive errors: %u", udp_receive_errors);
}

void wlan_command_info(cli_command_call_t *call)
{
	esp_netif_t *netif;
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
	wifi_ps_type_t ps_type;
	unsigned int ix, rv;

	assert(inited);
	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "WLAN INFO");

	string_format_append(call->result, "\ncurrent state: %s, since %u seconds ago",
			wlan_state_to_cstr(state), state_time);

	if((rv = esp_wifi_get_mode(&wlan_mode)))
	{
		util_warn_on_esp_err("esp_wifi_get_mode", rv);
		string_append_cstr(call->result, "no information");
		return;
	}

	string_append_cstr(call->result, "\noperating mode: ");

	if(wlan_mode == WIFI_MODE_AP)
	{
		string_append_cstr(call->result, "access point");
		netif = netif_ap;
	}
	else
	{
		string_append_cstr(call->result, "station");
		netif = netif_sta;
	}

	string_append_cstr(call->result, "\ninterface:");
	string_format_append(call->result, "\n- number of interfaces: %u", esp_netif_get_nr_of_ifs());
	string_format_append(call->result, "\n- index: %d", esp_netif_get_netif_impl_index(netif));

	util_abort_on_esp_err("esp_netif_get_netif_impl_name", esp_netif_get_netif_impl_name(netif, ifname));

	string_format_append(call->result, "\n- name: %s", ifname);

	key = esp_netif_get_ifkey(netif);

	string_format_append(call->result, "\n- key: %s", key ? key : "<invalid>");

	key = esp_netif_get_desc(netif);

	string_format_append(call->result, "\n- description: %s", key ? key : "<invalid>");

	string_append_cstr(call->result, "\n- flags:");

	if_flags = esp_netif_get_flags(netif);

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

	if((rv = esp_netif_get_mac(netif, mac)))
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

	if((rv = esp_netif_get_ip_info(netif, &ip_info)))
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

	rv = esp_netif_get_all_ip6(netif, esp_ip6_addr);

	for(ix = 0; ix < rv; ix++)
	{
		util_esp_ipv6_addr_to_string(ipv6_str, &esp_ip6_addr[ix]);
		string_format_append(call->result, "\n- address %u: %s (%s)", ix, string_cstr(ipv6_str), util_ipv6_address_type_string(&esp_ip6_addr[ix]));
	}

	string_append_cstr(call->result, "\nhostname: ");

	if((rv = esp_netif_get_hostname(netif, &hostname)))
	{
		util_warn_on_esp_err("esp_netif_get_hostname", rv);
		string_append_cstr(call->result, "<unknown>");
	}
	else
		string_append_cstr(call->result, hostname);

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

	if(wlan_mode == WIFI_MODE_STA)
	{
		uint8_t protocol_bitmap;
		uint16_t timeout;
		wifi_bandwidth_t wbw;
		wifi_phy_mode_t mode;

		string_append_cstr(call->result, "\nwlan STA status:");

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
				string_format_append(call->result, "%u+%u", ap_info.primary, ap_info.primary + 1U);
			else
				if(ap_info.second == WIFI_SECOND_CHAN_BELOW)
					string_format_append(call->result, "%u+%u", ap_info.primary, ap_info.primary - 1U);
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
					ap_info.country.cc, ap_info.country.schan, ap_info.country.nchan - ap_info.country.schan + 1U, ap_info.country.max_tx_power);
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
		string_format_append(call->result, "\n- TSF timestamp: %llu", (unsigned long long int)esp_wifi_get_tsf_time(WIFI_IF_STA));
		string_append_cstr(call->result, "\n- configured inactive time: ");

		if((rv = esp_wifi_get_inactive_time(WIFI_IF_STA, &timeout)))
		{
			util_warn_on_esp_err("esp_wifi_get_inactive_time", rv);
			string_append_cstr(call->result, "<invalid>");
		}
		else
			string_format_append(call->result, "%u", timeout);
	}
	else
	{
		uint8_t channel;
		wifi_second_chan_t secondary;
		wifi_country_t country;

		string_append_cstr(call->result, "\nwlan AP status:");

		if((rv = esp_wifi_get_channel(&channel, &secondary)))
		{
			util_warn_on_esp_err("esp_wifi_get_channel", rv);
			channel = 0;
		}

		string_format_append(call->result, "\n- channel: %u", channel);
		string_append_cstr(call->result, "\n- country: ");

		if((rv = esp_wifi_get_country(&country)))
		{
			util_warn_on_esp_err("esp_wifi_get_country", rv);
			string_append_cstr(call->result, "<invalid>");
		}
		else
			string_format_append(call->result, "%.2s", country.cc);
	}
}
