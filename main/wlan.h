#pragma once

#include "log.h"
#include "config.h"
#include "notify.h"
#include "system.h"

#include <string>
#include <map>
#include <cstdint>

#include <esp_timer.h>
#include <esp_netif.h>

class WLAN final
{
	public:

		explicit WLAN() = delete;
		explicit WLAN(const WLAN &) = delete;
		explicit WLAN(Log &, Config &, Notify &, System &);

		void run();

		WLAN &get();

		void set(const std::string &ssid, const std::string &passwd);
		void get(std::string &ssid, std::string &passwd);

		void set_ipv6_static(const std::string&);
		void get_ipv6_static(std::string &);

		void info(std::string &);

	private:

		static constexpr const char *key_ipv6_static_address = "ipv6-address";

		enum class state_t
		{
			invalid,
			init,
			associating,
			associated,
			ipv4_address_acquired,
			ipv6_link_local_address_acquired,
			ipv6_slaac_address_acquired,
			ipv6_static_address_active,
			rescue_ap_mode_init,
			rescue_ap_mode_idle,
			rescue_ap_mode_associated,
		};

		struct wlan_state_info_t
		{
			std::uint32_t valid_transitions;
			std::string name;
			Notify::Notification notification;
		};

		typedef std::map<state_t, wlan_state_info_t> state_info_map_t;
		static const state_info_map_t state_info;

		typedef std::map<int, std::string> wifi_auth_mode_map_t;
		static const wifi_auth_mode_map_t wifi_auth_mode_map;

		typedef std::map<int, std::string> wifi_cipher_type_map_t;
		static const wifi_cipher_type_map_t wifi_cipher_type_map;

		typedef std::map<int, std::string> wifi_phy_mode_map_t;
		static const wifi_phy_mode_map_t wifi_phy_mode_map;

		static WLAN *singleton;
		Log &log;
		Config &config;
		Notify &notify;
		System &system;

		esp_netif_t *netif_sta;
		esp_netif_t *netif_ap;
		state_t state;
		int state_time;
		TimerHandle_t state_timer;
		std::string static_ipv6_address;
		std::string hostname;

		std::string state_to_string(state_t);
		void set_state(state_t);

		static void state_callback_wrapper(TimerHandle_t);
		void state_callback(TimerHandle_t);

		static void wlan_event_handler_wrapper(void *, esp_event_base_t, std::int32_t, void *);
		void wlan_event_handler(void *, esp_event_base_t, std::int32_t, void *);

		static void ip_event_handler_wrapper(void *, esp_event_base_t, std::int32_t, void *);
		void ip_event_handler(void *, esp_event_base_t, std::int32_t, void *);
};
