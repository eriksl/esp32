#pragma once

#include "log.h"
#include "config.h"

#include <string>
#include <cstdint>

class Command;

class BT final
{
	public:

		explicit BT() = delete;
		explicit BT(const BT &) = delete;
		explicit BT(Log &, Config &);

		static BT &get();
		void set(Command *);
		void run();

		void send(const command_response_t *);
		std::string key();
		void key(const std::string &);
		void info(std::string &out);

	private:

		static constexpr int service_handle = 0xabf0;
		static constexpr int characteristics_handle = 0xabf1;
		static constexpr int bt_mtu = 484;

		static BT *singleton;
		Log &log;
		Config &config;
		Command *command;

		std::string hostname;
		std::string encryption_key;
		bool running;

		static void nimble_port_task(void *);

		static int gatt_value_event_wrapper(std::uint16_t connection_handle, std::uint16_t attribute_handle, struct ble_gatt_access_ctxt *context, void *arg);
		int gatt_value_event(std::uint16_t connection_handle, std::uint16_t attribute_handle, struct ble_gatt_access_ctxt *context, void *arg);

		static void callback_reset_wrapper(int);
		void callback_reset(int);

		static void callback_sync_wrapper();
		void callback_sync();

		static int gap_event_wrapper(struct ble_gap_event *event, void *arg);
		int gap_event(struct ble_gap_event *event, void *arg);

		static void gatt_svr_register_cb_wrapper(struct ble_gatt_register_ctxt *context, void *arg);
		void gatt_svr_register_cb(struct ble_gatt_register_ctxt *context, void *arg);

		std::uint8_t bt_host_address[6] = {0};
		std::uint16_t value_attribute_handle;
		std::uint8_t own_addr_type;

		int stats_sent_bytes;
		int stats_sent_packets;
		int stats_sent_encryption_failed;
		int stats_received_bytes;
		int stats_received_packets;
		int stats_received_null_packets;
		int stats_received_decryption_failed;
		int stats_received_invalid_packets;
		int stats_received_incomplete_packets;
		int stats_indication_error;
		int stats_indication_timeout;

		int gatt_init();
		void server_advertise();
		void received(unsigned int connection_handle, unsigned int attribute_handle, const struct os_mbuf *mbuf);
};
