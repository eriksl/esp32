#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <esp_log.h>

#include "string.h"
#include "cli.h"
#include "log.h"
#include "util.h"
#include "packet.h"
#include "config.h"
#include "cli-command.h"
#include "encryption.h"
#include "exception.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <nimble/nimble_port_freertos.h>
#include <nimble/nimble_port.h>
#include <store/config/ble_store_config.h>
#include <host/ble_hs.h>
#include <host/util/util.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#include <esp_timer.h>

#include "bt.h"

#include <string>
#include <exception>
#include <boost/format.hpp>

extern "C" void ble_store_config_init(void);

enum
{
	service_handle = 0xabf0,
	characteristics_handle = 0xabf1,
	bt_mtu = 484,
};

static int gap_event(struct ble_gap_event *event, void *arg);
static int gatt_value_event(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *context, void *arg);
static void bt_received(unsigned int connection_handle, unsigned int attribute_handle, const struct os_mbuf *mbuf);

static std::string hostname;
static std::string encryption_key;

static uint8_t bt_host_address[6] = {0};
static uint16_t value_attribute_handle;
static uint8_t own_addr_type;

static unsigned int stats_sent_bytes;
static unsigned int stats_sent_packets;
static unsigned int stats_sent_encryption_failed;

static unsigned int stats_received_bytes;
static unsigned int stats_received_packets;
static unsigned int stats_received_null_packets;
static unsigned int stats_received_decryption_failed;
static unsigned int stats_received_invalid_packets;
static unsigned int stats_received_incomplete_packets;

static unsigned int stats_indication_error;
static unsigned int stats_indication_timeout;

static bool inited = false;

static void nimble_port_task(void *param)
{
	assert(inited);

	nimble_port_run();
	nimble_port_freertos_deinit();
}

static int gatt_value_event(uint16_t connection_handle, uint16_t attribute_handle, struct ble_gatt_access_ctxt *context, void *arg)
{
	assert(inited);

	switch (context->op)
	{
		case(BLE_GATT_ACCESS_OP_WRITE_CHR):
		{
			bt_received(connection_handle, attribute_handle, context->om);
			break;
		}

		default:
		{
			log_format("bt: gatt_value_event: default callback: 0x%x", context->op);
			break;
		}
	}

	return(0);
}

static int gatt_init(void)
{
	static constexpr ble_uuid_t uuid16 = { .type = BLE_UUID_TYPE_16, };
	static struct ble_gatt_svc_def gatt_svc_definitions[2];
	static struct ble_gatt_chr_def gatt_chr_definitions[2];
	ble_gatt_svc_def *gatt_svc_definition;
	ble_gatt_chr_def *gatt_chr_definition;
	int rc = 0;

	static const ble_uuid16_t uuid_service_handle =
	{
		.u = uuid16,
		.value = service_handle,
	};

	static const ble_uuid16_t uuid_characteristics_handle =
	{
		.u = uuid16,
		.value = characteristics_handle,
	};

	ble_svc_gap_init();
	ble_svc_gatt_init();

	gatt_svc_definition = &gatt_svc_definitions[0];
	gatt_svc_definition->type = BLE_GATT_SVC_TYPE_PRIMARY;
	gatt_svc_definition->uuid = reinterpret_cast<const ble_uuid_t *>(&uuid_service_handle);
	gatt_svc_definition->includes = nullptr;
	gatt_svc_definition->characteristics = gatt_chr_definitions;

	gatt_svc_definition = &gatt_svc_definitions[1];
	gatt_svc_definition->type = 0;
	gatt_svc_definition->uuid = nullptr;
	gatt_svc_definition->includes = nullptr;
	gatt_svc_definition->characteristics = nullptr;

	gatt_chr_definition = &gatt_chr_definitions[0];
	gatt_chr_definition->uuid = reinterpret_cast<const ble_uuid_t *>(&uuid_characteristics_handle);
	gatt_chr_definition->access_cb = gatt_value_event,
	gatt_chr_definition->val_handle = &value_attribute_handle,
	gatt_chr_definition->flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,

	gatt_chr_definition = &gatt_chr_definitions[1];
	gatt_chr_definition->uuid = nullptr;
	gatt_chr_definition->access_cb = nullptr;
	gatt_chr_definition->val_handle = nullptr;
	gatt_chr_definition->flags = 0;

	if((rc = ble_gatts_count_cfg(gatt_svc_definitions)) != 0)
		return(rc);

	if((rc = ble_gatts_add_svcs(gatt_svc_definitions)) != 0)
		return rc;

	return(0);
}

static void server_advertise(void)
{
	struct ble_gap_adv_params adv_params;
	struct ble_hs_adv_fields fields;
	const char *name;
	int rc;

	memset(&fields, 0, sizeof(fields));

	fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

	fields.uuids16 = (ble_uuid16_t[]) { BLE_UUID16_INIT(service_handle) };
	fields.num_uuids16 = 1;
	fields.uuids16_is_complete = 1;

	name = ble_svc_gap_device_name();
	fields.name = (const uint8_t *)name;
	fields.name_len = strlen(name);
	fields.name_is_complete = 1;

	fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
	fields.tx_pwr_lvl_is_present = 1;

	util_abort_on_esp_err("ble_gap_adv_set_fields", ble_gap_adv_set_fields(&fields));

	memset(&adv_params, 0, sizeof(adv_params));
	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

	rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event, NULL);

	if((rc != 0) && (rc != BLE_HS_EALREADY))
		util_abort_on_esp_err("bt: ble_gap_adv_start", rc);
}

static void callback_reset(int reason)
{
	log_format("bt: resetting state, reason: 0x%x", (unsigned int)reason);
}

static void callback_sync(void)
{
	util_abort_on_esp_err("bt: ble_hs_util_ensure_addr", ble_hs_util_ensure_addr(0));
	util_abort_on_esp_err("bt: ble_hs_id_infer_auto", ble_hs_id_infer_auto(0, &own_addr_type));
	util_abort_on_esp_err("bt: ble_hId_copy_addr", ble_hs_id_copy_addr(own_addr_type, bt_host_address, NULL));

	server_advertise();
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
	assert(inited);

	switch(event->type)
	{
		case(BLE_GAP_EVENT_CONNECT):
		{
			if(event->connect.status != 0)
				server_advertise();

			break;
		}

		case(BLE_GAP_EVENT_DISCONNECT):
		{
			server_advertise();

			break;
		}

		case(BLE_GAP_EVENT_ADV_COMPLETE):
		{
			log("bt: gap event complete");

			server_advertise();

			break;
		}

		case(BLE_GAP_EVENT_REPEAT_PAIRING):
		{
			log("bt: GAP EVENT repeat pairing");
			break;
		}

		case(BLE_GAP_EVENT_PASSKEY_ACTION):
		{
			log("bt: GAP EVENT passkey action");
			break;
		}

		case(BLE_GAP_EVENT_NOTIFY_TX):
		{
			//log("BLE_GAP_EVENT_NOTIFY_TX");

			/* NOTE: this event doesn't mean the notification is actually sent! */
			/* it's just called synchronously from within ble_gatts_indicate_custom */

			break;
		}

		case(BLE_GAP_EVENT_CONN_UPDATE):
		{
			log("BLE_GAP_EVENT_CONN_UPDATE");
			break;
		}

		case(BLE_GAP_EVENT_ENC_CHANGE):
		{
			log("BLE_GAP_EVENT_ENC_CHANGE");
			break;
		}

		case(BLE_GAP_EVENT_SUBSCRIBE):
		{
			//log("BLE_GAP_EVENT_SUBSCRIBE");
			break;
		}

		case(BLE_GAP_EVENT_MTU):
		{
			//log("BLE_GAP_EVENT_MTU");
			break;
		}

		case(BLE_GAP_EVENT_AUTHORIZE):
		{
			log("BLE_GAP_EVENT_AUTHORIZE");
			break;
		}

		case(BLE_GAP_EVENT_TRANSMIT_POWER):
		{
			log("BLE_GAP_EVENT_TRANSMIT_POWER");
			break;
		}

		case(BLE_GAP_EVENT_PATHLOSS_THRESHOLD):
		{
			log("BLE_GAP_EVENT_PATHLOSS_THRESHOLD");
			break;
		}

		case(BLE_GAP_EVENT_PHY_UPDATE_COMPLETE):
		{
			log("BLE_GAP_EVENT_PHY_UPDATE_COMPLETE");
			break;
		}

		case(BLE_GAP_EVENT_PARING_COMPLETE):
		{
			log("BLE_GAP_EVENT_PARING_COMPLETE");
			break;
		}

		case(BLE_GAP_EVENT_DATA_LEN_CHG):
		{
			//log("BLE_GAP_EVENT_DATA_LEN_CHG");
			break;
		}

		case(BLE_GAP_EVENT_LINK_ESTAB):
		{
			//log("BLE_GAP_EVENT_LINK_ESTAB");
			break;
		}

		default:
		{
			log_format("bt: gap event unknown: 0x%x", event->type);

			break;
		}
	}

	return(0);
}

static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *context, void *arg)
{
	switch (context->op)
	{
		case(BLE_GATT_REGISTER_OP_SVC):
		{
			break;
		}

		case(BLE_GATT_REGISTER_OP_CHR):
		{
			break;
		}

		default:
		{
			log_format("bt: gatt event unknown: 0x%x", context->op);
			abort();

			break;
		}
	}
}

static void bt_received(unsigned int connection_handle, unsigned int attribute_handle, const struct os_mbuf *mbuf)
{
	unsigned int length;
	std::string receive_buffer, decrypt_buffer;
	uint16_t om_length;

	assert(inited);
	assert(mbuf);

	length = os_mbuf_len(mbuf);

	receive_buffer.resize(length);

	ble_hs_mbuf_to_flat(mbuf, receive_buffer.data(), receive_buffer.size(), &om_length);

	assert(om_length == receive_buffer.size());

	stats_received_packets++;

	if(length == 0)
	{
		stats_received_null_packets++;
		return;
	}

	try
	{
		decrypt_buffer = Encryption::aes256_decrypt(Encryption::password_to_aes256_key(encryption_key), receive_buffer);
	}
	catch(const hard_exception &e)
	{
		log_format("bt_received: %s", e.what());
		stats_received_decryption_failed++;
		return;
	}

	receive_buffer.clear();

	stats_received_bytes += decrypt_buffer.size();

	if(!Packet::valid(decrypt_buffer))
	{
		stats_received_invalid_packets++;
		return;
	}

	if(!Packet::complete(decrypt_buffer))
	{
		stats_received_incomplete_packets++;
		return;
	}

	command_response_t *command_response = new command_response_t;

	command_response->source = cli_source_bt;
	command_response->packetised = 1;
	command_response->mtu = bt_mtu;
	command_response->packet = decrypt_buffer;
	command_response->bt.connection_handle = connection_handle;
	command_response->bt.attribute_handle = attribute_handle;

	cli_receive_queue_push(command_response);

	command_response = nullptr;
}

void net_bt_send(const command_response_t *command_response)
{
	struct os_mbuf *txom;
	std::string encrypt_buffer;
	int attempt, rv;

	assert(inited);

	try
	{
		encrypt_buffer = Encryption::aes256_encrypt(Encryption::password_to_aes256_key(encryption_key), command_response->packet);
	}
	catch(const hard_exception &e)
	{
		log_format("bt_send: %s", e.what());
		stats_sent_encryption_failed++;
		return;
	}

	for(attempt = 16; attempt > 0; attempt--)
	{
		txom = ble_hs_mbuf_from_flat(encrypt_buffer.data(), encrypt_buffer.size());
		assert(txom);

		rv = ble_gatts_indicate_custom(command_response->bt.connection_handle, command_response->bt.attribute_handle, txom);

		if(rv == 0)
			break;

		if(rv != BLE_HS_ENOMEM)
		{
			stats_indication_error++;
			log_format("bt: send error: %d", rv);
			return;
		}
		else
			log("bt: HS_ENOMEM");

		util_sleep(100);

		log("bt: send: retry");
	}

	if(attempt == 0)
	{
		stats_indication_timeout++;
		return;
	}

	stats_sent_bytes += command_response->packet.size();
	stats_sent_packets++;
}

void bt_init(void)
{
	assert(!inited);

	try
	{
		hostname = Config::get_string("hostname");
	}
	catch(transient_exception &)
	{
		hostname = "esp32";
	}

	try
	{
		encryption_key = Config::get_string("bt.key");
	}
	catch(transient_exception &)
	{
		encryption_key = "default";
	}

	util_warn_on_esp_err("nimble_port_init", nimble_port_init());

	inited = true;

	ble_hs_cfg.reset_cb = callback_reset;
	ble_hs_cfg.sync_cb = callback_sync;
	ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
	ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
	ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
	ble_hs_cfg.sm_bonding = 0;
	ble_hs_cfg.sm_mitm = 0;
	ble_hs_cfg.sm_sc = 0;
	ble_hs_cfg.sm_our_key_dist  = 0;
	ble_hs_cfg.sm_their_key_dist = 0;

	util_abort_on_esp_err("gatt_init", gatt_init());
	util_abort_on_esp_err("ble_svc_gap_device_name_set", ble_svc_gap_device_name_set(hostname.c_str()));
	ble_store_config_init();
	nimble_port_freertos_init(nimble_port_task);
}

void bluetooth_command_info(cli_command_call_t *call)
{
	string_auto(string_addr, 32);

	assert(call->parameter_count == 0);

	util_mac_addr_to_string(string_addr, bt_host_address, true);

	call->result = "bluetooth information";

	call->result += (boost::format("\n  address: %s") % string_cstr(string_addr)).str();
	call->result += "\n  data sent:";
	call->result += (boost::format("\n  - packets: %u") % stats_sent_packets).str();
	call->result += (boost::format("\n  - bytes: %u") % stats_sent_bytes).str();
	call->result += (boost::format("\n  - encryption failed: %u") % stats_sent_encryption_failed).str();
	call->result += "\n  data received:";
	call->result += (boost::format("\n  - bytes: %u") % stats_received_bytes).str();
	call->result += (boost::format("\n  - packets: %u") % stats_received_packets).str();
	call->result += (boost::format("\n  - decryption failed: %u") % stats_received_decryption_failed).str();
	call->result += (boost::format("\n  - null packets: %u") % stats_received_null_packets).str();
	call->result += (boost::format("\n  - invalid packets: %u") % stats_received_invalid_packets).str();
	call->result += (boost::format("\n  - incomplete packets: %u") % stats_received_incomplete_packets).str();
	call->result += "\n  indications:";
	call->result += (boost::format("\n  - errors: %u") % stats_indication_error).str();
	call->result += (boost::format("\n  - timeouts: %u") % stats_indication_timeout).str();
}

void bluetooth_command_key(cli_command_call_t *call)
{
	std::string key;

	try
	{
		switch(call->parameter_count)
		{
			case(1):
			{
				Config::set_string("bt.key", call->parameters[0].str);
				encryption_key = call->parameters[0].str;
				[[fallthrough]];
			}
			case(0):
			{
				try
				{
					key = Config::get_string("bt.key");
				}
				catch(const transient_exception &)
				{
					key = encryption_key;
				}

				call->result = (boost::format("bluetooth key: %s") % key).str();
				break;
			}
			default:
			{
				util_abort("bluetooth-command-key: invalid parameter count");
				break;
			}
		}
	}
	catch(const e32if_exception &e)
	{
		call->result = std::string("bluetooth-command-key: ") + e.what();
	}
}
