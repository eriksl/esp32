#include "bt.h"

#include "log.h"
#include "packet.h"
#include "config.h"
#include "system.h"
#include "crypt.h"
#include "exception.h"
#include "command.h"
#include "cli-command.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <nimble/nimble_port_freertos.h>
#include <nimble/nimble_port.h>
#include <store/config/ble_store_config.h>
#include <host/ble_hs.h>
#include <host/util/util.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>

#include <format>
#include <thread>
#include <chrono>

extern "C" void ble_store_config_init();

BT *BT::singleton = nullptr;

BT::BT(Log &log_in, Config &config_in) : log(log_in), config(config_in)
{
	static constexpr ble_uuid_t uuid16 = { .type = BLE_UUID_TYPE_16, };
	static struct ble_gatt_svc_def gatt_svc_definitions[2];
	static struct ble_gatt_chr_def gatt_chr_definitions[2];
	ble_gatt_svc_def *gatt_svc_definition;
	ble_gatt_chr_def *gatt_chr_definition;

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

	esp_err_t rv;
	int rc;

	this->command = nullptr;

	if(singleton)
		throw(hard_exception("BT: already active"));

	try
	{
		this->hostname = this->config.get_string("hostname");
	}
	catch(transient_exception &)
	{
		this->hostname = "esp32";
	}

	try
	{
		this->encryption_key = this->config.get_string("bt.key");
	}
	catch(transient_exception &)
	{
		this->encryption_key = "default";
	}

	if((rv = ::nimble_port_init()) != ESP_OK)
		throw(hard_exception(log.esp_string_error(rv, "BT: nimble_port_init")));

	::ble_hs_cfg.reset_cb = this->callback_reset_wrapper;
	::ble_hs_cfg.sync_cb = this->callback_sync_wrapper;
	::ble_hs_cfg.gatts_register_cb = this->gatt_svr_register_cb_wrapper;
	::ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
	::ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
	::ble_hs_cfg.sm_bonding = 0;
	::ble_hs_cfg.sm_mitm = 0;
	::ble_hs_cfg.sm_sc = 0;
	::ble_hs_cfg.sm_our_key_dist  = 0;
	::ble_hs_cfg.sm_their_key_dist = 0;

	::ble_svc_gap_init();
	::ble_svc_gatt_init();

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
	gatt_chr_definition->access_cb = gatt_value_event_wrapper,
	gatt_chr_definition->val_handle = &value_attribute_handle,
	gatt_chr_definition->flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,

	gatt_chr_definition = &gatt_chr_definitions[1];
	gatt_chr_definition->uuid = nullptr;
	gatt_chr_definition->access_cb = nullptr;
	gatt_chr_definition->val_handle = nullptr;
	gatt_chr_definition->flags = 0;

	if((rc = ::ble_gatts_count_cfg(gatt_svc_definitions)) != ESP_OK)
		throw(hard_exception(std::format("BT: ble_gatts_count_cfg: error: {:#x}", rc)));

	if((rc = ::ble_gatts_add_svcs(gatt_svc_definitions)) != ESP_OK)
		throw(hard_exception(std::format("BT: ble_gatts_add_svcs: {:#x}", rc)));

	if((rc = ::ble_svc_gap_device_name_set(hostname.c_str())) != ESP_OK)
		throw(hard_exception(std::format("BT: ble_svc_gap_device_name_set: {:#x}", rc)));

	::ble_store_config_init();

	this->stats_sent_bytes = 0;
	this->stats_sent_packets = 0;
	this->stats_sent_encryption_failed = 0;
	this->stats_received_bytes = 0;
	this->stats_received_packets = 0;
	this->stats_received_null_packets = 0;
	this->stats_received_decryption_failed = 0;
	this->stats_received_invalid_packets = 0;
	this->stats_received_incomplete_packets = 0;
	this->stats_indication_error = 0;
	this->stats_indication_timeout = 0;

	running = false;

	singleton = this;
}

void BT::run()
{
	if(running)
		throw(hard_exception("BT::run: already running"));

	::nimble_port_freertos_init(this->nimble_port_task);

	running = true;
}

BT &BT::get()
{
	if(!BT::singleton)
		throw(hard_exception("BT::get: not active"));

	return(*BT::singleton);
}

void BT::set(Command *cmd)
{
	if(this->command)
		throw(hard_exception("BT::set: command already set"));

	this->command = cmd;
}

void BT::nimble_port_task(void *)
{
	::nimble_port_run();
	::nimble_port_freertos_deinit();
}

int BT::gatt_value_event_wrapper(uint16_t connection_handle, uint16_t attribute_handle, struct ble_gatt_access_ctxt *context, void *arg)
{
	return(BT::get().gatt_value_event(connection_handle, attribute_handle, context, arg));
}

int BT::gatt_value_event(uint16_t connection_handle, uint16_t attribute_handle, struct ble_gatt_access_ctxt *context, void *arg)
{
	switch (context->op)
	{
		case(BLE_GATT_ACCESS_OP_WRITE_CHR):
		{
			this->received(connection_handle, attribute_handle, context->om);
			break;
		}

		default:
		{
			this->log << std::format("bt: gatt_value_event: default callback: {:#x}", context->op);
			break;
		}
	}

	return(0);
}

void BT::server_advertise()
{
	struct ble_gap_adv_params adv_params;
	struct ble_hs_adv_fields fields;;
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

	if((rc = ble_gap_adv_set_fields(&fields)))
		throw(hard_exception(std::format("BT: ble_gap_adv_set_fields: error: {:#x}", rc)));

	memset(&adv_params, 0, sizeof(adv_params));
	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

	rc = ::ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, this->gap_event_wrapper, NULL);

	if((rc != 0) && (rc != BLE_HS_EALREADY))
		throw(hard_exception(std::format("BT: ble_gap_adv_start: error: {:#x}", rc)));
}

void BT::callback_reset_wrapper(int reason)
{
	BT::get().callback_reset(reason);
}

void BT::callback_reset(int reason)
{
	log << std::format("bt: resetting state, reason: {:#x}", reason);
}

void BT::callback_sync_wrapper()
{
	return(BT::get().callback_sync());
}

void BT::callback_sync()
{
	int rc;

	if((rc = ble_hs_util_ensure_addr(0)) != 0)
		throw(hard_exception(std::format("BT: ble_hs_util_ensure_addr: error: {:#x}", rc)));

	if((rc = ble_hs_id_infer_auto(0, &own_addr_type)) != 0)
		throw(hard_exception(std::format("BT: ble_hs_id_infer_auto: error: {:#x}", rc)));

	if((rc = ble_hs_id_copy_addr(own_addr_type, bt_host_address, NULL)) != 0)
		throw(hard_exception(std::format("BT: ble_hs_id_copy_addr: error: {:#x}", rc)));

	this->server_advertise();
}

int BT::gap_event_wrapper(struct ble_gap_event *event, void *arg)
{
	return(BT::get().gap_event(event, arg));
}

int BT::gap_event(struct ble_gap_event *event, void *arg)
{
	switch(event->type)
	{
		case(BLE_GAP_EVENT_CONNECT):
		{
			if(event->connect.status != 0)
				this->server_advertise();

			break;
		}

		case(BLE_GAP_EVENT_DISCONNECT):
		{
			this->server_advertise();

			break;
		}

		case(BLE_GAP_EVENT_ADV_COMPLETE):
		{
			log << "bt: gap event complete";

			server_advertise();

			break;
		}

		case(BLE_GAP_EVENT_REPEAT_PAIRING):
		{
			log << "bt: GAP EVENT repeat pairing";
			break;
		}

		case(BLE_GAP_EVENT_PASSKEY_ACTION):
		{
			log << "bt: GAP EVENT passkey action";
			break;
		}

		case(BLE_GAP_EVENT_NOTIFY_TX):
		{
			log << "BLE_GAP_EVENT_NOTIFY_TX";

			/* NOTE: this event doesn't mean the notification is actually sent! */
			/* it's just called synchronously from within ble_gatts_indicate_custom */

			break;
		}

		case(BLE_GAP_EVENT_CONN_UPDATE):
		{
			log << "BLE_GAP_EVENT_CONN_UPDATE";
			break;
		}

		case(BLE_GAP_EVENT_ENC_CHANGE):
		{
			log << "BLE_GAP_EVENT_ENC_CHANGE";
			break;
		}

		case(BLE_GAP_EVENT_SUBSCRIBE):
		{
			log << "BLE_GAP_EVENT_SUBSCRIBE";
			break;
		}

		case(BLE_GAP_EVENT_MTU):
		{
			log << "BLE_GAP_EVENT_MTU";
			break;
		}

		case(BLE_GAP_EVENT_AUTHORIZE):
		{
			log << "BLE_GAP_EVENT_AUTHORIZE";
			break;
		}

		case(BLE_GAP_EVENT_TRANSMIT_POWER):
		{
			log << "BLE_GAP_EVENT_TRANSMIT_POWER";
			break;
		}

		case(BLE_GAP_EVENT_PATHLOSS_THRESHOLD):
		{
			log << "BLE_GAP_EVENT_PATHLOSS_THRESHOLD";
			break;
		}

		case(BLE_GAP_EVENT_PHY_UPDATE_COMPLETE):
		{
			log << "BLE_GAP_EVENT_PHY_UPDATE_COMPLETE";
			break;
		}

		case(BLE_GAP_EVENT_PARING_COMPLETE):
		{
			log << "BLE_GAP_EVENT_PARING_COMPLETE";
			break;
		}

		case(BLE_GAP_EVENT_DATA_LEN_CHG):
		{
			log << "BLE_GAP_EVENT_DATA_LEN_CHG";
			break;
		}

		case(BLE_GAP_EVENT_LINK_ESTAB):
		{
			//log << "BLE_GAP_EVENT_LINK_ESTAB";
			break;
		}

		default:
		{
			log << std::format("bt: gap event unknown: {:#x}", event->type);

			break;
		}
	}

	return(0);
}

void BT::gatt_svr_register_cb_wrapper(struct ble_gatt_register_ctxt *context, void *arg)
{
	BT::get().gatt_svr_register_cb(context, arg);
}

void BT::gatt_svr_register_cb(struct ble_gatt_register_ctxt *context, void *arg)
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
			log  << std::format("bt: gatt event unknown: {:#x}", context->op);

			break;
		}
	}
}

void BT::received(unsigned int connection_handle, unsigned int attribute_handle, const struct os_mbuf *mbuf)
{
	unsigned int length;
	std::string receive_buffer, decrypt_buffer;
	uint16_t om_length;

	if(!mbuf)
		throw(hard_exception("BT::received: invalid mbuf"));

	length = os_mbuf_len(mbuf);

	receive_buffer.resize(length);

	ble_hs_mbuf_to_flat(mbuf, receive_buffer.data(), receive_buffer.size(), &om_length);

	if(om_length != receive_buffer.size())
		throw(hard_exception("BT::received:: invalid mbuf length"));

	this->stats_received_packets++;

	if(length == 0)
	{
		this->stats_received_null_packets++;
		return;
	}

	try
	{
		decrypt_buffer = Crypt::aes256(false, Crypt::password_to_aes256_key(encryption_key), receive_buffer);
	}
	catch(const hard_exception &e)
	{
		this->stats_received_decryption_failed++;
		return;
	}

	receive_buffer.clear();

	this->stats_received_bytes += decrypt_buffer.size();

	if(!Packet::valid(decrypt_buffer))
	{
		this->stats_received_invalid_packets++;
		return;
	}

	if(!Packet::complete(decrypt_buffer))
	{
		this->stats_received_incomplete_packets++;
		return;
	}

	command_response_t *command_response = new command_response_t;

	command_response->source = cli_source_bt;
	command_response->packetised = 1;
	command_response->mtu = bt_mtu;
	command_response->packet = decrypt_buffer;
	command_response->bt.connection_handle = connection_handle;
	command_response->bt.attribute_handle = attribute_handle;

	command->receive_queue_push(command_response);

	command_response = nullptr;
}

void BT::send(const command_response_t *command_response)
{
	struct os_mbuf *txom;
	std::string encrypt_buffer;
	int attempt, rv;

	try
	{
		encrypt_buffer = Crypt::aes256(true, Crypt::password_to_aes256_key(encryption_key), command_response->packet);
	}
	catch(const hard_exception &e)
	{
		this->stats_sent_encryption_failed++;
		return;
	}

	for(attempt = 16; attempt > 0; attempt--)
	{
		txom = ble_hs_mbuf_from_flat(encrypt_buffer.data(), encrypt_buffer.size());

		if(!txom)
			throw(hard_exception("BT::send: invalid mbuf"));

		rv = ble_gatts_indicate_custom(command_response->bt.connection_handle, command_response->bt.attribute_handle, txom);

		if(rv == 0)
			break;

		if(rv != BLE_HS_ENOMEM)
		{
			this->stats_indication_error++;
			log << std::format("bt: send error: {:#x}", rv);
			return;
		}
		else
			log << "bt: HS_ENOMEM";

		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		log << "bt: send: retry";
	}

	if(attempt == 0)
	{
		this->stats_indication_timeout++;
		return;
	}

	this->stats_sent_bytes += command_response->packet.size();
	this->stats_sent_packets++;
}

void BT::info(std::string &out)
{
	out += std::format("\n  address: {}", System::get().mac_addr_to_string(reinterpret_cast<const char *>(bt_host_address), true)); // FIXME
	out += "\n  data sent:";
	out += std::format("\n  - packets: {:d}", this->stats_sent_packets);
	out += std::format("\n  - bytes: {:d}", this->stats_sent_bytes);
	out += std::format("\n  - encryption failed: {:d}", this->stats_sent_encryption_failed);
	out += "\n  data received:";
	out += std::format("\n  - bytes: {:d}", this->stats_received_bytes);
	out += std::format("\n  - packets: {:d}", this->stats_received_packets);
	out += std::format("\n  - decryption failed: {:d}", this->stats_received_decryption_failed);
	out += std::format("\n  - null packets: {:d}", this->stats_received_null_packets);
	out += std::format("\n  - invalid packets: {:d}", this->stats_received_invalid_packets);
	out += std::format("\n  - incomplete packets: {:d}", this->stats_received_incomplete_packets);
	out += "\n  indications:";
	out += std::format("\n  - errors: {:d}", this->stats_indication_error);
	out += std::format("\n  - timeouts: {:d}", this->stats_indication_timeout);
}

void BT::key(const std::string &ekey)
{
	config.set_string("bt.key", ekey);
	encryption_key = ekey;
}

std::string BT::key()
{
	std::string ekey;

	try
	{
		ekey = config.get_string("bt.key");
	}
	catch(const transient_exception &)
	{
		ekey = this->encryption_key;
	}

	return(ekey);
}
