#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <esp_log.h>

#include "string.h"
#include "cli.h"
#include "bt.h"
#include "log.h"
#include "util.h"
#include "packet.h"
#include "config.h"
#include "bt_pair_pin.h"
#include "cli-command.h"
#include "packet_header.h"

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

void ble_store_config_init(void);

enum
{
	SERVICE_HANDLE = 0xabf0,
	CHARACTERISTICS_HANDLE = 0xabf1,
	KEY_HANDLE = 0xabf2,
	bt_mtu = 512,
	bt_max_chunk = bt_mtu + sizeof(packet_header_t) + 8 /* HCI metadata */,
	max_packet_size = 4096 + sizeof(packet_header_t) + 128,
};

static TimerHandle_t bt_defragmentation_timer;
static string_t bt_receive_buffer;
static bool bt_defragmentation_incomplete;

static int gap_event(struct ble_gap_event *event, void *arg);
static int gatt_value_event(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *context, void *arg);
static int gatt_key_event(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *context, void *arg);
static void bt_received(unsigned int connection_handle, unsigned int attribute_handle, const struct os_mbuf *mbuf);

static uint8_t bt_host_address[6] = {0};
static uint8_t key[12];
static uint16_t value_attribute_handle;
static uint16_t key_attribute_handle;
static uint8_t own_addr_type;
static bool inited = false;
static bool authorised = false;

static unsigned int bt_stats_authentication_attempts;
static unsigned int bt_stats_unauthorised_access;

static unsigned int bt_stats_indication_error;
static unsigned int bt_stats_indication_timeout;

static unsigned int bt_stats_sent_bytes;
static unsigned int bt_stats_sent_packets;

static unsigned int bt_stats_received_bytes;
static unsigned int bt_stats_received_packets;
static unsigned int bt_stats_received_raw_packets;
static unsigned int bt_stats_received_packetised_packets;
static unsigned int bt_stats_received_defragmentation_timeouts;

static const struct ble_gatt_svc_def gatt_definitions[] =
{
	{
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = BLE_UUID16_DECLARE(SERVICE_HANDLE),
		.characteristics = (struct ble_gatt_chr_def[])
		{
			{
				.uuid = BLE_UUID16_DECLARE(CHARACTERISTICS_HANDLE),
				.access_cb = gatt_value_event,
				.val_handle = &value_attribute_handle,
				.flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
			},
			{
				.uuid = BLE_UUID16_DECLARE(KEY_HANDLE),
				.access_cb = gatt_key_event,
				.val_handle = &key_attribute_handle,
				.flags = BLE_GATT_CHR_F_WRITE,
			},
			{
				0,
			}
		},
	},
	{
		0,
	},
};

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
			if(!authorised)
				bt_stats_unauthorised_access++;
			else
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

static int gatt_key_event(uint16_t connection_handle, uint16_t attribute_handle, struct ble_gatt_access_ctxt *context, void *arg)
{
	assert(inited);
	authorised = false;

	switch (context->op)
	{
		case(BLE_GATT_ACCESS_OP_WRITE_CHR):
		{
			string_auto(input_string, 16);
			string_auto(output_string, 16);
			unsigned int length;

			bt_stats_authentication_attempts++;

			length = string_assign_mbuf(input_string, context->om);

			if(length != 16)
				break;

			decrypt_aes_256(output_string, input_string);

			if(string_length(output_string) != 16)
				break;

			if((string_at(output_string, 12) != 0x04) ||
					(string_at(output_string, 13) != 0x04) ||
					(string_at(output_string, 14) != 0x04) ||
					(string_at(output_string, 15) != 0x04))
				break;

			string_truncate(output_string, sizeof(key));

			if(!string_equal_data(output_string, sizeof(key), key))
				break;

			authorised = true;

			break;
		}

		default:
		{
			log_format("bt: gatt_key_event: default callback: 0x%x", context->op);
			break;
		}
	}

	return(0);
}

static int gatt_init(void)
{
	int rc = 0;
	ble_svc_gap_init();
	ble_svc_gatt_init();

	if((rc = ble_gatts_count_cfg(gatt_definitions)) != 0)
		return(rc);

	if((rc = ble_gatts_add_svcs(gatt_definitions)) != 0)
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

	fields.uuids16 = (ble_uuid16_t[]) { BLE_UUID16_INIT(SERVICE_HANDLE) };
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
	uint8_t *keyptr;

	util_abort_on_esp_err("bt: ble_hs_util_ensure_addr", ble_hs_util_ensure_addr(0));
	util_abort_on_esp_err("bt: ble_hs_id_infer_auto", ble_hs_id_infer_auto(0, &own_addr_type));
	util_abort_on_esp_err("bt: ble_hId_copy_addr", ble_hs_id_copy_addr(own_addr_type, bt_host_address, NULL));

	keyptr = key;

	*(keyptr++) = bt_host_address[0] ^ 0x55;
	*(keyptr++) = bt_host_address[1] ^ 0x55;
	*(keyptr++) = bt_host_address[2] ^ 0x55;
	*(keyptr++) = bt_host_address[3] ^ 0x55;
	*(keyptr++) = bt_host_address[4] ^ 0x55;
	*(keyptr++) = bt_host_address[5] ^ 0x55;
	*(keyptr++) = bt_host_address[5] ^ 0xaa;
	*(keyptr++) = bt_host_address[4] ^ 0xaa;
	*(keyptr++) = bt_host_address[3] ^ 0xaa;
	*(keyptr++) = bt_host_address[2] ^ 0xaa;
	*(keyptr++) = bt_host_address[1] ^ 0xaa;
	*(keyptr++) = bt_host_address[0] ^ 0xaa;

	server_advertise();
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
	int rc;

	assert(inited);

	switch(event->type)
	{
		case(BLE_GAP_EVENT_CONNECT):
		{
			authorised = false;

			if(event->connect.status != 0)
				server_advertise();

			break;
		}

		case(BLE_GAP_EVENT_REPEAT_PAIRING):
		{
			struct ble_gap_conn_desc desc;

			log("bt: GAP EVENT repeat pairing");

			util_abort_on_esp_err("ble_gap_conn_find", ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc));
			ble_store_util_delete_peer(&desc.peer_id_addr);

			return(BLE_GAP_REPEAT_PAIRING_RETRY);
		}

		case(BLE_GAP_EVENT_PASSKEY_ACTION):
		{
			log("bt: GAP EVENT passkey action");

			 if(event->passkey.params.action == BLE_SM_IOACT_DISP)
			 {
				struct ble_sm_io pkey = {0};

				pkey.action = BLE_SM_IOACT_DISP;
				pkey.passkey = bt_pair_pin;

				if((rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey)) != 0)
					log_format("bt: passkey error: ble_sm_inject_io result: %d", rc);
			}
			else
				log_format("bt: passkey: unknown op: %d", event->passkey.params.action);

			//ble_gap_terminate(event->connect.conn_handle, BLE_ERR_CONN_LIMIT); // FIXME does this need to be here?

			break;
		}

		case(BLE_GAP_EVENT_DISCONNECT):
		{
			authorised = false;
			server_advertise();

			break;
		}

		case(BLE_GAP_EVENT_ADV_COMPLETE):
		{
			server_advertise();

			break;
		}

		case(BLE_GAP_EVENT_NOTIFY_TX):
		{
			/* NOTE: this event doesn't mean the notification is actually sent! */
			/* it's just called synchronously from within ble_gatts_indicate_custom */

			break;
		}

		case(BLE_GAP_EVENT_ENC_CHANGE):
		case(BLE_GAP_EVENT_CONN_UPDATE):
		case(BLE_GAP_EVENT_MTU):
		case(BLE_GAP_EVENT_SUBSCRIBE):
		case(BLE_GAP_EVENT_PHY_UPDATE_COMPLETE):
		case(BLE_GAP_EVENT_PARING_COMPLETE):
		case(BLE_GAP_EVENT_DATA_LEN_CHG):
		case(BLE_GAP_EVENT_LINK_ESTAB):
		{
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

static void bt_defragmentation_callback(TimerHandle_t handle)
{
	bt_stats_received_defragmentation_timeouts++;
	log("bt: defragmentation timed out");

	if(!bt_defragmentation_incomplete)
		log("bt: defragmentation while not active");

	bt_defragmentation_incomplete = false;
	string_clear(bt_receive_buffer);
}

static void bt_received(unsigned int connection_handle, unsigned int attribute_handle, const struct os_mbuf *mbuf)
{
	cli_buffer_t cli_buffer;
	unsigned int length;

	assert(inited);
	assert(mbuf);

	length = string_append_mbuf(bt_receive_buffer, mbuf);

	bt_stats_received_bytes += length;

	if(packet_valid(bt_receive_buffer))
	{
		bt_stats_received_packetised_packets++;

		if(packet_complete(bt_receive_buffer))
		{
			bt_defragmentation_incomplete = false;
			xTimerStop(bt_defragmentation_timer, portMAX_DELAY);
			cli_buffer.packetised = 1;
		}
		else
		{
			if(!bt_defragmentation_incomplete)
			{
				bt_defragmentation_incomplete = true;
				xTimerStart(bt_defragmentation_timer, portMAX_DELAY);
			}
		}
	}
	else
	{
		bt_stats_received_raw_packets++;
		bt_defragmentation_incomplete = false;
		cli_buffer.packetised = 0;
	}

	if(!bt_defragmentation_incomplete)
	{
		bt_stats_received_packets++;

		cli_buffer.source = cli_source_bt;
		cli_buffer.mtu = bt_mtu;
		cli_buffer.data = string_new(string_length(bt_receive_buffer));
		string_assign_string(cli_buffer.data, bt_receive_buffer);
		cli_buffer.bt.connection_handle = connection_handle;
		cli_buffer.bt.attribute_handle = attribute_handle;
		string_clear(bt_receive_buffer);
		bt_defragmentation_incomplete = false;

		cli_receive_queue_push(&cli_buffer);
	}
}

void bt_send(const cli_buffer_t *cli_buffer)
{
	struct os_mbuf *txom;
	int offset, chunk_length, rv, attempt;

	assert(inited);

	for(offset = 0; offset < string_length(cli_buffer->data);)
	{
		if((chunk_length = string_length(cli_buffer->data) - offset) < 0)
			break;

		if(chunk_length > bt_max_chunk)
			chunk_length = bt_max_chunk;

		for(attempt = 16; attempt > 0; attempt--)
		{
			txom = ble_hs_mbuf_from_flat(string_data(cli_buffer->data) + offset, chunk_length);
			assert(txom);

			rv = ble_gatts_indicate_custom(cli_buffer->bt.connection_handle, cli_buffer->bt.attribute_handle, txom);

			if(rv == 0)
				break;

			if(rv != BLE_HS_ENOMEM)
			{
				bt_stats_indication_error++;
				log_format("bt: send error: %d", rv);
				goto done;
			}

			util_sleep(100);
		}

		if(attempt == 0)
		{
			bt_stats_indication_timeout++;
			break;
		}

		offset += chunk_length;
		bt_stats_sent_bytes += chunk_length;
		bt_stats_sent_packets++;
	}

done:
}

void bt_init(void)
{
	string_auto_init(hostname_key, "hostname");
	string_auto(hostname, 16);

	assert(!inited);

	if(!config_get_string(hostname_key, hostname))
		string_assign_cstr(hostname, "esp32");

	util_abort_on_esp_err("nimble_port_init", nimble_port_init());

	inited = true;

	bt_defragmentation_incomplete = false;
	bt_receive_buffer = string_new(max_packet_size);

	bt_defragmentation_timer = xTimerCreate("bt-defrag", pdMS_TO_TICKS(10000), pdFALSE, (void *)0, bt_defragmentation_callback);
	assert(bt_defragmentation_timer);
	bt_defragmentation_incomplete = false;

	ble_hs_cfg.reset_cb = callback_reset;
	ble_hs_cfg.sync_cb = callback_sync;
	ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
	ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

	ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY;
	ble_hs_cfg.sm_bonding = 1;
	ble_hs_cfg.sm_mitm = 1;
	ble_hs_cfg.sm_sc = 1;
	ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
	ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

	util_abort_on_esp_err("gatt_init", gatt_init());
	util_abort_on_esp_err("ble_svc_gap_device_name_set", ble_svc_gap_device_name_set(string_cstr(hostname)));
	ble_store_config_init();
	nimble_port_freertos_init(nimble_port_task);
}

void bluetooth_command_info(cli_command_call_t *call)
{
	string_auto(string_addr, 32);

	assert(call->parameter_count == 0);

	util_mac_addr_to_string(string_addr, bt_host_address, true);

	string_format(call->result, "bluetooth information");

	string_format_append(call->result, "\n  address: %s", string_cstr(string_addr));
	string_format_append(call->result, "\n  authorised: %s", authorised ? "yes" : "no");
	string_format_append(call->result, "\n  authentication attempts: %u", bt_stats_authentication_attempts);
	string_format_append(call->result, "\n  unauthorised access: %u", bt_stats_unauthorised_access);
	string_format_append(call->result, "\n  data sent:");
	string_format_append(call->result, "\n  - packets: %u", bt_stats_sent_packets);
	string_format_append(call->result, "\n  - bytes: %u", bt_stats_sent_bytes);

	string_format_append(call->result, "\n  data received:");
	string_format_append(call->result, "\n  - packets: %u", bt_stats_received_packets);
	string_format_append(call->result, "\n  - bytes: %u", bt_stats_received_bytes);
	string_format_append(call->result, "\n  - defragmentation timeouts: %u", bt_stats_received_defragmentation_timeouts);
	string_format_append(call->result, "\n  - packetised packets: %u", bt_stats_received_packetised_packets);
	string_format_append(call->result, "\n  - raw packets: %u", bt_stats_received_raw_packets);

	string_format_append(call->result, "\n  indication:");
	string_format_append(call->result, "\n  - errors: %u", bt_stats_indication_error);
	string_format_append(call->result, "\n  - timeouts: %u", bt_stats_indication_timeout);
}
