#include "cli.h"
#include "bt.h"
#include "util.h"
#include "packet.h"

#include <stdint.h>
#include <stdbool.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_err.h>

#include <nimble/nimble_port_freertos.h>
#include <nimble/nimble_port.h>
#include <store/config/ble_store_config.h>
#include <host/ble_hs.h>
#include <host/util/util.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>

void ble_store_config_init(void);

enum
{
	SERVICE_HANDLE = 0xabf0,
	CHARACTERISTICS_HANDLE = 0xabf1,
};

enum
{
	reassembly_buffer_size = 4096 + sizeof(packet_header_t) + 32,
	reassembly_timeout_ms = 2000,
	reassembly_raw_stream_fragmented_size = 512,
	reassembly_expected_length_unknown = ~0UL,
};

static int gap_event(struct ble_gap_event *event, void *arg);
static int gatt_event(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *context, void *arg);
static void bt_received(unsigned int connection_handle, unsigned int attribute_handle, const struct os_mbuf *mbuf);

static uint16_t attribute_handle;
static uint8_t own_addr_type;
static bool inited = false;
static uint8_t *reassembly_buffer = (uint8_t *)0;
static unsigned int reassembly_offset = 0;
static unsigned int reassembly_expected_length = 0;
static uint64_t reassembly_timestamp_start = 0;

static const struct ble_gatt_svc_def gatt_definitions[] =
{
	{
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = BLE_UUID16_DECLARE(SERVICE_HANDLE),
		.characteristics = (struct ble_gatt_chr_def[])
		{
			{
				.uuid = BLE_UUID16_DECLARE(CHARACTERISTICS_HANDLE),
				.access_cb = gatt_event,
				.val_handle = &attribute_handle,
				.flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
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

static char *bt_addr_to_str(const void *addr, char *str, unsigned int size)
{
	const uint8_t *u8p = addr;

	snprintf(str, size, "%02x:%02x:%02x:%02x:%02x:%02x", u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);

	return(str);
}

static char *conn_info_to_str(const struct ble_gap_conn_desc *desc, char *buffer, unsigned int buffer_length)
{
	char addr[64];
	char line[128];

	snprintf(buffer, buffer_length, "handle: %d our_ota_addr_type: %d our_ota_addr: %s",
		desc->conn_handle,
		desc->our_ota_addr.type,
		bt_addr_to_str(desc->our_ota_addr.val, addr, sizeof(addr)));

	snprintf(line, sizeof(line), " our_id_addr_type: %d our_id_addr: %s",
		desc->our_id_addr.type,
		bt_addr_to_str(desc->our_id_addr.val, addr, sizeof(addr)));
	strncat(buffer, line, buffer_length - 1);

	snprintf(line, sizeof(line), " peer_ota_addr_type: %d peer_ota_addr: %s",
		desc->peer_ota_addr.type,
		bt_addr_to_str(desc->peer_ota_addr.val, addr, sizeof(addr)));
	strncat(buffer, line, buffer_length - 1);

	snprintf(line, sizeof(line), " peer_id_addr_type: %d peer_id_addr: %s",
		desc->peer_id_addr.type,
		bt_addr_to_str(desc->peer_id_addr.val, addr, sizeof(addr)));
	strncat(buffer, line, buffer_length - 1);

	snprintf(line, sizeof(line), " conn_itvl: %d conn_latency: %d supervision_timeout: %d encrypted: %d authenticated: %d bonded: %d\n",
			desc->conn_itvl, desc->conn_latency,
			desc->supervision_timeout,
			desc->sec_state.encrypted,
			desc->sec_state.authenticated,
			desc->sec_state.bonded);
	strncat(buffer, line, buffer_length - 1);

	return(buffer);
}

static inline void reassemble_reset(void)
{
	reassembly_offset = 0;
	reassembly_expected_length = 0;
	reassembly_timestamp_start = 0;
}

static void nimble_port_task(void *param)
{
	assert(inited);

	nimble_port_run();
	nimble_port_freertos_deinit();
}

static int gatt_event(uint16_t connection_handle, uint16_t attribute_handle, struct ble_gatt_access_ctxt *context, void *arg)
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
			ESP_LOGW("bt", "gatt handler: default callback: %d", context->op);
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
	fields.name = (uint8_t *)name;
	fields.name_len = strlen(name);
	fields.name_is_complete = 1;

	fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
	fields.tx_pwr_lvl_is_present = 1;

	ESP_ERROR_CHECK(ble_gap_adv_set_fields(&fields));

	memset(&adv_params, 0, sizeof(adv_params));
	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

	rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event, NULL);

	if((rc != 0) && (rc != BLE_HS_EALREADY))
	{
		ESP_LOGE("bt", "ble_gap_adv_start, error: %x", rc);
		abort();
	}
}

static void callback_reset(int reason)
{
	ESP_LOGW("bt", "resetting state; reason=0x%x", reason);
}

static void callback_sync(void)
{
	int rc;
	char addr[64];
	uint8_t addr_val[6] = {0};

	ESP_ERROR_CHECK(ble_hs_util_ensure_addr(0));

	if((rc = ble_hs_id_infer_auto(0, &own_addr_type)) != 0)
	{
		ESP_LOGW("bt", "sync: error determining address type; rc=%d", rc);
		return;
	}

	rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

	ESP_LOGI("bt", "sync: device address: %s", bt_addr_to_str(addr_val, addr, sizeof(addr)));
	
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
			reassemble_reset();

			if(event->connect.status != 0)
				server_advertise();

			break;
		}

		case(BLE_GAP_EVENT_REPEAT_PAIRING):
		{
			struct ble_gap_conn_desc desc;

			ESP_ERROR_CHECK(ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc));
			ble_store_util_delete_peer(&desc.peer_id_addr);

			return(BLE_GAP_REPEAT_PAIRING_RETRY);
		}

		case(BLE_GAP_EVENT_PASSKEY_ACTION):
		{
			 if(event->passkey.params.action == BLE_SM_IOACT_DISP)
			 {
				struct ble_sm_io pkey = {0};

				pkey.action = BLE_SM_IOACT_DISP;
				pkey.passkey = 28022;

				if((rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey)) != 0)
					ESP_LOGW("bt", "passkey error: ble_sm_inject_io result: %d", rc);
			}
			else
				ESP_LOGW("bt", "passkey: unknown op: %d", event->passkey.params.action);

			ble_gap_terminate(event->connect.conn_handle, BLE_ERR_CONN_LIMIT);

			break;
		}

		case(BLE_GAP_EVENT_DISCONNECT):
		{
			reassemble_reset();
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
		{
			break;
		}

		default:
		{
			ESP_LOGW("bt", "EVENT unknown: 0x%x", event->type);

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
			ESP_LOGE("bt", "gatt unknown event: %d", context->op);
			abort();

			break;
		}
	}
}

static void bt_received(unsigned int connection_handle, unsigned int attribute_handle, const struct os_mbuf *mbuf)
{
	cli_buffer_t cli_buffer;
	unsigned int reassembled_packet_length, chunk_length, current_write_offset;
	uint64_t reassembly_timestamp_now;
	uint16_t om_length;

	assert(inited);
	assert(reassembly_buffer);
	assert(mbuf);

	if((chunk_length = os_mbuf_len(mbuf)) > reassembly_buffer_size)
	{
		ESP_LOGW("bt", "reassembly: chunk too large: %u (> %u), drop chunk", chunk_length, reassembly_buffer_size);
		return(reassemble_reset());
	}

	reassembly_timestamp_now = esp_timer_get_time();

	if(!reassembly_offset || !reassembly_expected_length || !reassembly_timestamp_start)
	{
		assert(!reassembly_offset);
		assert(!reassembly_expected_length);
		assert(!reassembly_timestamp_start);

		ble_hs_mbuf_to_flat(mbuf, reassembly_buffer, chunk_length, &om_length);
		assert(om_length == chunk_length);

		reassembly_offset = chunk_length;

		if(packet_is_packet(chunk_length, reassembly_buffer) && (chunk_length < (reassembled_packet_length = packet_length(chunk_length, reassembly_buffer))))
		{
			if(!reassembled_packet_length)
				return(reassemble_reset());

			reassembly_expected_length = reassembled_packet_length;
			reassembly_timestamp_start = reassembly_timestamp_now;
			return;
		}
		else
		{
			if(!packet_is_packet(chunk_length, reassembly_buffer) && (chunk_length == reassembly_raw_stream_fragmented_size))
			{
				reassembly_expected_length = reassembly_expected_length_unknown;
				reassembly_timestamp_start = reassembly_timestamp_now;
				return;
			}
		}
	}
	else
	{
		assert(reassembly_offset);
		assert(reassembly_expected_length);
		assert(reassembly_timestamp_start);

		current_write_offset = reassembly_offset;
		reassembly_offset += chunk_length;

		if(reassembly_offset > reassembly_buffer_size)
		{
			ESP_LOGW("bt", "reassembly: buffer overrun: %u + %u = %u (> %u), dropped chunk",
					current_write_offset, chunk_length, reassembly_offset, reassembly_buffer_size);
			return(reassemble_reset());
		}

		ble_hs_mbuf_to_flat(mbuf, &reassembly_buffer[current_write_offset], chunk_length, &om_length);
		assert(om_length == chunk_length);

		if(((reassembly_timestamp_now - reassembly_timestamp_start) / 1000) >= reassembly_timeout_ms)
		{
			if(reassembly_expected_length != reassembly_expected_length_unknown)
				return(reassemble_reset());
		}
		else
		{
			if(reassembly_expected_length == reassembly_expected_length_unknown)
			{
				if(chunk_length == reassembly_raw_stream_fragmented_size)
					return;
			}
			else
			{
				if(reassembly_offset > reassembly_expected_length)
					return(reassemble_reset());
				else
				{
					if(reassembly_offset < reassembly_expected_length)
						return;
				}
			}
		}
	}

	cli_buffer.source = cli_source_bt;
	cli_buffer.length = reassembly_offset;
	cli_buffer.data_from_malloc = 1;
	assert((cli_buffer.data = heap_caps_malloc(cli_buffer.length ? cli_buffer.length : cli_buffer.length + 1, MALLOC_CAP_SPIRAM)));
	memcpy(cli_buffer.data, reassembly_buffer, cli_buffer.length);
	cli_buffer.bt.connection_handle = connection_handle;
	cli_buffer.bt.attribute_handle = attribute_handle;

	cli_receive_queue_push(&cli_buffer);
	reassemble_reset();
}

void bt_send(cli_buffer_t *cli_buffer)
{
	static const unsigned int max_chunk = /* netto data */ 512 + /* sizeof(packet_header_t) */ 32 + /* HCI headers */ 8;
	struct os_mbuf *txom;
	unsigned int offset, chunk, length, attempt;
	int rv;

	assert(inited);

	offset = 0;
	length = cli_buffer->length;

	while(length > 0)
	{
		chunk = length;

		if(chunk > max_chunk)
			chunk = max_chunk;

		for(attempt = 16; attempt > 0; attempt--)
		{
			txom = ble_hs_mbuf_from_flat(&cli_buffer->data[offset], chunk);
			assert(txom);

			rv = ble_gatts_indicate_custom(cli_buffer->bt.connection_handle, cli_buffer->bt.attribute_handle, txom);

			if(!rv)
				break;

			if(rv != BLE_HS_ENOMEM)
			{
				ESP_LOGW("bt", "indicate: error: %x", rv);
				return;
			}

			vTaskDelay(100 / portTICK_PERIOD_MS);
		}

		if(attempt == 0)
			ESP_LOGW("bt", "bt_send: no more attempts");

		length -= chunk;
		offset += chunk;
	}
}

esp_err_t bt_init(void)
{
	assert(!inited);

	assert((reassembly_buffer = heap_caps_malloc(reassembly_buffer_size, MALLOC_CAP_SPIRAM)));
	reassemble_reset();

	ESP_ERROR_CHECK(nimble_port_init());

	inited = true;

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

	ESP_ERROR_CHECK(gatt_init());
	ESP_ERROR_CHECK(ble_svc_gap_device_name_set("nimble-ble-spp-svr"));
	ble_store_config_init();
	nimble_port_freertos_init(nimble_port_task);

	return(0);
}
