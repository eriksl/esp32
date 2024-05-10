#include "cli.h"
#include "bt.h"
#include "util.h"

#include <stdint.h>

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

static int gap_event(struct ble_gap_event *event, void *arg);
static int gatt_event(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *context, void *arg);
static uint16_t attribute_handle;
static uint8_t own_addr_type;

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

static void nimble_port_task(void *param)
{
	nimble_port_run();
	nimble_port_freertos_deinit();
}

static int gatt_event(uint16_t connection_handle, uint16_t attribute_handle, struct ble_gatt_access_ctxt *context, void *arg)
{
	switch (context->op)
	{
		case(BLE_GATT_ACCESS_OP_WRITE_CHR):
		{
			cli_buffer_t cli_buffer;
			uint16_t length;

			//ESP_LOGW("bt", "gatt handler: data received in write event,connection_handle = 0x%x,attribute_handle = 0x%x", connection_handle, attribute_handle);
			cli_buffer.source = cli_source_bt;
			cli_buffer.length = os_mbuf_len(context->om);
			cli_buffer.data_from_malloc = 1;
			assert((cli_buffer.data = heap_caps_malloc(cli_buffer.length, MALLOC_CAP_SPIRAM)));
			ble_hs_mbuf_to_flat(context->om, cli_buffer.data, cli_buffer.length, &length);
			cli_buffer.bt.connection_handle = connection_handle;
			cli_buffer.bt.attribute_handle = attribute_handle;
			cli_receive_queue_push(&cli_buffer);

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

	if(((rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event, NULL)) != 0) && (rc != BLE_HS_EALREADY))
		ESP_ERROR_CHECK(rc);
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
	struct ble_gap_conn_desc desc;
	int rc;
	char buffer[512];

	switch(event->type)
	{
		case(BLE_GAP_EVENT_CONNECT):
		{
			ESP_LOGI("bt", "EVENT CONNECT: connection %s; status=%d ",
				event->connect.status == 0 ? "established" : "failed",
				event->connect.status);

			if (event->connect.status == 0)
			{
				ESP_ERROR_CHECK(ble_gap_conn_find(event->connect.conn_handle, &desc));
				ESP_LOGI("bt", "new connection: %s", conn_info_to_str(&desc, buffer, sizeof(buffer)));
			}

			if((event->connect.status != 0) || (CONFIG_BT_NIMBLE_MAX_CONNECTIONS > 1))
				server_advertise();

			break;
		}

		case(BLE_GAP_EVENT_REPEAT_PAIRING):
		{
			ESP_LOGI("bt", "%s", "EVENT REPEAT_PAIRING started");

			ESP_ERROR_CHECK(ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc));
			ble_store_util_delete_peer(&desc.peer_id_addr);

			return(BLE_GAP_REPEAT_PAIRING_RETRY);
		}

		case(BLE_GAP_EVENT_PASSKEY_ACTION):
		{
			 if(event->passkey.params.action == BLE_SM_IOACT_DISP)
			 {
				struct ble_sm_io pkey = {0};

				ESP_LOGI("bt", "%s", "passkey: BLE_SM_IOACT_DISP");
				ESP_LOGI("bt", "Enter passkey %lu on the peer side", pkey.passkey);
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

		case(BLE_GAP_EVENT_ENC_CHANGE):
		{
			ESP_ERROR_CHECK(ble_gap_conn_find(event->enc_change.conn_handle, &desc));
			ESP_LOGI("bt", "EVENT ENC CHANGE: status: %d, %s", event->enc_change.status, conn_info_to_str(&desc, buffer, sizeof(buffer)));

			break;
		}

		case(BLE_GAP_EVENT_DISCONNECT):
		{
			ESP_LOGI("bt", "EVENT DISCONNECT: reason: 0x%x\n%s",
					event->disconnect.reason,
					conn_info_to_str(&event->disconnect.conn, buffer, sizeof(buffer)));

			server_advertise();

			break;
		}

		case(BLE_GAP_EVENT_CONN_UPDATE):
		{
			ESP_LOGI("bt", "EVENT CONN UPDATE: connection updated; status=%d\n%s",
					event->conn_update.status,
					conn_info_to_str(&desc, buffer, sizeof(buffer)));
			ESP_ERROR_CHECK(ble_gap_conn_find(event->conn_update.conn_handle, &desc));

			break;
		}

		case(BLE_GAP_EVENT_ADV_COMPLETE):
		{
			ESP_LOGI("bt", "EVENT ADV COMPLETE: advertise complete; reason=%d", event->adv_complete.reason);
			server_advertise();

			break;
		}

		case(BLE_GAP_EVENT_MTU):
		{
			ESP_LOGI("bt", "EVENT MTU: mtu update event; conn_handle=%d cid=%d mtu=%d",
					event->mtu.conn_handle,
					event->mtu.channel_id,
					event->mtu.value);

			break;
		}

		case(BLE_GAP_EVENT_SUBSCRIBE):
		{
			//ESP_LOGI("bt", "EVENT SUBSCRIBE: subscribe event; conn_handle=%d attr_handle=%d "
					//"reason=%d prevn=%d curn=%d previ=%d curi=%d",
					//event->subscribe.conn_handle,
					//event->subscribe.attr_handle,
					//event->subscribe.reason,
					//event->subscribe.prev_notify,
					//event->subscribe.cur_notify,
					//event->subscribe.prev_indicate,
					//event->subscribe.cur_indicate);

			break;
		}

		case(BLE_GAP_EVENT_PHY_UPDATE_COMPLETE):
		{
			ESP_LOGI("bt", "%s", "EVENT PHY UPDATE COMPLETE");

			break;
		}

		case(BLE_GAP_EVENT_NOTIFY_TX):
		{
			/* NOTE: this event doesn't mean the notification is actually sent! */
			/* it's just called synchronously from within ble_gatts_indicate_custom */

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
	char buf[BLE_UUID_STR_LEN];

	switch (context->op)
	{
		case(BLE_GATT_REGISTER_OP_SVC):
		{
			ESP_LOGI("bt", "gatt svc: registered service %s with handle=%d",
					ble_uuid_to_str(context->svc.svc_def->uuid, buf),
					context->svc.handle);
			break;
		}

		case(BLE_GATT_REGISTER_OP_CHR):
		{
			ESP_LOGI("bt", "gatt chr: registering characteristic %s with def_handle=%d val_handle=%d",
					ble_uuid_to_str(context->chr.chr_def->uuid, buf),
					context->chr.def_handle,
					context->chr.val_handle);
			break;
		}

		default:
		{
			ESP_LOGE("bt", "gatt unknown: event: %d", context->op);
			abort();

			break;
		}
	}
}

void bt_send(cli_buffer_t *cli_buffer)
{
	static const unsigned int max_chunk = /* netto data */ 512 + /* sizeof(packet_header_t) */ 32 + /* HCI headers */ 8;
	struct os_mbuf *txom;
	unsigned int offset, chunk, length, attempt;
	int rv;

	offset = 0;
	length = cli_buffer->length;

	ESP_LOGI("bt", "bt_send(%u, %u)", offset, length);

	while(length > 0)
	{
		chunk = length;

		if(chunk > max_chunk)
			chunk = max_chunk;

		ESP_LOGI("bt", "sending chunk from %u length %u from %u", offset, chunk, length);

		for(attempt = 32; attempt > 0; attempt--)
		{
			ESP_LOGI("bt", "bt_send: attempt: %u", attempt);

			txom = ble_hs_mbuf_from_flat(&cli_buffer->data[offset], chunk);
			assert(txom);

			ESP_LOGI("bt", "bt_send: call indicate");

			if(!(rv = ble_gatts_indicate_custom(cli_buffer->bt.connection_handle, cli_buffer->bt.attribute_handle, txom)))
				break;

			ESP_LOGI("bt", "bt_send: wait: %lu", 10 / portTICK_PERIOD_MS);
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}

		if(attempt == 0)
			ESP_LOGW("bt", "bt_send: no more attempts");

		length -= chunk;
		offset += chunk;
	}
}

esp_err_t bt_init(void)
{
	ESP_ERROR_CHECK(nimble_port_init());

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
