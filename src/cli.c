#include "cli.h"
#include "util.h"
#include "bt.h"
#include "packet.h"

#include <errno.h>
#include <string.h>
#include <stdbool.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <mbedtls/sha1.h>
#include <mbedtls/sha256.h>

#include <esp_log.h>
#include <esp_check.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_image_format.h>
#include <esp_err.h>

enum
{
	receive_queue_size = 8,
	send_queue_size = 8,
	result_size = 4096,
	result_oob_size = 4096,
};

enum
{
	parameters_size = 16,
};

typedef enum
{
	cli_parameter_none = 0,
	cli_parameter_unsigned_int,
	cli_parameter_signed_int,
	cli_parameter_float,
	cli_parameter_string,
	cli_parameter_size,
} cli_parameter_type_description_t;

typedef struct
{
	unsigned int lower_bound;
	unsigned int upper_bound;
} cli_unsigned_int_description_t;

typedef struct
{
	int lower_bound;
	int upper_bound;
} cli_signed_int_description_t;

typedef struct
{
	float lower_bound;
	float upper_bound;
} cli_float_description_t;

typedef struct
{
	unsigned int lower_length_bound;
	unsigned int upper_length_bound;
} cli_string_description_t;

typedef struct
{
	cli_parameter_type_description_t type:4;

	unsigned int base:5;
	unsigned int value_required:1;
	unsigned int lower_bound_required:1;
	unsigned int upper_bound_required:1;

	union
	{
		cli_unsigned_int_description_t	unsigned_int;
		cli_signed_int_description_t	signed_int;
		cli_float_description_t			fp;
		cli_string_description_t		string;
	};
} cli_parameter_description_t;

typedef struct
{
	unsigned int count;
	cli_parameter_description_t parameters[parameters_size];
} cli_parameters_description_t;

typedef struct
{
	cli_parameter_type_description_t type:4;
	unsigned int has_value:1;

	union
	{
		unsigned int	unsigned_int;
		int				signed_int;
		float			fp;
		const char *	string;
	};
} cli_parameter_t;

typedef struct
{
	unsigned int count;
	cli_parameter_t parameters[parameters_size];
} cli_parameters_t;

typedef struct
{
	const cli_parameters_t *	parameters;
	unsigned int				oob_data_length;
	uint8_t *					oob_data;
	unsigned int				result_size;
	char *						result;
	unsigned int				result_oob_size;
	unsigned int				result_oob_length;
	uint8_t *					result_oob;
} cli_function_call_t;

typedef void(cli_process_function_t)(cli_function_call_t *);

typedef struct
{
	const char *name;
	const char *alias;
	cli_process_function_t *function;
	cli_parameters_description_t parameters;
} cli_function_t;

#if 0
static const char *parameter_type_to_string(unsigned int type)
{
	static const char *type_string[cli_parameter_size] =
	{
		"invalid parameter type",
		"unsigned int",
		"signed int",
		"float",
		"string",
	};

	if(type >= cli_parameter_size)
		type = cli_parameter_none;

	return(type_string[type]);
}
#endif

static void process_flash_bench(cli_function_call_t *call)
{
	unsigned int length;

	assert(call->result_oob_size >= 4096);
	assert(call->parameters->count == 1);

	if((length = call->parameters->parameters[0].unsigned_int) > 4096)
	{
		snprintf(call->result, call->result_size, "ERROR: flash-bench: length %d should be <= 4096", length);
		return;
	}

	memset(call->result_oob, 0, length);
	call->result_oob_length = length;

	snprintf(call->result, call->result_size, "OK flash-bench: sending %u bytes", length);
}

static void process_flash_checksum(cli_function_call_t *call)
{
	int rv;
	unsigned int start_sector, offset, length, current;
	mbedtls_sha1_context ctx;
	uint8_t output[20];

	assert(call->result_oob_size >= 4096);
	assert(call->parameters->count == 2);

	mbedtls_sha1_init(&ctx);
	mbedtls_sha1_starts(&ctx);

	start_sector = call->parameters->parameters[0].unsigned_int;
	length = call->parameters->parameters[1].unsigned_int;

	for(current = start_sector; current < (start_sector + length); current++)
	{
		if((rv = esp_flash_read((esp_flash_t *)0, call->result_oob, current, 4096)) != 0)
		{
			snprintf(call->result, call->result_size, "ERROR: esp_flash_read from %u returned error %d", start_sector, rv);
			return;
		}

		if((rv = mbedtls_sha1_update(&ctx, call->result_oob, 4096)) < 0)
		{
			snprintf(call->result, call->result_size, "ERROR: mbedtls_sha1_update on sector %u returned error %d", start_sector, rv);
			return;
		}
	}

	if((rv = mbedtls_sha1_finish(&ctx, output)) < 0)
	{
		snprintf(call->result, call->result_size, "ERROR: mbedtls_sha1_finish returned error %d", rv);
		return;
	}

	snprintf(call->result, call->result_size, "OK flash-checksum: checksummed %u sectors from sector %u, checksum: ", current - start_sector, start_sector);

	offset = strlen(call->result);

	for(current = 0; current < sizeof(output); current++)
	{
		length = snprintf(call->result + offset, call->result_size - offset, "%02x", output[current]);
		offset += length;
	}
}

static void process_flash_info(cli_function_call_t *call)
{
	esp_partition_iterator_t partition_iterator;
	const esp_partition_t *partition;
	const esp_partition_t *boot;
	const esp_partition_t *running;
	const esp_partition_t *next;
	unsigned int slot[2];

	assert(call->parameters->count == 0);

	slot[0] = slot[1] = 0;

	if(!(boot = esp_ota_get_boot_partition()))
	{
		snprintf(call->result, call->result_size, "ERROR: esp_ota_get_boot_partition failed");
		return;
	}

	if(!(running = esp_ota_get_running_partition()))
	{
		snprintf(call->result, call->result_size, "ERROR: esp_ota_get_running_partition failed");
		return;
	}

	if(!(next = esp_ota_get_next_update_partition((const esp_partition_t *)0)))
	{
		snprintf(call->result, call->result_size, "ERROR: esp_ota_get_next_update_partition failed");
		return;
	}

	if(!(partition_iterator = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, (const char *)0)))
	{
		snprintf(call->result, call->result_size, "ERROR: esp_partition_find failed");
		return;
	}

	for(; partition_iterator; partition_iterator = esp_partition_next(partition_iterator))
	{
		if(!(partition = esp_partition_get(partition_iterator)))
		{
			snprintf(call->result, call->result_size, "ERROR: esp_partition_get failed");
			return;
		}

		if(partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0)
			slot[0] = partition->address / 4096;

		if(partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1)
			slot[1] = partition->address / 4096;
	}

	esp_partition_iterator_release(partition_iterator);

	snprintf(call->result, call->result_size, "OK esp32 ota available, "
			"slots: %u, "
			"current: %u, "
			"next: %u, "
			"sectors: [ %u, %u ], "
			"display: %ux%upx@%u\n",
			esp_ota_get_app_partition_count(),
			running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0 ? 0 : 1,
			next->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0 ? 0 : 1,
			slot[0], slot[1],
			0, 0, 0); // FIXME
}

static void process_flash_read(cli_function_call_t *call)
{
	esp_err_t rv;
	unsigned int sector;

	assert(call->parameters->count == 1);
	assert(call->result_oob_size >= 4096);

	sector = call->parameters->parameters[0].unsigned_int;

	if((rv = esp_flash_read((esp_flash_t *)0, call->result_oob, sector * 4096, 4096)))
	{
		snprintf(call->result, call->result_size, "ERROR: esp_flash_read from %u returned error %u", sector, rv);
		return;
	}

	call->result_oob_length = 4096;
	snprintf(call->result, call->result_size, "OK flash-read: read sector %u", sector);
}

static bool ota_handle_active = false;
static esp_ota_handle_t ota_handle;
static bool ota_sha256_ctx_active = false;
static mbedtls_sha256_context ota_sha256_ctx;
static unsigned int ota_length = 0;

static void ota_abort(void)
{
	esp_err_t rv;

	if(ota_handle_active)
	{
		if((rv = esp_ota_abort(ota_handle)))
			ESP_LOGE("cli", "ota_abort: esp_ota_abort returns error: %x", rv);

		ota_handle_active = false;
	}

	if(ota_sha256_ctx_active)
	{
		mbedtls_sha256_free(&ota_sha256_ctx);
		ota_sha256_ctx_active = false;
	}

	ota_length = 0;
}

static void process_flash_ota_start(cli_function_call_t *call)
{
	esp_err_t rv;
	unsigned int address, length, simulate;
	const esp_partition_t *partition;
	unsigned int slot;

	assert(call->parameters->count == 3);

	address = call->parameters->parameters[0].unsigned_int;
	length = call->parameters->parameters[1].unsigned_int;
	simulate = call->parameters->parameters[2].unsigned_int;

	if(!(partition = esp_ota_get_next_update_partition((const esp_partition_t *)0)))
	{
		snprintf(call->result, call->result_size, "ERROR: no valid OTA partition");
		return;
	}

	if(partition->type != ESP_PARTITION_TYPE_APP)
	{
		snprintf(call->result, call->result_size, "error: partition %s is not APP", partition->label);
		return;
	}

	if(partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0)
		slot = 0;
	else
		if(partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1)
			slot = 1;
		else
		{
			snprintf(call->result, call->result_size, "error: partition %s is not OTA", partition->label);
			return;
		}

	if(address != partition->address)
	{
		snprintf(call->result, call->result_size, "error: start address unexpected: %u vs. %lu", address, partition->address);
		return;
	}

	if(length > partition->size)
	{
		snprintf(call->result, call->result_size, "error: ota partition too small for image: %u vs. %lu", length, partition->size);
		return;
	}

	if(ota_handle_active || ota_sha256_ctx_active)
	{
		ESP_LOGW("cli", "flash-ota-start: ota already active, first aborting session");
		ota_abort();
	}

	if(!simulate)
	{
		if((rv = esp_ota_begin(partition, length, &ota_handle)))
		{
			snprintf(call->result, call->result_size, "ERROR: esp_ota_begin: %s (0x%x)", esp_err_to_name(rv), rv);
			return(ota_abort());
		}

		ota_handle_active = true;
	}

	mbedtls_sha256_init(&ota_sha256_ctx);
	mbedtls_sha256_starts(&ota_sha256_ctx, /* no SHA-224 */ 0);
	ota_sha256_ctx_active = true;

	ota_length = length;

	snprintf(call->result, call->result_size, "OK start flash ota partition %s %u", partition->label, slot);
}

static void process_flash_ota_write(cli_function_call_t *call)
{
	esp_err_t rv;
	unsigned length, checksum_chunk, simulate;

	assert(call->parameters->count == 3);

	length = call->parameters->parameters[0].unsigned_int;
	checksum_chunk = call->parameters->parameters[1].unsigned_int;
	simulate = call->parameters->parameters[2].unsigned_int;

	if(!ota_sha256_ctx_active)
	{
		snprintf(call->result, call->result_size, "ERROR: sha256 context not active");
		return(ota_abort());
	}

	if(!simulate && !ota_handle_active)
	{
		snprintf(call->result, call->result_size, "ERROR: ota write context not active");
		return(ota_abort());
	}

	if(call->oob_data_length != length)
	{
		snprintf(call->result, call->result_size, "ERROR: lengths do not match (%u vs. %u)", length, call->oob_data_length);
		return(ota_abort());
	}

	if(checksum_chunk && (length != 32))
	{
		snprintf(call->result, call->result_size, "ERROR: invalid checksum chunk length (%u vs. %u)", length, 32);
		return(ota_abort());
	}

	if(!simulate && ((rv = esp_ota_write(ota_handle, call->oob_data, call->oob_data_length))))
	{
		snprintf(call->result, call->result_size, "ERROR: esp_ota_write returned error %u", rv);
		return(ota_abort());
	}

	if(!checksum_chunk)
		mbedtls_sha256_update(&ota_sha256_ctx, call->oob_data, call->oob_data_length);

	snprintf(call->result, call->result_size, "OK write flash ota");
	return;
}

static void process_flash_ota_finish(cli_function_call_t *call)
{
	esp_err_t rv;
	unsigned int simulate;
	unsigned char ota_sha256_hash[32];
	char ota_sha256_hash_text[(sizeof(ota_sha256_hash) * 2) + 1];

	assert(call->parameters->count == 1);

	simulate = call->parameters->parameters[0].unsigned_int;

	if(!ota_sha256_ctx_active)
	{
		snprintf(call->result, call->result_size, "ERROR: sha256 context not active");
		return(ota_abort());
	}

	if(!simulate && !ota_handle_active)
	{
		snprintf(call->result, call->result_size, "error: ota write context not active");
		return(ota_abort());
	}

	mbedtls_sha256_finish(&ota_sha256_ctx, ota_sha256_hash);
	mbedtls_sha256_free(&ota_sha256_ctx);
	ota_sha256_ctx_active = false;
	util_hash_to_text(sizeof(ota_sha256_hash), ota_sha256_hash, sizeof(ota_sha256_hash_text), ota_sha256_hash_text);

	if(!simulate && ((rv = esp_ota_end(ota_handle))))
	{
		snprintf(call->result, call->result_size, "error: esp_ota_end failed: %s (0x%x)", esp_err_to_name(rv), rv);
		return(ota_abort());
	}

	ota_handle_active = false;

	snprintf(call->result, call->result_size, "OK finish flash ota, checksum: %s", ota_sha256_hash_text);
}

static void process_flash_ota_commit(cli_function_call_t *call)
{
	esp_err_t rv;
	const char *partition_name;
	unsigned char local_sha256_hash[32];
	char local_sha256_hash_text[(sizeof(local_sha256_hash) * 2) + 1];
	const char *remote_sha256_hash_text;
	unsigned int simulate;
	const esp_partition_t *partition;
	esp_partition_pos_t partition_pos;
	esp_image_metadata_t image_metadata;

	assert(call->parameters->count == 3);

	partition_name =			call->parameters->parameters[0].string;
	remote_sha256_hash_text =	call->parameters->parameters[1].string;
	simulate =					call->parameters->parameters[2].unsigned_int;

	if(!(partition = esp_partition_find_first(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, partition_name)))
	{
		snprintf(call->result, call->result_size, "error: esp_partition_find_first failed");
		return;
	}

	if((rv = esp_partition_get_sha256(partition, local_sha256_hash)))
	{
		snprintf(call->result, call->result_size, "error: esp_partition_get_sha256 failed: %u", rv);
		return;
	}

	util_hash_to_text(sizeof(local_sha256_hash), local_sha256_hash, sizeof(local_sha256_hash_text), local_sha256_hash_text);

	if(!simulate)
	{
		ESP_LOGI("cli", "remote hash[%u]: %s", sizeof(remote_sha256_hash_text), remote_sha256_hash_text);
		ESP_LOGI("cli", "local hash[%u]: %s", sizeof(local_sha256_hash_text), local_sha256_hash_text);
		ESP_LOGI("cli", "result size: %u", call->result_size);

		if(strcmp(local_sha256_hash_text, remote_sha256_hash_text))
		{
			snprintf(call->result, call->result_size, "error: checksum mismatch: %s vs. %s", remote_sha256_hash_text, local_sha256_hash_text);
			return;
		}

		if((rv = esp_ota_set_boot_partition(partition)))
		{
			snprintf(call->result, call->result_size, "error: esp_ota_set_boot_partition failed: %u", rv);
			return;
		}

		if(!(partition = esp_ota_get_boot_partition()))
		{
			snprintf(call->result, call->result_size, "error: esp_ota_get_boot_partition");
			return;
		}

		partition_pos.offset = partition->address;
		partition_pos.size = partition->size;

		if((rv = esp_image_verify(ESP_IMAGE_VERIFY, &partition_pos, &image_metadata)))
		{
			snprintf(call->result, call->result_size, "error: esp_image_verify failed: %u", rv);
			return;
		}
	}

	snprintf(call->result, call->result_size, "OK commit flash ota, address: %lu", partition->address);
}

static void process_flash_ota_confirm(cli_function_call_t *call)
{
	esp_err_t rv;
	unsigned int address;
	unsigned int simulate;
	const esp_partition_t *partition;

	assert(call->parameters->count == 2);

	address = call->parameters->parameters[0].unsigned_int;
	simulate = call->parameters->parameters[1].unsigned_int;

	if(!(partition = esp_ota_get_running_partition()))
	{
		snprintf(call->result, call->result_size, "error: esp_ota_get_running_partition failed");
		return;
	}

	if(!simulate)
	{
		if(partition->address != address)
		{
			snprintf(call->result, call->result_size, "error: address of running slot (%lu) not equal to requested slot (%u), boot failed", partition->address, address);
			return;
		}

		if((rv = esp_ota_mark_app_valid_cancel_rollback()))
		{
			snprintf(call->result, call->result_size, "error: esp_ota_mark_app_valid_cancel_rollback failed: %u", rv);
			return;
		}

		if(!(partition = esp_ota_get_boot_partition()))
		{
			snprintf(call->result, call->result_size, "error: esp_ota_get_boot_partition failed: %u", rv);
			return;
		}

		if(partition->address != address)
		{
			snprintf(call->result, call->result_size, "error: address of next boot slot (%lu) not equal to requested slot (%u), boot failed", partition->address, address);
			return;
		}
	}

	snprintf(call->result, call->result_size, "OK confirm flash ota, next boot slot address: %lu", simulate ? address : partition->address);
}

static void process_flash_write(cli_function_call_t *call)
{
	//esp_err_t rv;
	unsigned int simulate;
	unsigned int sector;
	unsigned int same, erased;

	assert(call->parameters->count == 2);
	assert(call->result_oob_size >= 4096);

	simulate = call->parameters->parameters[0].unsigned_int;
	sector = call->parameters->parameters[1].unsigned_int;

	same = erased = 0; // FIXME

	//if((rv = esp_flash_read((esp_flash_t *)0, call->result_oob, sector, 4096)) != 0)
	//{
		//snprintf(call->result, call->result_size, "ERROR: esp_flash_read from %u returned error %u", sector, rv);
		//return;
	//}

	call->result_oob_length = 0;
	snprintf(call->result, call->result_size, "OK flash-write: written mode %u, sector %u, same %u, erased %u",
			simulate, sector, same, erased);
}

static void process_reset(cli_function_call_t *call)
{
	assert(call->parameters->count == 0);

	esp_restart();
}

static void process_stat_firmware(cli_function_call_t *call)
{
	const esp_app_desc_t *desc;

	if(!(desc = esp_app_get_description()))
	{
		snprintf(call->result, call->result_size, "ERROR: esp_app_get_description failed");
		return;
	}

	snprintf(call->result, call->result_size,
			"> firmware\n"
			">   date: %s %s\n"
			">   build start: %s %s\n",
			__DATE__, __TIME__, desc->date, desc->time);
}

static void process_stat_flash(cli_function_call_t *call)
{
	esp_err_t rv;
	esp_partition_iterator_t partition_iterator;
	const esp_partition_t *partition;
	const char *type, *subtype;
	unsigned int index, offset;
	const esp_partition_t *boot_partition, *running_partition;
	esp_ota_img_states_t ota_state;
	const char *ota_state_text;
	unsigned char sha256_hash[32];
	char sha256_hash_text[(sizeof(sha256_hash) * 2) + 1];

	assert(call->parameters->count == 0);

	if(!(boot_partition = esp_ota_get_boot_partition()))
	{
		snprintf(call->result, call->result_size, "ERROR: esp_ota_get_boot_partition failed");
		return;
	}

	if(!(running_partition = esp_ota_get_running_partition()))
	{
		snprintf(call->result, call->result_size, "ERROR: esp_ota_get_running_partition failed");
		return;
	}

	if(!(partition_iterator = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, (const char *)0)))
	{
		snprintf(call->result, call->result_size, "ERROR: esp_partition_find failed");
		return;
	}

	offset = snprintf(call->result, call->result_size, "Partitions:\n");

	for(index = 0; partition_iterator; index++, partition_iterator = esp_partition_next(partition_iterator))
	{
		if(!(partition = esp_partition_get(partition_iterator)))
		{
			snprintf(call->result, call->result_size, "ERROR: esp_partition_get failed");
			return;
		}

		switch(partition->type)
		{
			case(ESP_PARTITION_TYPE_APP):
			{
				type = "app";
				break;
			}

			case(ESP_PARTITION_TYPE_DATA):
			{
				type = "data";
				break;
			}

			default:
			{
				type = "unknown";
				break;
			}
		}

		switch(partition->subtype)
		{
			case(ESP_PARTITION_SUBTYPE_APP_FACTORY):
			//case(ESP_PARTITION_SUBTYPE_DATA_OTA): // these are using the same value
			{
				if(partition->type == ESP_PARTITION_TYPE_APP)
					subtype = "factory";
				else
					subtype = "ota data";

				break;
			}

			case(ESP_PARTITION_SUBTYPE_APP_OTA_0):
			{
				subtype = "ota 0";
				break;
			}

			case(ESP_PARTITION_SUBTYPE_APP_OTA_1):
			{
				subtype = "ota 1";
				break;
			}

			case(ESP_PARTITION_SUBTYPE_DATA_NVS):
			{
				subtype = "nvs";
				break;
			}

			default:
			{
				subtype = "unknown";
				break;
			}
		}

		ota_state_text = "";

		if((partition->type == ESP_PARTITION_TYPE_APP) &&
			((partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) || (partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1)))
		{
			if(!esp_ota_get_state_partition(partition, &ota_state))
			{
				switch(ota_state)
				{
					case(ESP_OTA_IMG_NEW):				{ ota_state_text = "new"; break; }
					case(ESP_OTA_IMG_PENDING_VERIFY):	{ ota_state_text = "pending verify"; break; }
					case(ESP_OTA_IMG_VALID):			{ ota_state_text = "valid"; break; }
					case(ESP_OTA_IMG_INVALID):			{ ota_state_text = "invalid"; break; }
					case(ESP_OTA_IMG_ABORTED):			{ ota_state_text = "aborted"; break; }
					case(ESP_OTA_IMG_UNDEFINED):		{ ota_state_text = "undefined"; break; }
					default:							{ ota_state_text = "unknown"; break; }
				}
			}
			else
				ota_state_text = "not found";
		}

		if((rv = esp_partition_get_sha256(partition, sha256_hash)))
		{
			snprintf(call->result, call->result_size, "error: esp_partition_get_sha256 failed: %u", rv);
			return;
		}

		util_hash_to_text(sizeof(sha256_hash), sha256_hash, sizeof(sha256_hash_text), sha256_hash_text);

		offset += snprintf(call->result + offset, call->result_size - offset, "%s  %2u %1s %1s %-8s %06lx %4lu %-7s %-8s %-64s %s",
				(index > 0) ? "\n" : "",
				index,
				(partition->address == boot_partition->address) ? "b" : " ",
				(partition->address == running_partition->address) ? "r" : " ",
				partition->label, partition->address, partition->size / 1024, type, subtype, sha256_hash_text, ota_state_text);
	}

	esp_partition_iterator_release(partition_iterator);
}

static void process_stat_memory(cli_function_call_t *call)
{
	unsigned offset;

	offset = 0;

	offset += snprintf(call->result + offset, call->result_size - offset, "%-30s %-4s\n", "Type of memory", "kB");
	offset += snprintf(call->result + offset, call->result_size - offset, "  %-28s %lu\n", "free heap total",			esp_get_free_heap_size() / 1024);
	offset += snprintf(call->result + offset, call->result_size - offset, "  %-28s %lu\n", "minimum free heap",			esp_get_minimum_free_heap_size() / 1024);
	offset += snprintf(call->result + offset, call->result_size - offset, "  %-28s %u\n",  "heap executable",			heap_caps_get_free_size(MALLOC_CAP_EXEC) / 1024);
	offset += snprintf(call->result + offset, call->result_size - offset, "  %-28s %u\n",  "heap 32 bit addressable",	heap_caps_get_free_size(MALLOC_CAP_32BIT) / 1024);
	offset += snprintf(call->result + offset, call->result_size - offset, "  %-28s %u\n",  "heap 8 bit addressable",		heap_caps_get_free_size(MALLOC_CAP_8BIT) / 1024);
	offset += snprintf(call->result + offset, call->result_size - offset, "  %-28s %u\n",  "heap DMA adressable",		heap_caps_get_free_size(MALLOC_CAP_DMA) / 1024);
	offset += snprintf(call->result + offset, call->result_size - offset, "  %-28s %u\n",  "heap SPI RAM",				heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024);
	offset += snprintf(call->result + offset, call->result_size - offset, "  %-28s %u\n",  "heap internal RAM",			heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024);
	offset += snprintf(call->result + offset, call->result_size - offset, "  %-28s %u\n",  "heap default",				heap_caps_get_free_size(MALLOC_CAP_DEFAULT) / 1024);
	offset += snprintf(call->result + offset, call->result_size - offset, "  %-28s %u\n",  "heap IRAM 8 bit adressable",	heap_caps_get_free_size(MALLOC_CAP_IRAM_8BIT) / 1024);
	offset += snprintf(call->result + offset, call->result_size - offset, "  %-28s %u\n",  "heap retention",				heap_caps_get_free_size(MALLOC_CAP_RETENTION) / 1024);
	offset += snprintf(call->result + offset, call->result_size - offset, "  %-28s %u\n",  "heap RTC RAM",				heap_caps_get_free_size(MALLOC_CAP_RTCRAM) / 1024);
	offset += snprintf(call->result + offset, call->result_size - offset, "  %-28s %u\n",  "heap TCM",					heap_caps_get_free_size(MALLOC_CAP_TCM) / 1024);
}

static void process_stat_process(cli_function_call_t *call)
{
	unsigned int ix, length, offset, processes;
	unsigned long runtime;
	TaskStatus_t *process_info;
	const TaskStatus_t *pip;
	const char *name;
	const char *state;

	offset = 0;

	processes = uxTaskGetNumberOfTasks();
	process_info = heap_caps_malloc(sizeof(*process_info) * processes , MALLOC_CAP_SPIRAM);
	assert(uxTaskGetSystemState(process_info, processes, &runtime) == processes);

	length = snprintf(call->result + offset, call->result_size - offset, "processes: %u\n", processes);
	offset += length;
	length = snprintf(call->result + offset, call->result_size - offset, "  %2s  %-12s %-10s %-4s %-5s\n", "#", "name", "state", "prio", "stack");
	offset += length;

	for(ix = 0; ix < processes; ix++)
	{
		pip = &process_info[ix];
		name = pip->pcTaskName ? pip->pcTaskName : "(null)";

		switch(pip->eCurrentState)
		{
			case(eRunning):
			{
				state = "running";
				break;
			}

			case(eReady):
			{
				state = "ready";
				break;
			}

			case(eBlocked):
			{
				state = "blocked";
				break;
			}

			case(eSuspended):
			{
				state = "suspended";
				break;
			}

			case(eDeleted):
			{
				state = "deleted";
				break;
			}

			default:
			{
				state = "invalid";
				break;
			}
		}

		length = snprintf(call->result + offset, call->result_size - offset, "  %2u: %-12s %-10s %4u %5u\n",
				pip->xTaskNumber,
				name,
				state,
				pip->uxCurrentPriority,
				(unsigned int)pip->usStackHighWaterMark);
		offset += length;
	}

	free(process_info);
}

static void process_stat_system(cli_function_call_t *call)
{
	esp_chip_info_t chip_info;
	uint32_t flash_size;
	unsigned int offset;

	offset = 0;

	esp_chip_info(&chip_info);

	offset += snprintf(call->result + offset, call->result_size - offset, "SoC: %s with %d cores\nRF: %s%s%s%s\n",
		CONFIG_IDF_TARGET,
		chip_info.cores,
		(chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
		(chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
		(chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
		(chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

	unsigned major_rev = chip_info.revision / 100;
	unsigned minor_rev = chip_info.revision % 100;
	offset += snprintf(call->result + offset, call->result_size - offset, "Revision: %d.%d\n", major_rev, minor_rev);

	esp_flash_get_size(NULL, &flash_size);

	offset += snprintf(call->result + offset, call->result_size - offset, "Flash: %lu MB %s\n", flash_size / (1024 * 1024),
		(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
}

static void process_test(cli_function_call_t *call)
{
	unsigned int ix, at;
	const cli_parameter_t *parameter;

	at = 0;

	for(ix = 0; ix < call->parameters->count; ix++)
	{
		parameter = &call->parameters->parameters[ix];

		if(!parameter->has_value)
		{
			if(at < call->result_size)
				at += snprintf(call->result + at, call->result_size - at, "ERROR: parameter %u has no value\n", ix);

			continue;
		}

		switch(parameter->type)
		{
			case(cli_parameter_none):
			case(cli_parameter_size):
			{
				if(at < call->result_size)
					at += snprintf(call->result + at, call->result_size - at, "ERROR: invalid parameter %u\n", ix);

				break;
			}

			case(cli_parameter_unsigned_int):
			{
				if(at < call->result_size)
					at += snprintf(call->result + at, call->result_size - at, "unsigned int parameter: %u\n", parameter->unsigned_int);

				break;
			}

			case(cli_parameter_signed_int):
			{
				if(at < call->result_size)
					at += snprintf(call->result + at, call->result_size - at, "signed int parameter: %d\n", parameter->signed_int);

				break;
			}

			case(cli_parameter_float):
			{
				if(at < call->result_size)
					at += snprintf(call->result + at, call->result_size - at, "float parameter: %f\n", parameter->fp);

				break;
			}

			case(cli_parameter_string):
			{
				if(at < call->result_size)
					at += snprintf(call->result + at, call->result_size - at, "string parameter: \"%s\"\n", parameter->string);

				break;
			}
		}
	}
}

static const cli_function_t cli_functions[] =
{
	{ "flash-checksum",	(const char*)0,	process_flash_checksum,		{	2,
																		{
																			{ cli_parameter_unsigned_int,	0,	1,	0,	0, {} },
																			{ cli_parameter_unsigned_int,	0,	1,	0,	0, {} },
																		},
																	}},
	{ "flash-bench",	(const char*)0,	process_flash_bench,		{	1,
																		{
																			{ cli_parameter_unsigned_int,	0,	1,	1,	1, .unsigned_int = { 0, 4096 }},
																		},
																	}},
	{ "flash-info",		(const char*)0,	process_flash_info,			{}	},
	{ "flash-read",		(const char*)0,	process_flash_read,			{	1,
																		{
																			{ cli_parameter_unsigned_int,	0,	1,	0,	0, {} },
																		},
																	}},
	{ "flash-ota-start",(const char*)0,	process_flash_ota_start,	{	3,
																		{
																			{ cli_parameter_unsigned_int,	0,	1,	0,	0, {} },
																			{ cli_parameter_unsigned_int,	0,	1,	0,	0, {} },
																			{ cli_parameter_unsigned_int,	0,	1,	1,	1, .unsigned_int = { 0, 1 }},
																		},
																	}},
	{ "flash-ota-write",(const char*)0,	process_flash_ota_write,	{	3,
																		{
																			{ cli_parameter_unsigned_int,	0,	1,	0,	0, {} },
																			{ cli_parameter_unsigned_int,	0,	1,	1,	1, .unsigned_int = { 0, 1 }},
																			{ cli_parameter_unsigned_int,	0,	1,	1,	1, .unsigned_int = { 0, 1 }},
																		},
																	}},
	{ "flash-ota-finish",(const char*)0, process_flash_ota_finish,	{	1,
																		{
																			{ cli_parameter_unsigned_int,	0,	1,	1,	1, .unsigned_int = { 0, 1 }},
																		},
																	}},
	{ "flash-ota-commit",(const char*)0, process_flash_ota_commit,	{	3,
																		{
																			{ cli_parameter_string,			0,	1,	1,	1, .string = { 0, 16 }},
																			{ cli_parameter_string,			0,	1,	1,	1, .string = { 64, 64 }},
																			{ cli_parameter_unsigned_int,	0,	1,	1,	1, .unsigned_int = { 0, 1}},
																		},
																	}},
	{ "flash-ota-confirm", (const char*)0, process_flash_ota_confirm, {	2,
																		{
																			{ cli_parameter_unsigned_int,	0,	1,	0,	0, {} },
																			{ cli_parameter_unsigned_int,	0,	1,	1,	1, .unsigned_int = { 0, 1 }},
																		},
																	}},
	{ "flash-write",	(const char*)0,	process_flash_write,		{	2,
																		{
																			{ cli_parameter_unsigned_int,	0,	1,	1,	1, .unsigned_int = { 0, 1 }},
																			{ cli_parameter_unsigned_int,	0,	1,	0,	0, {} },
																	},
																	}},
	{ "reset",			"r",			process_reset,				{}	},
	{ "stats",			"s",			process_stat_firmware,		{}	},
	{ "stat-flash",		"sf",			process_stat_flash,			{}	},
	{ "stat-memory",	"sm",			process_stat_memory,		{}	},
	{ "stat-process",	"sp",			process_stat_process,		{}	},
	{ "stat-system",	"ss",			process_stat_system,		{}	},
	{ "test",			"t",			process_test,				{	4,
																		{
																			{ cli_parameter_unsigned_int,	0,	1,	1,	1,	.unsigned_int =	{ 1, 10 }},
																			{ cli_parameter_signed_int,		0,	1,	1,	1,	.signed_int =	{ 1, 10 }},
																			{ cli_parameter_float,			0,	1,	1,	1,	.fp =			{ 1, 10 }},
																			{ cli_parameter_string,			0,	1,	1,	1,	.string =		{ 1, 10 }},
																		}
																	}},
	{ (char *)0,	(char *)0,	(cli_process_function_t *)0,		{}	},
};

static QueueHandle_t receive_queue_handle;
static QueueHandle_t send_queue_handle;
static bool inited = false;

static void receive_queue_pop(cli_buffer_t *cli_buffer)
{
	assert(inited);

	xQueueReceive(receive_queue_handle, cli_buffer, portMAX_DELAY);
}

static void send_queue_push(cli_buffer_t *cli_buffer)
{
	assert(inited);

	xQueueSend(send_queue_handle, cli_buffer, portMAX_DELAY);
}

static void send_queue_pop(cli_buffer_t *cli_buffer)
{
	assert(inited);

	xQueueReceive(send_queue_handle, cli_buffer, portMAX_DELAY);
}

static void run_receive_queue(void *)
{
	cli_buffer_t						cli_buffer;
	cli_parameters_t					parameters;
	unsigned int						oob_data_length;
	char								*data;
	uint8_t								*oob_data;
	unsigned int						count, current, ix;
	char								*token, *saveptr;
	const cli_function_t				*cli_function;
	const cli_parameter_description_t	*parameter_description;
	cli_parameter_t						*parameter;
	static cli_function_call_t			call;
	static char							error[128];

	assert(inited);

	for(;;)
	{
		receive_queue_pop(&cli_buffer);
		packet_decapsulate(&cli_buffer, &data, &oob_data_length, &oob_data);

		if(cli_buffer.data_from_malloc && cli_buffer.data)
			free(cli_buffer.data);
		cli_buffer.length = 0;
		cli_buffer.data = (uint8_t *)0;
		cli_buffer.data_from_malloc = 0;

		saveptr = (char *)0;
		if(!(token = strtok_r((char *)data, " \r\n", &saveptr)))
		{
			snprintf(error, sizeof(error), "ERROR: empty line");
			packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		for(ix = 0;; ix++)
		{
			cli_function = &cli_functions[ix];

			if(!cli_function->name || !strcmp(cli_function->name, token))
				break;

			if(cli_function->alias && !strcmp(cli_function->alias, token))
				break;
		}

		if(!cli_function->name)
		{
			snprintf(error, sizeof(error), "ERROR: unknown command \"%s\"", token);
			packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		count = cli_function->parameters.count;

		if(count > parameters_size)
			count = parameters_size;

		for(current = 0; current < count; current++)
		{
			parameter_description = &cli_function->parameters.parameters[current];
			parameter = &parameters.parameters[current];

			parameter->type = cli_parameter_none;
			parameter->has_value = 0;

			token = strtok_r((char *)0, " \r\n", &saveptr);

			if(!token)
			{
				if(!parameter_description->value_required)
				{
					continue;
				}
				else
				{
					snprintf(error, sizeof(error), "ERROR: missing required parameter %u", current + 1);
					packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
					send_queue_push(&cli_buffer);
					goto error;
				}
			}
			else
			{
				switch(parameter_description->type)
				{
					case(cli_parameter_none):
					case(cli_parameter_size):
					{
						snprintf(error, sizeof(error), "ERROR: parameter with invalid type %u", parameter_description->type);
						packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
						send_queue_push(&cli_buffer);
						goto error;
					}

					case(cli_parameter_unsigned_int):
					{
						char *endptr;
						unsigned int value;

						errno = 0;
						value = strtoul(token, &endptr, parameter_description->base);

						if(errno || !*token || *endptr)
						{
							snprintf(error, sizeof(error), "ERROR: invalid unsigned integer value: %s", token);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->lower_bound_required) && (value < parameter_description->unsigned_int.lower_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid unsigned integer value: %u, smaller than lower bound: %u", value, parameter_description->unsigned_int.lower_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->upper_bound_required) && (value > parameter_description->unsigned_int.upper_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid unsigned integer value: %u, larger than upper bound: %u", value, parameter_description->unsigned_int.upper_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						parameter->type = cli_parameter_unsigned_int;
						parameter->has_value = 1;
						parameter->unsigned_int = value;

						break;
					}

					case(cli_parameter_signed_int):
					{
						char *endptr;
						int value;

						errno = 0;
						value = strtol(token, &endptr, parameter_description->base);

						if(errno || !*token || *endptr)
						{
							snprintf(error, sizeof(error), "ERROR: invalid signed integer value: %s", token);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->lower_bound_required) && (value < parameter_description->signed_int.lower_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid signed integer value: %d, smaller than lower bound: %d", value, parameter_description->signed_int.lower_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->upper_bound_required) && (value > parameter_description->signed_int.upper_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid signed integer value: %d, larger than upper bound: %d", value, parameter_description->signed_int.upper_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						parameter->type = cli_parameter_signed_int;
						parameter->has_value = 1;
						parameter->signed_int = value;

						break;
					}

					case(cli_parameter_float):
					{
						char *endptr;
						float value;

						errno = 0;
						value = strtof(token, &endptr);

						if(errno || !*token || *endptr)
						{
							snprintf(error, sizeof(error), "ERROR: invalid float value: %s", token);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->lower_bound_required) && (value < parameter_description->fp.lower_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid float value: %f, smaller than lower bound: %f", value, parameter_description->fp.lower_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->upper_bound_required) && (value > parameter_description->fp.upper_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid float value: %f, larger than upper bound: %f", value, parameter_description->fp.upper_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						parameter->type = cli_parameter_float;
						parameter->has_value = 1;
						parameter->fp = value;

						break;
					}

					case(cli_parameter_string):
					{
						unsigned int length;

						length = strlen(token);

						if((parameter_description->lower_bound_required) && (length < parameter_description->string.lower_length_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid string length: %u, smaller than lower bound: %u", length, parameter_description->string.lower_length_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->upper_bound_required) && (length > parameter_description->string.upper_length_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid string length: %u, larger than upper bound: %u", length, parameter_description->string.upper_length_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						parameter->type = cli_parameter_string;
						parameter->has_value = 1;
						parameter->string = token;

						break;
					}
				}
			}
		}

		if(current >= parameters_size)
		{
			snprintf(error, sizeof(error), "ERROR: too many parameters: %u", current);
			packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		if(current < cli_function->parameters.count)
		{
			snprintf(error, sizeof(error), "ERROR: missing paramters");
			packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		if(strtok_r((char *)0, " \r\n", &saveptr))
		{
			snprintf(error, sizeof(error), "ERROR: too many parameters");
			packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		parameters.count = current;

		call.parameters =			&parameters;
		call.oob_data_length =		oob_data_length;
		call.oob_data =				oob_data;
		call.result_size =			result_size;
		call.result =				heap_caps_malloc(call.result_size, MALLOC_CAP_SPIRAM);
		call.result_oob_size =		result_oob_size;
		call.result_oob_length =	0;
		call.result_oob =			heap_caps_malloc(call.result_oob_size, MALLOC_CAP_SPIRAM);

		cli_function->function(&call);

		packet_encapsulate(&cli_buffer, call.result, call.result_oob_length, call.result_oob);
		send_queue_push(&cli_buffer);

		if(call.result)
		{
			free(call.result);
			call.result = (char *)0;
		}

		if(call.result_oob)
		{
			free(call.result_oob);
			call.result_oob = (uint8_t *)0;
		}

error:
		cli_buffer.source = cli_source_none;
		cli_buffer.length = 0;
		cli_buffer.data_from_malloc = 0;
		cli_buffer.data = (uint8_t *)0;

		if(data)
		{
			free(data);
			data = (char *)0;
		}

		if(oob_data)
		{
			free(oob_data);
			oob_data = (uint8_t *)0;
		}
	}
}

static void run_send_queue(void *)
{
	cli_buffer_t cli_buffer;

	assert(inited);

	for(;;)
	{
		send_queue_pop(&cli_buffer);

		switch(cli_buffer.source)
		{
			case(cli_source_none):
			{
				ESP_LOGE("cli", "invalid source type: %u", cli_buffer.source);
				break;
			}

			case(cli_source_bt):
			{
				bt_send(&cli_buffer);

				break;
			}

			case(cli_source_station):
			{
				break;
			}

			case(cli_source_ap):
			{
				break;
			}

			case(cli_source_console):
			{
				break;
			}
		}

		if(cli_buffer.data_from_malloc && cli_buffer.data)
			free(cli_buffer.data);
		cli_buffer.source = cli_source_none;
		cli_buffer.length = 0;
		cli_buffer.data_from_malloc = 0;
		cli_buffer.data = (uint8_t *)0;
	}
}

void cli_receive_queue_push(const cli_buffer_t *buffer)
{
	assert(inited);

	xQueueSend(receive_queue_handle, buffer, portMAX_DELAY);
}

void cli_init(void)
{
	assert(!inited);

	cli_buffer_t *queue_data;
	StaticQueue_t *queue;

	queue_data = heap_caps_malloc(receive_queue_size * sizeof(cli_buffer_t) , MALLOC_CAP_SPIRAM);
	assert(queue_data);

	queue = heap_caps_malloc(sizeof(StaticQueue_t), MALLOC_CAP_SPIRAM);
	assert(queue);

	if(!(receive_queue_handle = xQueueCreateStatic(receive_queue_size, sizeof(cli_buffer_t), (void *)queue_data, queue)))
	{
		ESP_LOGE("cli", "xQueueCreateStatic receive queue init");
		abort();
	}

	queue_data = heap_caps_malloc(send_queue_size * sizeof(cli_buffer_t) , MALLOC_CAP_SPIRAM);
	assert(queue_data);

	queue = heap_caps_malloc(sizeof(StaticQueue_t), MALLOC_CAP_SPIRAM);
	assert(queue);

	if(!(send_queue_handle = xQueueCreateStatic(send_queue_size, sizeof(cli_buffer_t), (void *)queue_data, queue)))
	{
		ESP_LOGE("cli", "xQueueCreateStatic send queue init");
		abort();
	}

	inited = true;

	if(xTaskCreatePinnedToCore(run_receive_queue, "cli-recv", 4096, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
	{
		ESP_LOGE("cli", "xTaskCreatePinnedToNode run_receive_queue");
		abort();
	}

	if(xTaskCreatePinnedToCore(run_send_queue, "cli-send", 4096, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
	{
		ESP_LOGE("cli", "xTaskCreatePinnedToNode run_send_queue");
		abort();
	}
}
