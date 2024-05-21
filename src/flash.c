#include "cli-command.h"
#include "flash.h"

#include "util.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <mbedtls/sha1.h>
#include <mbedtls/sha256.h>

#include <esp_flash.h>
#include <esp_ota_ops.h>

void command_flash_bench(cli_command_call_t *call)
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

void command_flash_checksum(cli_command_call_t *call)
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

void command_flash_info(cli_command_call_t *call)
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

		slot[util_partition_to_slot(partition)] = partition->address / 4096;
	}

	esp_partition_iterator_release(partition_iterator);

	snprintf(call->result, call->result_size, "OK esp32 ota available, "
			"slots: %u, "
			"current: %u, "
			"next: %u, "
			"sectors: [ %u, %u ], "
			"display: %ux%upx@%u\n",
			esp_ota_get_app_partition_count(),
			util_partition_to_slot(running),
			util_partition_to_slot(next),
			slot[0], slot[1],
			0, 0, 0); // FIXME
}

void command_flash_read(cli_command_call_t *call)
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

void command_flash_write(cli_command_call_t *call)
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