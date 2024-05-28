#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "string.h"
#include "cli-command.h"
#include "flash.h"
#include "log.h"
#include "util.h"

#include <mbedtls/sha1.h>
#include <mbedtls/sha256.h>

#include <esp_flash.h>
#include <esp_ota_ops.h>

static uint8_t flash_buffer[4096];

void command_flash_bench(cli_command_call_t *call)
{
	unsigned int length;

	assert(string_size(call->result_oob) > 4096);
	assert(call->parameter_count == 1);

	if((length = call->parameters[0].unsigned_int) > 4096)
	{
		string_format(call->result, "ERROR: flash-bench: length %d should be <= 4096", length);
		return;
	}

	string_fill(call->result_oob, length, '\0');
	string_format(call->result, "OK flash-bench: sending %u bytes", length);
}

void command_flash_checksum(cli_command_call_t *call)
{
	int rv;
	unsigned int start_sector, length, current;
	mbedtls_sha1_context ctx;
	uint8_t output[20];

	assert(string_size(call->result_oob) > 4096);
	assert(call->parameter_count == 2);

	mbedtls_sha1_init(&ctx);
	mbedtls_sha1_starts(&ctx);

	start_sector = call->parameters[0].unsigned_int;
	length = call->parameters[1].unsigned_int;

	for(current = start_sector; current < (start_sector + length); current++)
	{
		if((rv = esp_flash_read((esp_flash_t *)0, flash_buffer, current, sizeof(flash_buffer))) != 0)
		{
			string_format(call->result, "ERROR: esp_flash_read from %u returned error %d", start_sector, rv);
			return;
		}

		if((rv = mbedtls_sha1_update(&ctx, flash_buffer, sizeof(flash_buffer))) < 0)
		{
			string_format(call->result, "ERROR: mbedtls_sha1_update on sector %u returned error %d", start_sector, rv);
			return;
		}
	}

	if((rv = mbedtls_sha1_finish(&ctx, output)) < 0)
	{
		string_format(call->result, "ERROR: mbedtls_sha1_finish returned error %d", rv);
		return;
	}

	string_format(call->result, "OK flash-checksum: checksummed %u sectors from sector %u, checksum: ", current - start_sector, start_sector);
	string_format_append(call->result, "%02x", output[current]);
}

void command_flash_info(cli_command_call_t *call)
{
	esp_partition_iterator_t partition_iterator;
	const esp_partition_t *partition;
	const esp_partition_t *boot;
	const esp_partition_t *running;
	const esp_partition_t *next;
	unsigned int slot[2];

	assert(call->parameter_count == 0);

	slot[0] = slot[1] = 0;

	if(!(boot = esp_ota_get_boot_partition()))
	{
		string_format(call->result, "ERROR: esp_ota_get_boot_partition failed");
		return;
	}

	if(!(running = esp_ota_get_running_partition()))
	{
		string_format(call->result, "ERROR: esp_ota_get_running_partition failed");
		return;
	}

	if(!(next = esp_ota_get_next_update_partition((const esp_partition_t *)0)))
	{
		string_format(call->result, "ERROR: esp_ota_get_next_update_partition failed");
		return;
	}

	if(!(partition_iterator = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, (const char *)0)))
	{
		string_format(call->result, "ERROR: esp_partition_find failed");
		return;
	}

	for(; partition_iterator; partition_iterator = esp_partition_next(partition_iterator))
	{
		if(!(partition = esp_partition_get(partition_iterator)))
		{
			string_format(call->result, "ERROR: esp_partition_get failed");
			return;
		}

		slot[util_partition_to_slot(partition)] = partition->address / 4096;
	}

	esp_partition_iterator_release(partition_iterator);

	string_format(call->result, "OK esp32 ota available, slots: %u, current: %u, next: %u, sectors: [ %u, %u ], display: %ux%upx@%u\n",
			esp_ota_get_app_partition_count(),
			util_partition_to_slot(running),
			util_partition_to_slot(next),
			slot[0], slot[1],
			0, 0, 0); // FIXME
}

void command_flash_read(cli_command_call_t *call)
{
	int rv;
	unsigned int sector;

	assert(call->parameter_count == 1);
	assert(string_size(call->result_oob) > 4096);

	sector = call->parameters[0].unsigned_int;

	if((rv = esp_flash_read((esp_flash_t *)0, flash_buffer, sector * sizeof(flash_buffer), sizeof(flash_buffer))))
	{
		string_format(call->result, "ERROR: esp_flash_read from %u returned error 0x%x", sector, rv);
		return;
	}

	string_assign_data(call->result_oob, sizeof(flash_buffer), flash_buffer);

	string_format(call->result, "OK flash-read: read sector %u", sector);
}

void command_flash_write(cli_command_call_t *call)
{
	unsigned int simulate;
	unsigned int sector;
	unsigned int same, erased;

	assert(call->parameter_count == 2);
	assert(string_size(call->result_oob) > 4096);

	simulate = call->parameters[0].unsigned_int;
	sector = call->parameters[1].unsigned_int;

	same = erased = 0; // FIXME

	//if((rv = esp_flash_read((esp_flash_t *)0, call->result_oob, sector, 4096)) != 0)
	//{
		//string_format(call->result, "ERROR: esp_flash_read from %u returned error %u", sector, rv);
		//return;
	//}

	string_format(call->result, "OK flash-write: written mode %u, sector %u, same %u, erased %u",
			simulate, sector, same, erased);
}
