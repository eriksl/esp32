#include <stdint.h>
#include <stdbool.h>

#include "string.h"
#include "cli-command.h"
#include "otacli.h"
#include "log.h"
#include "util.h"

#include <mbedtls/sha256.h>

#include <esp_ota_ops.h>
#include <esp_image_format.h>

static bool ota_handle_active = false;
static const esp_partition_t *ota_partition = (const esp_partition_t *)0;
static esp_ota_handle_t ota_handle;

static bool ota_sha256_ctx_active = false;
static mbedtls_sha256_context ota_sha256_ctx;
static unsigned int ota_length = 0;

static void ota_abort(void)
{
	if(ota_handle_active)
	{
		util_warn_on_esp_err("otacli: ota_abort: esp_ota_abort returns error", esp_ota_abort(ota_handle));

		ota_partition = (const esp_partition_t *)0;
		ota_handle_active = false;
	}

	if(ota_sha256_ctx_active)
	{
		mbedtls_sha256_free(&ota_sha256_ctx);
		ota_sha256_ctx_active = false;
	}

	ota_length = 0;
}

void command_ota_start(cli_command_call_t *call)
{
	int rv;
	unsigned int length;
	const esp_partition_t *partition;

	assert(call->parameter_count == 1);

	length = call->parameters[0].unsigned_int;

	if(!(partition = esp_ota_get_next_update_partition((const esp_partition_t *)0)))
	{
		string_format(call->result, "ERROR: no valid OTA partition");
		return;
	}

	if(partition->type != ESP_PARTITION_TYPE_APP)
	{
		string_format(call->result, "ERROR: partition %s is not APP", partition->label);
		return;
	}

	if(length > partition->size)
	{
		string_format(call->result, "ERROR: ota partition too small for image: %u vs. %lu", length, partition->size);
		return;
	}

	if(ota_handle_active || ota_sha256_ctx_active)
	{
		log("otacli: ota-start: ota already active, first aborting session");
		ota_abort();
	}

	if((rv = esp_ota_begin(partition, length, &ota_handle)))
	{
		string_format(call->result, "ERROR: esp_ota_begin: %s (0x%x)", esp_err_to_name(rv), rv);
		return(ota_abort());
	}

	ota_partition = partition;
	ota_handle_active = true;

	mbedtls_sha256_init(&ota_sha256_ctx);
	mbedtls_sha256_starts(&ota_sha256_ctx, /* no SHA-224 */ 0);
	ota_sha256_ctx_active = true;

	ota_length = length;

	string_format(call->result, "OK start write ota partition %s %u", partition->label, util_partition_to_slot(partition));
}

void command_ota_write(cli_command_call_t *call)
{
	int rv;
	unsigned length, checksum_chunk;

	assert(call->parameter_count == 2);

	length = call->parameters[0].unsigned_int;
	checksum_chunk = call->parameters[1].unsigned_int;

	if(!ota_sha256_ctx_active)
	{
		string_format(call->result, "ERROR: sha256 context not active");
		return(ota_abort());
	}

	if(!ota_handle_active)
	{
		string_format(call->result, "ERROR: ota write context not active");
		return(ota_abort());
	}

	if(string_length(call->oob) != length)
	{
		string_format(call->result, "ERROR: lengths do not match (%u vs. %u)", length, string_length(call->oob));
		return(ota_abort());
	}

	if(checksum_chunk && (length != 32))
	{
		string_format(call->result, "ERROR: invalid checksum chunk length (%u vs. %u)", length, 32);
		return(ota_abort());
	}

	if((rv = esp_ota_write(ota_handle, string_data(call->oob), string_length(call->oob))))
	{
		string_format(call->result, "ERROR: esp_ota_write returned error %u", rv);
		return(ota_abort());
	}

	if(!checksum_chunk)
		mbedtls_sha256_update(&ota_sha256_ctx, string_data(call->oob), string_length(call->oob));

	string_format(call->result, "OK write ota");
}

void command_ota_finish(cli_command_call_t *call)
{
	int rv;
	unsigned char ota_sha256_hash[32];
	string_auto(ota_sha256_hash_text, (sizeof(ota_sha256_hash) * 2) + 1);

	assert(call->parameter_count == 0);

	if(!ota_sha256_ctx_active)
	{
		string_format(call->result, "ERROR: sha256 context not active");
		return(ota_abort());
	}

	if(!ota_handle_active)
	{
		string_format(call->result, "ERROR: ota write context not active");
		return(ota_abort());
	}

	mbedtls_sha256_finish(&ota_sha256_ctx, ota_sha256_hash);
	mbedtls_sha256_free(&ota_sha256_ctx);
	ota_sha256_ctx_active = false;
	util_hash_to_string(ota_sha256_hash_text, sizeof(ota_sha256_hash), ota_sha256_hash);

	if((rv = esp_ota_end(ota_handle)))
	{
		string_format(call->result, "ERROR: esp_ota_end failed: %s (0x%x)", esp_err_to_name(rv), rv);
		return(ota_abort());
	}

	ota_handle_active = false;

	string_format(call->result, "OK finish ota, checksum: %s", string_cstr(ota_sha256_hash_text));
}

void command_ota_commit(cli_command_call_t *call)
{
	esp_err_t rv;
	unsigned char local_sha256_hash[32];
	string_auto(local_sha256_hash_text, (sizeof(local_sha256_hash) * 2) + 1);
	string_t remote_sha256_hash_text;
	const esp_partition_t *boot_partition;
	esp_partition_pos_t partition_pos;
	esp_image_metadata_t image_metadata;

	assert(call->parameter_count == 1);

	remote_sha256_hash_text = call->parameters[0].string;

	if(!ota_partition)
	{
		string_format(call->result, "ERROR: commit: no active OTA partition");
		return;
	}

	if((rv = esp_partition_get_sha256(ota_partition, local_sha256_hash)))
	{
		string_format(call->result, "ERROR: esp_partition_get_sha256 failed: %u", rv);
		return;
	}

	util_hash_to_string(local_sha256_hash_text, sizeof(local_sha256_hash), local_sha256_hash);

	if(!string_equal_string(remote_sha256_hash_text, local_sha256_hash_text))
	{
		string_format(call->result, "ERROR: checksum mismatch: %s vs. %s", string_cstr(remote_sha256_hash_text), string_cstr(local_sha256_hash_text));
		return;
	}

	if((rv = esp_ota_set_boot_partition(ota_partition)))
	{
		string_format(call->result, "ERROR: esp_ota_set_boot_partition failed: %u", rv);
		return;
	}

	ota_partition = (const esp_partition_t *)0;

	if(!(boot_partition = esp_ota_get_boot_partition()))
	{
		string_format(call->result, "ERROR: esp_ota_get_boot_partition");
		return;
	}

	partition_pos.offset = boot_partition->address;
	partition_pos.size = boot_partition->size;

	if((rv = esp_image_verify(ESP_IMAGE_VERIFY, &partition_pos, &image_metadata)))
	{
		string_format(call->result, "ERROR: esp_image_verify failed: %u", rv);
		return;
	}

	string_format(call->result, "OK commit ota");
}

void command_ota_confirm(cli_command_call_t *call)
{
	int rv;
	const esp_partition_t *boot_partition, *running_partition;
	unsigned int slot;

	assert(call->parameter_count == 1);

	slot = call->parameters[0].unsigned_int;

	if(!(running_partition = esp_ota_get_running_partition()))
	{
		string_format(call->result, "ERROR: esp_ota_get_running_partition failed");
		return;
	}

	if(util_partition_to_slot(running_partition) != slot)
	{
		string_format(call->result, "ERROR: address of running slot (%u) not equal to updated slot (%u), boot failed",
				util_partition_to_slot(running_partition), slot);
		return;
	}

	if((rv = esp_ota_mark_app_valid_cancel_rollback()))
	{
		string_format(call->result, "ERROR: esp_ota_mark_app_valid_cancel_rollback failed: %u", rv);
		return;
	}

	if(!(boot_partition = esp_ota_get_boot_partition()))
	{
		string_format(call->result, "ERROR: esp_ota_get_boot_partition failed");
		return;
	}

	if(util_partition_to_slot(boot_partition) != slot)
	{
		string_format(call->result, "ERROR: address of boot slot (%u) not equal to updated slot (%u), confirm failed",
				util_partition_to_slot(boot_partition), slot);
		return;
	}

	string_format(call->result, "OK confirm ota");
}
