#include <stdint.h>
#include <stdbool.h>

#include "string.h"
#include "log.h"
#include "util.h"
#include "encryption.h"
#include "cli-command.h"

#include <esp_ota_ops.h>
#include <esp_image_format.h>

#include <string>
#include <boost/format.hpp>

static bool ota_handle_active = false;
static const esp_partition_t *ota_partition = (const esp_partition_t *)0;
static esp_ota_handle_t ota_handle;

static Encryption *encryption = nullptr;
static unsigned int ota_length = 0;

static void ota_abort(void)
{
	if(ota_handle_active)
	{
		util_warn_on_esp_err("otacli: ota_abort: esp_ota_abort returns error", esp_ota_abort(ota_handle));

		ota_partition = nullptr;
		ota_handle_active = false;
	}

	if(encryption)
	{
		delete encryption;
		encryption = nullptr;
	}

	ota_length = 0;
}

void command_ota_start(cli_command_call_t *call)
{
	unsigned int rv;
	unsigned int length;
	const esp_partition_t *partition;

	assert(call->parameter_count == 1);

	length = call->parameters[0].unsigned_int;

	if(!(partition = esp_ota_get_next_update_partition((const esp_partition_t *)0)))
	{
		call->result = "ERROR: no valid OTA partition";
		return;
	}

	if(partition->type != ESP_PARTITION_TYPE_APP)
	{
		call->result = (boost::format("ERROR: partition %s is not APP") % partition->label).str();
		return;
	}

	if(length > partition->size)
	{
		call->result = (boost::format("ERROR: ota partition too small for image: %u vs. %lu") % length % partition->size).str();
		return;
	}

	if(ota_handle_active || encryption)
	{
		log("otacli: ota-start: ota already active, first aborting session");
		ota_abort();
	}

	if((rv = esp_ota_begin(partition, length, &ota_handle)))
	{
		call->result = (boost::format("ERROR: esp_ota_begin: %s (0x%x)") % esp_err_to_name(rv) % rv).str();
		return(ota_abort());
	}

	ota_partition = partition;
	ota_handle_active = true;
	ota_length = length;

	assert(!encryption);

	encryption = new Encryption;
	encryption->sha256_init();

	call->result = (boost::format("OK start write ota to partition %u/%s") % util_partition_to_slot(partition) % partition->label).str();
}

void command_ota_write(cli_command_call_t *call)
{
	int rv;
	unsigned length, checksum_chunk;

	assert(call->parameter_count == 2);

	length = call->parameters[0].unsigned_int;
	checksum_chunk = call->parameters[1].unsigned_int;

	if(!encryption)
	{
		call->result = "ERROR: hash context not active";
		return(ota_abort());
	}

	if(!ota_handle_active)
	{
		call->result = "ERROR: ota write context not active";
		return(ota_abort());
	}

	if(call->oob.length() != length)
	{
		call->result = (boost::format("ERROR: lengths do not match (%u vs. %u)") % length % call->oob.length()).str();
		return(ota_abort());
	}

	if(checksum_chunk && (length != 32))
	{
		call->result = (boost::format("ERROR: invalid checksum chunk length (%u vs. %u)") % length % 32).str();
		return(ota_abort());
	}

	if((rv = esp_ota_write(ota_handle, call->oob.data(), call->oob.length())))
	{
		call->result = (boost::format("ERROR: esp_ota_write returned error %d") % rv).str();
		return(ota_abort());
	}

	if(!checksum_chunk)
		encryption->sha256_update(call->oob);

	call->result = "OK write ota";
}

void command_ota_finish(cli_command_call_t *call)
{
	unsigned int rv;
	std::string hash;
	std::string hash_text;

	if(!encryption)
	{
		call->result = "ERROR: hash context not active";
		return(ota_abort());
	}

	if(!ota_handle_active)
	{
		call->result = "ERROR: ota write context not active";
		return(ota_abort());
	}

	hash = encryption->sha256_finish();
	hash_text = util_hash_to_string(hash);

	delete encryption;
	encryption = nullptr;

	if((rv = esp_ota_end(ota_handle)))
	{
		call->result = (boost::format("ERROR: esp_ota_end failed: %s (0x%x)") % esp_err_to_name(rv) % rv).str();
		return(ota_abort());
	}

	ota_handle_active = false;

	call->result = (boost::format("OK finish ota, checksum: %s") % hash_text).str();
}

void command_ota_commit(cli_command_call_t *call)
{
	unsigned int rv;
	std::string local_hash;
	std::string local_hash_text;
	std::string remote_hash_text;
	const esp_partition_t *boot_partition;
	esp_partition_pos_t partition_pos;
	esp_image_metadata_t image_metadata;

	assert(call->parameter_count == 1);

	remote_hash_text = call->parameters[0].str;

	if(!ota_partition)
	{
		call->result = "ERROR: commit: no active OTA partition";
		return;
	}

	local_hash.resize(32);

	if((rv = esp_partition_get_sha256(ota_partition, reinterpret_cast<uint8_t *>(local_hash.data()))))
	{
		call->result = (boost::format("ERROR: esp_partition_get_sha256 failed: %u") % rv).str();
		return;
	}

	local_hash_text = util_hash_to_string(local_hash);

	if(remote_hash_text != local_hash_text)
	{
		call->result = (boost::format("ERROR: checksum mismatch: %s vs. %s") % remote_hash_text % local_hash_text).str();
		return;
	}

	if((rv = esp_ota_set_boot_partition(ota_partition)))
	{
		call->result = (boost::format("ERROR: esp_ota_set_boot_partition failed: %u") % rv).str();
		return;
	}

	ota_partition = (const esp_partition_t *)0;

	if(!(boot_partition = esp_ota_get_boot_partition()))
	{
		call->result = "ERROR: esp_ota_get_boot_partition";
		return;
	}

	partition_pos.offset = boot_partition->address;
	partition_pos.size = boot_partition->size;

	if((rv = esp_image_verify(ESP_IMAGE_VERIFY, &partition_pos, &image_metadata)))
	{
		call->result = (boost::format("ERROR: esp_image_verify failed: %u") % rv).str();
		return;
	}

	call->result = "OK commit ota";
}

void command_ota_confirm(cli_command_call_t *call)
{
	unsigned int rv;

	assert(call->parameter_count == 0);

	if((rv = esp_ota_mark_app_valid_cancel_rollback()))
	{
		call->result = (boost::format("ERROR: esp_ota_mark_app_valid_cancel_rollback failed: %u") % rv).str();
		return;
	}

	call->result = "OK confirm ota";
}
