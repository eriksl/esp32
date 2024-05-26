#include <stdbool.h>
#include <string.h>

#include "cli-command.h"
#include "config.h"
#include "log.h"
#include "util.h"

#include <freertos/FreeRTOS.h>
#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>

static bool inited = false;

void config_init(void)
{
	esp_err_t rv;

	assert(!inited);

	rv = nvs_flash_init();

	if((rv == ESP_ERR_NVS_NO_FREE_PAGES) || (rv == ESP_ERR_NVS_NEW_VERSION_FOUND))
	{
		log("init: erase and reinit flash");
		util_abort_on_esp_err("nvs_flash_erase", nvs_flash_erase());
		util_abort_on_esp_err("nvs_flash_init", nvs_flash_init());
	}
	else
		util_abort_on_esp_err("nvs_flash_init", rv);

	inited = true;
}

static bool find_key(const char *namespace, const char *key, nvs_entry_info_t *info)
{
	int rv;
	nvs_iterator_t iterator;

	iterator = (nvs_iterator_t)0;

	if((rv = nvs_entry_find("nvs", namespace, NVS_TYPE_ANY, &iterator)) == ESP_ERR_NVS_NOT_FOUND)
		return(false);

	util_abort_on_esp_err("nvs_entry_find", rv);

	for(;;)
	{
		util_abort_on_esp_err("nvs_entry_info", nvs_entry_info(iterator, info));

		if(!strcmp(info->key, key))
			break;

		util_abort_on_esp_err("nvs_entry_next", rv);
		if((rv = nvs_entry_next(&iterator)) == ESP_ERR_NVS_NOT_FOUND)
			goto error;
	}

	return(true);

error:
	nvs_release_iterator(iterator);
	return(false);
}

static bool get_value_as_integer(const char *namespace, const char *key, const nvs_entry_info_t *their_info, const char **type, int64_t *value)
{
	esp_err_t rv;
	nvs_entry_info_t info;
	nvs_handle_t handle;

	if(their_info)
		info = *their_info;
	else
	{
		if(!find_key(namespace, key, &info) && !find_key((const char *)0, key, &info))
		{
			if(type)
				*type = "not found";
			return(false);
		}
	}

	util_abort_on_esp_err("nvs_open", nvs_open(namespace, NVS_READONLY, &handle));

	switch(info.type)
	{
		case(NVS_TYPE_U8):
		{
			uint8_t raw_value;
			if(type)
				*type = "uint8";
			util_abort_on_esp_err("nvs_get_u8", nvs_get_u8(handle, info.key, &raw_value));
			*value = raw_value;
			break;
		}
		case(NVS_TYPE_I8):
		{
			int8_t raw_value;
			if(type)
				*type = "int8";
			util_abort_on_esp_err("nvs_get_i8", nvs_get_i8(handle, info.key, &raw_value));
			*value = raw_value;
			break;
		}
		case(NVS_TYPE_U16):
		{
			uint16_t raw_value;
			if(type)
				*type = "uint16";
			util_abort_on_esp_err("nvs_get_u16", nvs_get_u16(handle, info.key, &raw_value));
			*value = raw_value;
			break;
		}
		case(NVS_TYPE_I16):
		{
			int16_t raw_value;
			if(type)
				*type = "int16";
			util_abort_on_esp_err("nvs_get_i16", nvs_get_i16(handle, info.key, &raw_value));
			*value = raw_value;
			break;
		}
		case(NVS_TYPE_U32):
		{
			uint32_t raw_value;
			if(type)
				*type = "uint32";
			util_abort_on_esp_err("nvs_get_u32", nvs_get_u32(handle, info.key, &raw_value));
			*value = raw_value;
			break;
		}
		case(NVS_TYPE_I32):
		{
			int32_t raw_value;
			if(type)
				*type = "int32";
			util_abort_on_esp_err("nvs_get_i32", nvs_get_i32(handle, info.key, &raw_value));
			*value = raw_value;
			break;
		}
		case(NVS_TYPE_U64):
		{
			uint64_t raw_value;
			if(type)
				*type = "uint64";
			util_abort_on_esp_err("nvs_get_u64", nvs_get_u64(handle, info.key, &raw_value));
			*value = raw_value;
			break;
		}
		case(NVS_TYPE_I64):
		{
			int64_t raw_value;
			if(type)
				*type = "int64";
			util_abort_on_esp_err("nvs_get_i64", nvs_get_i64(handle, info.key, &raw_value));
			*value = raw_value;
			break;
		}
		case(NVS_TYPE_STR):
		{
			char raw_value[32];
			char *endptr;
			unsigned length;

			if(type)
				*type = "string";

			rv = nvs_get_str(handle, info.key, raw_value, &length);

			if(rv == ESP_ERR_NVS_INVALID_LENGTH)
				*value = 0;
			else
			{
				util_abort_on_esp_err("nvs_get_str", rv);
				*value = strtoll(raw_value, &endptr, 0);
			}

			break;
		}
		case(NVS_TYPE_BLOB):
		{
			char raw_value[32];
			char *endptr;
			unsigned length;

			if(type)
				*type = "blob";

			rv = nvs_get_blob(handle, info.key, raw_value, &length);

			if(rv == ESP_ERR_NVS_INVALID_LENGTH)
				*value = 0;
			else
			{
				util_abort_on_esp_err("nvs_get_blob", rv);
				*value = strtoll(raw_value, &endptr, 0);
			}
			break;
		}
		default:
		{
			if(type)
				*type = "unknown";
			*value = 0;
			break;
		}
	}

	nvs_close(handle);

	return(true);
}

static bool get_value_as_string(const char *namespace, const char *key, const nvs_entry_info_t *their_info, const char **type, unsigned int string_size, char *string)
{
	esp_err_t rv;
	nvs_entry_info_t our_info;
	const nvs_entry_info_t *info;
	nvs_handle_t handle;

	if(!namespace)
		namespace = "config";

	if(their_info)
		info = their_info;
	else
	{
		if(!find_key(namespace, key, &our_info) && !find_key((const char *)0, key, &our_info))
		{
			if(type)
				*type = "not found";
			return(false);
		}

		info = &our_info;
	}

	util_abort_on_esp_err("nvs_open", nvs_open(namespace, NVS_READONLY, &handle));

	switch(info->type)
	{
		case(NVS_TYPE_U8):
		{
			uint8_t raw_value;
			if(type)
				*type = "uint8";
			util_abort_on_esp_err("nvs_get_u8", nvs_get_u8(handle, info->key, &raw_value));
			snprintf(string, string_size, "%u (%#02x)", raw_value, raw_value);
			break;
		}
		case(NVS_TYPE_I8):
		{
			int8_t raw_value;
			if(type)
				*type = "int8";
			util_abort_on_esp_err("nvs_get_i8", nvs_get_i8(handle, info->key, &raw_value));
			snprintf(string, string_size, "%d (%#02x)", raw_value, raw_value);
			break;
		}
		case(NVS_TYPE_U16):
		{
			uint16_t raw_value;
			if(type)
				*type = "uint16";
			util_abort_on_esp_err("nvs_get_u16", nvs_get_u16(handle, info->key, &raw_value));
			snprintf(string, string_size, "%u (%#04x)", raw_value, raw_value);
			break;
		}
		case(NVS_TYPE_I16):
		{
			int16_t raw_value;
			if(type)
				*type = "int16";
			util_abort_on_esp_err("nvs_get_i16", nvs_get_i16(handle, info->key, &raw_value));
			snprintf(string, string_size, "%d (%#04x)", raw_value, raw_value);
			break;
		}
		case(NVS_TYPE_U32):
		{
			uint32_t raw_value;
			if(type)
				*type = "uint32";
			util_abort_on_esp_err("nvs_get_u32", nvs_get_u32(handle, info->key, &raw_value));
			snprintf(string, string_size, "%lu (%#08lx)", raw_value, raw_value);
			break;
		}
		case(NVS_TYPE_I32):
		{
			int32_t raw_value;
			if(type)
				*type = "int32";
			util_abort_on_esp_err("nvs_get_i32", nvs_get_i32(handle, info->key, &raw_value));
			snprintf(string, string_size, "%ld (%#08lx)", raw_value, raw_value);
			break;
		}
		case(NVS_TYPE_U64):
		{
			uint64_t raw_value;
			if(type)
				*type = "uint64";
			util_abort_on_esp_err("nvs_get_u64", nvs_get_u64(handle, info->key, &raw_value));
			snprintf(string, string_size, "%llu (%#016llx)", raw_value, raw_value);
			break;
		}
		case(NVS_TYPE_I64):
		{
			int64_t raw_value;
			if(type)
				*type = "int64";
			util_abort_on_esp_err("nvs_get_i64", nvs_get_i64(handle, info->key, &raw_value));
			snprintf(string, string_size, "%lld (%#016llx)", raw_value, raw_value);
			break;
		}
		case(NVS_TYPE_STR):
		{
			char raw_value[32];
			unsigned length;

			if(type)
				*type = "string";

			length = sizeof(raw_value);
			rv = nvs_get_str(handle, info->key, raw_value, &length);

			if(rv == ESP_ERR_NVS_INVALID_LENGTH)
				length = 0;
			else
				util_abort_on_esp_err("nvs_get_str", rv);

			snprintf(string, string_size, "%.*s", length, raw_value);
			break;
		}
		case(NVS_TYPE_BLOB):
		{
			char raw_value[32];
			unsigned length, ix, offset;

			if(type)
				*type = "blob";

			length = sizeof(raw_value);
			rv = nvs_get_blob(handle, info->key, raw_value, &length);

			if(rv == ESP_ERR_NVS_INVALID_LENGTH)
				length = 0;
			else
				util_abort_on_esp_err("nvs_get_blob", rv);

			offset = snprintf(string, string_size, "[%d]", length);
			for(ix = 0; ix < length; ix++)
				offset += snprintf(string + offset, string_size - offset, " %02x", (uint8_t)raw_value[ix]);
			break;
		}
		default:
		{
			if(type)
				*type = "unknown";
			break;
		}
	}

	nvs_close(handle);

	return(true);
}

bool config_get_int(const char *key, int32_t *value)
{
	int64_t raw_value;

	assert(inited);

	if(!get_value_as_integer("config", key, (const nvs_entry_info_t *)0, (const char **)0, &raw_value))
	{
		*value = 0;
		return(false);
	}

	*value = raw_value;
	return(true);
}

bool config_get_uint(const char *key, uint32_t *value)
{
	int64_t raw_value;

	assert(inited);

	if(!get_value_as_integer("config", key, (const nvs_entry_info_t *)0, (const char **)0, &raw_value))
	{
		*value = 0;
		return(false);
	}

	*value = raw_value;
	return(true);
}

bool config_get_string(const char *key, unsigned int string_size, char *string)
{
	assert(inited);
	assert(string);
	assert(string_size);

	if(!get_value_as_string((const char *)0, key, (nvs_entry_info_t *)0, (const char **)0, string_size, string))
	{
		string[0] = '\0';
		return(false);
	}

	return(true);
}

void config_set_uint(const char *key, uint32_t value)
{
	nvs_handle_t handle;

	util_abort_on_esp_err("nvs_open", nvs_open("config", NVS_READWRITE, &handle));
	util_abort_on_esp_err("nvs_set_u32", nvs_set_u32(handle, key, value));
	util_abort_on_esp_err("nvs_commit", nvs_commit(handle));
	nvs_close(handle);
}

void config_set_int(const char *key, int32_t value)
{
	nvs_handle_t handle;

	util_abort_on_esp_err("nvs_open", nvs_open("config", NVS_READWRITE, &handle));
	util_abort_on_esp_err("nvs_set_i32", nvs_set_i32(handle, key, value));
	util_abort_on_esp_err("nvs_commit", nvs_commit(handle));
	nvs_close(handle);
}

void config_set_string(const char *key, const char *value)
{
	nvs_handle_t handle;

	util_abort_on_esp_err("nvs_open", nvs_open("config", NVS_READWRITE, &handle));
	util_abort_on_esp_err("nvs_set_str", nvs_set_str(handle, key, value));
	util_abort_on_esp_err("nvs_commit", nvs_commit(handle));
	nvs_close(handle);
}

bool config_erase(const char *key)
{
	int rv;
	nvs_handle_t handle;

	util_abort_on_esp_err("nvs_open", nvs_open("config", NVS_READWRITE, &handle));

	if((rv = nvs_erase_key(handle, key)) == ESP_ERR_NVS_NOT_FOUND)
	{
		nvs_close(handle);
		return(false);
	}

	util_abort_on_esp_err("nvs_erase_key", rv);
	util_abort_on_esp_err("nvs_commit", nvs_commit(handle));
	nvs_close(handle);

	return(true);
}

void command_info_config(cli_command_call_t *call)
{
	unsigned int offset;
	nvs_stats_t stats;

	assert(inited);
	assert(call->parameters->count == 0);

	util_abort_on_esp_err("nvs_get_stats", nvs_get_stats((const char *)0, &stats));

	offset = snprintf(call->result, call->result_size, "CONFIG INFO");
	offset += snprintf(call->result + offset, call->result_size - offset, "\nentries:");
	offset += snprintf(call->result + offset, call->result_size - offset, "\n- used: %u", stats.used_entries);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n- free: %u", stats.free_entries);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n- available: %u", stats.available_entries);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n- total: %u", stats.total_entries);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n- namespaces: %u", stats.namespace_count);
}

void command_config_set_uint(cli_command_call_t *call)
{
	int64_t value;
	const char *type;
	assert(inited);
	assert(call->parameters->count == 2);

	config_set_uint(call->parameters->parameters[0].string, call->parameters->parameters[1].unsigned_int);

	if(get_value_as_integer("config", call->parameters->parameters[0].string, (const nvs_entry_info_t *)0, &type, &value))
		snprintf(call->result, call->result_size, "%s[%s]=%lld", call->parameters->parameters[0].string, type, value);
	else
		snprintf(call->result, call->result_size, "ERROR: %s not found", call->parameters->parameters[0].string);
}

void command_config_set_int(cli_command_call_t *call)
{
	int64_t value;
	const char *type;
	assert(inited);
	assert(call->parameters->count == 2);

	config_set_int(call->parameters->parameters[0].string, call->parameters->parameters[1].signed_int);

	if(get_value_as_integer("config", call->parameters->parameters[0].string, (const nvs_entry_info_t *)0, &type, &value))
		snprintf(call->result, call->result_size, "%s[%s]=%lld", call->parameters->parameters[0].string, type, value);
	else
		snprintf(call->result, call->result_size, "ERROR: %s not found", call->parameters->parameters[0].string);
}

void command_config_set_string(cli_command_call_t *call)
{
	char value[64];
	const char *type;

	assert(inited);
	assert(call->parameters->count == 2);

	config_set_string(call->parameters->parameters[0].string, call->parameters->parameters[1].string);

	if(get_value_as_string((const char *)0, call->parameters->parameters[0].string, (nvs_entry_info_t *)0, &type, sizeof(value), value))
		snprintf(call->result, call->result_size, "%s[%s]=%s", call->parameters->parameters[0].string, type, value);
	else
		snprintf(call->result, call->result_size, "ERROR: %s not found", call->parameters->parameters[0].string);
}

void command_config_erase(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameters->count == 1);

	if(config_erase(call->parameters->parameters[0].string))
		snprintf(call->result, call->result_size, "erase %s OK", call->parameters->parameters[0].string);
	else
		snprintf(call->result, call->result_size, "erase %s not found", call->parameters->parameters[0].string);
}

static void config_dump(cli_command_call_t *call, const char *namespace)
{
	int rv;
	nvs_iterator_t iterator;
	nvs_entry_info_t info;
	unsigned int offset;
	char value[64];
	const char *type;

	assert(inited);

	offset = snprintf(call->result, call->result_size, "SHOW CONFIG namespace %s", namespace ? namespace : "ALL");

	if((rv = nvs_entry_find("nvs", namespace, NVS_TYPE_ANY, &iterator)) == ESP_ERR_NVS_NOT_FOUND)
		return;

	util_abort_on_esp_err("nvs_entry_find", rv);

	for(;;)
	{
		util_abort_on_esp_err("nvs_entry_info", nvs_entry_info(iterator, &info));

		if(!get_value_as_string(info.namespace_name, info.key, &info, &type, sizeof(value), value))
			snprintf(value, sizeof(value), "%s", "<not found>");

		if(namespace)
			offset += snprintf(call->result + offset, call->result_size - offset, "\n- %-7s %-14s %s", type, info.key, value);
		else
			offset += snprintf(call->result + offset, call->result_size - offset, "\n- %-12s %-7s %-14s %s", info.namespace_name, type, info.key, value);

		if((rv = nvs_entry_next(&iterator)) == ESP_ERR_NVS_NOT_FOUND)
			break;

		util_abort_on_esp_err("nvs_entry_next", rv);
	}

	nvs_release_iterator(iterator);
}

void command_config_dump(cli_command_call_t *call)
{
	assert(call->parameters->count == 0);

	return(config_dump(call, (const char *)0));
}

void command_config_show(cli_command_call_t *call)
{
	assert(call->parameters->count == 0);

	return(config_dump(call, "config"));
}
