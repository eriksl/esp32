#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "string.h"
#include "config.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"

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

static bool find_key(const char *namespace, const string_t key, nvs_entry_info_t *info)
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

		if(string_equal_cstr(key, info->key))
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

static bool get_value_as_integer(const char *namespace, const string_t key, const nvs_entry_info_t *their_info, const char **type, int64_t *value)
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

static bool get_value_as_string(const char *namespace, const string_t key, const nvs_entry_info_t *their_info, const char **type, string_t dst)
{
	esp_err_t rv;
	nvs_entry_info_t our_info;
	const nvs_entry_info_t *info;
	nvs_handle_t handle;

	assert(dst);

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
			string_format(dst, "%u (%#02x)", raw_value, raw_value);
			break;
		}
		case(NVS_TYPE_I8):
		{
			int8_t raw_value;
			if(type)
				*type = "int8";
			util_abort_on_esp_err("nvs_get_i8", nvs_get_i8(handle, info->key, &raw_value));
			string_format(dst, "%d (%#02x)", raw_value, (unsigned int)raw_value);
			break;
		}
		case(NVS_TYPE_U16):
		{
			uint16_t raw_value;
			if(type)
				*type = "uint16";
			util_abort_on_esp_err("nvs_get_u16", nvs_get_u16(handle, info->key, &raw_value));
			string_format(dst, "%u (%#04x)", raw_value, raw_value);
			break;
		}
		case(NVS_TYPE_I16):
		{
			int16_t raw_value;
			if(type)
				*type = "int16";
			util_abort_on_esp_err("nvs_get_i16", nvs_get_i16(handle, info->key, &raw_value));
			string_format(dst, "%d (%#04x)", raw_value, (unsigned int)raw_value);
			break;
		}
		case(NVS_TYPE_U32):
		{
			uint32_t raw_value;
			if(type)
				*type = "uint32";
			util_abort_on_esp_err("nvs_get_u32", nvs_get_u32(handle, info->key, &raw_value));
			string_format(dst, "%lu (%#08lx)", raw_value, raw_value);
			break;
		}
		case(NVS_TYPE_I32):
		{
			int32_t raw_value;
			if(type)
				*type = "int32";
			util_abort_on_esp_err("nvs_get_i32", nvs_get_i32(handle, info->key, &raw_value));
			string_format(dst, "%ld (%#08x)", raw_value, (unsigned int)raw_value);
			break;
		}
		case(NVS_TYPE_U64):
		{
			uint64_t raw_value;
			if(type)
				*type = "uint64";
			util_abort_on_esp_err("nvs_get_u64", nvs_get_u64(handle, info->key, &raw_value));
			string_format(dst, "%llu (%#016llx)", raw_value, raw_value);
			break;
		}
		case(NVS_TYPE_I64):
		{
			int64_t raw_value;
			if(type)
				*type = "int64";
			util_abort_on_esp_err("nvs_get_i64", nvs_get_i64(handle, info->key, &raw_value));
			string_format(dst, "%lld (%#016llx)", raw_value, (uint64_t)raw_value);
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

			string_format(dst, "%.*s", (int)length, raw_value);
			break;
		}
		case(NVS_TYPE_BLOB):
		{
			if(type)
				*type = "blob";

			string_assign_cstr(dst, "<blob>");
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

bool config_get_uint(const string_t key, uint32_t *value)
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

bool config_get_uint_cstr(const char *key, uint32_t *value)
{
	string_auto(key_string, 32);

	string_assign_cstr(key_string, key);

	return(config_get_uint(key_string, value));
}

bool config_get_int(const string_t key, int32_t *value)
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

bool config_get_int_cstr(const char *key, int32_t *value)
{
	string_auto(key_string, 32);

	string_assign_cstr(key_string, key);

	return(config_get_int(key_string, value));
}

bool config_get_string(const string_t key, string_t dst)
{
	assert(inited);
	assert(dst);

	return(get_value_as_string((const char *)0, key, (nvs_entry_info_t *)0, (const char **)0, dst));
}

bool config_get_string_cstr(const char *key, string_t dst)
{
	string_auto(key_string, 32);

	string_assign_cstr(key_string, key);

	return(config_get_string(key_string, dst));
}

void config_set_uint(const string_t key, uint32_t value)
{
	nvs_handle_t handle;

	util_abort_on_esp_err("nvs_open", nvs_open("config", NVS_READWRITE, &handle));
	util_abort_on_esp_err("nvs_set_u32", nvs_set_u32(handle, string_cstr(key), value));
	util_abort_on_esp_err("nvs_commit", nvs_commit(handle));
	nvs_close(handle);
}

void config_set_uint_cstr(const char *key, uint32_t value)
{
	string_auto(key_string, 32);

	string_assign_cstr(key_string, key);

	config_set_uint(key_string, value);
}

void config_set_int(const string_t key, int32_t value)
{
	nvs_handle_t handle;

	util_abort_on_esp_err("nvs_open", nvs_open("config", NVS_READWRITE, &handle));
	util_abort_on_esp_err("nvs_set_i32", nvs_set_i32(handle, string_cstr(key), value));
	util_abort_on_esp_err("nvs_commit", nvs_commit(handle));
	nvs_close(handle);
}

void config_set_int_cstr(const char *key, int32_t value)
{
	string_auto(key_string, 32);

	string_assign_cstr(key_string, key);

	config_set_int(key_string, value);
}

void config_set_string(const string_t key, string_t value)
{
	nvs_handle_t handle;

	util_abort_on_esp_err("nvs_open", nvs_open("config", NVS_READWRITE, &handle));
	util_abort_on_esp_err("nvs_set_str", nvs_set_str(handle, string_cstr(key), string_cstr(value)));
	util_abort_on_esp_err("nvs_commit", nvs_commit(handle));
	nvs_close(handle);
}

void config_set_string_cstr(const char *key, string_t value)
{
	string_auto(key_string, 32);

	string_assign_cstr(key_string, key);

	config_set_string(key_string, value);
}

bool config_erase(const string_t key)
{
	int rv;
	nvs_handle_t handle;

	util_abort_on_esp_err("nvs_open", nvs_open("config", NVS_READWRITE, &handle));

	if((rv = nvs_erase_key(handle, string_cstr(key))) == ESP_ERR_NVS_NOT_FOUND)
	{
		nvs_close(handle);
		return(false);
	}

	util_abort_on_esp_err("nvs_erase_key", rv);
	util_abort_on_esp_err("nvs_commit", nvs_commit(handle));
	nvs_close(handle);

	return(true);
}

bool config_erase_cstr(const char *key)
{
	string_auto(key_string, 32);

	string_assign_cstr(key_string, key);

	return(config_erase(key_string));
}

bool config_erase_wildcard(const string_t key)
{
	int rv;
	nvs_handle_t handle;
	nvs_iterator_t iterator;
	nvs_entry_info_t info;

	assert(inited);

	util_abort_on_esp_err("nvs_open", nvs_open("config", NVS_READWRITE, &handle));

	if((rv = nvs_entry_find("nvs", "config", NVS_TYPE_ANY, &iterator)) == ESP_ERR_NVS_NOT_FOUND)
		return(true);

	util_abort_on_esp_err("nvs_entry_find", rv);

	for(;;)
	{
		util_abort_on_esp_err("nvs_entry_info", nvs_entry_info(iterator, &info));

		if((strlen(info.key) >= string_length(key)) && string_equal_data(key, string_length(key), (const uint8_t *)info.key))
			util_abort_on_esp_err("nvs_erase_key", nvs_erase_key(handle, info.key));

		if((rv = nvs_entry_next(&iterator)) == ESP_ERR_NVS_NOT_FOUND)
			break;

		util_abort_on_esp_err("nvs_entry_next", rv);
	}

	nvs_release_iterator(iterator);

	util_abort_on_esp_err("nvs_commit", nvs_commit(handle));
	nvs_close(handle);

	return(true);
}

bool config_erase_wildcard_cstr(const char *key)
{
	string_auto(key_string, 32);

	string_assign_cstr(key_string, key);

	return(config_erase_wildcard(key_string));
}

static void config_dump(cli_command_call_t *call, const char *namespace)
{
	int rv;
	nvs_iterator_t iterator;
	nvs_entry_info_t info;
	string_auto(dst, 64);
	string_auto(key, 16);
	const char *type;

	assert(inited);

	string_format(call->result, "SHOW CONFIG namespace %s", namespace ? namespace : "ALL");

	if((rv = nvs_entry_find("nvs", namespace, NVS_TYPE_ANY, &iterator)) == ESP_ERR_NVS_NOT_FOUND)
		return;

	util_abort_on_esp_err("nvs_entry_find", rv);

	for(;;)
	{
		util_abort_on_esp_err("nvs_entry_info", nvs_entry_info(iterator, &info));

		string_assign_cstr(key, info.key);

		if(!get_value_as_string(info.namespace_name, key, &info, &type, dst))
			string_assign_cstr(dst, "<not found");

		if(namespace)
			string_format_append(call->result, "\n- %-7s %-20s %s", type, string_cstr(key), string_cstr(dst));
		else
			string_format_append(call->result, "\n- %-12s %-7s %-20s %s", info.namespace_name, type, string_cstr(key), string_cstr(dst));

		if((rv = nvs_entry_next(&iterator)) == ESP_ERR_NVS_NOT_FOUND)
			break;

		util_abort_on_esp_err("nvs_entry_next", rv);
	}

	nvs_release_iterator(iterator);
}

void config_command_info(cli_command_call_t *call)
{
	nvs_stats_t stats;

	assert(inited);
	assert(call->parameter_count == 0);

	util_abort_on_esp_err("nvs_get_stats", nvs_get_stats((const char *)0, &stats));

	string_format(call->result, "CONFIG INFO");
	string_format_append(call->result, "\nentries:");
	string_format_append(call->result, "\n- used: %u", stats.used_entries);
	string_format_append(call->result, "\n- free: %u", stats.free_entries);
	string_format_append(call->result, "\n- available: %u", stats.available_entries);
	string_format_append(call->result, "\n- total: %u", stats.total_entries);
	string_format_append(call->result, "\n- namespaces: %u", stats.namespace_count);
}

void config_command_set_uint(cli_command_call_t *call)
{
	int64_t value;
	const char *type;
	assert(inited);
	assert(call->parameter_count == 2);

	config_set_uint(call->parameters[0].string, call->parameters[1].unsigned_int);

	if(get_value_as_integer("config", call->parameters[0].string, (const nvs_entry_info_t *)0, &type, &value))
		string_format(call->result, "%s[%s]=%lld", string_cstr(call->parameters[0].string), type, value);
	else
		string_format(call->result, "ERROR: %s not found", string_cstr(call->parameters[0].string));
}

void config_command_set_int(cli_command_call_t *call)
{
	int64_t value;
	const char *type;
	assert(inited);
	assert(call->parameter_count == 2);

	config_set_int(call->parameters[0].string, call->parameters[1].signed_int);

	if(get_value_as_integer("config", call->parameters[0].string, (const nvs_entry_info_t *)0, &type, &value))
		string_format(call->result, "%s[%s]=%lld", string_cstr(call->parameters[0].string), type, value);
	else
		string_format(call->result, "ERROR: %s not found", string_cstr(call->parameters[0].string));
}

void config_command_set_string(cli_command_call_t *call)
{
	string_auto(dst, 64);
	const char *type;

	assert(inited);
	assert(call->parameter_count == 2);

	config_set_string(call->parameters[0].string, call->parameters[1].string);

	if(get_value_as_string((const char *)0, call->parameters[0].string, (nvs_entry_info_t *)0, &type, dst))
		string_format(call->result, "%s[%s]=%s", string_cstr(call->parameters[0].string), type, string_cstr(dst));
	else
		string_format(call->result, "ERROR: %s not found", string_cstr(call->parameters[0].string));
}

void config_command_erase(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 1);

	if(config_erase(call->parameters[0].string))
		string_format(call->result, "erase %s OK", string_cstr(call->parameters[0].string));
	else
		string_format(call->result, "erase %s not found", string_cstr(call->parameters[0].string));
}

void config_command_dump(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	return(config_dump(call, (const char *)0));
}

void config_command_show(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	return(config_dump(call, "config"));
}
