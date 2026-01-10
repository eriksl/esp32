#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "string.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "config.h"

#include <freertos/FreeRTOS.h>
#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>

#include <string>
#include <boost/format.hpp>

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

static bool find_key(const std::string &name_space, const std::string &key, nvs_entry_info_t *info)
{
	int rv;
	nvs_iterator_t iterator;

	iterator = nullptr;

	if((rv = nvs_entry_find("nvs", name_space.c_str(), NVS_TYPE_ANY, &iterator)) == ESP_ERR_NVS_NOT_FOUND)
		return(false);

	util_abort_on_esp_err("nvs_entry_find", rv);

	for(;;)
	{
		util_abort_on_esp_err("nvs_entry_info", nvs_entry_info(iterator, info));

		if(key == info->key)
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

static bool get_value_as_integer(const std::string &name_space, const std::string &key, const nvs_entry_info_t *their_info, std::string &type, int64_t &value)
{
	esp_err_t rv;
	nvs_entry_info_t info;
	nvs_handle_t handle;

	if(their_info)
		info = *their_info;
	else
	{
		if(!find_key(name_space.c_str(), key.c_str(), &info) && !find_key("", key.c_str(), &info))
		{
			type = "not found";
			return(false);
		}
	}

	util_abort_on_esp_err("nvs_open", nvs_open(name_space.c_str(), NVS_READONLY, &handle));

	switch(info.type)
	{
		case(NVS_TYPE_U8):
		{
			uint8_t raw_value;
			type = "uint8";
			util_abort_on_esp_err("nvs_get_u8", nvs_get_u8(handle, info.key, &raw_value));
			value = raw_value;
			break;
		}
		case(NVS_TYPE_I8):
		{
			int8_t raw_value;
			type = "int8";
			util_abort_on_esp_err("nvs_get_i8", nvs_get_i8(handle, info.key, &raw_value));
			value = raw_value;
			break;
		}
		case(NVS_TYPE_U16):
		{
			uint16_t raw_value;
			type = "uint16";
			util_abort_on_esp_err("nvs_get_u16", nvs_get_u16(handle, info.key, &raw_value));
			value = raw_value;
			break;
		}
		case(NVS_TYPE_I16):
		{
			int16_t raw_value;
			type = "int16";
			util_abort_on_esp_err("nvs_get_i16", nvs_get_i16(handle, info.key, &raw_value));
			value = raw_value;
			break;
		}
		case(NVS_TYPE_U32):
		{
			uint32_t raw_value;
			type = "uint32";
			util_abort_on_esp_err("nvs_get_u32", nvs_get_u32(handle, info.key, &raw_value));
			value = raw_value;
			break;
		}
		case(NVS_TYPE_I32):
		{
			int32_t raw_value;
			type = "int32";
			util_abort_on_esp_err("nvs_get_i32", nvs_get_i32(handle, info.key, &raw_value));
			value = raw_value;
			break;
		}
		case(NVS_TYPE_U64):
		{
			uint64_t raw_value;
			type = "uint64";
			util_abort_on_esp_err("nvs_get_u64", nvs_get_u64(handle, info.key, &raw_value));
			value = raw_value;
			break;
		}
		case(NVS_TYPE_I64):
		{
			int64_t raw_value;
			type = "int64";
			util_abort_on_esp_err("nvs_get_i64", nvs_get_i64(handle, info.key, &raw_value));
			value = raw_value;
			break;
		}
		case(NVS_TYPE_STR):
		{
			char raw_value[32];
			char *endptr;
			unsigned length;

			type = "string";

			rv = nvs_get_str(handle, info.key, raw_value, &length);

			if(rv == ESP_ERR_NVS_INVALID_LENGTH)
				value = 0;
			else
			{
				util_abort_on_esp_err("nvs_get_str", rv);
				value = strtoll(raw_value, &endptr, 0);
			}

			break;
		}
		case(NVS_TYPE_BLOB):
		{
			char raw_value[32];
			char *endptr;
			unsigned length;

			type = "blob";

			rv = nvs_get_blob(handle, info.key, raw_value, &length);

			if(rv == ESP_ERR_NVS_INVALID_LENGTH)
				value = 0;
			else
			{
				util_abort_on_esp_err("nvs_get_blob", rv);
				value = strtoll(raw_value, &endptr, 0);
			}
			break;
		}
		default:
		{
			type = "unknown";
			value = 0;
			break;
		}
	}

	nvs_close(handle);

	return(true);
}

static bool get_value_as_string(std::string name_space, const std::string &key, const nvs_entry_info_t *their_info, std::string &type, std::string &dst)
{
	esp_err_t rv;
	nvs_entry_info_t our_info;
	const nvs_entry_info_t *info;
	nvs_handle_t handle;

	if(name_space == "")
		name_space = "config";

	if(their_info)
		info = their_info;
	else
	{
		if(!find_key(name_space, key, &our_info) && !find_key("", key, &our_info))
		{
			type = "not found";
			return(false);
		}

		info = &our_info;
	}

	util_abort_on_esp_err("nvs_open", nvs_open(name_space.c_str(), NVS_READONLY, &handle));

	switch(info->type)
	{
		case(NVS_TYPE_U8):
		{
			uint8_t raw_value;
			type = "uint8";
			util_abort_on_esp_err("nvs_get_u8", nvs_get_u8(handle, info->key, &raw_value));
			dst = (boost::format("%u (%#02x)") % raw_value % raw_value).str();
			break;
		}
		case(NVS_TYPE_I8):
		{
			int8_t raw_value;
			type = "int8";
			util_abort_on_esp_err("nvs_get_i8", nvs_get_i8(handle, info->key, &raw_value));
			dst = (boost::format("%d (%#02x)") % raw_value % raw_value).str();
			break;
		}
		case(NVS_TYPE_U16):
		{
			uint16_t raw_value;
			type = "uint16";
			util_abort_on_esp_err("nvs_get_u16", nvs_get_u16(handle, info->key, &raw_value));
			dst = (boost::format("%u (%#04x)") % raw_value % raw_value).str();
			break;
		}
		case(NVS_TYPE_I16):
		{
			int16_t raw_value;
			type = "int16";
			util_abort_on_esp_err("nvs_get_i16", nvs_get_i16(handle, info->key, &raw_value));
			dst = (boost::format("%d (%#04x)") % raw_value % raw_value).str();
			break;
		}
		case(NVS_TYPE_U32):
		{
			uint32_t raw_value;
			type = "uint32";
			util_abort_on_esp_err("nvs_get_u32", nvs_get_u32(handle, info->key, &raw_value));
			dst = (boost::format("%lu (%#08lx)") % raw_value % raw_value).str();
			break;
		}
		case(NVS_TYPE_I32):
		{
			int32_t raw_value;
			type = "int32";
			util_abort_on_esp_err("nvs_get_i32", nvs_get_i32(handle, info->key, &raw_value));
			dst = (boost::format("%ld (%#08x)") % raw_value % raw_value).str();
			break;
		}
		case(NVS_TYPE_U64):
		{
			uint64_t raw_value;
			type = "uint64";
			util_abort_on_esp_err("nvs_get_u64", nvs_get_u64(handle, info->key, &raw_value));
			dst = (boost::format("%llu (%#016llx)") % raw_value % raw_value).str();
			break;
		}
		case(NVS_TYPE_I64):
		{
			int64_t raw_value;
			type = "int64";
			util_abort_on_esp_err("nvs_get_i64", nvs_get_i64(handle, info->key, &raw_value));
			dst = (boost::format("%lld (%#016llx)") % raw_value % raw_value).str();
			break;
		}
		case(NVS_TYPE_STR):
		{
			char raw_value[32];
			unsigned length;

			type = "string";

			length = sizeof(raw_value);
			rv = nvs_get_str(handle, info->key, raw_value, &length);

			if(rv == ESP_ERR_NVS_INVALID_LENGTH)
				length = 0;
			else
				util_abort_on_esp_err("nvs_get_str", rv);

			dst.assign(raw_value, length);

			break;
		}
		case(NVS_TYPE_BLOB):
		{
			type = "blob";
			dst = "<blob>";

			break;
		}
		default:
		{
			type = "unknown";
			break;
		}
	}

	nvs_close(handle);

	return(true);
}

//

bool config_get_uint(const std::string &key, uint32_t &value)
{
	int64_t raw_value;
	std::string type;

	assert(inited);

	if(!get_value_as_integer("config", key, nullptr, type, raw_value))
	{
		value = 0;
		return(false);
	}

	value = raw_value;
	return(true);
}

bool config_get_uint(string_t key, uint32_t &value)
{
	return(config_get_uint(std::string(string_cstr(key)), value));
}

//

bool config_get_int(const std::string &key, int32_t &value)
{
	int64_t raw_value;
	std::string type;

	assert(inited);

	if(!get_value_as_integer("config", key, nullptr, type, raw_value))
	{
		value = 0;
		return(false);
	}

	value = raw_value;
	return(true);
}

bool config_get_int(const string_t key, int32_t &value)
{
	return(config_get_int(std::string(string_cstr(key)), value));
}

//

bool config_get_string(const std::string &key, std::string &dst)
{
	std::string type;

	assert(inited);

	return(get_value_as_string("config", key, nullptr, type, dst));
}

bool config_get_string(const string_t key, string_t dst)
{
	assert(inited);
	std::string tmp_dst;
	bool rv;

	if((rv = config_get_string(std::string(string_cstr(key)), tmp_dst)))
		string_assign_cstr(dst, tmp_dst.c_str());

	return(rv);
}

//

void config_set_uint(const std::string &key, uint32_t value)
{
	nvs_handle_t handle;

	util_abort_on_esp_err("nvs_open", nvs_open("config", NVS_READWRITE, &handle));
	util_abort_on_esp_err("nvs_set_u32", nvs_set_u32(handle, key.c_str(), value));
	util_abort_on_esp_err("nvs_commit", nvs_commit(handle));
	nvs_close(handle);
}

void config_set_uint(const string_t key, uint32_t value)
{
	config_set_uint(std::string(string_cstr(key)), value);
}

//

void config_set_int(const std::string &key, int32_t value)
{
	nvs_handle_t handle;

	util_abort_on_esp_err("nvs_open", nvs_open("config", NVS_READWRITE, &handle));
	util_abort_on_esp_err("nvs_set_i32", nvs_set_i32(handle, key.c_str(), value));
	util_abort_on_esp_err("nvs_commit", nvs_commit(handle));
	nvs_close(handle);
}

void config_set_int(const string_t key, int32_t value)
{
	config_set_int(std::string(string_cstr(key)), value);
}

//

void config_set_string(const std::string &key, const std::string &value)
{
	nvs_handle_t handle;

	util_abort_on_esp_err("nvs_open", nvs_open("config", NVS_READWRITE, &handle));
	util_abort_on_esp_err("nvs_set_str", nvs_set_str(handle, key.c_str(), value.c_str()));
	util_abort_on_esp_err("nvs_commit", nvs_commit(handle));
	nvs_close(handle);
}

void config_set_string(const string_t key, string_t value)
{
	config_set_string(std::string(string_cstr(key)), std::string(string_cstr(key)));
}

//

bool config_erase(const std::string &key)
{
	int rv;
	nvs_handle_t handle;

	util_abort_on_esp_err("nvs_open", nvs_open("config", NVS_READWRITE, &handle));

	if((rv = nvs_erase_key(handle, key.c_str())) == ESP_ERR_NVS_NOT_FOUND)
	{
		nvs_close(handle);
		return(false);
	}

	util_abort_on_esp_err("nvs_erase_key", rv);
	util_abort_on_esp_err("nvs_commit", nvs_commit(handle));
	nvs_close(handle);

	return(true);
}

bool config_erase(const string_t key)
{
	return(config_erase(std::string(string_cstr(key))));
}

//

bool config_erase_wildcard(const std::string &key)
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

		if((strlen(info.key) >= key.length()) && (key.compare(0, std::string::npos, info.key, key.length()) == 0))
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

bool config_erase_wildcard(const string_t key)
{
	return(config_erase_wildcard(std::string(string_cstr(key))));
}

//

static void config_dump(cli_command_call_t *call, const char *name_space)
{
	int rv;
	nvs_iterator_t iterator;
	nvs_entry_info_t info;
	std::string dst;
	std::string key;
	std::string type;

	assert(inited);

	string_format(call->result, "SHOW CONFIG namespace %s", name_space ? name_space : "ALL");

	if((rv = nvs_entry_find("nvs", name_space, NVS_TYPE_ANY, &iterator)) == ESP_ERR_NVS_NOT_FOUND)
		return;

	util_abort_on_esp_err("nvs_entry_find", rv);

	for(;;)
	{
		util_abort_on_esp_err("nvs_entry_info", nvs_entry_info(iterator, &info));

		key = info.key;

		if(!get_value_as_string(info.namespace_name, key, &info, type, dst))
			dst = "<not found>";

		if(name_space)
			string_format_append(call->result, "\n- %-7s %-20s %s", type.c_str(), key.c_str(), dst.c_str());
		else
			string_format_append(call->result, "\n- %-12s %-7s %-20s %s", info.namespace_name, type.c_str(), key.c_str(), dst.c_str());

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

	util_abort_on_esp_err("nvs_get_stats", nvs_get_stats(nullptr, &stats));

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
	std::string type;

	assert(inited);
	assert(call->parameter_count == 2);

	config_set_uint(call->parameters[0].str, call->parameters[1].unsigned_int);

	if(get_value_as_integer("config", call->parameters[0].str, nullptr, type, value))
		string_format(call->result, "%s[%s]=%lld", call->parameters[0].str.c_str(), type.c_str(), value);
	else
		string_format(call->result, "ERROR: %s not found", call->parameters[0].str.c_str());
}

void config_command_set_int(cli_command_call_t *call)
{
	int64_t value;
	std::string type;

	assert(inited);
	assert(call->parameter_count == 2);

	config_set_int(call->parameters[0].str, call->parameters[1].signed_int);

	if(get_value_as_integer("config", call->parameters[0].str, nullptr, type, value))
		string_format(call->result, "%s[%s]=%lld", call->parameters[0].str.c_str(), type.c_str(), value);
	else
		string_format(call->result, "ERROR: %s not found", call->parameters[0].str.c_str());
}

void config_command_set_string(cli_command_call_t *call)
{
	std::string dst;
	std::string type;

	assert(inited);
	assert(call->parameter_count == 2);

	config_set_string(call->parameters[0].str, call->parameters[1].str);

	if(get_value_as_string("", call->parameters[0].str, (nvs_entry_info_t *)0, type, dst))
		string_format(call->result, "%s[%s]=%s", call->parameters[0].str.c_str(), type.c_str(), dst.c_str());
	else
		string_format(call->result, "ERROR: %s not found", call->parameters[0].str.c_str());
}

void config_command_erase(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 1);

	if(config_erase(std::string(call->parameters[0].str.c_str())))
		string_format(call->result, "erase %s OK", call->parameters[0].str.c_str());
	else
		string_format(call->result, "erase %s not found", call->parameters[0].str.c_str());
}

void config_command_dump(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	return(config_dump(call, nullptr));
}

void config_command_show(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	return(config_dump(call, "config"));
}
