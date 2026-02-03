#include "config.h"

#include <nvs_flash.h>

#include <stdint.h>

#include <string>
#include <format>

Config *Config::singleton = nullptr;

Config::Config(std::string_view default_name_space_in) :
		default_name_space(default_name_space_in)
{
	esp_err_t rv;

	if(this->singleton)
		throw(hard_exception("Config: already activated"));

	rv = nvs_flash_init();

	if((rv == ESP_ERR_NVS_NO_FREE_PAGES) || (rv == ESP_ERR_NVS_NEW_VERSION_FOUND))
	{
		if((rv = nvs_flash_erase()) != ESP_OK)
			throw(hard_exception(std::format("Config::Config: nvs_flash_erase failed: {} ({})", rv, esp_err_to_name(rv))));

		rv = nvs_flash_init();
	}

	if(rv != ESP_OK)
		throw(hard_exception(std::format("Config::Config: nvs_flash_init failed: {} ({})", rv, esp_err_to_name(rv))));

	this->singleton = this;
}

std::string Config::make_exception_text(esp_err_t err,
		std::string_view fn, std::string_view message1, std::string_view message2,
		std::string_view key, std::string_view name_space)
{
	return(std::format("Config::{}: {}{}, key: {}, namespace: {}, error: {:#x} \"{}\"",
			fn, message1, message2, key, name_space, err, esp_err_to_name(err)));
}

void Config::set_value(std::string_view key_in, std::string_view name_space_in, int64_t *int_value, const std::string *string_value)
{
	nvs_handle_t local_nvs_handle;
	esp_err_t rv = ESP_OK;
	std::string key(key_in);
	std::string name_space;

	name_space = name_space_in == "" ? this->default_name_space : name_space_in;

	try
	{
		if((rv = nvs_open(name_space.c_str(), NVS_READWRITE, &local_nvs_handle)) != ESP_OK)
			throw(hard_exception("nvs_open"));

		if(int_value)
		{
			if((rv = nvs_set_i64(local_nvs_handle, key.c_str(), *int_value)) != ESP_OK)
				throw(hard_exception("nvs_set_i64"));
		}
		else
			if(string_value)
				if((rv = nvs_set_str(local_nvs_handle, key.c_str(), string_value->c_str())) != ESP_OK)
					throw(hard_exception("nvs_set_str"));

		if((rv = nvs_commit(local_nvs_handle)) != ESP_OK)
			throw(hard_exception("nvs_commit"));
	}
	catch(const transient_exception &e)
	{
		nvs_close(local_nvs_handle);
		throw(transient_exception(this->make_exception_text(rv, "set-value", "error in ", e.what(), key, name_space)));
	}
	catch(const hard_exception &e)
	{
		nvs_close(local_nvs_handle);
		throw(hard_exception(this->make_exception_text(rv, "set-value", "error in ", e.what(), key, name_space)));
	}

	nvs_close(local_nvs_handle);
}

void Config::get_value(std::string_view key, std::string_view name_space,
		int64_t *int_value_in, std::string *string_value_in, std::string *formatted_value_in, std::string *type_in,
		const nvs_entry_info_t *their_info)
{
	nvs_entry_info_t our_info;
	const nvs_entry_info_t *info = nullptr;
	nvs_iterator_t nvs_iterator = nullptr;
	nvs_handle_t local_nvs_handle;
	esp_err_t rv = ESP_OK;
	int64_t int_value;
	std::string string_value;
	std::string formatted_value;
	std::string type;
	const char *name_space_ptr;
	std::string name_space_name;

	if(name_space == "*")
	{
		name_space_name = "ALL";
		name_space_ptr = nullptr;
	}
	else
	{
		if(name_space == "")
		{
			name_space_name = this->default_name_space;
			name_space_ptr = this->default_name_space.c_str();
		}
		else
		{
			name_space_name = name_space;
			name_space_ptr = name_space_name.c_str();
		}
	}

	try
	{
		if(their_info)
			info = their_info;
		else
		{
			if((rv = nvs_entry_find("nvs", name_space_ptr, NVS_TYPE_ANY, &nvs_iterator)) == ESP_ERR_NVS_NOT_FOUND)
				throw(transient_exception("nvs_entry_find"));

			if(rv != ESP_OK)
				throw(hard_exception("nvs_entry_find"));

			for(;;)
			{
				if((rv = nvs_entry_info(nvs_iterator, &our_info)) != ESP_OK)
					throw(hard_exception("nvs_entry_info"));

				if(key == our_info.key)
					break;

				if((rv = nvs_entry_next(&nvs_iterator)) == ESP_ERR_NVS_NOT_FOUND)
					throw(transient_exception("nvs_entry_next"));

				if(rv != ESP_OK)
					throw(hard_exception("nvs_entry_next"));
			}

			if(nvs_iterator)
			{
				nvs_release_iterator(nvs_iterator);
				nvs_iterator = nullptr;
			}

			info = &our_info;
		}

		if((rv = nvs_open(name_space_ptr, NVS_READONLY, &local_nvs_handle)) != ESP_OK)
			throw(hard_exception("nvs_open"));

		type = "none";
		int_value = 0;
		string_value = "";

		switch(info->type)
		{
			case(NVS_TYPE_U8):
			{
				uint8_t raw_value;

				if((rv = nvs_get_u8(local_nvs_handle, info->key, &raw_value)) != ESP_OK)
					throw(transient_exception("nvs_get_u8"));

				type = "uint8";
				int_value = raw_value;

				break;
			}
			case(NVS_TYPE_I8):
			{
				int8_t raw_value;

				if((rv = nvs_get_i8(local_nvs_handle, info->key, &raw_value)) != ESP_OK)
					throw(transient_exception("nvs_get_i8"));

				type = "int8";
				int_value = raw_value;

				break;
			}
			case(NVS_TYPE_U16):
			{
				uint16_t raw_value;

				if((rv = nvs_get_u16(local_nvs_handle, info->key, &raw_value)) != ESP_OK)
					throw(transient_exception("nvs_get_u16"));

				type = "uint16";
				int_value = raw_value;

				break;
			}
			case(NVS_TYPE_I16):
			{
				int16_t raw_value;

				if((rv = nvs_get_i16(local_nvs_handle, info->key, &raw_value)) != ESP_OK)
					throw(transient_exception("nvs_get_i16"));
				type = "int16";
				int_value = raw_value;

				break;
			}
			case(NVS_TYPE_U32):
			{
				uint32_t raw_value;

				if((rv = nvs_get_u32(local_nvs_handle, info->key, &raw_value)) != ESP_OK)
					throw(transient_exception("nvs_get_u16"));
				type = "uint32";
				int_value = raw_value;

				break;
			}
			case(NVS_TYPE_I32):
			{
				int32_t raw_value;

				if((rv = nvs_get_i32(local_nvs_handle, info->key, &raw_value)) != ESP_OK)
					throw(transient_exception("nvs_get_i32"));
				type = "int32";
				int_value = raw_value;

				break;
			}
			case(NVS_TYPE_U64):
			{
				uint64_t raw_value;

				if((rv = nvs_get_u64(local_nvs_handle, info->key, &raw_value)) != ESP_OK)
					throw(transient_exception("nvs_get_u64"));

				type = "uint64";
				int_value = raw_value;

				break;
			}
			case(NVS_TYPE_I64):
			{
				int64_t raw_value;

				if((rv = nvs_get_i64(local_nvs_handle, info->key, &raw_value)) != ESP_OK)
					throw(transient_exception("nvs_get_i64"));

				type = "int64";
				int_value = raw_value;

				break;
			}
			case(NVS_TYPE_STR):
			{
				std::string raw_value;
				size_t length;

				if((rv = nvs_get_str(local_nvs_handle, info->key, nullptr, &length)) != ESP_OK)
					throw(transient_exception("nvs_get_str 1"));

				if(length < 1)
					throw(hard_exception("nvs_get_str length < 1"));

				raw_value.resize(length);

				if(nvs_get_str(local_nvs_handle, info->key, raw_value.data(), &length) != ESP_OK)
					throw(hard_exception("nvs_get_str 2"));

				type = "string";
				string_value = raw_value.substr(0, length - 1);

				break;
			}
			case(NVS_TYPE_BLOB):
			{
				type = "blob";

				break;
			}
			default:
			{
				type = "unknown";

				break;
			}
		}

		switch(info->type)
		{
			case(NVS_TYPE_U8):
			case(NVS_TYPE_I8):
			case(NVS_TYPE_U16):
			case(NVS_TYPE_I16):
			case(NVS_TYPE_U32):
			case(NVS_TYPE_I32):
			case(NVS_TYPE_U64):
			case(NVS_TYPE_I64):
			{
				string_value = std::to_string(int_value);
				formatted_value = std::format("{:d} ({:#x})", int_value, int_value);

				break;
			}

			case(NVS_TYPE_STR):
			{
				try
				{
					int_value = std::stoll(string_value);
				}
				catch (const std::invalid_argument &)
				{
					int_value = 0;
				}
				catch (const std::out_of_range &)
				{
					int_value = 0;
				}

				formatted_value = std::format("{} ({:d})", string_value, string_value.length());

				break;
			}

			case(NVS_TYPE_BLOB):
			{
				formatted_value = "<blob>";

				break;
			}

			default:
			{
				formatted_value = "<unknown>";

				break;
			}
		}
	}
	catch(const transient_exception &e)
	{
		if(nvs_iterator)
			nvs_release_iterator(nvs_iterator);

		nvs_close(local_nvs_handle);

		throw(transient_exception(this->make_exception_text(rv, "get-value", "error in ", e.what(), key, name_space_name)));
	}
	catch(const hard_exception &e)
	{
		if(nvs_iterator)
			nvs_release_iterator(nvs_iterator);

		nvs_close(local_nvs_handle);

		throw(hard_exception(this->make_exception_text(rv, "get-value", "error in ", e.what(), key, name_space_name)));
	}

	if(nvs_iterator)
		nvs_release_iterator(nvs_iterator);

	nvs_close(local_nvs_handle);

	if(type_in)
		*type_in = type;

	if(int_value_in)
		*int_value_in = int_value;

	if(string_value_in)
		*string_value_in = string_value;

	if(formatted_value_in)
		*formatted_value_in = formatted_value;
}

Config &Config::get()
{
	if(!Config::singleton)
		throw(hard_exception("Config: not activated"));

	return(*Config::singleton);
}

void Config::set_int(const std::string &key, int64_t value, std::string_view name_space)
{
	this->set_value(key, name_space, &value, nullptr);
}

void Config::set_string(const std::string &key, const std::string &value, std::string_view name_space)
{
	this->set_value(key, name_space, nullptr, &value);
}

int64_t Config::get_int(const std::string &key, std::string *type, std::string_view name_space)
{
	int64_t value;

	this->get_value(key, name_space, &value, nullptr, nullptr, type);

	return(value);
}

std::string Config::get_string(const std::string &key, std::string *type, std::string_view name_space)
{
	std::string value;

	this->get_value(key, name_space, nullptr, &value, nullptr, type);

	return(value);
}

void Config::erase(const std::string &key, std::string_view name_space)
{
	esp_err_t rv = ESP_OK;
	nvs_handle_t local_nvs_handle;
	const char *name_space_ptr;
	std::string name_space_name;

	if(name_space == "*")
	{
		name_space_name = "ALL";
		name_space_ptr = nullptr;
	}
	else
	{
		if(name_space == "")
		{
			name_space_name = this->default_name_space;
			name_space_ptr = this->default_name_space.c_str();
		}
		else
		{
			name_space_name = name_space;
			name_space_ptr = name_space_name.c_str();
		}
	}

	try
	{
		if((rv = nvs_open(name_space_ptr, NVS_READWRITE, &local_nvs_handle)) != ESP_OK)
			throw(hard_exception("nvs_open"));

		if((rv = nvs_erase_key(local_nvs_handle, key.c_str())) == ESP_ERR_NVS_NOT_FOUND)
			throw(transient_exception("nvs_erase_key"));

		if(rv != ESP_OK)
			throw(hard_exception("nvs_erase_key"));

		if((rv = nvs_commit(local_nvs_handle)) != ESP_OK)
			throw(hard_exception("nvs_commit"));
	}
	catch(const transient_exception &e)
	{
		nvs_close(local_nvs_handle);
		throw(transient_exception(this->make_exception_text(rv, "erase", "error in ", e.what(), key, name_space_name)));
	}
	catch(const hard_exception &e)
	{
		nvs_close(local_nvs_handle);
		throw(hard_exception(this->make_exception_text(rv, "erase", "error in ", e.what(), key, name_space_name)));
	}

	nvs_close(local_nvs_handle);
}

void Config::erase_wildcard(const std::string &key, std::string_view name_space)
{
	esp_err_t rv = ESP_OK;
	nvs_handle_t local_nvs_handle;
	nvs_iterator_t nvs_iterator;
	nvs_entry_info_t nvs_entry_info_local;
	const char *name_space_ptr;
	std::string name_space_name;

	if(name_space == "*")
	{
		name_space_name = "ALL";
		name_space_ptr = nullptr;
	}
	else
	{
		if(name_space == "")
		{
			name_space_name = this->default_name_space;
			name_space_ptr = this->default_name_space.c_str();
		}
		else
		{
			name_space_name = name_space;
			name_space_ptr = name_space_name.c_str();
		}
	}

	try
	{
		if((rv = nvs_open(name_space_ptr, NVS_READWRITE, &local_nvs_handle)) != ESP_OK)
			throw(hard_exception("nvs_open"));

		if((rv = nvs_entry_find("nvs", name_space_ptr, NVS_TYPE_ANY, &nvs_iterator)) != ESP_OK)
			throw(hard_exception("nvs_entry_find"));

		for(;;)
		{
			if((rv = nvs_entry_info(nvs_iterator, &nvs_entry_info_local)) != ESP_OK)
				throw(hard_exception("nvs_entry_info"));

			if((strlen(nvs_entry_info_local.key) >= key.length()) && (key.compare(0, std::string::npos, nvs_entry_info_local.key, key.length()) == 0))
				if((rv = nvs_erase_key(local_nvs_handle, nvs_entry_info_local.key)) != ESP_OK)
					throw(hard_exception("nvs_erase_key"));

			if((rv = nvs_entry_next(&nvs_iterator)) == ESP_ERR_NVS_NOT_FOUND)
				break;

			if(rv != ESP_OK)
				throw(hard_exception("nvs_entry_next"));
		}

		if((rv = nvs_commit(local_nvs_handle)) != ESP_OK)
			throw(hard_exception("nvs_commit"));
	}
	catch(const hard_exception &e)
	{
		nvs_release_iterator(nvs_iterator);
		nvs_close(local_nvs_handle);

		throw(hard_exception(this->make_exception_text(rv, "erase-wildcard", "error in ", e.what(), key, name_space_name)));
	}

	nvs_release_iterator(nvs_iterator);
	nvs_close(local_nvs_handle);
}

void Config::dump(std::string &dst, std::string_view name_space)
{
	esp_err_t rv = ESP_OK;
	nvs_iterator_t nvs_iterator;
	nvs_entry_info_t info;
	std::string key;
	std::string string_value;
	int64_t int_value;
	std::string formatted_value;
	std::string type;
	const char *name_space_ptr;
	std::string name_space_name;

	if(name_space == "*")
	{
		name_space_name = "ALL";
		name_space_ptr = nullptr;
	}
	else
	{
		if(name_space == "")
		{
			name_space_name = this->default_name_space;
			name_space_ptr = this->default_name_space.c_str();
		}
		else
		{
			name_space_name = name_space;
			name_space_ptr = name_space_name.c_str();
		}
	}

	try
	{
		dst = std::format("SHOW CONFIG namespace {}", name_space_name);

		if((rv = nvs_entry_find("nvs", name_space_ptr, NVS_TYPE_ANY, &nvs_iterator)) == ESP_ERR_NVS_NOT_FOUND)
			return;

		if(rv != ESP_OK)
			throw(hard_exception("nvs_entry_find"));

		dst += std::format("\n- {:<16} {:<40} {:<6} {}", "KEY", "VALUE", "TYPE", "NAMESPACE");

		for(;;)
		{
			if((rv = nvs_entry_info(nvs_iterator, &info)) != ESP_OK)
				throw(hard_exception("nvs_entry_info"));

			key = info.key;
			name_space_name = info.namespace_name;

			this->get_value(key, name_space_name, &int_value, &string_value, &formatted_value, &type, &info);

			dst += std::format("\n- {:<16} {:<40} {:<6} {}", key, formatted_value, type, info.namespace_name);

			if((rv = nvs_entry_next(&nvs_iterator)) == ESP_ERR_NVS_NOT_FOUND)
				break;

			if(rv != ESP_OK)
				throw(hard_exception("nvs_entry_next"));
		}
	}
	catch(const hard_exception &e)
	{
		nvs_release_iterator(nvs_iterator);
		throw(hard_exception(this->make_exception_text(rv, "erase-wildcard", "error in ", e.what(), key, name_space_name)));
	}

	nvs_release_iterator(nvs_iterator);
}
