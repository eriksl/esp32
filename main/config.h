#pragma once

#include <nvs.h>

#include <string>

class Config final
{
	public:

		Config() = delete;
		Config(const Config &) = delete;
		Config(std::string_view default_name_space);

		static Config &get();
		void set_int(const std::string &key, int64_t value, std::string_view name_space = "");
		void set_string(const std::string &key, const std::string &value, std::string_view name_space = "");
		int64_t get_int(const std::string &key, std::string *type = nullptr, const std::string_view name_space = "");
		std::string get_string(const std::string &key, std::string *type = nullptr, std::string_view name_space = "");
		void erase(const std::string &key, std::string_view name_space = "");
		void erase_wildcard(const std::string &key, std::string_view name_space = "");
		void dump(std::string &dst, std::string_view name_space = "");

	private:

		std::string default_name_space;
		static Config *singleton;

		std::string make_exception_text(esp_err_t err,
				std::string_view fn, std::string_view message1, std::string_view message2,
				std::string_view key, std::string_view name_space);

		void set_value(std::string_view key, std::string_view name_space,
				int64_t *int_value = nullptr, const std::string *string_value = nullptr);

		void get_value(std::string_view key, std::string_view name_space,
				int64_t *int_value_in = nullptr, std::string *string_value_in = nullptr,
				std::string *formatted_value_in = nullptr, std::string *type_in = nullptr,
				const nvs_entry_info_t *their_info = nullptr);
};
