#pragma once

#include "config.h"

#include <time.h>

#include <esp_err.h>

#include <string>

class Util final
{
	public:

		explicit Util() = delete;
		explicit Util(Config &);
		explicit Util(const Util &) = delete;

		static Util &get();

		std::string_view yesno(bool);

		std::string time_to_string(const time_t &stamp, std::string_view format = "{:%Y/%m/%d %H:%M:%S}");
		void set_timezone(std::string_view in);
		std::string get_timezone();

		void info(std::string &);

	private:

		static Util *singleton;

		Config &config;

		std::string timezone;
};
