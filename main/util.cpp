#include "util.h"

#include "exception.h"

#include <string>
#include <format>
#include <chrono>

Util* Util::singleton = nullptr;

Util::Util(Config &config_in) : config(config_in)
{
	std::string tz;

	if(this->singleton)
		throw(hard_exception("Util::Util: already active"));

	try
	{
		tz = this->config.get_string("timezone");
	}
	catch(const transient_exception &)
	{
		tz = "UTC";
	}

	this->timezone = tz;

	this->singleton = this;
}

Util &Util::get()
{
	if(!Util::singleton)
		throw(hard_exception("Util::get: not active"));

	return(*Util::singleton);
}

std::string Util::time_to_string(const time_t &stamp, std::string_view format)
{
	try
	{
		std::chrono::zoned_time chrono_stamp{this->timezone, std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::from_time_t(stamp))};
		return(std::vformat(format, std::make_format_args(chrono_stamp)));
	}
	catch(const std::format_error &e)
	{
		return(std::string("[util_time_to_string: ") + e.what() + ", format string: " + std::string(format) + "]");
	}
	catch(...)
	{
	}

	if(this->timezone != "UTC")
	{
		this->timezone = "UTC";

		try
		{
			return(this->time_to_string(stamp, format));
		}
		catch(...)
		{
			return("***");
		}
	}

	return("###");
}

std::string_view Util::yesno(bool yesno_)
{
	static const char *no = "no";
	static const char *yes = "yes";

	return(yesno_ ? yes : no);
}

void Util::info(std::string &out)
{
	out += std::format("- timezone: {}", this->timezone);
}

void Util::set_timezone(std::string_view in)
{
	this->timezone = in;
	this->config.set_string("timezone", std::string(in));
}

std::string Util::get_timezone()
{
	return(this->timezone);
}
