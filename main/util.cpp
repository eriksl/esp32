#include "util.h"

#include "log.h"

#include <sys/socket.h>

#include <string>
#include <format>
#include <chrono>

static bool inited;

std::string util_time_to_string(std::string_view format, const time_t &stamp)
{
	// FIXME: timezone
	std::chrono::zoned_time chrono_stamp{"Europe/Amsterdam", std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::from_time_t(stamp))};

	try
	{
		return(std::vformat(format, std::make_format_args(chrono_stamp)));
	}
	catch(const std::format_error& e)
	{
		return(std::string("[util_time_to_string: ") + e.what() + ", format string: " + std::string(format) + "]");
	}
}

std::string util_time_to_string(const time_t &stamp)
{
	return(util_time_to_string("{:%Y/%m/%d %H:%M:%S}", stamp));
}

std::string_view yesno(bool yesno_)
{
	static const char *no = "no";
	static const char *yes = "yes";

	return(yesno_ ? yes : no);
}

void util_warn_on_esp_err(std::string_view what, unsigned int rv)
{
	if(rv == ESP_OK)
		return;

	try
	{
		Log::get().log_esperr(rv, what);
	}
	catch(...)
	{
	}
}

void util_abort_on_esp_err(std::string_view what, int rv)
{
	if(rv == ESP_OK)
		return;

	try
	{
		Log::get().setmonitor(true);
		util_warn_on_esp_err(what, rv);
	}
	catch(...)
	{
	}

	abort();
}

void util_abort(std::string_view what)
{
	try
	{
		Log::get().setmonitor(true);
		Log::get().log(std::format("abort: %s", what));
	}
	catch(...)
	{
	}

	abort();
}

std::string util_esp_string_error(esp_err_t e, const std::string &message)
{
	return(message + ": " + std::to_string(e) + "\"" + esp_err_to_name(e) + "\"");
}

void util_init(void)
{
	assert(!inited);

	setenv("TZ", "CEST-1CET,M3.5.0/2:00:00,M10.5.0/2:00:00", 1); // FIXME
	tzset();

	inited = true;
}
