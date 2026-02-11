#pragma once

#include "config.h"
#include "console.h"
#include "ledpixel.h"
#include "ledpwm.h"
#include "notify.h"
#include "log.h"
#include "system.h"
#include "pdm.h"
#include "exception.h"
#include "cli-command.h"

#include <string>

class Command final
{
	public:

		Command(Config &, Console &, Ledpixel &, LedPWM &, Notify &, Log &, System &, Util &, PDM &);
		Command() = delete;
		Command(const Command &) = delete;

		static void config_info(cli_command_call_t *);
		static void config_set_int(cli_command_call_t *);
		static void config_set_string(cli_command_call_t *);
		static void config_erase(cli_command_call_t *);
		static void config_dump(cli_command_call_t *);
		static void config_show(cli_command_call_t *);
		static void console_info(cli_command_call_t *);
		static void ledpixel_info(cli_command_call_t *);
		static void ledpwm_info(cli_command_call_t *);
		static void notify_info(cli_command_call_t *);
		static void log_info(cli_command_call_t *);
		static void log_log(cli_command_call_t *);
		static void log_clear(cli_command_call_t *);
		static void log_monitor(cli_command_call_t *);
		static void system_info(cli_command_call_t *);
		static void system_identify(cli_command_call_t *);
		static void system_partitions(cli_command_call_t *);
		static void system_memory(cli_command_call_t *);
		static void system_process_list(cli_command_call_t *);
		static void system_process_stop(cli_command_call_t *);
		static void util_info(cli_command_call_t *);
		static void util_timezone(cli_command_call_t *);
		static void pdm_info(cli_command_call_t *);

	private:

		Config &config;
		Console &console;
		Ledpixel &ledpixel;
		LedPWM &ledpwm;
		Notify &notify;
		Log &log;
		System &system;
		Util &util;
		PDM &pdm;

		static Command *singleton;
		static Config *config_;
		static Console *console_;
		static Ledpixel *ledpixel_;
		static LedPWM *ledpwm_;
		static Notify *notify_;
		static Log *log_;
		static System *system_;
		static Util *util_;
		static PDM *pdm_;

		std::string make_exception_text(std::string_view fn, std::string_view message1, std::string_view message2);
};
