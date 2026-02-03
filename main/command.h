#pragma once

#include "config.h"
#include "console.h"
#include "exception.h"
#include "cli-command.h"

#include <string>

class Command final
{
	public:

		Command(Config &, Console &);
		Command() = delete;
		Command(const Command &) = delete;

		static void config_info(cli_command_call_t *call);
		static void config_set_int(cli_command_call_t *call);
		static void config_set_string(cli_command_call_t *call);
		static void config_erase(cli_command_call_t *call);
		static void config_dump(cli_command_call_t *call);
		static void config_show(cli_command_call_t *call);
		static void console_info(cli_command_call_t *call);

	private:

		Config &config;
		Console &console;

		static Command *singleton;
		static Config *config_;
		static Console *console_;

		std::string make_exception_text(std::string_view fn, std::string_view message1, std::string_view message2);
};
