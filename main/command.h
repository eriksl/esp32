#pragma once

#include "exception.h"
#include "cli-command.h"

#include <string>

class Command
{
	public:

		Command();
		Command(const Command &) = delete;

		static void config_info(cli_command_call_t *call);
		static void config_set_int(cli_command_call_t *call);
		static void config_set_string(cli_command_call_t *call);
		static void config_erase(cli_command_call_t *call);
		static void config_dump(cli_command_call_t *call);
		static void config_show(cli_command_call_t *call);

	private:

		std::string make_exception_text(std::string_view fn, std::string_view message1, std::string_view message2);
};
