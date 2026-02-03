#include "command.h"

#include <format>

Command *Command::singleton = nullptr;
Config *Command::config_ = nullptr;
Console *Command::console_ = nullptr;
Ledpixel *Command::ledpixel_ = nullptr;
LedPWM *Command::ledpwm_ = nullptr;

Command::Command(Config &config_in, Console &console_in, Ledpixel &ledpixel_in, LedPWM &ledpwm_in) :
		config(config_in), console(console_in), ledpixel(ledpixel_in), ledpwm(ledpwm_in)
{
	if(this->singleton)
		throw(hard_exception("Command: already activated"));

	this->singleton = this;
	this->config_ = &config;
	this->console_ = &console;
	this->ledpixel_ = &ledpixel;
	this->ledpwm_ = &ledpwm;
}

std::string Command::make_exception_text(std::string_view fn, std::string_view message1, std::string_view message2)
{
	return(std::format("{}: {}{}", fn, message1, message2));
}

void Command::config_set_int(cli_command_call_t *call)
{
	int64_t value;
	std::string type;

	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	try
	{
		config_->set_int(call->parameters[0].str, call->parameters[1].signed_int);
		value = config_->get_int(call->parameters[0].str, &type);
		call->result = std::format("{}[{}]={:d}", call->parameters[0].str, type, value);
	}
	catch(const e32if_exception &e)
	{
		call->result = singleton->make_exception_text("config-set-int", "ERROR: ", e.what());
		return;
	}

	call->result = std::format("{}[{}]={:d}", call->parameters[0].str, type, value);
}

void Command::config_set_string(cli_command_call_t *call)
{
	std::string value;
	std::string type;

	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	try
	{
		config_->set_string(call->parameters[0].str, call->parameters[1].str);
		value = config_->get_string(call->parameters[0].str, &type);
		call->result = std::format("{}[{}]={}", call->parameters[0].str, type, value);
	}
	catch(const e32if_exception &e)
	{
		call->result = singleton->make_exception_text("config-set-str", "ERROR: ", e.what());
		return;
	}

	call->result = std::format("{}[{}]={}", call->parameters[0].str, type, value);
}

void Command::config_erase(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	try
	{
		if(call->parameter_count == 2)
			config_->erase(call->parameters[0].str, call->parameters[1].str);
		else
			config_->erase(call->parameters[0].str);
	}
	catch(const e32if_exception &e)
	{
		call->result = singleton->make_exception_text("config-erase", "ERROR: ", e.what());
		return;
	}

	call->result = std::format("erase {} OK", call->parameters[0].str);
}

void Command::config_dump(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "ALL CONFIG entries\n";

	Command::config_->dump(call->result, "*");
}

void Command::config_show(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "CONFIG ENTRIES\n";

	Command::config_->dump(call->result);
}

void Command::config_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "CONFIG INFO\n";

	Command::config_->info(call->result);
}

void Command::console_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "CONSOLE STATISTICS\n";

	Command::console_->info(call->result);
}

void Command::ledpixel_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "LEDPIXEL INFO\n";

	Command::ledpixel_->info(call->result);
}

void Command::ledpwm_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "LEDPWM INFO\n";

	Command::ledpwm_->info(call->result);
}
