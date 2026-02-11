#include "command.h"

#include "util.h"

#include <format>

Command *Command::singleton = nullptr;
Config *Command::config_ = nullptr;
Console *Command::console_ = nullptr;
Ledpixel *Command::ledpixel_ = nullptr;
LedPWM *Command::ledpwm_ = nullptr;
Notify *Command::notify_ = nullptr;
Log *Command::log_ = nullptr;
System *Command::system_ = nullptr;
Util *Command::util_ = nullptr;
PDM *Command::pdm_ = nullptr;

Command::Command(Config &config_in, Console &console_in, Ledpixel &ledpixel_in, LedPWM &ledpwm_in,
		Notify &notify_in, Log &log_in, System &system_in, Util &util_in, PDM &pdm_in)
	:
		config(config_in), console(console_in), ledpixel(ledpixel_in), ledpwm(ledpwm_in),
		notify(notify_in), log(log_in), system(system_in), util(util_in), pdm(pdm_in)
{
	if(this->singleton)
		throw(hard_exception("Command: already activated"));

	this->singleton = this;
	this->config_ = &config;
	this->console_ = &console;
	this->ledpixel_ = &ledpixel;
	this->ledpwm_ = &ledpwm;
	this->notify_ = &notify;
	this->log_ = &log;
	this->system_ = &system;
	this->util_ = &util;
	this->pdm_ = &pdm;
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

void Command::notify_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "NOTIFY INFO\n";

	Command::notify_->info(call->result);
}

void Command::log_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "LOG INFO\n";

	Command::log_->info(call->result);
}

void Command::log_log(cli_command_call_t *call)
{
	int entry;

	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	if(call->parameter_count == 1)
		entry = call->parameters[0].unsigned_int;
	else
		entry = -1;

	Command::log_->command_log(call->result, entry);
}

void Command::log_clear(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	Command::log_->clear();

	call->result = "log cleared";
}

void Command::log_monitor(cli_command_call_t *call)
{
	bool monitor;

	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	if(call->parameter_count == 1)
		Command::log_->setmonitor(call->parameters[0].unsigned_int > 0);

	monitor = Command::log_->getmonitor();

	call->result = std::format("log monitor: {}", Util::get().yesno(monitor));
}

void Command::system_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	Command::system_->info(call->result);
}

void Command::system_memory(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	Command::system_->memory(call->result);
}

void Command::system_identify(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	Command::system_->identify(call->result, call->mtu);
}

void Command::system_partitions(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	if(call->parameter_count == 0)
		Command::system_->partitions(call->result);
	else
		Command::system_->partitions(call->result, call->parameters[0].unsigned_int);
}

void Command::system_process_list(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	if(call->parameter_count == 0)
		Command::system_->process_list(call->result);
	else
		Command::system_->process_list(call->result, call->parameters[0].unsigned_int);
}

void Command::system_process_stop(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	Command::system_->process_list(call->result, call->parameters[0].unsigned_int);
}

void Command::util_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "UTIL INFO\n";

	Command::util_->info(call->result);
}

void Command::util_timezone(cli_command_call_t *call)
{
	std::string tz;

	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	if(call->parameter_count > 0)
	{
		try
		{
			util_->set_timezone(call->parameters[0].str);
		}
		catch(const transient_exception &e)
		{
			call->result = "set timezone: failed: %s", e.what();
			return;
		}
	}

	try
	{
		tz = util_->get_timezone();
	}
	catch(const transient_exception &e)
	{
		call->result = "get timezone: failed: %s", e.what();
		return;
	}

	call->result = std::format("TZ: {}", tz);
}

void Command::pdm_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "PDM INFO\n";

	Command::pdm_->info(call->result);
}
