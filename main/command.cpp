#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "config.h"
#include "command.h"

#include <esp_err.h>

#include <string>
#include <format>

static Command *singleton = nullptr;
static Config *config_ = nullptr; // FIXME

Command::Command(Config &config_in) :
		config(config_in)
{
	if(singleton)
		throw(hard_exception("Command: already constructed"));

	::singleton = &*this;
	::config_ = &config;
}

std::string Command::make_exception_text(std::string_view fn, std::string_view message1, std::string_view message2)
{
	return(std::format("{}: {}{}", fn, message1, message2));
}


void Command::config_info(cli_command_call_t *call)
{
	nvs_stats_t stats;

	if(nvs_get_stats(nullptr, &stats) != ESP_OK)
		throw(hard_exception("Command::config_info:nvs_get_stats error"));

	call->result = "CONFIG INFO";
	call->result += "\nentries:";
	call->result += std::format("\n- used: {}", stats.used_entries);
	call->result += std::format("\n- free: {}", stats.free_entries);
	call->result += std::format("\n- available: {}", stats.available_entries);
	call->result += std::format("\n- total: {}", stats.total_entries);
	call->result += std::format("\n- namespaces: {}", stats.namespace_count);
}

void Command::config_set_int(cli_command_call_t *call)
{
	int64_t value;
	std::string type;

	if(!singleton)
		throw(hard_exception("Command: not constructed"));

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

	if(!singleton)
		throw(hard_exception("Command: not constructed"));

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
	if(!singleton)
		throw(hard_exception("Command: not constructed"));

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
	config_->dump(call->result, "*");
}

void Command::config_show(cli_command_call_t *call)
{
	config_->dump(call->result);
}
