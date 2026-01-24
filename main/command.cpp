#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "config.h"
#include "command.h"

#include <esp_err.h>

#include <string>
#include <boost/format.hpp> // FIXME use std::formatter

static Command *singleton = nullptr;

Command::Command()
{
	if(singleton)
		throw(hard_exception("Command: already constructed"));

	singleton = &*this;
}

std::string Command::make_exception_text(esp_err_t err, std::string_view fn, std::string_view message1, std::string_view message2)
{
	return((boost::format("Config::%s: %s%s, error: %u \"%s\"") %
			fn % message1 % message2 % err % esp_err_to_name(err)).str());
}


void Command::config_info(cli_command_call_t *call)
{
	nvs_stats_t stats;

	if(nvs_get_stats(nullptr, &stats) != ESP_OK)
		throw(hard_exception("Command::config_info:nvs_get_stats error"));

	call->result = "CONFIG INFO";
	call->result += "\nentries:";
	call->result += (boost::format("\n- used: %u") % stats.used_entries).str();
	call->result += (boost::format("\n- free: %u") % stats.free_entries).str();
	call->result += (boost::format("\n- available: %u") % stats.available_entries).str();
	call->result += (boost::format("\n- total: %u") % stats.total_entries).str();
	call->result += (boost::format("\n- namespaces: %u") % stats.namespace_count).str();
}

void Command::config_set_int(cli_command_call_t *call)
{
	int64_t value;
	std::string type;

	if(!singleton)
		throw(hard_exception("Command: not constructed"));

	try
	{
		Config::set_int(call->parameters[0].str, call->parameters[1].signed_int);
		value = Config::get_int(call->parameters[0].str, &type);
		call->result = (boost::format("%s[%s]=%lld") % call->parameters[0].str % type % value).str();
	}
	catch(const e32if_exception &e)
	{
		call->result = singleton->make_exception_text(ESP_OK, "config-set-int", "ERROR: ", e.what());
		return;
	}

	call->result = (boost::format("%s[%s]=%lld") % call->parameters[0].str % type % value).str();
}

void Command::config_set_string(cli_command_call_t *call)
{
	std::string value;
	std::string type;

	if(!singleton)
		throw(hard_exception("Command: not constructed"));

	try
	{
		Config::set_string(call->parameters[0].str, call->parameters[1].str);
		value = Config::get_string(call->parameters[0].str, &type);
		call->result = (boost::format("%s[%s]=%s") % call->parameters[0].str % type % value).str();
	}
	catch(const e32if_exception &e)
	{
		call->result = singleton->make_exception_text(ESP_OK, "config-set-str", "ERROR: ", e.what());
		return;
	}

	call->result = (boost::format("%s[%s]=%s") % call->parameters[0].str % type % value).str();
}

void Command::config_erase(cli_command_call_t *call)
{
	if(!singleton)
		throw(hard_exception("Command: not constructed"));

	try
	{
		Config::erase(std::string(call->parameters[0].str));
	}
	catch(const e32if_exception &e)
	{
		call->result = singleton->make_exception_text(ESP_OK, "config-erase", "ERROR: ", e.what());
		return;
	}

	call->result = (boost::format("erase %s OK") % call->parameters[0].str).str();
}

void Command::config_dump(cli_command_call_t *call)
{
	Config::dump(call->result, "*");
}

void Command::config_show(cli_command_call_t *call)
{
	Config::dump(call->result);
}
