#include <string>
#include <map>

#include "string.h"
#include "cli-command.h"
#include "log.h"
#include "alias.h"

#include <string>
#include <boost/format.hpp>

class Alias
{
	private:

		typedef std::map<std::string, std::string> string_string_map;
		string_string_map aliases;

	public:

		void command(cli_command_call_t *call);
		void expand(std::string &data) const;
};

void Alias::command(cli_command_call_t *call)
{
	string_string_map::const_iterator it;

	switch(call->parameter_count)
	{
		case(0):
		{
			break;
		}

		case(1):
		{
			if((it = aliases.find(call->parameters[0].str)) != aliases.end())
				aliases.erase(it);

			break;
		}

		case(2):
		{
			aliases.insert_or_assign(call->parameters[0].str, call->parameters[1].str);
			break;
			break;
		}

		default:
		{
			assert(call->parameter_count < 3);
			break;
		}
	}

	call->result = "ALIASES";

	for(const auto &ref : aliases)
		call->result += (boost::format("\n  %s: %s") % ref.first % ref.second).str();

}

void Alias::expand(std::string &data) const
{
	std::string command;
	std::string parameters;
	unsigned int delimiter;
	string_string_map::const_iterator it;

	if(data.length() == 0)
		return;

	for(delimiter = 0; delimiter < data.length(); delimiter++)
		if(data.at(delimiter) <= ' ')
			break;

	if(delimiter == 0)
		return;

	if(delimiter >= data.length())
		command = data;
	else
		command = data.substr(0, delimiter);

	if((it = aliases.find(command)) == aliases.end())
		return;

	parameters = data.substr(delimiter);

	data = it->second + parameters;
}

static Alias *alias_singleton = nullptr;

void alias_init()
{
	assert(!alias_singleton);
	alias_singleton = new Alias;
	assert(alias_singleton);
}

void command_alias(cli_command_call_t *call)
{
	assert(alias_singleton);
	alias_singleton->command(call);
}

void alias_expand(std::string &data)
{
	assert(alias_singleton);
	alias_singleton->expand(data);
}
