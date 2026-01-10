#include <string>
#include <map>

#include "string.h"
#include "cli-command.h"
#include "log.h"
#include "alias.h"

class Alias
{
	private:

		std::map<std::string, std::string> aliases;

	public:

		void command(cli_command_call_t *call);
		void expand(string_t *data) const;
};

static Alias alias;

void Alias::command(cli_command_call_t *call)
{
	switch(call->parameter_count)
	{
		case(0):
		{
			break;
		}

		case(1):
		{
			auto it = aliases.find(call->parameters[0].str);

			if(it != aliases.end())
				aliases.erase(it);

			break;
		}

		case(2):
		{
			aliases[call->parameters[0].str.c_str()] = call->parameters[1].str.c_str(); // FIXME
			break;
		}

		default:
		{
			assert(call->parameter_count < 3);
			break;
		}
	}

	string_assign_cstr(call->result, "ALIASES");

	for(auto ref : aliases)
		string_format_append(call->result, "\n  %s: %s", ref.first.c_str(), ref.second.c_str());

}

void Alias::expand(string_t *data) const
{
	string_t command;
	string_t new_data;
	unsigned int offset;

	offset = 0;

	if(string_length(*data) == 0)
		return;

	if(!(command = string_parse(*data, &offset)))
		return;

	if((offset == 0) || (string_length(command) == 0))
	{
		string_free(&command);
		return;
	}

	auto current_alias = aliases.find(string_cstr(command));

	if(current_alias == aliases.end())
	{
		string_free(&command);
		return;
	}

	new_data = string_new(string_length(*data) - offset + ' ' + current_alias->first.length());

	string_format(new_data, "%s%s", current_alias->second.c_str(), string_data(*data) + offset);

	string_free(data);
	string_free(&command);
	*data = new_data;
}

void command_alias(cli_command_call_t *call)
{
	alias.command(call);
}

void alias_expand(string_t *data)
{
	alias.expand(data);
}
