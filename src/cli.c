#include <stdio.h>
#include <stdbool.h>
#include <errno.h>

#include "string.h"
#include "cli-command.h"
#include "stringcli.h"
#include "cli.h"
#include "otacli.h"
#include "flash.h"
#include "info.h"
#include "log.h"
#include "bt.h"
#include "console.h"
#include "config.h"
#include "wlan.h"
#include "packet.h"
#include "util.h"
#include "fs.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

enum
{
	receive_queue_size = 8,
	send_queue_size = 8,
	result_size = 4096,
	result_oob_size = 4096 + 16,
};

typedef struct
{
	unsigned int lower_bound;
	unsigned int upper_bound;
} cli_unsigned_int_description_t;

typedef struct
{
	int lower_bound;
	int upper_bound;
} cli_signed_int_description_t;

typedef struct
{
	float lower_bound;
	float upper_bound;
} cli_float_description_t;

typedef struct
{
	unsigned int lower_length_bound;
	unsigned int upper_length_bound;
} cli_string_description_t;

typedef struct
{
	cli_parameter_type_description_t type:4;

	unsigned int base:5;
	unsigned int value_required:1;
	unsigned int lower_bound_required:1;
	unsigned int upper_bound_required:1;

	const char *description;

	union
	{
		cli_unsigned_int_description_t	unsigned_int;
		cli_signed_int_description_t	signed_int;
		cli_float_description_t			fp;
		cli_string_description_t		string;
	};
} cli_parameter_description_t;

typedef struct
{
	unsigned int count;
	cli_parameter_description_t entries[parameters_size];
} cli_parameters_description_t;

typedef struct
{
	const char *name;
	const char *alias;
	const char *help;
	cli_command_function_t *function;
	cli_parameters_description_t parameters_description;
} cli_command_t;

static QueueHandle_t receive_queue_handle;
static QueueHandle_t send_queue_handle;
static unsigned int cli_stats_commands_received;
static unsigned int cli_stats_commands_received_packet;
static unsigned int cli_stats_commands_received_raw;
static unsigned int cli_stats_replies_sent;
static unsigned int cli_stats_replies_sent_packet;
static unsigned int cli_stats_replies_sent_raw;
static bool inited = false;

static const char *parameter_type_to_string(unsigned int type)
{
	static const char *type_string[cli_parameter_size] =
	{
		"invalid parameter type",
		"u_int",
		"s_int",
		"float",
		"string",
	};

	if(type >= cli_parameter_size)
		type = cli_parameter_none;

	return(type_string[type]);
}

static void help(cli_command_call_t *call);

static void command_help(cli_command_call_t *call)
{
	return(help(call));
}

#if 0
static void command_test_parse(cli_command_call_t *call)
{
	unsigned int ix, length;
	string_t token;

	string_auto(description, 512);

	assert(call->parameter_count == 1);

	string_assign_cstr(description, call->parameters[0].string);

	length = string_length(description);

	for(ix = 0; ix < length; ix++)
		if(string_at(description, ix) == '_')
			string_assign(description, ix, ' ');

	string_assign_cstr(call->result, "line: \"");
	string_append_string(call->result, description);
	string_append_cstr(call->result, "\"");

	for(ix = 0; ix < 16; ix++)
	{
		if(!(token = string_parse(description, ix)))
			break;

		string_format_append(call->result, "\n%2u \"%s\"", ix, string_cstr(token));
		string_free(token);
	}
}
#endif

static void command_hostname(cli_command_call_t *call)
{
	string_auto(hostname, 64);
	string_auto(description, 64);
	string_auto_init(key_hostname, "hostname");
	string_auto_init(key_description, "hostname_desc");

	assert(call->parameter_count < 3);

	if(call->parameter_count > 1)
	{
		string_replace(call->parameters[1].string, 0, ~0, '_', ' ');
		config_set_string(key_description, call->parameters[1].string);
	}

	if(call->parameter_count > 0)
		config_set_string(key_hostname, call->parameters[0].string);

	if(!config_get_string(key_hostname, hostname))
		string_assign_cstr(hostname, "<unset>");

	if(!config_get_string(key_description, description))
		string_assign_cstr(description, "<unset>");

	string_format(call->result, "hostname: %s (%s)", string_cstr(hostname), string_cstr(description));
}

static void command_reset(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	esp_restart();
}

static void command_info_cli(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	string_format(call->result, "commands received:");
	string_format_append(call->result, "\n- total: %u", cli_stats_commands_received);
	string_format_append(call->result, "\n- packetised: %u", cli_stats_commands_received_packet);
	string_format_append(call->result, "\n- raw: %u", cli_stats_commands_received_raw);
	string_format_append(call->result, "\nreplies sent:");
	string_format_append(call->result, "\n- total: %u", cli_stats_replies_sent);
	string_format_append(call->result, "\n- packetised: %u", cli_stats_replies_sent_packet);
	string_format_append(call->result, "\n- raw: %u", cli_stats_replies_sent_raw);
}

static const cli_command_t cli_commands[] =
{
	{ "config-dump", "cd", "dump all nvs keys", command_config_dump,
		{}
	},

	{ "config-set-int", "csi", "set a signed int config value", command_config_set_int,
		{	2,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "key", {} },
				{ cli_parameter_signed_int, 0, 1, 0, 0, "value", {} },
			},
		}
	},

	{ "config-set-uint", "csu", "set an unsigned int config value", command_config_set_uint,
		{	2,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "key", {} },
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "value", {} },
			},
		}
	},

	{ "config-set-string", "css", "set a string config value", command_config_set_string,
		{	2,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "key", {} },
				{ cli_parameter_string, 0, 1, 0, 0, "value", {} },
			},
		}
	},

	{ "config-erase", "ce", "erase a config entry", command_config_erase,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "key", {} },
			},
		}
	},


	{ "flash-bench", (const char*)0, "benchmark flash+transport", command_flash_bench,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 1, 1, 1, "length", .unsigned_int = { 0, 4096 }},
			},
		}
	},

	{ "flash-checksum", (const char*)0, "obtain checksum of sectors in flash", command_flash_checksum,
		{	2,
			{
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "start sector", {} },
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "length", {} },
			},
		}
	},

	{ "flash-info", (const char*)0, "show info about flash memory", command_flash_info,
		{}
	},

	{ "flash-read", (const char*)0, "read sectors from flash", command_flash_read,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "sector", {} },
			},
		}
	},

	{ "flash-write", (const char*)0, "write sectors to flash", command_flash_write,
		{	2,
			{
				{ cli_parameter_unsigned_int, 0, 1, 1, 1, "simulate", .unsigned_int = { 0, 1 }},
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "sector", {} },
			}
		}
	},

	{ "fs-read", (const char*)0, "read chunk from a file on the littlefs filesystem", command_fs_read,
		{	3,
			{
				{ cli_parameter_unsigned_int, 0, 1, 1, 1, "length", .unsigned_int = { 0, 4096 }},
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "offset", {} },
				{ cli_parameter_string, 0, 1, 1, 1, "file", .string = { 1, 64 }},
			}
		}
	},

	{ "fs-append", (const char*)0, "append chunk to file on the littlefs filesystem", command_fs_append,
		{	2,
			{
				{ cli_parameter_unsigned_int, 0, 1, 1, 1, "length", .unsigned_int = { 0, 4096 }},
				{ cli_parameter_string, 0, 1, 1, 1, "file", .string = { 1, 64 }},
			}
		}
	},

	{ "fs-checksum", (const char*)0, "checksum file on the littlefs filesystem", command_fs_checksum,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 1, 1, "file", .string = { 1, 64 }},
			}
		}
	},

	{ "fs-erase", (const char*)0, "erase file on the littlefs filesystem", command_fs_erase,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 1, 1, "file", .string = { 1, 64 }},
			}
		}
	},

	{ "fs-format", "fsf", "format the littlefs filesystem", command_fs_format,
		{}
	},

	{ "fs-info", "fsi", "show info about the littlefs filesystem", command_fs_info,
		{}
	},

	{ "fs-list", "ls", "show all files on the littlefs filesystem", command_fs_ls,
		{}
	},

	{ "help", "?", "this help", command_help,
		{	1,
			{
				{ cli_parameter_string, 0, 0, 0, 0, "command to show help about", {} },
			},
		}
	},

	{ "hostname", (const char*)0, "set hostname and description", command_hostname,
		{	2,
			{
				{ cli_parameter_string, 0, 0, 1, 1, "hostname", .string = { 0, 12 }},
				{ cli_parameter_string, 0, 0, 1, 1, "description", .string = { 0, 32 }},
			},
		}
	},

	{ "info", "i", "show some general information", command_info_firmware,
		{}
	},

	{ "info-bt", "ib", "show information about bluetooth", command_info_bluetooth,
		{}
	},

	{ "info-cli", "ic", "show information about the cli", command_info_cli,
		{}
	},

	{ "info-console", "icon", "show information about the console", console_command_info,
		{}
	},

	{ "info-config", "icf", "show information about the configuration", command_info_config,
		{}
	},

	{ "info-flash", "if", "show information about the flash", command_info_flash,
		{}
	},

	{ "info-log", "il", "show information about the log", command_info_log,
		{}
	},

	{ "info-memory", "im", "show information about memory", command_info_memory,
		{}
	},

	{ "info-process", "ip", "show information about running processes", command_info_process,
		{}
	},

	{ "info-system", "is", "show information about the system", command_info_system,
		{}
	},

	{ "info-string", "istr", "show information about all string", command_info_string,
		{}
	},

	{ "log", "l", "show log", command_log,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "start entry", { .unsigned_int = { 0, 128 }} },
			},
		}
	},

	{ "log-clear", "lc", "show log and clear it", command_log_clear,
		{}
	},

	{ "ota-start", (const char*)0, "start ota session", command_ota_start,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "length", {} },
			},
		}
	},

	{ "ota-write", (const char*)0, "write one sector of ota data", command_ota_write,
		{	2,
			{
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "length", {} },
				{ cli_parameter_unsigned_int, 0, 1, 1, 1, "checksum flag", .unsigned_int = { 0, 1 }},
			},
		}
	},

	{ "ota-finish", (const char*)0, "finish ota session", command_ota_finish,
		{}
	},

	{ "ota-commit", (const char*)0, "verify and select finished ota session", command_ota_commit,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 1, 1, "checksum", .string = { 64, 64 }},
			},
		}
	},

	{ "ota-confirm", (const char*)0, "confirm ota image runs correctly", command_ota_confirm,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 1, 1, 1, "slot", .unsigned_int = { 0, 1 }},
			},
		}
	},

	{ "reset", "r", "reset", command_reset,
		{}
	},

	{ "show-config", "sc", "show config", command_config_show,
		{}
	},

	{ "stats", "s", "show some general information", command_info_firmware,
		{}
	},

#if 0
	{ "test", "t", "test parsing", command_test_parse,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "line", {} },
			},
		}
	},
#endif

	{ "wlan-client-config", "wcc", "set wireless ssid and password in client mode", command_wlan_client_config,
		{	2,
			{
				{ cli_parameter_string, 0, 0, 1, 1, "ssid", .string = { 0, 63 }},
				{ cli_parameter_string, 0, 0, 1, 1, "password", .string = { 0, 63 }},
			},
		}
	},

	{ (const char *)0, (const char *)0, (const char *)0, (cli_command_function_t *)0,
		{}
	},
};

static void help(cli_command_call_t *call)
{
	unsigned int command_index, parameter_index;
	const cli_command_t *command;
	const cli_parameter_description_t *parameter;
	const char *delimiter[2];
	string_t command_name;

	string_format(call->result, "help");

	if(call->parameter_count == 0)
		command_name = (string_t)0;
	else
		command_name = call->parameters[0].string;

	for(command_index = 0; cli_commands[command_index].name; command_index++)
	{
		command = &cli_commands[command_index];

		if(command_name && !string_equal_cstr(command_name, command->name) && (!command->alias || !string_equal_cstr(command_name, command->alias)))
			continue;

		string_format_append(call->result, "\n  %-18s %-4s %s", command->name,
				command->alias ? command->alias : "",
				command->help ? command->help : "");

		for(parameter_index = 0; parameter_index < command->parameters_description.count; parameter_index++)
		{
			parameter = &command->parameters_description.entries[parameter_index];

			if(parameter->value_required)
			{
				delimiter[0] = "[";
				delimiter[1] = "]";
			}
			else
			{
				delimiter[0] = "(";
				delimiter[1] = ")";
			}

			string_format_append(call->result, " %s%s %s%s",
					delimiter[0],
					parameter_type_to_string(parameter->type),
					parameter->description ? parameter->description : "",
					delimiter[1]);
		}
	}
}

static void receive_queue_pop(cli_buffer_t *cli_buffer)
{
	assert(inited);

	xQueueReceive(receive_queue_handle, cli_buffer, portMAX_DELAY);
	cli_stats_commands_received++;
}

static void send_queue_push(cli_buffer_t *cli_buffer)
{
	assert(inited);

	xQueueSend(send_queue_handle, cli_buffer, portMAX_DELAY);

	if(cli_buffer->packetised)
		cli_stats_replies_sent_packet++;
	else
		cli_stats_replies_sent_raw++;

	cli_stats_replies_sent++;
}

static void send_queue_pop(cli_buffer_t *cli_buffer)
{
	assert(inited);

	xQueueReceive(send_queue_handle, cli_buffer, portMAX_DELAY);
}

static void run_receive_queue(void *)
{
	cli_buffer_t						cli_buffer;
	string_t							data;
	string_t							oob_data;
	string_t							command;
	string_t							token;
	unsigned int						count, current, ix;
	const cli_command_t					*cli_command;
	unsigned int						parameter_count;
	const cli_parameter_description_t	*parameter_description;
	cli_command_call_t					call;
	cli_parameter_t						*parameter;
	string_auto(error, 128);

	call.parameter_count =		0;
	call.oob =					(string_t)0;
	call.result =				string_new(result_size);
	call.result_oob =			string_new(result_oob_size);

	assert(inited);

	for(;;)
	{
		command = (string_t)0;

		receive_queue_pop(&cli_buffer);
		packet_decapsulate(&cli_buffer, &data, &oob_data);

		if(cli_buffer.packetised)
			cli_stats_commands_received_packet++;
		else
			cli_stats_commands_received_raw++;

		if(cli_buffer.data_from_malloc && cli_buffer.data)
			free(cli_buffer.data);
		cli_buffer.length = 0;
		cli_buffer.data = (uint8_t *)0;
		cli_buffer.data_from_malloc = 0;

		if(!(command = string_parse(data, 0)))
		{
			string_format(error, "ERROR: empty line");
			packet_encapsulate(&cli_buffer, error, (string_t)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		for(ix = 0;; ix++)
		{
			cli_command = &cli_commands[ix];

			if(!cli_command->name || string_equal_cstr(command, cli_command->name))
				break;

			if(cli_command->alias && string_equal_cstr(command, cli_command->alias))
				break;
		}

		if(!cli_command->name)
		{
			string_format(error, "ERROR: unknown command \"%s\"", string_cstr(command)); // FIXME
			packet_encapsulate(&cli_buffer, error, (string_t)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		count = cli_command->parameters_description.count;

		if(count > parameters_size)
			count = parameters_size;

		parameter_count = 0;

		for(current = 0; current < count; current++)
		{
			parameter_description = &cli_command->parameters_description.entries[current];
			parameter = &call.parameters[current];

			parameter->type = cli_parameter_none;
			parameter->has_value = 0;

			if(!(token = string_parse(data, current + 1)))
			{
				if(!parameter_description->value_required)
					continue;
				else
				{
					string_format(error, "ERROR: missing required parameter %u", current + 1);
					packet_encapsulate(&cli_buffer, error, (string_t)0);
					send_queue_push(&cli_buffer);
					goto error;
				}
			}
			else
			{
				parameter_count++;
				
				parameter->string = token;

				switch(parameter_description->type)
				{
					case(cli_parameter_none):
					case(cli_parameter_size):
					{
						string_format(error, "ERROR: parameter with invalid type %u", parameter_description->type);
						packet_encapsulate(&cli_buffer, error, (string_t)0);
						send_queue_push(&cli_buffer);
						goto error;
					}

					case(cli_parameter_unsigned_int):
					{
						unsigned int value;

						if(!string_uint(parameter->string, parameter_description->base, &value))
						{
							string_format(error, "ERROR: invalid unsigned integer value: %s", string_cstr(parameter->string)); // FIXME
							packet_encapsulate(&cli_buffer, error, (string_t)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->lower_bound_required) && (value < parameter_description->unsigned_int.lower_bound))
						{
							string_format(error, "ERROR: invalid unsigned integer value: %u, smaller than lower bound: %u", value, parameter_description->unsigned_int.lower_bound);
							packet_encapsulate(&cli_buffer, error, (string_t)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->upper_bound_required) && (value > parameter_description->unsigned_int.upper_bound))
						{
							string_format(error, "ERROR: invalid unsigned integer value: %u, larger than upper bound: %u", value, parameter_description->unsigned_int.upper_bound);
							packet_encapsulate(&cli_buffer, error, (string_t)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						parameter->type = cli_parameter_unsigned_int;
						parameter->has_value = 1;
						parameter->unsigned_int = value;

						break;
					}

					case(cli_parameter_signed_int):
					{
						int value;

						if(!string_int(parameter->string, parameter_description->base, &value))
						{
							string_format(error, "ERROR: invalid signed integer value: %s", string_cstr(parameter->string));
							packet_encapsulate(&cli_buffer, error, (string_t)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->lower_bound_required) && (value < parameter_description->signed_int.lower_bound))
						{
							string_format(error, "ERROR: invalid signed integer value: %d, smaller than lower bound: %d", value, parameter_description->signed_int.lower_bound);
							packet_encapsulate(&cli_buffer, error, (string_t)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->upper_bound_required) && (value > parameter_description->signed_int.upper_bound))
						{
							string_format(error, "ERROR: invalid signed integer value: %d, larger than upper bound: %d", value, parameter_description->signed_int.upper_bound);
							packet_encapsulate(&cli_buffer, error, (string_t)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						parameter->type = cli_parameter_signed_int;
						parameter->has_value = 1;
						parameter->signed_int = value;

						break;
					}

					case(cli_parameter_float):
					{
						float value;

						if(!string_float(parameter->string, &value))
						{
							string_format(error, "ERROR: invalid float value: %s", string_cstr(parameter->string));
							packet_encapsulate(&cli_buffer, error, (string_t)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->lower_bound_required) && (value < parameter_description->fp.lower_bound))
						{
							string_format(error, "ERROR: invalid float value: %f, smaller than lower bound: %f", value, parameter_description->fp.lower_bound);
							packet_encapsulate(&cli_buffer, error, (string_t)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->upper_bound_required) && (value > parameter_description->fp.upper_bound))
						{
							string_format(error, "ERROR: invalid float value: %f, larger than upper bound: %f", value, parameter_description->fp.upper_bound);
							packet_encapsulate(&cli_buffer, error, (string_t)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						parameter->type = cli_parameter_float;
						parameter->has_value = 1;
						parameter->fp = value;

						break;
					}

					case(cli_parameter_string):
					{
						unsigned int length;

						length = string_length(parameter->string);

						if((parameter_description->lower_bound_required) && (length < parameter_description->string.lower_length_bound))
						{
							string_format(error, "ERROR: invalid string length: %u, smaller than lower bound: %u", length, parameter_description->string.lower_length_bound);
							packet_encapsulate(&cli_buffer, error, (string_t)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->upper_bound_required) && (length > parameter_description->string.upper_length_bound))
						{
							string_format(error, "ERROR: invalid string length: %u, larger than upper bound: %u", length, parameter_description->string.upper_length_bound);
							packet_encapsulate(&cli_buffer, error, (string_t)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						parameter->type = cli_parameter_string;
						parameter->has_value = 1;

						break;
					}
				}
			}
		}

		if(current >= parameters_size)
		{
			string_format(error, "ERROR: too many parameters: %u", current);
			packet_encapsulate(&cli_buffer, error, (string_t)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		if(current < cli_command->parameters_description.count)
		{
			string_format(error, "ERROR: missing parameters");
			packet_encapsulate(&cli_buffer, error, (string_t)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		if((token = string_parse(data, current + 1)))
		{
			string_free(token);
			string_format(error, "ERROR: too many parameters");
			packet_encapsulate(&cli_buffer, error, (string_t)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		call.parameter_count =	parameter_count;
		call.oob =				oob_data;

		string_clear(call.result);
		string_clear(call.result_oob);

		cli_command->function(&call);

		packet_encapsulate(&cli_buffer, call.result, call.result_oob);
		send_queue_push(&cli_buffer);

error:
		if(command)
		{
			string_free(command);
			command = (string_t)0;
		}

		for(ix = 0; ix < call.parameter_count; ix++)
		{
			parameter = &call.parameters[ix];

			if(parameter->string)
			{
				string_free(parameter->string);
				parameter->string = (string_t)0;
			}
		}

		cli_buffer.source = cli_source_none;
		cli_buffer.length = 0;
		cli_buffer.data_from_malloc = 0;
		cli_buffer.data = (uint8_t *)0;

		if(data)
		{
			string_free(data);
			data = (string_t)0;
		}

		if(oob_data)
		{
			string_free(oob_data);
			oob_data = (string_t)0;
		}
	}
}

static void run_send_queue(void *)
{
	cli_buffer_t cli_buffer;

	assert(inited);

	for(;;)
	{
		send_queue_pop(&cli_buffer);

		switch(cli_buffer.source)
		{
			case(cli_source_none):
			{
				log_format("cli: invalid source type: %u", cli_buffer.source);
				break;
			}

			case(cli_source_bt):
			{
				bt_send(&cli_buffer);

				break;
			}

			case(cli_source_station):
			{
				break;
			}

			case(cli_source_ap):
			{
				break;
			}

			case(cli_source_console):
			{
				console_send(&cli_buffer);

				break;
			}
		}

		if(cli_buffer.data_from_malloc && cli_buffer.data)
			free(cli_buffer.data);
		cli_buffer.source = cli_source_none;
		cli_buffer.length = 0;
		cli_buffer.data_from_malloc = 0;
		cli_buffer.data = (uint8_t *)0;
	}
}

void cli_receive_queue_push(const cli_buffer_t *buffer)
{
	assert(inited);

	xQueueSend(receive_queue_handle, buffer, portMAX_DELAY);
}

void cli_init(void)
{
	assert(!inited);

	cli_buffer_t *queue_data;
	StaticQueue_t *queue;

	queue_data = heap_caps_malloc(receive_queue_size * sizeof(cli_buffer_t) , MALLOC_CAP_SPIRAM);
	assert(queue_data);

	queue = heap_caps_malloc(sizeof(StaticQueue_t), MALLOC_CAP_SPIRAM);
	assert(queue);

	if(!(receive_queue_handle = xQueueCreateStatic(receive_queue_size, sizeof(cli_buffer_t), (void *)queue_data, queue)))
		util_abort("cli: xQueueCreateStatic receive queue init");

	queue_data = heap_caps_malloc(send_queue_size * sizeof(cli_buffer_t) , MALLOC_CAP_SPIRAM);
	assert(queue_data);

	queue = heap_caps_malloc(sizeof(StaticQueue_t), MALLOC_CAP_SPIRAM);
	assert(queue);

	if(!(send_queue_handle = xQueueCreateStatic(send_queue_size, sizeof(cli_buffer_t), (void *)queue_data, queue)))
		util_abort("cli: xQueueCreateStatic send queue init");

	inited = true;

	if(xTaskCreatePinnedToCore(run_receive_queue, "cli-recv", 4096, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("cli: xTaskCreatePinnedToNode run_receive_queue");

	if(xTaskCreatePinnedToCore(run_send_queue, "cli-send", 4096, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("cli: xTaskCreatePinnedToNode run_send_queue");
}
