#include <stdio.h>
#include <stdbool.h>
#include <errno.h>

#include "string.h"
#include "config.h"
#include "log.h"
#include "util.h"
#include "cli.h"
#include "packet.h"
#include "bt.h"
#include "console.h"
#include "wlan.h"
#include "cli-command.h"
#include "alias.h"

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
		[cli_parameter_none] =			"invalid parameter type",
		[cli_parameter_unsigned_int] =	"u_int",
		[cli_parameter_signed_int] =	"s_int",
		[cli_parameter_float] =			"float",
		[cli_parameter_string] =		"string",
		[cli_parameter_string_raw] =	"raw string",
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

static void command_write(cli_command_call_t *call)
{
	if(call->parameter_count == 1)
		string_assign_string(call->result, call->parameters[0].string);
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
	{ "alias", nullptr, "set alias", command_alias,
		{	2,
			{
				{ cli_parameter_string, 0, 0, 0, 0, "alias", {} },
				{ cli_parameter_string_raw, 0, 0, 0, 0, "substitution text", {} },
			},
		},
	},

	{ "bt-info", "bi", "show information about bluetooth", bluetooth_command_info,
		{}
	},

	{ "config-dump", "cd", "dump all nvs keys", config_command_dump,
		{}
	},

	{ "config-erase", "ce", "erase a config entry", config_command_erase,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "key", {} },
			},
		}
	},

	{ "config-info", "ci", "show information about the configuration", config_command_info,
		{}
	},

	{ "config-set-int", "csi", "set a signed int config value", config_command_set_int,
		{	2,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "key", {} },
				{ cli_parameter_signed_int, 0, 1, 0, 0, "value", {} },
			},
		}
	},

	{ "config-set-uint", "csu", "set an unsigned int config value", config_command_set_uint,
		{	2,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "key", {} },
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "value", {} },
			},
		}
	},

	{ "config-set-string", "css", "set a string config value", config_command_set_string,
		{	2,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "key", {} },
				{ cli_parameter_string, 0, 1, 0, 0, "value", {} },
			},
		}
	},

	{ "config-show", "cs", "show config", config_command_show,
		{}
	},

	{ "console-info", "coni", "show information about the console", console_command_info,
		{}
	},

	{ "display-brightness", "db", "display brightness", command_display_brightness,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 1, 1, 1, "brightness percentage", .unsigned_int = { 0, 100 }},
			}
		}
	},

	{ "display-configure", "dc", "configure display", command_display_configure,
		{	7,
			{
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "display type", .unsigned_int = { 0, 2 }},
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "interface", .unsigned_int = { 0, 1 }},
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "x-size", .unsigned_int = { 16, 1024 }},
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "y-size", .unsigned_int = { 16, 1024 }},
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "flip", .unsigned_int = { 0, 1 }},
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "invert", .unsigned_int = { 0, 1 }},
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "rotate", .unsigned_int = { 0, 1 }},
			}
		}
	},

	{ "display-erase", "de", "erase display configuration", command_display_erase,
		{},
	},

	{ "display-info", "di", "display information", command_display_info,
		{},
	},

	{ "display-page-add-text", "dpat", "add text page to display", command_display_page_add_text,
		{	3,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "page name", {} },
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "timeout", {} },
				{ cli_parameter_string_raw, 0, 1, 0, 0, "text", {} },
			},
		},
	},

	{ "display-page-add-image", "dpai", "add image page to display", command_display_page_add_image,
		{	4,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "page name", {}},
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "timeout", {} },
				{ cli_parameter_string, 0, 1, 0, 0, "filename", {} },
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "file length", {} },
			},
		},
	},

	{ "display-page-remove", "dpr", "remove page from display", command_display_page_remove,
		{
			1,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "page name", {} },
			},
		}
	},

	{ "fs-read", (const char*)0, "read chunk from a file", fs_command_read,
		{	3,
			{
				{ cli_parameter_unsigned_int, 0, 1, 1, 1, "length", .unsigned_int = { 0, 4096 }},
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "offset", {} },
				{ cli_parameter_string, 0, 1, 1, 1, "file", .string = { 1, 64 }},
			}
		}
	},

	{ "fs-checksum", (const char*)0, "checksum file on the littlefs filesystem", fs_command_checksum,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 1, 1, "file", .string = { 1, 64 }},
			}
		}
	},

	{ "fs-erase", (const char*)0, "erase file on the filesystem", fs_command_erase,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 1, 1, "file", .string = { 1, 64 }},
			}
		}
	},

	{ "fs-format", "fsf", "format the littlefs filesystem", fs_command_format,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "partition name of fs to format", {} },
			},
		}
	},

	{ "fs-info", "fsi", "show info about the littlefs filesystem", fs_command_info,
		{}
	},

	{ "fs-list", "ls", "show all files on the littlefs filesystem", fs_command_list,
		{	2,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "directory to list", {} },
				{ cli_parameter_string, 0, 0, 0, 0, "option [-l]", {} },
			},
		}
	},

	{ "fs-write", (const char*)0, "write to a file on the filesystem", fs_command_write,
		{	3,
			{
				{ cli_parameter_unsigned_int, 0, 1, 1, 1, "mode, 0 = truncate, 1 = append", .unsigned_int = { 0, 1 }},
				{ cli_parameter_unsigned_int, 0, 1, 1, 1, "length", .unsigned_int = { 0, 4096 }},
				{ cli_parameter_string, 0, 1, 1, 1, "file", .string = { 1, 64 }},
			}
		}
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

	{ "i2c-info", "i2i", "info about the I2C interfaces", command_i2c_info,
		{},
	},

	{ "info", (const char *)0, "show some generic information", info_command_info,
		{}
	},

	{ "info-board", "ib", "BSP info", info_command_info_board,
		{}
	},

	{ "info-cli", "ic", "show information about the cli", command_info_cli,
		{}
	},

	{ "info-partitions", "ip", "show information about partitions", info_command_info_partitions,
		{}
	},

	{ "info-memory", "im", "show information about memory", info_command_info_memory,
		{}
	},

	{ "io-dump", "iod", "dump everything known about found IOs", command_io_dump,
		{}
	},

	{ "io-read", "ior", "read from I/O pin", command_io_read,
		{	2,
			{
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "I/O id", {} },
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "pin", {} },
			},
		}
	},

	{ "io-stats", "ios", "statistics about IOs", command_io_stats,
		{}
	},

	{ "io-write", "iow", "write to I/O pin", command_io_write,
		{	3,
			{
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "I/O id", {} },
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "pin", {} },
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "value", {} },
			},
		}
	},

	{ "ipv6-slaac", "ip6sl", "enable force slaac address retrieval", wlan_command_ipv6_slaac,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "address", .unsigned_int = { 0, 1 }},
			},
		}
	},

	{ "ipv6-static", "ip6st", "set ipv6 static address", wlan_command_ipv6_static,
		{	1,
			{
				{ cli_parameter_string, 0, 0, 1, 1, "address", .string = { 0, 64 }},
			},
		}
	},

	{ "ledpixel-info", "lpxi", "info about LEDpixels channels", command_ledpixel_info,
		{}
	},

	{ "ledpwm-info", "lpi", "info about LED PWM channels and timers", command_ledpwm_info,
		{}
	},

	{ "log", "l", "show log", log_command_log,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "start entry", { .unsigned_int = { 0, 128 }} },
			},
		}
	},

	{ "log-clear", "lc", "show log and clear it", log_command_log_clear,
		{}
	},

	{ "log-info", "li", "show information about the log", log_command_info,
		{}
	},

	{ "log-monitor", "lm", "enable/disable output log to console", log_command_log_monitor,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "activate", { .unsigned_int = { 0, 1 }} },
			},
		},
	},

	{ "mcpwm-info", "mpi", "info about MCPWM channels and timers", command_mcpwm_info,
		{}
	},

	{ "ota-commit", (const char*)0, "verify and select finished ota session", command_ota_commit,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 1, 1, "checksum", { .string = { 64, 64 }} },
			},
		}
	},

	{ "ota-confirm", (const char*)0, "confirm ota image runs correctly", command_ota_confirm,
		{}
	},

	{ "ota-finish", (const char*)0, "finish ota session", command_ota_finish,
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

	{ "pdm-info", "pin", "info about pdm channels", command_pdm_info,
		{}
	},

	{ "process-list", "ps", "show information about running processes", command_process_list,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "core id", .unsigned_int = { 0, 1 }},
			}
		},
	},

	{ "process-stop", "kill", "stop running process", command_process_kill,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "process", {} },
			}
		},
	},

	{ "reset", "r", "reset", command_reset,
		{}
	},

	{ "run", nullptr, "run a script", command_run,
		{	5,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "script name", {} },
				{ cli_parameter_string, 0, 0, 0, 0, "parameter 1", {} },
				{ cli_parameter_string, 0, 0, 0, 0, "parameter 2", {} },
				{ cli_parameter_string, 0, 0, 0, 0, "parameter 3", {} },
				{ cli_parameter_string, 0, 0, 0, 0, "parameter 4", {} },
			},
		},
	},

	{ "sensor-dump", "sd", "dump registered sensors", command_sensor_dump,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 0, 1, 0, "sensor index to dump", .unsigned_int = { 0, 0 } },
			},
		},
	},

	{ "sensor-info", "si", "info about registered sensors", command_sensor_info,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "include disabled devices", .unsigned_int = { 0, 1 }},
			}
		},
	},

	{ "sensor-json", "sj", "sensors values in json layout", command_sensor_json,
		{},
	},

	{ "sensor-stats", "ss", "sensors statistics", command_sensor_stats,
		{},
	},

	{ "string-info", "sti", "show information about all strings", string_command_info,
		{}
	},

	{ "wlan-client-config", "wcc", "set wireless ssid and password in client mode", wlan_command_client_config,
		{	2,
			{
				{ cli_parameter_string, 0, 0, 1, 1, "ssid", .string = { 0, 63 }},
				{ cli_parameter_string, 0, 0, 1, 1, "password", .string = { 0, 63 }},
			},
		}
	},

	{ "wlan-info", "wi", "show information about wlan", wlan_command_info,
		{}
	},

	{ "wlan-ip-info", "wii", "show information about tcp/udp over wlan", wlan_command_ip_info,
		{}
	},

	{ "write", "w", "write to output", command_write,
		{	1,
			{
				{ cli_parameter_string_raw, 0, 1, 0, 0, "text", {} },
			},
		},
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

	string_format(call->result, "HELP");

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


		if(command_name)
		{
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
}

static void receive_queue_pop(cli_buffer_t *cli_buffer)
{
	assert(inited);
	assert(cli_buffer);
	assert(receive_queue_handle);

	xQueueReceive(receive_queue_handle, cli_buffer, portMAX_DELAY);

	assert(cli_buffer->magic_number_head == cli_buffer_magic_number_head);
	assert(cli_buffer->magic_number_tail == cli_buffer_magic_number_tail);

	cli_stats_commands_received++;
}

static void send_queue_push(cli_buffer_t *cli_buffer)
{
	assert(inited);
	assert(cli_buffer);
	assert(send_queue_handle);

	cli_buffer->magic_number_head = cli_buffer_magic_number_head;
	cli_buffer->magic_number_tail = cli_buffer_magic_number_tail;

	xQueueSendToBack(send_queue_handle, cli_buffer, portMAX_DELAY);

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

	assert(cli_buffer->magic_number_head == cli_buffer_magic_number_head);
	assert(cli_buffer->magic_number_tail == cli_buffer_magic_number_tail);
}

static void run_receive_queue(void *)
{
	cli_buffer_t						cli_buffer;
	string_t							data = (string_t)0;
	string_t							oob_data = (string_t)0;
	string_t							command = (string_t)0;
	string_t							token = (string_t)0;
	unsigned int						count, current, ix, string_parse_offset, previous_string_parse_offset;
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
		string_free(&cli_buffer.data);

		if(cli_buffer.packetised)
			cli_stats_commands_received_packet++;
		else
			cli_stats_commands_received_raw++;

		alias_expand(&data);

		string_parse_offset = 0;
		previous_string_parse_offset = 0;

		if(!(command = string_parse(data, &string_parse_offset)))
		{
			string_format(error, "ERROR: empty line");
			packet_encapsulate(&cli_buffer, error, (string_t)0);
			send_queue_push(&cli_buffer);
			goto error1;
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
			string_format(error, "ERROR: unknown command \"%s\"", string_cstr(command));
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

			previous_string_parse_offset = string_parse_offset;

			if(!(token = string_parse(data, &string_parse_offset)))
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
							string_format(error, "ERROR: invalid unsigned integer value: %s", string_cstr(parameter->string));
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
							string_format(error, "ERROR: invalid float value: %f, smaller than lower bound: %f", (double)value, (double)parameter_description->fp.lower_bound);
							packet_encapsulate(&cli_buffer, error, (string_t)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->upper_bound_required) && (value > parameter_description->fp.upper_bound))
						{
							string_format(error, "ERROR: invalid float value: %f, larger than upper bound: %f", (double)value, (double)parameter_description->fp.upper_bound);
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

					case(cli_parameter_string_raw):
					{
						unsigned int length;

						string_free(&parameter->string);
						parameter->string = string_new(string_length(data));

						while((previous_string_parse_offset < string_length(data)) && (string_at(data, previous_string_parse_offset) == ' '))
							previous_string_parse_offset++;

						string_cut(parameter->string, data, previous_string_parse_offset, ~0);
						string_parse_offset = string_length(data);

						length = string_length(parameter->string);

						if((parameter_description->lower_bound_required) && (length < parameter_description->string.lower_length_bound))
						{
							string_format(error, "ERROR: invalid raw string length: %u, smaller than lower bound: %u", length, parameter_description->string.lower_length_bound);
							packet_encapsulate(&cli_buffer, error, (string_t)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->upper_bound_required) && (length > parameter_description->string.upper_length_bound))
						{
							string_format(error, "ERROR: invalid raw string length: %u, larger than upper bound: %u", length, parameter_description->string.upper_length_bound);
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

		if((token = string_parse(data, &string_parse_offset)))
		{
			string_free(&token);
			string_format(error, "ERROR: too many parameters");
			packet_encapsulate(&cli_buffer, error, (string_t)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		call.source =			cli_buffer.source;
		call.mtu =				cli_buffer.mtu;
		call.parameter_count =	parameter_count;
		call.oob =				oob_data;

		string_clear(call.result);
		string_clear(call.result_oob);

		cli_command->function(&call);

		packet_encapsulate(&cli_buffer, call.result, call.result_oob);

		send_queue_push(&cli_buffer);

error:
		string_free(&command);
error1:
		for(ix = 0; ix < call.parameter_count; ix++)
		{
			parameter = &call.parameters[ix];

			if(parameter->string)
				string_free(&parameter->string);
		}

		cli_buffer.source = cli_source_none;
		string_free(&data);

		if(oob_data)
			string_free(&oob_data);
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
			case(cli_source_bt):
			{
				bt_send(&cli_buffer);
				break;
			}

			case(cli_source_console):
			{
				console_send(&cli_buffer);
				break;
			}

			case(cli_source_wlan_tcp):
			{
				wlan_tcp_send(&cli_buffer);
				break;
			}

			case(cli_source_wlan_udp):
			{
				wlan_udp_send(&cli_buffer);
				break;
			}

			case(cli_source_script):
			{
				if(string_at_back(cli_buffer.data) == '\n')
					string_pop_back(cli_buffer.data);

				if(string_length(cli_buffer.data) > 0)
					log_format("%s: %s", cli_buffer.script.name, string_cstr(cli_buffer.data));

				break;
			}

			default:
			{
				log_format("cli: invalid source type: %u", cli_buffer.source);
				break;
			}
		}

		string_free(&cli_buffer.data);

		if(cli_buffer.source == cli_source_script)
		{
			assert(cli_buffer.script.task);
			xTaskNotifyGive(cli_buffer.script.task);
			cli_buffer.script.name[0] = '\0';
			cli_buffer.script.task = nullptr;
		}

		cli_buffer.source = cli_source_none;
	}
}

void cli_receive_queue_push(cli_buffer_t *buffer)
{
	assert(inited);

	buffer->magic_number_head = cli_buffer_magic_number_head;
	buffer->magic_number_tail = cli_buffer_magic_number_tail;

	xQueueSendToBack(receive_queue_handle, buffer, portMAX_DELAY);
}

void cli_init(void)
{
	assert(!inited);

	cli_buffer_t *queue_data;
	StaticQueue_t *queue;

	queue_data = util_memory_alloc_spiram(receive_queue_size * sizeof(cli_buffer_t));
	queue = util_memory_alloc_spiram(sizeof(StaticQueue_t));

	if(!(receive_queue_handle = xQueueCreateStatic(receive_queue_size, sizeof(cli_buffer_t), (void *)queue_data, queue)))
		util_abort("cli: xQueueCreateStatic receive queue init");

	queue_data = util_memory_alloc_spiram(send_queue_size * sizeof(cli_buffer_t));
	queue = util_memory_alloc_spiram(sizeof(StaticQueue_t));

	if(!(send_queue_handle = xQueueCreateStatic(send_queue_size, sizeof(cli_buffer_t), (void *)queue_data, queue)))
		util_abort("cli: xQueueCreateStatic send queue init");

	inited = true;

	if(xTaskCreatePinnedToCore(run_receive_queue, "cli-recv", 4096, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("cli: xTaskCreatePinnedToNode run_receive_queue");

	if(xTaskCreatePinnedToCore(run_send_queue, "cli-send", 4096, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("cli: xTaskCreatePinnedToNode run_send_queue");
}
