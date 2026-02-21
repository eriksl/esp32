#include "command.h"

#include "util.h"
#include "config.h"
#include "log.h"
#include "console.h"
#include "wlan.h"
#include "i2c.h"
#include "bt.h"
#include "udp.h"
#include "tcp.h"
#include "bt.h"
#include "exception.h"
#include "packet.h"

#include "log.h"
#include "command.h"
#include "cli-command.h"

#include <format>
#include <thread>
#include <map>
#include <algorithm>

#include <esp_pthread.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

//FIXME
void flash_command_bench(cli_command_call_t *call);
void flash_command_checksum(cli_command_call_t *call);
void flash_command_info(cli_command_call_t *call);
void flash_command_read(cli_command_call_t *call);
void flash_command_write(cli_command_call_t *call);
void command_ota_start(cli_command_call_t *call);
void command_ota_write(cli_command_call_t *call);
void command_ota_finish(cli_command_call_t *call);
void command_ota_commit(cli_command_call_t *call);
void command_ota_confirm(cli_command_call_t *call);
void command_display_brightness(cli_command_call_t *call);
void command_display_configure(cli_command_call_t *call);
void command_display_erase(cli_command_call_t *call);
void command_display_info(cli_command_call_t *call);
void command_display_page_add_text(cli_command_call_t *call);
void command_display_page_add_image(cli_command_call_t *call);
void command_display_page_remove(cli_command_call_t *call);
void command_i2c_info(cli_command_call_t *call);
void command_i2c_speed(cli_command_call_t *call);
void command_sensor_dump(cli_command_call_t *call);
void command_sensor_info(cli_command_call_t *call);
void command_sensor_json(cli_command_call_t *call);
void command_sensor_stats(cli_command_call_t *call);
void command_io_dump(cli_command_call_t *call);
void command_io_read(cli_command_call_t *call);
void command_io_stats(cli_command_call_t *call);
void command_io_write(cli_command_call_t *call);
void command_alias(cli_command_call_t *call);
void command_run(cli_command_call_t *call);
void net_tcp_command_info(cli_command_call_t *call);

//FIXME
int Command::cli_stats_commands_received = 0;
int Command::cli_stats_commands_received_packet = 0;
int Command::cli_stats_commands_received_raw = 0;
int Command::cli_stats_replies_sent = 0;
int Command::cli_stats_replies_sent_packet = 0;
int Command::cli_stats_replies_sent_raw = 0;

const std::map<cli_parameter_type_description_t, std::string> Command::parameter_type_to_string
{
	{	cli_parameter_none,			"invalid parameter type" },
	{	cli_parameter_unsigned_int,	"u_int" },
	{	cli_parameter_signed_int,	"s_int" },
	{	cli_parameter_float,		"float" },
	{	cli_parameter_string,		"string" },
	{	cli_parameter_string_raw,	"raw string" },
};

const Command::cli_command_t Command::cli_commands[] =
{
	{ "alias", nullptr, "set alias", Command::alias,
		{	2,
			{
				{ cli_parameter_string, 0, 0, 0, 0, "alias", {}},
				{ cli_parameter_string_raw, 0, 0, 0, 0, "substitution text", {}},
			},
		},
	},

	{ "bt-info", "bi", "show information about bluetooth", Command::bluetooth_info, {}},

	{ "bt-key", "bk", "show or set bluetooth encryption key", Command::bluetooth_key,
		{	1,
			{
				{ cli_parameter_string, 0, 0, 0, 0, "key", {}},
			},
		}
	},

	{ "command-info", "comi", "show info about command processing", Command::info, {}},
	{ "config-dump", "cd", "dump all nvs keys", Command::config_dump, {}},

	{ "config-erase", "ce", "erase a config entry", Command::config_erase,
		{	2,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "key", {}},
				{ cli_parameter_string, 0, 0, 0, 0, "namespace", {}},
			},
		}
	},

	{ "config-info", "ci", "show information about the configuration", Command::config_info, {}},

	{ "config-set-int", "csi", "set a signed int config value", Command::config_set_int,
		{	2,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "key", {}},
				{ cli_parameter_signed_int, 0, 1, 0, 0, "value", {}},
			},
		}
	},

	{ "config-set-string", "css", "set a string config value", Command::config_set_string,
		{	2,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "key", {}},
				{ cli_parameter_string, 0, 1, 0, 0, "value", {}},
			},
		}
	},

	{ "config-show", "cs", "show config", Command::config_show, {}},
	{ "console-info", "coni", "show information about the console", Command::console_info, {}},

	{ "display-brightness", "db", "display brightness", Command::display_brightness,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 1, 1, 1, "brightness percentage", { 0, 100 }},
			},
		}
	},

	{ "display-configure", "dc", "configure display", Command::display_configure,
		{	7,
			{
				cli_parameter_unsigned_int, 0, 0, 1, 1, "display type", { .unsigned_int = { 0, 2 }},
				cli_parameter_unsigned_int, 0, 0, 1, 1, "interface", { .unsigned_int = { 0, 1 }},
				cli_parameter_unsigned_int, 0, 0, 1, 1, "x-size", { .unsigned_int = { 16, 1024 }},
				cli_parameter_unsigned_int, 0, 0, 1, 1, "y-size", { .unsigned_int = { 16, 1024 }},
				cli_parameter_unsigned_int, 0, 0, 1, 1, "flip", { .unsigned_int = { 0, 1 }},
				cli_parameter_unsigned_int, 0, 0, 1, 1, "invert", { .unsigned_int = { 0, 1 }},
				cli_parameter_unsigned_int, 0, 0, 1, 1, "rotate", { .unsigned_int = { 0, 1 }},
			}
		}
	},

	{ "display-erase", "de", "erase display configuration", Command::display_erase, {}},
	{ "display-info", "di", "display information", Command::display_info, {}},

	{ "display-page-add-text", "dpat", "add text page to display", Command::display_page_add_text,
		{	3,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "page name", {}},
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "timeout", {}},
				{ cli_parameter_string_raw, 0, 1, 0, 0, "text", {}},
			},
		},
	},

	{ "display-page-add-image", "dpai", "add image page to display", Command::display_page_add_image,
		{	4,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "page name", {}},
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "timeout", {}},
				{ cli_parameter_string, 0, 1, 0, 0, "filename", {}},
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "file length", {}},
			},
		},
	},

	{ "display-page-remove", "dpr", "remove page from display", Command::display_page_remove,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "page name", {}},
			},
		}
	},

	{ "fs-read", nullptr, "read chunk from a file", Command::fs_read,
		{	3,
			{
				{ cli_parameter_unsigned_int, 0, 1, 1, 1, "length",  { .unsigned_int = { 0, 32768 }}},
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "offset", {}},
				{ cli_parameter_string, 0, 1, 1, 1, "file", { .string = { 1, 64 }}},
			}
		}
	},

	{ "fs-checksum", nullptr, "checksum a file", Command::fs_checksum,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 1, 1, "file", { .string = { 1, 64 }}},
			}
		}
	},

	{ "fs-erase", "rm", "erase file", Command::fs_erase,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 1, 1, "file", { .string = { 1, 64 }}},
			}
		}
	},

	{ "fs-format", nullptr, "format/wipe a filesystem", Command::fs_format,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "partition name of fs to format", {}},
			},
		}
	},

	{ "fs-info", "fsi", "show info about the filesystems", Command::fs_info, {}},

	{ "fs-list", "ls", "show the files on a filesystem", Command::fs_list,
		{	2,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "directory to list", {}},
				{ cli_parameter_string, 0, 0, 0, 0, "option [-l]", {}},
			},
		}
	},

	{ "fs-rename", "mv", "rename file", Command::fs_rename,
		{	2,
			{
				{ cli_parameter_string, 0, 1, 1, 1, "from file", { .string = { 1, 64 }}},
				{ cli_parameter_string, 0, 1, 1, 1, "to file", { .string = { 1, 64 }}},
			}
		}
	},

	{ "fs-truncate", nullptr, "truncate a file", Command::fs_truncate,
		{	3,
			{
				{ cli_parameter_string,			0, 1, 1, 1, "file", { .string = { 1, 64 }}},
				{ cli_parameter_unsigned_int,	0, 1, 0, 0, "length", {}},
			}
		}
	},

	{ "fs-write", nullptr, "write to a file", Command::fs_write,
		{	3,
			{
				{ cli_parameter_unsigned_int, 0, 1, 1, 1, "mode, 0 = truncate, 1 = append", { .unsigned_int = { 0, 1 }}},
				{ cli_parameter_unsigned_int, 0, 1, 1, 1, "length", { .unsigned_int = { 0, 32768 }}},
				{ cli_parameter_string, 0, 1, 1, 1, "file", { .string = { 1, 64 }}},
			}
		}
	},

	{ "help", "?", "this help", Command::command_help,
		{	1,
			{
				{ cli_parameter_string, 0, 0, 0, 0, "command to show help about", {}},
			},
		}
	},

	{ "hostname", (const char*)0, "set hostname and description", Command::hostname,
		{	2,
			{
				{ cli_parameter_string, 0, 0, 1, 1, "hostname", { .string = { 0, 12 }}},
				{ cli_parameter_string, 0, 0, 1, 1, "description", { .string = { 0, 32 }}},
			},
		}
	},

	{ "i2c-info", "i2i", "info about the I2C interfaces", Command::i2c_info, {}},

	{ "i2c-speed", "i2s", "set speed of I2C interface", Command::i2c_speed,
		{	2,
			{
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "module", { .unsigned_int = { i2c_module_first, i2c_module_last }}},
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "speed in kHz", { .unsigned_int = { 0, 500 }}},
			},
		}
	},

	{ "info", (const char *)0, "show some generic information", Command::system_info, {}},
	{ "info-board", "ib", "BSP info", Command::system_identify, {}},
	{ "info-partitions", "ip", "show information about partitions", Command::system_partitions,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "partition #", { .unsigned_int = { 0, 16 }}},
			},
		}
	},
	{ "info-memory", "im", "show information about memory", Command::system_memory, {}},
	{ "io-dump", "iod", "dump everything known about found IOs", Command::io_dump, {}},

	{ "io-read", "ior", "read from I/O pin", Command::io_read,
		{	2,
			{
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "I/O id", {}},
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "pin", {}},
			},
		}
	},

	{ "io-stats", "ios", "statistics about IOs", Command::io_stats, {}},

	{ "io-write", "iow", "write to I/O pin", Command::io_write,
		{	3,
			{
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "I/O id", {}},
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "pin", {}},
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "value", {}},
			},
		}
	},

	{ "ipv6-static", "ip6st", "set ipv6 static address", Command::wlan_ipv6_static,
		{	1,
			{
				{ cli_parameter_string, 0, 0, 1, 1, "address", { .string = { 0, 64 }}},
			},
		}
	},

	{ "ledpixel-info", "lpxi", "info about LEDpixels channels", Command::ledpixel_info, {}},
	{ "ledpwm-info", "lpi", "info about LED PWM channels and timers", Command::ledpwm_info, {}},

	{ "log", "l", "show log", Command::log_log,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "start entry", { .unsigned_int = { 0, 128 }}},
			},
		}
	},

	{ "log-clear", "lc", "show log and clear it", Command::log_clear, {}},
	{ "log-info", "li", "show information about the log", Command::log_info, {}},

	{ "log-monitor", "lm", "enable/disable output log to console", Command::log_monitor,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "activate", { .unsigned_int = { 0, 1 }}},
			},
		},
	},

	{ "mcpwm-info", "mpi", "info about MCPWM channels and timers", Command::mcpwm_info, {}},

	{ "notify-info", "ni", "notification system info", Command::notify_info, {}},

	{ "ota-commit", (const char*)0, "verify and select finished ota session", Command::ota_commit,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 1, 1, "checksum", { .string = { 64, 64 }}},
			},
		}
	},

	{ "ota-confirm", (const char*)0, "confirm ota image runs correctly", Command::ota_confirm, {}},
	{ "ota-finish", (const char*)0, "finish ota session", Command::ota_finish, {}},

	{ "ota-start", (const char*)0, "start ota session", Command::ota_start,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "length", {}},
			},
		}
	},

	{ "ota-write", (const char*)0, "write one sector of ota data", Command::ota_write,
		{	2,
			{
				{ cli_parameter_unsigned_int, 0, 1, 0, 0, "length", {}},
				{ cli_parameter_unsigned_int, 0, 1, 1, 1, "checksum flag", { .unsigned_int = { 0, 1 }}},
			},
		}
	},

	{ "pdm-info", "pin", "info about pdm channels", Command::pdm_info, {}},

	{ "process-list", "ps", "show information about running processes", Command::system_process_list,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "core id", { .unsigned_int = { 0, 1 }}},
			}
		},
	},

	{ "process-stop", "kill", "stop running process", Command::system_process_stop,
		{	1,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "process name or id", {}},
			}
		},
	},

	{ "reset", "r", "reset", Command::reset, {}},

	{ "run", nullptr, "run a script", Command::run,
		{	5,
			{
				{ cli_parameter_string, 0, 1, 0, 0, "script name", {}},
				{ cli_parameter_string, 0, 0, 0, 0, "parameter 1", {}},
				{ cli_parameter_string, 0, 0, 0, 0, "parameter 2", {}},
				{ cli_parameter_string, 0, 0, 0, 0, "parameter 3", {}},
				{ cli_parameter_string, 0, 0, 0, 0, "parameter 4", {}},
			},
		},
	},

	{ "sensor-dump", "sd", "dump registered sensors", Command::sensor_dump,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 0, 1, 0, "sensor index to dump", { .unsigned_int = { 0, 0 }}},
			},
		},
	},

	{ "sensor-info", "si", "info about registered sensors", Command::sensor_info,
		{	1,
			{
				{ cli_parameter_unsigned_int, 0, 0, 1, 1, "include disabled devices", { .unsigned_int = { 0, 1 }}},
			}
		},
	},

	{ "sensor-json", "sj", "sensors values in json layout", Command::sensor_json, {}},
	{ "sensor-stats", "ss", "sensors statistics", Command::sensor_stats, {}},
	{ "tcp-info", "ti", "show information about tcp", Command::tcp_info, {}},
	{ "udp-info", "ui", "show information about udp", Command::udp_info, {}},
	{ "util-info", "uti", "show information about utils the module", Command::util_info, {}},
	{ "util-tz", "ut", "get or set the timezone for date representation", Command::util_timezone,
		{	1,
			{
				{ cli_parameter_string, 0, 0, 0, 0, "timezone", {}},
			},
		},
	},
	{ "wlan-client-config", "wcc", "set wireless ssid and password in client mode", Command::wlan_client_config,
		{	2,
			{
				{ cli_parameter_string, 0, 0, 1, 1, "ssid", { .string = { 0, 63 }}},
				{ cli_parameter_string, 0, 0, 1, 1, "password", { .string = { 0, 63 }}},
			},
		}
	},

	{ "wlan-info", "wi", "show information about wlan", Command::wlan_info, {}},

	{ "write", "w", "write to output", Command::write,
		{	1,
			{
				{ cli_parameter_string_raw, 0, 1, 0, 0, "text", {}},
			},
		},
	},
	{ nullptr, nullptr, nullptr, nullptr, {} },
};

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
MCPWM *Command::mcpwm_ = nullptr;
FS *Command::fs_ = nullptr;
BT *Command::bt_ = nullptr;
WLAN *Command::wlan_ = nullptr;
UDP *Command::udp_ = nullptr;

Command::Command(Config &config_in, Console &console_in, Ledpixel &ledpixel_in, LedPWM &ledpwm_in,
		Notify &notify_in, Log &log_in, System &system_in, Util &util_in, PDM &pdm_in, MCPWM &mcpwm_in,
		FS &fs_in, BT &bt_in, WLAN &wlan_in, UDP &udp_in)
	:
		config(config_in), console(console_in), ledpixel(ledpixel_in), ledpwm(ledpwm_in),
		notify(notify_in), log(log_in), system(system_in), util(util_in), pdm(pdm_in), mcpwm(mcpwm_in),
		fs(fs_in), bt(bt_in), wlan(wlan_in), udp(udp_in)
{
	if(this->singleton)
		throw(hard_exception("Command: already activated"));

	if(!(receive_queue_handle = xQueueCreate(receive_queue_size, sizeof(command_response_t *))))
		throw(hard_exception("Command: xQueueCreateStatic receive queue init failed"));

	if(!(send_queue_handle = xQueueCreate(send_queue_size, sizeof(command_response_t *))))
		throw(hard_exception("Command: xQueueCreateStatic send queue init failed"));

	this->running = false;

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
	this->mcpwm_ = &mcpwm;
	this->fs_ = &fs;
	this->bt_ = &bt;
	this->wlan_ = &wlan;
	this->udp_ = &udp;
}

Command &Command::get()
{
	if(!Command::singleton)
		throw(hard_exception("Command::get: not active"));

	return(*Command::singleton);
}

void Command::run()
{
	esp_pthread_cfg_t thread_config;

	if(this->running)
		throw(hard_exception("Command::run: already running"));

	thread_config = esp_pthread_get_default_config();
	thread_config.thread_name = "cmd recv";
	thread_config.pin_to_core = 1;
	thread_config.stack_size = 6 * 1024;
	thread_config.prio = 1;
	//thread_config.stack_alloc_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
	esp_pthread_set_cfg(&thread_config);

	std::thread receive_thread(run_receive_queue_wrapper, this);

	receive_thread.detach();

	thread_config = esp_pthread_get_default_config();
	thread_config.thread_name = "cmd send";
	thread_config.pin_to_core = 1;
	thread_config.stack_size = 3 * 1024;
	thread_config.prio = 1;
	//thread_config.stack_alloc_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
	esp_pthread_set_cfg(&thread_config);

	std::thread send_thread(run_send_queue_wrapper, this);

	send_thread.detach();
}

void Command::help(std::string &out, const std::string &filter)
{
	unsigned int command_index, parameter_index;
	const cli_command_t *command;
	const cli_parameter_description_t *parameter;
	std::string delimiter[2];

	for(command_index = 0; cli_commands[command_index].name; command_index++)
	{
		command = &cli_commands[command_index];

		if(filter.length() && (filter != command->name) && (!command->alias || (filter != command->alias)))
			continue;

		out += std::format("\n  {:<18s} {:<4s} {}", command->name, command->alias ? command->alias : "", command->help ? command->help : "");

		if(filter.length())
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

				out += std::format(" {}{} {}{}", delimiter[0], this->parameter_type_to_string.at(parameter->type),
						parameter->description ? parameter->description : "", delimiter[1]);
			}
		}
	}
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

	call->result = std::format("log monitor: {}", util_->yesno(monitor));
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

void Command::mcpwm_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "MCPWM INFO\n";

	Command::mcpwm_->info(call->result);
}

void Command::fs_list(cli_command_call_t *call)
{
	bool option_long;

	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	if(call->parameter_count == 2)
	{
		if(call->parameters[1].str == "-l")
			option_long = true;
		else
		{
			call->result = std::format("FS::fs-list: unknown option: {}\n", call->parameters[1].str);
			return;
		}
	}
	else
		option_long = false;

	call->result = std::format("DIRECTORY {}", call->parameters[0].str);

	try
	{
		Command::fs_->list(call->result, call->parameters[0].str, option_long);
	}
	catch(const transient_exception &e)
	{
		call->result = std::format("fs-list: {}", e.what());
	}
}

void Command::fs_format(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	try
	{
		Command::fs_->format(call->parameters[0].str);
	}
	catch(const transient_exception &e)
	{
		call->result = std::format("fs-format: {}", e.what());
		return;
	}

	call->result = std::format("format {} OK", call->parameters[0].str);
}

void Command::fs_read(cli_command_call_t *call)
{
	int length;

	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	try
	{
		length = Command::fs_->read(call->result_oob, call->parameters[2].str, call->parameters[1].unsigned_int, call->parameters[0].unsigned_int);
	}
	catch(const transient_exception &e)
	{
		call->result = std::format("fs-read: {}", e.what());
		return;
	}

	call->result = std::format("OK chunk read: {:d}", length);
}

void Command::fs_write(cli_command_call_t *call)
{
	int length;

	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	try
	{
		length = Command::fs_->write(call->oob, call->parameters[2].str, !!call->parameters[0].unsigned_int, call->parameters[1].unsigned_int);
	}
	catch(const transient_exception &e)
	{
		call->result = std::format("fs-write: {}", e.what());
		return;
	}

	call->result = std::format("OK file length: {:d}", length);
}

void Command::fs_erase(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	try
	{
		Command::fs_->erase(call->parameters[0].str);
	}
	catch(const transient_exception &e)
	{
		call->result = std::format("fs-erase: {}", e.what());
		return;
	}

	call->result = "OK file erased";
}

void Command::fs_rename(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	try
	{
		Command::fs_->rename(call->parameters[0].str, call->parameters[1].str);
	}
	catch(const transient_exception &e)
	{
		call->result = std::format("fs-rename: {}", e.what());
		return;
	}

	call->result = "OK file renamed";
}

void Command::fs_truncate(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	try
	{
		Command::fs_->truncate(call->parameters[0].str, call->parameters[1].unsigned_int);
	}
	catch(const transient_exception &e)
	{
		call->result = std::format("fs-truncate: {}", e.what());
		return;
	}

	call->result = "OK truncated";
}

void Command::fs_checksum(cli_command_call_t *call)
{
	std::string checksum;

	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	try
	{
		checksum = Command::fs_->checksum(call->parameters[0].str);
	}
	catch(const transient_exception &e)
	{
		call->result = std::format("fs-checksum: {}", e.what());
		return;
	}

	call->result = std::format("OK checksum: {}", checksum);
}

void Command::fs_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "FS INFO\n";

	Command::fs_->info(call->result);
}

void Command::command_help(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "HELP";

	if(call->parameter_count == 2)
		singleton->help(call->result, call->parameters[1].str);
	else
		singleton->help(call->result);

}

void Command::hostname(cli_command_call_t *call)
{
	std::string hostname;
	std::string description;

	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	if(call->parameter_count > 1)
	{
		description = call->parameters[1].str;
		std::replace(description.begin(), description.end(), '_', ' ');
		config_->set_string("hostname_desc", description);
	}

	if(call->parameter_count > 0)
		config_->set_string("hostname", call->parameters[0].str);

	try
	{
		hostname = config_->get_string("hostname");
	}
	catch(const transient_exception &)
	{
		hostname = "<unset>";
	}

	try
	{
		description = config_->get_string("hostname_desc");
	}
	catch(const transient_exception &)
	{
		description = "<unset>";
	}

	call->result = std::format("hostname: {} ({})", hostname, description);
}

void Command::reset(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	esp_restart();
}

void Command::write(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	if(call->parameter_count == 1)
		call->result = call->parameters[0].str;
}

void Command::info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "commands received:";
	call->result += std::format("\n- total: {:d}", cli_stats_commands_received);
	call->result += std::format("\n- packetised: {:d}", cli_stats_commands_received_packet);
	call->result += std::format("\n- raw: {:d}", cli_stats_commands_received_raw);
	call->result += "\nreplies sent:";
	call->result += std::format("\n- total: {:d}", cli_stats_replies_sent);
	call->result += std::format("\n- packetised: {:d}", cli_stats_replies_sent_packet);
	call->result += std::format("\n- raw: {:d}", cli_stats_replies_sent_raw);
}

void Command::bluetooth_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "BLUETOOTH INFO";

	bt_->info(call->result);
}

void Command::bluetooth_key(cli_command_call_t *call)
{
	std::string key;

	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	if(call->parameter_count == 1)
		bt_->key(call->parameters[0].str);

	key = bt_->key();

	call->result = std::format("bluetooth key: {}", key);
}

void Command::ota_start(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_ota_start(call); // FIXME
}

void Command::ota_write(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_ota_write(call); // FIXME
}

void Command::ota_finish(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_ota_finish(call); // FIXME
}

void Command::ota_commit(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_ota_commit(call); // FIXME
}

void Command::ota_confirm(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_ota_confirm(call); // FIXME
}

void Command::display_brightness(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_display_brightness(call); // FIXME
}

void Command::display_configure(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_display_configure(call); // FIXME
}

void Command::display_erase(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_display_erase(call); // FIXME
}

void Command::display_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_display_info(call); // FIXME
}

void Command::display_page_add_text(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_display_page_add_text(call); // FIXME
}

void Command::display_page_add_image(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_display_page_add_image(call); // FIXME
}

void Command::display_page_remove(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_display_page_remove(call); // FIXME
}

void Command::i2c_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_i2c_info(call); // FIXME
}

void Command::i2c_speed(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_i2c_speed(call); // FIXME
}

void Command::sensor_dump(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_sensor_dump(call); // FIXME
}

void Command::sensor_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_sensor_info(call); // FIXME
}

void Command::sensor_json(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_sensor_json(call); // FIXME
}

void Command::sensor_stats(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_sensor_stats(call); // FIXME
}

void Command::io_dump(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_io_dump(call); // FIXME
}

void Command::io_read(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_io_read(call); // FIXME
}

void Command::io_stats(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_io_stats(call); // FIXME
}

void Command::io_write(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_io_write(call); // FIXME
}

void Command::alias(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	Command::singleton->alias_command(call);
}

void Command::run(cli_command_call_t *call) // FIXME
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	command_run(call); // FIXME
}

void Command::tcp_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	net_tcp_command_info(call); // FIXME
}

void Command::alias_command(cli_command_call_t *call)
{
	string_string_map::const_iterator it;

	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

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
		call->result += std::format("\n  {}: {}", ref.first, ref.second);
}

void Command::wlan_client_config(cli_command_call_t *call)
{
	std::string ssid;
	std::string passwd;

	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	if(call->parameter_count == 1)
		throw(hard_exception("wlan_client_config: need to set both ssid and passwd"));

	if(call->parameter_count > 2)
		throw(hard_exception("wlan_client_config: invalid arguments"));

	if(call->parameter_count == 2)
		wlan_->set(call->parameters[0].str, call->parameters[1].str);

	wlan_->get(ssid, passwd);

	call->result = std::format("wlan client, ssid: {}, password: {}", ssid, passwd);
}

void Command::wlan_ipv6_static(cli_command_call_t *call)
{
	std::string address;

	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	if(call->parameter_count == 1)
		wlan_->set_ipv6_static(call->parameters[0].str);

	wlan_->get_ipv6_static(address);

	call->result = std::format("ipv6 static address: {}", address);
}

void Command::wlan_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "WLAN INFO";

	wlan_->info(call->result);
}

void Command::udp_info(cli_command_call_t *call)
{
	if(!Command::singleton)
		throw(hard_exception("Command: not activated"));

	call->result = "UDP INFO";

	udp_->info(call->result);
}

command_response_t *Command::receive_queue_pop()
{
	command_response_t *command_response = nullptr;

	if(!this->receive_queue_handle)
		throw(hard_exception("Command::receive_queue_pop: queue inactive"));

	xQueueReceive(this->receive_queue_handle, &command_response, portMAX_DELAY);

	this->cli_stats_commands_received++;

	return(command_response);
}

void Command::send_queue_push(command_response_t *command_response)
{
	if(!this->send_queue_handle)
		throw(hard_exception("Command::send_queue_push: queue inactive"));

	if(!command_response)
		throw(hard_exception("Command::send_queue_push: invalid argument"));

	xQueueSendToBack(this->send_queue_handle, &command_response, portMAX_DELAY);

	if(command_response->packetised)
		this->cli_stats_replies_sent_packet++;
	else
		this->cli_stats_replies_sent_raw++;

	this->cli_stats_replies_sent++;
}

command_response_t *Command::send_queue_pop()
{
	command_response_t *command_response = nullptr;

	xQueueReceive(send_queue_handle, &command_response, portMAX_DELAY);

	return(command_response);
}

void Command::run_receive_queue_wrapper(void *command_)
{
	Command *command;

	if(!command_)
		throw(hard_exception("Command::run_receive_queue_wrapper: invalid argument"));

	command = reinterpret_cast<Command *>(command_);

	command->run_receive_queue();
}

void Command::run_receive_queue()
{
	command_response_t					*command_response;
	std::string							data;
	std::string::const_iterator			data_iterator;
	std::string							oob_data;
	std::string							command;
	unsigned int						count, current, ix;
	const cli_command_t					*cli_command;
	cli_parameter_t						*parameter;
	const cli_parameter_description_t	*parameter_description;
	cli_command_call_t					call;

	try
	{
		for(;;)
		{
			command_response = receive_queue_pop();
			Packet::decapsulate(command_response->packetised, command_response->packet, data, oob_data);

			if(command_response->packetised)
				cli_stats_commands_received_packet++;
			else
				cli_stats_commands_received_raw++;

			try
			{
				call.parameter_count = 0;

				if(data.length() == 0)
					throw(transient_exception("ERROR: empty line"));

				this->alias_expand(data);

				command.clear();

				for(data_iterator = data.begin(); data_iterator != data.end(); data_iterator++)
					if(*data_iterator > ' ')
						command.append(1, *data_iterator);
					else
						break;

				for(ix = 0;; ix++)
				{
					cli_command = &cli_commands[ix];

					if(!cli_command->name || (command == cli_command->name))
						break;

					if(cli_command->alias && (command == cli_command->alias))
						break;
				}

				if(!cli_command->name)
					throw(transient_exception(std::format("ERROR: unknown command \"{}\"", command)));

				count = cli_command->parameters_description.count;

				if(count > parameters_size)
					throw(hard_exception("Command::run_receive_queue: parameter count > parameters_size"));

				for(current = 0; current < count; current++)
				{
					parameter_description = &cli_command->parameters_description.entries[current];
					parameter = &call.parameters[current];

					parameter->type = cli_parameter_none;
					parameter->has_value = 0;

					for(; data_iterator != data.end(); data_iterator++)
						if(*data_iterator > ' ')
							break;

					if(data_iterator == data.end())
					{
						if(!parameter_description->value_required)
							continue;
						else
							throw(transient_exception(std::format("ERROR: missing required parameter {:d}", current + 1)));
					}
					else
					{
						call.parameter_count++;

						parameter->str.clear();

						for(; data_iterator != data.end(); data_iterator++)
							if(*data_iterator > ' ')
								parameter->str.append(1, *data_iterator);
							else
								break;

						switch(parameter_description->type)
						{
							case(cli_parameter_none):
							case(cli_parameter_size):
							{
								throw(transient_exception(std::format("ERROR: parameter with invalid type {:d}", static_cast<int>(parameter_description->type))));
							}

							case(cli_parameter_unsigned_int):
							{
								unsigned int value;

								try
								{
									value = std::stoul(parameter->str, nullptr, parameter_description->base);
								}
								catch(...)
								{
									throw(transient_exception(std::format("ERROR: invalid unsigned integer value: {}", parameter->str)));
								}

								if((parameter_description->lower_bound_required) && (value < parameter_description->unsigned_int.lower_bound))
									throw(transient_exception(std::format("ERROR: invalid unsigned integer value: {:d}, smaller than lower bound: {:d}",
											value, parameter_description->unsigned_int.lower_bound)));

								if((parameter_description->upper_bound_required) && (value > parameter_description->unsigned_int.upper_bound))
									throw(transient_exception(std::format("ERROR: invalid unsigned integer value: {:d}, larger than upper bound: {:d}",
											value, parameter_description->unsigned_int.upper_bound)));

								parameter->type = cli_parameter_unsigned_int;
								parameter->has_value = 1;
								parameter->unsigned_int = value;

								break;
							}

							case(cli_parameter_signed_int):
							{
								int value;

								try
								{
									value = std::stol(parameter->str, nullptr, parameter_description->base);
								}
								catch(...)
								{
									throw(transient_exception(std::format("ERROR: invalid signed integer value: {}", parameter->str)));
								}

								if((parameter_description->lower_bound_required) && (value < parameter_description->signed_int.lower_bound))
									throw(transient_exception(std::format("ERROR: invalid signed integer value: {:d}, smaller than lower bound: {:d}",
											value, parameter_description->signed_int.lower_bound)));

								if((parameter_description->upper_bound_required) && (value > parameter_description->signed_int.upper_bound))
									throw(transient_exception(std::format("ERROR: invalid signed integer value: {:d}, larger than upper bound: {:d}",
											value, parameter_description->signed_int.upper_bound)));

								parameter->type = cli_parameter_signed_int;
								parameter->has_value = 1;
								parameter->signed_int = value;

								break;
							}

							case(cli_parameter_float):
							{
								float value;

								try
								{
									value = std::stod(parameter->str, nullptr);
								}
								catch(...)
								{
									throw(transient_exception(std::format("ERROR: invalid float value: {}", parameter->str)));
								}

								if((parameter_description->lower_bound_required) && (value < parameter_description->fp.lower_bound))
									throw(transient_exception(std::format("ERROR: invalid float value: {:f}, smaller than lower bound: {:f}",
											value, parameter_description->fp.lower_bound)));

								if((parameter_description->upper_bound_required) && (value > parameter_description->fp.upper_bound))
									throw(transient_exception(std::format("ERROR: invalid float value: {:f}, larger than upper bound: {:f}",
											value, parameter_description->fp.upper_bound)));

								parameter->type = cli_parameter_float;
								parameter->has_value = 1;
								parameter->fp = value;

								break;
							}

							case(cli_parameter_string):
							{
								unsigned int length;

								length = parameter->str.length();

								if((parameter_description->lower_bound_required) && (length < parameter_description->string.lower_length_bound))
									throw(transient_exception(std::format("ERROR: invalid string length: {:d}, smaller than lower bound: {:d}",
											length, parameter_description->string.lower_length_bound)));

								if((parameter_description->upper_bound_required) && (length > parameter_description->string.upper_length_bound))
									throw(transient_exception(std::format("ERROR: invalid string length: {:d}, larger than upper bound: {:d}",
											length, parameter_description->string.upper_length_bound)));

								parameter->type = cli_parameter_string;
								parameter->has_value = 1;

								break;
							}

							case(cli_parameter_string_raw):
							{
								unsigned int length;

								for(; data_iterator != data.end(); data_iterator++)
									parameter->str.append(1, *data_iterator);

								length = parameter->str.length();

								if((parameter_description->lower_bound_required) && (length < parameter_description->string.lower_length_bound))
									throw(transient_exception(std::format("ERROR: invalid raw string length: {:d}, smaller than lower bound: {:d}",
											length, parameter_description->string.lower_length_bound)));

								if((parameter_description->upper_bound_required) && (length > parameter_description->string.upper_length_bound))
									throw(transient_exception(std::format("ERROR: invalid raw string length: {:d}, larger than upper bound: {:d}",
											length, parameter_description->string.upper_length_bound)));

								parameter->type = cli_parameter_string;
								parameter->has_value = 1;

								break;
							}
						}
					}
				}

				if(current >= parameters_size)
					throw(transient_exception(std::format("ERROR: too many parameters: {}", current)));

				if(current < cli_command->parameters_description.count)
					throw(transient_exception("ERROR: missing parameters"));

				for(; data_iterator != data.end(); data_iterator++)
					if(*data_iterator > ' ')
						break;

				if(data_iterator != data.end())
					throw(transient_exception("ERROR: too many parameters"));

				call.source =			command_response->source;
				call.mtu =				command_response->mtu;
				call.oob =				oob_data;
				call.result.clear();
				call.result_oob.clear();

				cli_command->function(&call);
			}
			catch(const transient_exception &e)
			{
				call.result = std::format("WARNING: {}", e.what());
				*log_ << std::format("cli: transient exception: {}", e.what());
				call.result_oob.clear();
			}
			catch(const hard_exception &e)
			{
				call.result = std::format("ERROR: {}", e.what());
				*log_ << std::format("cli: hard exception: {}", e.what());
				call.result_oob.clear();
			}

			if(call.result_oob.empty() && (call.result.size() > command_response->mtu))
				call.result.resize(command_response->mtu);

			if(call.result_oob.size() > command_response->mtu)
			{
				call.result = std::format("ERROR: packet mtu overflow, payload: {:d}, oob: {:d}, packet overhead: {:d}, mtu: {:d}",
						call.result.size(), call.result_oob.size(), Packet::packet_header_size(), command_response->mtu);
				*log_ << std::format("cli: {}", call.result);
				call.result_oob.clear();
			}

			command_response->packet = Packet::encapsulate(command_response->packetised, call.result, call.result_oob);
			send_queue_push(command_response);

			for(ix = 0; ix < call.parameter_count; ix++)
				call.parameters[ix].str.clear();
		}
	}
	catch(const hard_exception &e)
	{
		log_->abort(std::format("Command::run_receive_queue: uncaught hard exception: {}", e.what()));
	}
	catch(const transient_exception &e)
	{
		log_->abort(std::format("Command::run_receive_queue: uncaught transient exception: {}", e.what()));
	}
	catch(const std::exception &e)
	{
		log_->abort(std::format("Command::run_receive_queue: uncaught generic exception: {}", e.what()));
	}
	catch(...)
	{
		log_->abort("Command::run_receive_queue: uncaught unknown exception");
	}

	for(;;) // prevent compiler error
		(void)0;
}

void Command::run_send_queue_wrapper(void *command_)
{
	Command *command;

	if(!command_)
		throw(hard_exception("Command::run_send_queue_wrapper: invalid argument"));

	command = reinterpret_cast<Command *>(command_);

	command->run_send_queue();
}

void Command::run_send_queue()
{
	command_response_t *command_response;

	try
	{
		for(;;)
		{
			command_response = this->send_queue_pop();

			switch(command_response->source)
			{
				case(cli_source_bt):
				{
					this->bt.send(command_response);
					break;
				}

				case(cli_source_console):
				{
					this->console.send(*command_response);
					break;
				}

				case(cli_source_wlan_tcp):
				{
					net_tcp_send(command_response);
					break;
				}

				case(cli_source_wlan_udp):
				{
					this->udp.send(command_response);
					break;
				}

				case(cli_source_script):
				{
					if(!command_response->packet.empty() && command_response->packet.back() == '\n') // FIXME
						command_response->packet.pop_back();

					if(!command_response->packet.empty())
						*log_ << std::format("script: {}: {}", command_response->script.name, command_response->packet);

					break;
				}

				default:
				{
					*log_ << std::format("cli: invalid source type: {:d}", static_cast<int>(command_response->source));
					break;
				}
			}

			if(command_response->source == cli_source_script)
			{
				if(!command_response->script.task)
					throw(hard_exception("Command::run_send_queue: invalid script task"));

				xTaskNotifyGive(static_cast<TaskHandle_t>(command_response->script.task));
				command_response->script.name[0] = '\0';
				command_response->script.task = nullptr;
			}

			command_response->source = cli_source_none;
			delete command_response;
			command_response = nullptr;
		}
	}
	catch(const hard_exception &e)
	{
		log_->abort(std::format("Command::run_send_queue: uncaught hard exception: {}", e.what()));
	}
	catch(const transient_exception &e)
	{
		log_->abort(std::format("Command::run_send_queue: uncaught transient exception: {}", e.what()));
	}
	catch(const std::exception &e)
	{
		log_->abort(std::format("Command::run_send_queue: uncaught generic exception: {}", e.what()));
	}
	catch(...)
	{
		log_->abort("Command::run_send_queue: uncaught unknown exception");
	}

	for(;;) // prevent compiler error
		(void)0;
}

void Command::receive_queue_push(command_response_t *command_response)
{
	xQueueSendToBack(this->receive_queue_handle, &command_response, portMAX_DELAY);
}

void Command::alias_expand(std::string &data) const
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
