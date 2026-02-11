#pragma once

#include <string>

#include "cli.h"

enum
{
	parameters_size = 16,
};

typedef enum
{
	cli_parameter_none = 0,
	cli_parameter_unsigned_int,
	cli_parameter_signed_int,
	cli_parameter_float,
	cli_parameter_string,
	cli_parameter_string_raw,
	cli_parameter_size,
} cli_parameter_type_description_t;

typedef struct
{
	cli_parameter_type_description_t type:4;
	unsigned int has_value:1;

	union
	{
		unsigned int	unsigned_int;
		int				signed_int;
		float			fp;
	};

	std::string str;
} cli_parameter_t;

typedef struct
{
	cli_source_t		source;
	unsigned int		mtu;
	unsigned int		parameter_count;
	cli_parameter_t		parameters[parameters_size];
	std::string			oob;
	std::string			result;
	std::string			result_oob;
} cli_command_call_t;

typedef void(cli_command_function_t)(cli_command_call_t *);

void bluetooth_command_info(cli_command_call_t *call);
void bluetooth_command_key(cli_command_call_t *call);

void flash_command_bench(cli_command_call_t *call);
void flash_command_checksum(cli_command_call_t *call);
void flash_command_info(cli_command_call_t *call);
void flash_command_read(cli_command_call_t *call);
void flash_command_write(cli_command_call_t *call);

void fs_command_erase(cli_command_call_t *call);
void fs_command_checksum(cli_command_call_t *call);
void fs_command_format(cli_command_call_t *call);
void fs_command_info(cli_command_call_t *call);
void fs_command_list(cli_command_call_t *call);
void fs_command_read(cli_command_call_t *call);
void fs_command_write(cli_command_call_t *call);
void fs_command_rename(cli_command_call_t *call);
void fs_command_truncate(cli_command_call_t *call);

void command_ota_start(cli_command_call_t *call);
void command_ota_write(cli_command_call_t *call);
void command_ota_finish(cli_command_call_t *call);
void command_ota_commit(cli_command_call_t *call);
void command_ota_confirm(cli_command_call_t *call);

void wlan_command_client_config(cli_command_call_t *call);
void wlan_command_info(cli_command_call_t *call);
void wlan_command_ipv6_static(cli_command_call_t *call);

void command_display_brightness(cli_command_call_t *call);
void command_display_configure(cli_command_call_t *call);
void command_display_erase(cli_command_call_t *call);
void command_display_info(cli_command_call_t *call);
void command_display_page_add_text(cli_command_call_t *call);
void command_display_page_add_image(cli_command_call_t *call);
void command_display_page_remove(cli_command_call_t *call);

void command_mcpwm_info(cli_command_call_t *call);

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

void net_udp_command_info(cli_command_call_t *call);

void net_tcp_command_info(cli_command_call_t *call);
