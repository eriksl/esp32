#pragma once

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

	string_t string;
} cli_parameter_t;

_Static_assert(sizeof(cli_parameter_t) == 12);

typedef struct
{
	cli_source_t		source;
	unsigned int		mtu;
	unsigned int		parameter_count;
	cli_parameter_t		parameters[parameters_size];
	string_t			oob;
	string_t			result;
	string_t			result_oob;
} cli_command_call_t;

_Static_assert(sizeof(cli_command_call_t) == 216);

typedef void(cli_command_function_t)(cli_command_call_t *);

void bluetooth_command_info(cli_command_call_t *call);

void config_command_info(cli_command_call_t *call);
void config_command_dump(cli_command_call_t *call);
void config_command_show(cli_command_call_t *call);
void config_command_set_int(cli_command_call_t *call);
void config_command_set_uint(cli_command_call_t *call);
void config_command_set_string(cli_command_call_t *call);
void config_command_erase(cli_command_call_t *call);

void console_command_info(cli_command_call_t *call);

void flash_command_bench(cli_command_call_t *call);
void flash_command_checksum(cli_command_call_t *call);
void flash_command_info(cli_command_call_t *call);
void flash_command_read(cli_command_call_t *call);
void flash_command_write(cli_command_call_t *call);

void fs_command_info(cli_command_call_t *call);
void fs_command_list(cli_command_call_t *call);
void fs_command_format(cli_command_call_t *call);
void fs_command_read(cli_command_call_t *call);
void fs_command_append(cli_command_call_t *call);
void fs_command_erase(cli_command_call_t *call);
void fs_command_checksum(cli_command_call_t *call);

void info_command_info(cli_command_call_t *call);
void info_command_info_board(cli_command_call_t *call);
void info_command_info_memory(cli_command_call_t *call);
void info_command_info_partitions(cli_command_call_t *call);
void info_command_info_process(cli_command_call_t *call);

void log_command_info(cli_command_call_t *call);
void log_command_log(cli_command_call_t *call);
void log_command_log_clear(cli_command_call_t *call);

void command_ota_start(cli_command_call_t *call);
void command_ota_write(cli_command_call_t *call);
void command_ota_finish(cli_command_call_t *call);
void command_ota_commit(cli_command_call_t *call);
void command_ota_confirm(cli_command_call_t *call);

void string_command_info(cli_command_call_t *call);

void wlan_command_client_config(cli_command_call_t *call);
void wlan_command_info(cli_command_call_t *call);
void wlan_command_ip_info(cli_command_call_t *call);
void wlan_command_ipv6_config(cli_command_call_t *call);

void command_display_brightness(cli_command_call_t *call);
void command_display_configure(cli_command_call_t *call);
void command_display_erase(cli_command_call_t *call);
void command_display_info(cli_command_call_t *call);
void command_display_page_add_text(cli_command_call_t *call);
void command_display_page_add_image(cli_command_call_t *call);
void command_display_page_remove(cli_command_call_t *call);

void command_ledpwm_info(cli_command_call_t *call);

void command_mcpwm_info(cli_command_call_t *call);

void command_pdm_info(cli_command_call_t *call);

void command_ledpixel_info(cli_command_call_t *call);

void command_i2c_info(cli_command_call_t *call);

void command_sensor_dump(cli_command_call_t *call);
void command_sensor_info(cli_command_call_t *call);
void command_sensor_stats(cli_command_call_t *call);

void command_io_dump(cli_command_call_t *call);
void command_io_read(cli_command_call_t *call);
void command_io_stats(cli_command_call_t *call);
void command_io_write(cli_command_call_t *call);
