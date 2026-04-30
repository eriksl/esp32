#pragma once

#include <string>

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

void command_ota_start(cli_command_call_t *call);
void command_ota_write(cli_command_call_t *call);
void command_ota_finish(cli_command_call_t *call);
void command_ota_commit(cli_command_call_t *call);
void command_ota_confirm(cli_command_call_t *call);

void command_io_dump(cli_command_call_t *call);
void command_io_read(cli_command_call_t *call);
void command_io_stats(cli_command_call_t *call);
void command_io_write(cli_command_call_t *call);
