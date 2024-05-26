#pragma once

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
		const char *	string;
	};
} cli_parameter_t;

typedef struct
{
	unsigned int count;
	cli_parameter_t parameters[parameters_size];
} cli_parameters_t;

typedef struct
{
	const cli_parameters_t *	parameters;
	unsigned int				oob_data_length;
	uint8_t *					oob_data;
	unsigned int				result_size;
	char *						result;
	unsigned int				result_oob_size;
	unsigned int				result_oob_length;
	uint8_t *					result_oob;
} cli_command_call_t;

typedef void(cli_command_function_t)(cli_command_call_t *);
