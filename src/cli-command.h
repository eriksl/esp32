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
	};

	string_t string;
} cli_parameter_t;

_Static_assert(sizeof(cli_parameter_t) == 12);

typedef struct
{
	unsigned int		parameter_count;
	cli_parameter_t		parameters[parameters_size];
	string_t			oob;
	string_t			result;
	string_t			result_oob;
} cli_command_call_t;

_Static_assert(sizeof(cli_command_call_t) == 208);

typedef void(cli_command_function_t)(cli_command_call_t *);
