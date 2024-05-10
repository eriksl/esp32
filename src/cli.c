#include "cli.h"
#include "util.h"
#include "bt.h"
#include "packet.h"

#include <errno.h>
#include <string.h>
#include <stdbool.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <esp_log.h>
#include <esp_check.h>

enum
{
	receive_queue_size = 8,
	send_queue_size = 8,
	result_size = 1024,
	result_oob_size = 0,
};

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

	unsigned int base:4;
	unsigned int value_required:1;
	unsigned int lower_bound_required:1;
	unsigned int upper_bound_required:1;

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
	cli_parameter_description_t parameters[parameters_size];
} cli_parameters_description_t;

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
} cli_function_call_t;

typedef void(cli_process_function_t)(cli_function_call_t *);

typedef struct
{
	const char *name;
	const char *alias;
	cli_process_function_t *function;
	cli_parameters_description_t parameters;
} cli_function_t;

#if 0
static const char *parameter_type_to_string(unsigned int type)
{
	static const char *type_string[cli_parameter_size] =
	{
		"invalid parameter type",
		"unsigned int",
		"signed int",
		"float",
		"string",
	};
		
	if(type >= cli_parameter_size)
		type = cli_parameter_none;

	return(type_string[type]);
}
#endif

static void process_stat(cli_function_call_t *)
{
}

static void process_test(cli_function_call_t *call)
{
	unsigned int ix, at;
	const cli_parameter_t *parameter;

	at = 0;

	for(ix = 0; ix < call->parameters->count; ix++)
	{
		parameter = &call->parameters->parameters[ix];

		if(!parameter->has_value)
		{
			if(at < call->result_size)
				at += snprintf(call->result + at, call->result_size - at, "ERROR: parameter %u has no value\n", ix);

			continue;
		}

		switch(parameter->type)
		{
			case(cli_parameter_none):
			case(cli_parameter_size):
			{
				if(at < call->result_size)
					at += snprintf(call->result + at, call->result_size - at, "ERROR: invalid parameter %u\n", ix);

				break;
			}

			case(cli_parameter_unsigned_int):
			{
				if(at < call->result_size)
					at += snprintf(call->result + at, call->result_size - at, "unsigned int parameter: %u\n", parameter->unsigned_int);

				break;
			}

			case(cli_parameter_signed_int):
			{
				if(at < call->result_size)
					at += snprintf(call->result + at, call->result_size - at, "signed int parameter: %d\n", parameter->signed_int);

				break;
			}

			case(cli_parameter_float):
			{
				if(at < call->result_size)
					at += snprintf(call->result + at, call->result_size - at, "float parameter: %f\n", parameter->fp);

				break;
			}

			case(cli_parameter_string):
			{
				if(at < call->result_size)
					at += snprintf(call->result + at, call->result_size - at, "string parameter: \"%s\"\n", parameter->string);

				break;
			}
		}
	}
}

static const cli_function_t cli_functions[] =
{
	{ "stat",		"s",		process_stat, {} },
	{ "test",		"t",		process_test,	{ 4,
													{
														{ cli_parameter_unsigned_int,	0, 1, 1, 1, .unsigned_int =	{ 1, 10 }},
														{ cli_parameter_signed_int,		0, 1, 1, 1, .signed_int =		{ 1, 10 }},
														{ cli_parameter_float,			0, 1, 1, 1, .fp =				{ 1, 10 }},
														{ cli_parameter_string,			0, 1, 1, 1, .string =			{ 1, 10 }},
													}
												}},
	{ (char *)0,	(char *)0,	(cli_process_function_t *)0, {} },
};

static bool inited = false;
static QueueHandle_t receive_queue_handle;
static QueueHandle_t send_queue_handle;
static cli_buffer_t receive_queue_data[receive_queue_size];
static cli_buffer_t send_queue_data[send_queue_size];
static StaticQueue_t receive_queue;
static StaticQueue_t send_queue;

static void receive_queue_pop(cli_buffer_t *cli_buffer)
{
	assert(inited);

	xQueueReceive(receive_queue_handle, cli_buffer, portMAX_DELAY);
}

static void send_queue_push(cli_buffer_t *cli_buffer)
{
	assert(inited);

	xQueueSend(send_queue_handle, cli_buffer, portMAX_DELAY);
}

static void send_queue_pop(cli_buffer_t *cli_buffer)
{
	assert(inited);

	xQueueReceive(send_queue_handle, cli_buffer, portMAX_DELAY);
}

static void run_receive_queue(void *)
{
	cli_buffer_t						cli_buffer;
	cli_parameters_t					parameters;
	unsigned int						oob_data_length;
	char								*data;
	uint8_t								*oob_data;
	unsigned int						count, current, ix;
	char								*token, *saveptr;
	const cli_function_t				*cli_function;
	const cli_parameter_description_t	*parameter_description;
	cli_parameter_t						*parameter;
	static cli_function_call_t			call;
	static char							error[128];

	assert(inited);

	for(;;)
	{
		util_stack_usage_update("run_receive_queue");

		receive_queue_pop(&cli_buffer);
		packet_decapsulate(&cli_buffer, &data, &oob_data_length, &oob_data);

		if(cli_buffer.data_from_malloc && cli_buffer.data)
			free(cli_buffer.data);
		cli_buffer.length = 0;
		cli_buffer.data = (uint8_t *)0;
		cli_buffer.data_from_malloc = 0;

		saveptr = (char *)0;
		if(!(token = strtok_r((char *)data, " \r\n", &saveptr)))
		{
			snprintf(error, sizeof(error), "ERROR: empty line");
			packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		for(ix = 0;; ix++)
		{
			cli_function = &cli_functions[ix];

			if(!cli_function->name || !strcmp(cli_function->name, token))
				break;

			if(cli_function->alias && !strcmp(cli_function->alias, token))
				break;
		}

		if(!cli_function->name)
		{
			snprintf(error, sizeof(error), "ERROR: unknown command \"%s\"", token);
			packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		count = cli_function->parameters.count;

		if(count > parameters_size)
			count = parameters_size;

		for(current = 0; current < count; current++)
		{
			parameter_description = &cli_function->parameters.parameters[current];
			parameter = &parameters.parameters[current];

			parameter->type = cli_parameter_none;
			parameter->has_value = 0;

			token = strtok_r((char *)0, " \r\n", &saveptr);

			if(!token)
			{
				if(!parameter_description->value_required)
				{
					continue;
				}
				else
				{
					snprintf(error, sizeof(error), "ERROR: missing required parameter %u", current + 1);
					packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
					send_queue_push(&cli_buffer);
					goto error;
				}
			}
			else
			{
				switch(parameter_description->type)
				{
					case(cli_parameter_none):
					case(cli_parameter_size):
					{
						snprintf(error, sizeof(error), "ERROR: parameter with invalid type %u", parameter_description->type);
						packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
						send_queue_push(&cli_buffer);
						goto error;
					}

					case(cli_parameter_unsigned_int):
					{
						char *endptr;
						unsigned int value;

						errno = 0;
						value = strtoul(token, &endptr, parameter_description->base);

						if(errno || !*token || *endptr)
						{
							snprintf(error, sizeof(error), "ERROR: invalid unsigned integer value: %s", token);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->lower_bound_required) && (value < parameter_description->unsigned_int.lower_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid unsigned integer value: %u, smaller than lower bound: %u", value, parameter_description->unsigned_int.lower_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->upper_bound_required) && (value > parameter_description->unsigned_int.upper_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid unsigned integer value: %u, larger than upper bound: %u", value, parameter_description->unsigned_int.upper_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
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
						char *endptr;
						int value;

						errno = 0;
						value = strtol(token, &endptr, parameter_description->base);

						if(errno || !*token || *endptr)
						{
							snprintf(error, sizeof(error), "ERROR: invalid signed integer value: %s", token);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->lower_bound_required) && (value < parameter_description->signed_int.lower_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid signed integer value: %d, smaller than lower bound: %d", value, parameter_description->signed_int.lower_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->upper_bound_required) && (value > parameter_description->signed_int.upper_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid signed integer value: %d, larger than upper bound: %d", value, parameter_description->signed_int.upper_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
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
						char *endptr;
						float value;

						errno = 0;
						value = strtof(token, &endptr);

						if(errno || !*token || *endptr)
						{
							snprintf(error, sizeof(error), "ERROR: invalid float value: %s", token);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->lower_bound_required) && (value < parameter_description->fp.lower_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid float value: %f, smaller than lower bound: %f", value, parameter_description->fp.lower_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->upper_bound_required) && (value > parameter_description->fp.upper_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid float value: %f, larger than upper bound: %f", value, parameter_description->fp.upper_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
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

						length = strlen(token);

						if((parameter_description->lower_bound_required) && (length < parameter_description->string.lower_length_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid string length: %u, smaller than lower bound: %u", length, parameter_description->string.lower_length_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						if((parameter_description->upper_bound_required) && (length > parameter_description->string.upper_length_bound))
						{
							snprintf(error, sizeof(error), "ERROR: invalid string length: %u, larger than upper bound: %u", length, parameter_description->string.upper_length_bound);
							packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
							send_queue_push(&cli_buffer);
							goto error;
						}

						parameter->type = cli_parameter_string;
						parameter->has_value = 1;
						parameter->string = token;

						break;
					}
				}
			}
		}

		if(current >= parameters_size)
		{
			snprintf(error, sizeof(error), "ERROR: too many parameters: %u", current);
			packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		if(current < cli_function->parameters.count)
		{
			snprintf(error, sizeof(error), "ERROR: missing paramters");
			packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		if(strtok_r((char *)0, " \r\n", &saveptr))
		{
			snprintf(error, sizeof(error), "ERROR: too many parameters");
			packet_encapsulate(&cli_buffer, error, 0, (uint8_t *)0);
			send_queue_push(&cli_buffer);
			goto error;
		}

		parameters.count = current;

		call.parameters =			&parameters;
		call.oob_data_length =		oob_data_length;
		call.oob_data =				oob_data;
		call.result_size =			result_size;
		call.result =				heap_caps_malloc(call.result_size, MALLOC_CAP_SPIRAM);
		call.result_oob_size =		result_oob_size;
		call.result_oob_length =	0;
		call.result_oob =			heap_caps_malloc(call.result_oob_size, MALLOC_CAP_SPIRAM);

		cli_function->function(&call);

		packet_encapsulate(&cli_buffer, call.result, call.result_oob_length, call.result_oob);
		send_queue_push(&cli_buffer);

		if(call.result)
		{
			free(call.result);
			call.result = (char *)0;
		}

		if(call.result_oob)
		{
			free(call.result_oob);
			call.result_oob = (uint8_t *)0;
		}

error:
		cli_buffer.source = cli_source_none;
		cli_buffer.length = 0;
		cli_buffer.data_from_malloc = 0;
		cli_buffer.data = (uint8_t *)0;

		if(data)
		{
			free(data);
			data = (char *)0;
		}

		if(oob_data)
		{
			free(oob_data);
			oob_data = (uint8_t *)0;
		}
	}
}

static void run_send_queue(void *)
{
	cli_buffer_t cli_buffer;

	assert(inited);

	for(;;)
	{
		util_stack_usage_update("run_send_queue");

		send_queue_pop(&cli_buffer);

		switch(cli_buffer.source)
		{
			case(cli_source_none):
			{
				ESP_LOGE("cli", "invalid source type: %u", cli_buffer.source);
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

	if(!(receive_queue_handle = xQueueCreateStatic(receive_queue_size, sizeof(cli_buffer_t), (void *)&receive_queue_data, &receive_queue)))
	{
		ESP_LOGE("cli", "xQueueCreateStatic receive queue init");
		abort();
	}

	if(!(send_queue_handle = xQueueCreateStatic(send_queue_size, sizeof(cli_buffer_t), (void *)&send_queue_data, &send_queue)))
	{
		ESP_LOGE("cli", "xQueueCreateStatic receive queue init");
		abort();
	}

	inited = true;

	if(xTaskCreatePinnedToCore(run_receive_queue, "cli-recv", 3072, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
	{
		ESP_LOGE("cli", "xTaskCreatePinnedToNode run_receive_queue");
		abort();
	}

	if(xTaskCreatePinnedToCore(run_send_queue, "cli-send", 3072, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
	{
		ESP_LOGE("cli", "xTaskCreatePinnedToNode run_send_queue");
		abort();
	}

	util_stack_usage_update("cli_init");
}
