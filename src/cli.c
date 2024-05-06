#include "cli.h"

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <esp_log.h>
#include <esp_check.h>

enum
{
	receive_queue_size = 8,
	send_queue_size = 8,
};

enum
{
	parameters_size = 16,
};

typedef enum
{
	cli_source_none = 0,
	cli_source_bt,
	cli_source_station,
	cli_source_ap,
	cli_source_usb,
	cli_source_uart,
} cli_source_t;

typedef struct
{
	cli_source_t	source;
	unsigned int	length;
	uint8_t			*data;
} cli_buffer_t;

typedef enum
{
	cli_parameter_none = 0,
	cli_parameter_unsigned_int,
	cli_parameter_signed_int,
	cli_parameter_float,
	cli_parameter_string,
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
	cli_parameter_type_description_t type;

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
	cli_parameter_type_description_t type;

	union
	{
		unsigned int	unsigned_int;
		int				signed_int;
		float			fp;
		const char *	string;
	};
} cli_parameter_t;

typedef cli_parameter_t cli_parameters_t[parameters_size];

typedef void(cli_process_function_t)(const cli_parameters_t *, unsigned int size, uint8_t *result);

typedef struct
{
	const char *name;
	const char *alias;
	cli_process_function_t *function;
	cli_parameters_description_t parameters;
} cli_function_t;

static void process_stat(const cli_parameters_t *parameters, unsigned int size, uint8_t *result)
{
}

static void process_test(const cli_parameters_t *parameters, unsigned int size, uint8_t *result)
{
}

static const cli_function_t cli_functions[] =
{
	{ "stat",		"s",		process_stat, {} },
	{ "test",		"t",		process_test, { 1, {{ cli_parameter_unsigned_int, 1, 1, .unsigned_int = { 1, 10 }}}}},
	{ (char *)0,	(char *)0,	(cli_process_function_t *)0, {} },
};

static bool inited = false;
static QueueHandle_t receive_queue_handle;
//static QueueHandle_t send_queue_handle;
static cli_buffer_t receive_queue_data[receive_queue_size];
//static cli_buffer_t send_queue_data[send_queue_size];
static StaticQueue_t receive_queue;
//static StaticQueue_t send_queue;

static void cli_receive_queue_pop(cli_source_t *source, unsigned int *length, uint8_t **data);

static void cli_run(void *)
{
	cli_source_t 			source;
	unsigned int			length, current;
	uint8_t					*data;
	char					*token, *saveptr;
	const cli_function_t	*cli_function;

	source = cli_source_none;
	length = 0;
	data = (uint8_t *)0;

	ESP_LOGI("cli", "stack run: %d", uxTaskGetStackHighWaterMark(0));
	ESP_LOGW("cli", "cli run started");

	for(;;)
	{
		cli_receive_queue_pop(&source, &length, &data);
		ESP_LOGI("cli", "pop item: \"%.*s\"", (int)length, data);

		saveptr = (char *)0;
		if(!(token = strtok_r((char *)data, " \r\n", &saveptr)))
			goto next;

		for(cli_function = cli_functions; cli_function->name; cli_function++)
		{
			ESP_LOGW("cli", "compare name: \"%s\" \"%s\"\n", token, cli_function->name);

			if(!strcmp(cli_function->name, token))
			{
				ESP_LOGW("cli", "match name");
				break;
			}

			if(cli_function->alias)
			{
				ESP_LOGW("cli", "compare alias: \"%s\" \"%s\"\n", token, cli_function->alias);

				if(!strcmp(cli_function->alias, token))
				{
					ESP_LOGW("cli", "match alias");
					break;
				}
			}
		}

		if(!cli_function->name)
		{
			ESP_LOGW("cli", "no match");
			goto next;
		}

		for(current = 0; current < cli_function->parameters.count; current++)
		{
			if(!(token = strtok_r((char *)0, " \r\n", &saveptr)))
				break;

			ESP_LOGW("cli", "token: \"%s\"", token);
		}

		if(current < cli_function->parameters.count)
		{
			ESP_LOGW("cli", "missing parameters");
			goto next;
		}

		for(current = 0; strtok_r((char *)0, " \r\n", &saveptr); current++);

		if(current)
		{
			ESP_LOGW("cli", "too many parameters: %u", current);
			goto next;
		}

next:
		free(data);
	}
}

void cli_init(void)
{
	ESP_LOGI("cli", "stack init: %d", uxTaskGetStackHighWaterMark(0));

	if(inited)
	{
		ESP_LOGE("cli", "cli init: already inited");
		abort();
	}

	if(!(receive_queue_handle = xQueueCreateStatic(receive_queue_size, sizeof(cli_buffer_t), (void *)&receive_queue_data, &receive_queue)))
	{
		ESP_LOGE("cli", "xQueueCreateStatic");
		abort();
	}

	inited = true;

	if(xTaskCreatePinnedToCore(cli_run, "cli", 2048, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
	{
		ESP_LOGE("cli", "xTaskCreatePinnedToNode");
		abort();
	}
}

void cli_receive_from_bt_queue_push(unsigned int length, uint8_t *data)
{
	cli_buffer_t buffer;

	if(!inited)
	{
		ESP_LOGE("cli", "cli receive bt queue push: not inited");
		abort();
	}

	buffer.source = cli_source_bt;
	buffer.length = length;
	buffer.data = data;

	ESP_LOGI("cli", "push item: \"%.*s\"", (int)buffer.length, buffer.data);

	xQueueSend(receive_queue_handle, &buffer, portMAX_DELAY);
}

static void cli_receive_queue_pop(cli_source_t *source, unsigned int *length, uint8_t **data)
{
	cli_buffer_t buffer;

	if(!inited)
	{
		ESP_LOGE("cli", "cli receive queue pop: not inited");
		abort();
	}

	xQueueReceive(receive_queue_handle, &buffer, portMAX_DELAY);

	*source = buffer.source;
	*length = buffer.length;
	*data = buffer.data;
}
