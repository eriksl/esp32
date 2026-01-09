#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_random.h>

#include "log.h"

extern "C"
{
#include "string.h"
#include "util.h"
#include "cli-command.h"
}

#include "console.h"
#include "cli.h"

#include <errno.h>

enum
{
	log_buffer_size = 8192 - 32 - CONFIG_ULP_COPROC_RESERVE_MEM,
	log_buffer_entries = 62,
	log_buffer_data_size = 120,
	log_buffer_magic_word = 0x4afbcafe,
};

typedef struct
{
	time_t timestamp;
	char data[log_buffer_data_size];
} log_entry_t;

static_assert(sizeof(log_entry_t) == 128);

typedef struct
{
	uint32_t magic_word;
	uint32_t random_salt;
	uint32_t magic_word_salted;
	unsigned int entries;
	unsigned int in;
	unsigned int out;
	log_entry_t entry[log_buffer_entries];
} log_t;

static_assert(sizeof(log_t) == 7960);
static_assert(sizeof(log_t) < log_buffer_size);

RTC_NOINIT_ATTR static char rtc_slow_memory[log_buffer_size];
static bool inited = false;
static bool monitor = false;
static log_t *log_buffer = (log_t *)0;
static QueueHandle_t log_display_queue = (QueueHandle_t)0;
static SemaphoreHandle_t data_mutex;

static inline void data_mutex_take(void)
{
	xSemaphoreTake(data_mutex, portMAX_DELAY);
}

static inline void data_mutex_give(void)
{
	xSemaphoreGive(data_mutex);
}

static void log_clear(void)
{
	uint32_t random_value;

	random_value = esp_random();

	data_mutex_take();

	log_buffer->magic_word = log_buffer_magic_word;
	log_buffer->random_salt = random_value;
	log_buffer->magic_word_salted = log_buffer_magic_word ^ random_value;
	log_buffer->entries = log_buffer_entries;
	log_buffer->in = 0;
	log_buffer->out = 0;

	data_mutex_give();
}

static void log_signal_display(unsigned int item)
{
	if(log_display_queue)
		xQueueSend(log_display_queue, &item, (TickType_t)0);
}

static void _log_cstr(bool append_strerror, const char *string)
{
	unsigned int current;
	log_entry_t *entry;

	if(inited)
	{
		data_mutex_take();

		current = log_buffer->in;

		entry = &log_buffer->entry[current];

		entry->timestamp = time((time_t *)0);

		if(append_strerror)
			snprintf(entry->data, sizeof(entry->data), "%s: %s (%d)",
				string, strerror(errno), errno);
		else
			snprintf(entry->data, sizeof(entry->data), "%s", string);

		if(log_buffer->in++ >= log_buffer_entries)
			log_buffer->in = 0;

		data_mutex_give();

		if(monitor)
		{
			if(strlen(string) >= sizeof(entry->data))
				console_write_line(string);
			else
				console_write_line(entry->data);
		}

		log_signal_display(current);
	}
	else
		console_write_line(string);
}

void log_cstr(const char *line)
{
	_log_cstr(false, line);
}

void log_errno(const char *line)
{
	_log_cstr(true, line);
}

void log_cstr_errno(const char *line)
{
	_log_cstr(true, line);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsuggest-attribute=format"

static void _log_format(bool append_strerror, const char *fmt, va_list ap)
{
	unsigned int offset, current;
	log_entry_t *entry;

	if(inited)
	{
		data_mutex_take();

		current = log_buffer->in;

		entry = &log_buffer->entry[current];

		entry->timestamp = time((time_t *)0);

		vsnprintf(entry->data, sizeof(entry->data), fmt, ap);

		if(append_strerror)
		{
			offset = strlen(entry->data);
			snprintf(entry->data + offset, sizeof(entry->data) - offset, ": %s (%d)",
					strerror(errno), errno);
		}

		if(log_buffer->in++ >= log_buffer_entries)
			log_buffer->in = 0;

		data_mutex_give();

		if(monitor)
			console_write_line(entry->data);

		log_signal_display(current);
	}
	else
		console_write_line(fmt);
}

#pragma GCC diagnostic pop

void log_format(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	_log_format(false, fmt, ap);
	va_end(ap);
}

void log_format_errno(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	_log_format(true, fmt, ap);
	va_end(ap);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsuggest-attribute=format"

static int logging_function(const char *fmt, va_list ap)
{
	char buffer[log_buffer_data_size];
	char *start;
	char *end;
	int length;

	assert(inited);

	length = vsnprintf(buffer, sizeof(buffer), fmt, ap);

	for(start = buffer; *start; start++)
		if(*start == ':')
			break;

	if((start[0] == ':') && (start[1]))
		start = start + 1;
	else
		start = buffer;

	for(end = start; *end; end++)
		if(*end < ' ')
			break;

	*end = '\0';

	if(start != end)
		log_cstr(start);

	return(length);
}

#pragma GCC diagnostic pop

QueueHandle_t log_get_display_queue(void)
{
	return(log_display_queue);
}

void log_get_entry(unsigned int entry_index, time_t *stamp, unsigned int text_buffer_size, char *text_buffer)
{
	assert(inited);
	assert(text_buffer_size);
	assert(text_buffer);

	const log_entry_t *entry;

	*stamp = (time_t)0;
	text_buffer[0] = '\0';

	data_mutex_take();

	if(entry_index >= log_buffer->entries)
		goto exit;

	entry = &log_buffer->entry[entry_index];

	*stamp = entry->timestamp;
	strlcpy(text_buffer, entry->data, text_buffer_size);

exit:
	data_mutex_give();
}

void log_init(void)
{
	assert(!inited);

	log_buffer = (log_t *)(void *)rtc_slow_memory;

	data_mutex = xSemaphoreCreateMutex();
	assert(data_mutex);

	inited = true;

	if((log_buffer->magic_word != log_buffer_magic_word) ||
		(log_buffer->magic_word_salted != (log_buffer_magic_word ^ log_buffer->random_salt)))
	{
		log_clear();
		log("log: log buffer corrupt, reinit");
	}

	esp_log_set_vprintf(logging_function);

	log_display_queue = xQueueCreate(log_buffer_entries, sizeof(unsigned int));

	log("boot");
}

void log_setmonitor(bool val)
{
	monitor = !!val;
}

void log_command_info(cli_command_call_t *call)
{
	assert(inited);

	data_mutex_take();

	string_format(call->result, "logging");
	string_format_append(call->result, "\n  buffer: 0x%08lx", (uint32_t)log_buffer);
	string_format_append(call->result, "\n  magic word: %08lx", log_buffer->magic_word);
	string_format_append(call->result, "\n  random salt: %08lx", log_buffer->random_salt);
	string_format_append(call->result, "\n  magic word salted: %08lx", log_buffer->magic_word_salted);
	string_format_append(call->result, "\n  entries: %u", log_buffer->entries);
	string_format_append(call->result, "\n  last entry added: %u", log_buffer->in);
	string_format_append(call->result, "\n  last entry viewed: %u", log_buffer->out);

	data_mutex_give();
}

void log_command_log(cli_command_call_t *call)
{
	string_auto(timestring, 64);
	unsigned int entries, amount;

	assert(inited);
	assert((call->parameter_count == 0) || (call->parameter_count == 1));

	if(call->parameter_count == 1)
		log_buffer->out = call->parameters[0].unsigned_int;

	data_mutex_take();

	if(log_buffer->in > log_buffer->out)
		entries = log_buffer->in - log_buffer->out;
	else
		entries = log_buffer->in + (log_buffer_entries - log_buffer->out);

	if(entries == log_buffer_entries)
		entries = 0;

	string_format(call->result, "%u entries:", entries);
	amount = 0;

	for(amount = 0; (amount < 24) && (amount < entries); amount++)
	{
		util_time_to_string(timestring, &log_buffer->entry[log_buffer->out].timestamp);

		string_format_append(call->result, "\n%3u %s %s",
				log_buffer->out,
				string_cstr(timestring),
				log_buffer->entry[log_buffer->out].data);

		if(++log_buffer->out >= log_buffer_entries)
			log_buffer->out = 0;
	}

	data_mutex_give();

	if(amount != entries)
		string_format_append(call->result, "\n[%u more]", entries - amount);
}

void log_command_log_clear(cli_command_call_t *call)
{
	assert(inited);

	log_command_log(call);

	log_clear();

	string_format_append(call->result, "\nlog cleared");
}

void log_command_log_monitor(cli_command_call_t *call)
{
	assert(inited);

	if(call->parameter_count == 1)
		monitor = call->parameters[0].unsigned_int != 0;

	string_format_append(call->result, "log monitor: %s", monitor ? "yes" : "no");
}
