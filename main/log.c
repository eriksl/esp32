#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_random.h>

#include "string.h"
#include "cli.h"
#include "log.h"
#include "console.h"
#include "util.h"
#include "cli-command.h"

#include <errno.h>

enum
{
	log_buffer_size = 8192,
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
static log_t *log_buffer = (log_t *)0;
static QueueHandle_t log_display_queue;

static void log_clear(void)
{
	uint32_t random_value;

	random_value = esp_random();

	log_buffer->magic_word = log_buffer_magic_word;
	log_buffer->random_salt = random_value;
	log_buffer->magic_word_salted = log_buffer_magic_word ^ random_value;
	log_buffer->entries = log_buffer_entries;
	log_buffer->in = 0;
	log_buffer->out = 0;
}

static void log_signal_display(unsigned int item)
{
	xQueueSend(log_display_queue, &item, (TickType_t)0);
}

void _log_cstr(bool append_strerror, const char *string)
{
	if(inited)
	{
		log_entry_t *entry;

		entry = &log_buffer->entry[log_buffer->in];

		entry->timestamp = time((time_t *)0);

		if(append_strerror)
			snprintf(entry->data, sizeof(entry->data), "%s: %s (%u)",
				string, strerror(errno), errno);
		else
			snprintf(entry->data, sizeof(entry->data), "%s", string);

		log_signal_display(log_buffer->in);

		if(log_buffer->in++ >= log_buffer_entries)
			log_buffer->in = 0;

		console_write_line(entry->data);
	}
	else
		console_write_line(string);
}

void _log_format(bool append_strerror, const char *fmt, ...)
{
	va_list ap;
	unsigned int offset;

	if(inited)
	{
		log_entry_t *entry;

		entry = &log_buffer->entry[log_buffer->in];

		entry->timestamp = time((time_t *)0);

		va_start(ap, fmt);
		vsnprintf(entry->data, sizeof(entry->data), fmt, ap);
		va_end(ap);

		if(append_strerror)
		{
			offset = strlen(entry->data);
			snprintf(entry->data + offset, sizeof(entry->data) - offset, ": %s (%u)",
					strerror(errno), errno);
		}

		log_signal_display(log_buffer->in);

		if(log_buffer->in++ >= log_buffer_entries)
			log_buffer->in = 0;

		console_write_line(entry->data);
	}
	else
		console_write_line(fmt);
}

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

void log_get_display_queue(QueueHandle_t *handle)
{
	assert(log_display_queue);

	*handle = log_display_queue;
}

void log_get_entry(unsigned int entry_index, time_t *stamp, unsigned int text_buffer_size, char *text_buffer)
{
	assert(inited);
	assert(text_buffer_size);
	assert(text_buffer);

	const log_entry_t *entry;

	*stamp = (time_t)0;
	text_buffer[0] = '\0';

	if(entry_index >= log_buffer->entries)
		return;

	entry = &log_buffer->entry[entry_index];

	*stamp = entry->timestamp;
	strlcpy(text_buffer, entry->data, text_buffer_size);
}

void log_init(void)
{
	assert(!inited);
	assert(rtc_slow_memory == (char *)0x50000000);

	log_buffer = (log_t *)rtc_slow_memory;

	inited = true;

	if((log_buffer->magic_word != log_buffer_magic_word) ||
		(log_buffer->magic_word_salted != (log_buffer_magic_word ^ log_buffer->random_salt)))
	{
		log_clear();
		log("log: log buffer corrupt, reinit");
	}

	esp_log_set_vprintf(logging_function);

	assert((log_display_queue = xQueueCreate(log_buffer_entries, sizeof(unsigned int))));

	log("boot");
}

void log_command_info(cli_command_call_t *call)
{
	assert(inited);

	string_format(call->result, "logging");
	string_format_append(call->result, "\n  buffer: 0x%08lx", (uint32_t)log_buffer);
	string_format_append(call->result, "\n  magic word: %08lx", log_buffer->magic_word);
	string_format_append(call->result, "\n  random salt: %08lx", log_buffer->random_salt);
	string_format_append(call->result, "\n  magic word salted: %08lx", log_buffer->magic_word_salted);
	string_format_append(call->result, "\n  entries: %u", log_buffer->entries);
	string_format_append(call->result, "\n  last entry added: %u", log_buffer->in);
	string_format_append(call->result, "\n  last entry viewed: %u", log_buffer->out);
}

void log_command_log(cli_command_call_t *call)
{
	string_auto(timestring, 64);
	unsigned int entries, amount;

	assert(inited);
	assert((call->parameter_count == 0) || (call->parameter_count == 1));

	if(call->parameter_count == 1)
		log_buffer->out = call->parameters[0].unsigned_int;

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

		string_format_append(call->result, "\n%3d %s %s",
				log_buffer->out,
				string_cstr(timestring),
				log_buffer->entry[log_buffer->out].data);

		if(++log_buffer->out >= log_buffer_entries)
			log_buffer->out = 0;
	}

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
