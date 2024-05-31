#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_random.h>

#include "string.h"
#include "cli-command.h"
#include "cli.h"
#include "log.h"
#include "console.h"
#include "util.h"

#include <freertos/FreeRTOS.h>

#include <sys/time.h>

enum
{
	log_buffer_size = 7 * 1024,
	log_buffer_entries = 55,
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

static_assert(sizeof(log_t) < log_buffer_size);
static_assert(sizeof(log_t) == 7064);

static bool inited = false;
static log_t *log_buffer = (log_t *)0;

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

void log_cstr(const char *string)
{
	if(inited)
	{
		log_entry_t *entry;

		entry = &log_buffer->entry[log_buffer->in];

		entry->timestamp = time((time_t *)0);
		strlcpy(entry->data, string, log_buffer_data_size);
		entry->data[log_buffer_data_size - 1] = '\0';

		if(log_buffer->in++ >= log_buffer_entries)
			log_buffer->in = 0;
	}

	console_write_line(string);
}

void log_format(const char *fmt, ...)
{
	va_list ap;

	if(inited)
	{
		log_entry_t *entry;

		entry = &log_buffer->entry[log_buffer->in];

		entry->timestamp = time((time_t *)0);

		va_start(ap, fmt);
		vsnprintf(entry->data, sizeof(entry->data), fmt, ap);
		va_end(ap);

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

void log_init(void)
{
	assert(!inited);
	assert((log_buffer = (log_t *)heap_caps_malloc(sizeof(log_t), MALLOC_CAP_RTCRAM)));
	assert(log_buffer == (log_t *)0x600fe198);

	inited = true;

	if((log_buffer->magic_word != log_buffer_magic_word) ||
		(log_buffer->magic_word_salted != (log_buffer_magic_word ^ log_buffer->random_salt)))
	{
		log_clear();
		log("log: log buffer corrupt, reinit");
	}

	esp_log_set_vprintf(logging_function);

	log("boot");
}

void command_info_log(cli_command_call_t *call)
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

void command_log(cli_command_call_t *call)
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

void command_log_clear(cli_command_call_t *call)
{
	assert(inited);

	command_log(call);

	log_clear();

	string_format_append(call->result, "\nlog cleared");
}
