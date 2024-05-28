#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "cli-command.h"
#include "cli.h"
#include "log.h"
#include "console.h"
#include "util.h"

#include <freertos/FreeRTOS.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_random.h>

enum
{
	log_buffer_size = 7 * 1024,
	log_buffer_entries = 110,
	log_buffer_data_size = 56,
	log_buffer_magic_word = 0x4afbcafe,
};

typedef struct
{
	uint64_t timestamp;
	char data[log_buffer_data_size];
} log_entry_t;

static_assert(sizeof(log_entry_t) == 64);

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

		entry->timestamp = esp_timer_get_time();
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

		entry->timestamp = esp_timer_get_time();

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

	assert(inited);

	vsnprintf(buffer, sizeof(buffer), fmt, ap);

	if((start = strchr(buffer, ':')))
		start++;
	else
		start = buffer;

	if(*start == ' ')
		start++;

	if((end = strchr(start, '\n')))
		*end = '\0';

	log_cstr(start);

	return(strlen(start));
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
	unsigned int offset;

	assert(inited);

	offset = snprintf(call->result, call->result_size, "logging");

	offset += snprintf(call->result + offset, call->result_size - offset, "\n  buffer: 0x%08lx", (uint32_t)log_buffer);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n  magic word: %08lx", log_buffer->magic_word);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n  random salt: %08lx", log_buffer->random_salt);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n  magic word salted: %08lx", log_buffer->magic_word_salted);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n  entries: %u", log_buffer->entries);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n  last entry added: %u", log_buffer->in);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n  last entry viewed: %u", log_buffer->out);
}

void command_log(cli_command_call_t *call)
{
	unsigned int entries, offset, amount;

	assert(inited);
	assert((call->parameters->count == 0) || (call->parameters->count == 1));

	if(call->parameters->count == 1)
		log_buffer->out = call->parameters->parameters[0].unsigned_int;

	if(log_buffer->in > log_buffer->out)
		entries = log_buffer->in - log_buffer->out;
	else
		entries = log_buffer->in + (log_buffer_entries - log_buffer->out);

	if(entries == log_buffer_entries)
		entries = 0;

	offset = snprintf(call->result, call->result_size, "%u entries:", entries);
	amount = 0;

	for(amount = 0; (amount < 24) && (amount < entries) && (offset < call->result_size); amount++)
	{
		offset += snprintf(call->result + offset, call->result_size - offset, "\n%3d %llu %s",
				log_buffer->out,
				log_buffer->entry[log_buffer->out].timestamp,
				log_buffer->entry[log_buffer->out].data);

		if(++log_buffer->out >= log_buffer_entries)
			log_buffer->out = 0;
	}

	if(amount != entries)
		snprintf(call->result + offset, call->result_size - offset, "\n[%u more]", entries - amount);
}

void command_log_clear(cli_command_call_t *call)
{
	unsigned int offset;

	assert(inited);

	command_log(call);

	log_clear();

	offset = strlen(call->result);
	snprintf(call->result + offset, call->result_size - offset, "\nlog cleared");
}
