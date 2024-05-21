#include "cli-command.h"
#include "log.h"
#include "util.h"

#include <freertos/FreeRTOS.h>

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <esp_timer.h>
#include <esp_log.h>
#include <esp_random.h>

enum
{
	log_buffer_size = 7 * 1024,
	log_buffer_entries = 110,
	log_buffer_magic_word = 0x4afbcafe,
};

typedef struct
{
	uint64_t timestamp;
	char data[56];
} log_entry_t;

static_assert(sizeof(log_entry_t) == 64);

typedef struct
{
	uint32_t magic_word;
	uint32_t random_salt;
	uint32_t magic_word_salted;
	unsigned int entries;
	unsigned int current_entry;
	log_entry_t entry[log_buffer_entries];
} log_t;

static_assert(sizeof(log_t) < log_buffer_size);
static_assert(sizeof(log_t) == 7064);

static bool inited = false;
static log_t *log_buffer = (log_t *)0;

void log_simple(const char *string)
{
	log_entry_t *entry;

	assert(inited);

	entry = &log_buffer->entry[log_buffer->current_entry];

	entry->timestamp = esp_timer_get_time();
	strncpy(entry->data, string, sizeof(entry->data));
	entry->data[sizeof(entry->timestamp) - 1] = '\0';

	if(log_buffer->current_entry++ >= log_buffer_entries)
		log_buffer->current_entry = 0;
}

void log_vargs(const char *fmt, ...)
{
	va_list ap;
	log_entry_t *entry;

	assert(inited);

	entry = &log_buffer->entry[log_buffer->current_entry];

	entry->timestamp = esp_timer_get_time();

	va_start(ap, fmt);
	vsnprintf(entry->data, sizeof(entry->data), fmt, ap);
	va_end(ap);

	if(log_buffer->current_entry++ >= log_buffer_entries)
		log_buffer->current_entry = 0;
}

void log_init(void)
{
	uint32_t random_value;

	assert((log_buffer = (log_t *)heap_caps_malloc(sizeof(log_t), MALLOC_CAP_RTCRAM)));
	assert(log_buffer == (log_t *)0x600fe198);

	if((log_buffer->magic_word != log_buffer_magic_word) ||
		(log_buffer->magic_word_salted != (log_buffer_magic_word ^ log_buffer->random_salt)))
	{
		random_value = esp_random();

		ESP_LOGW("log", "log buffer corrupt, using random value 0x%lx to reinit", random_value); // FIXME

		log_buffer->magic_word = log_buffer_magic_word;
		log_buffer->random_salt = random_value;
		log_buffer->magic_word_salted = log_buffer_magic_word ^ random_value;
		log_buffer->entries = log_buffer_entries;
		log_buffer->current_entry = 0;
	}

	inited = true;

	log("boot");
	log("boot %u", 2);
}

void command_info_log(cli_command_call_t *call)
{
	unsigned int offset;

	assert(inited);

	offset = snprintf(call->result, call->result_size, "logging");

	offset += snprintf(call->result + offset, call->result_size - offset, "\n  buffer: 0x%lx", (uint32_t)log_buffer);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n  magic word: %04lx", log_buffer->magic_word);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n  random salt: %04lx", log_buffer->random_salt);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n  magic word salted: %04lx", log_buffer->magic_word_salted);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n  entries: %u", log_buffer->entries);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n  current entry: %u", log_buffer->current_entry);
}

void command_log(cli_command_call_t *call)
{
	unsigned int offset, current;

	assert(inited);

	offset = snprintf(call->result, call->result_size, "%u entries:", log_buffer->current_entry);

	for(current = 0; (current < log_buffer->current_entry) && (offset < call->result_size); current++)
		offset += snprintf(call->result + offset, call->result_size - offset, "\n%llu %s",
				log_buffer->entry[current].timestamp,
				log_buffer->entry[current].data);
}

void command_log_clear(cli_command_call_t *call)
{
	unsigned int offset;

	assert(inited);

	command_log(call);
	offset = strlen(call->result);
	memset(log_buffer, 0, sizeof(log_t));
	snprintf(call->result + offset, call->result_size - offset, "\nlog cleared");
}
