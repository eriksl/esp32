#include "cli-command.h"
#include "console.h"
#include "log.h"
#include "config.h"
#include "util.h"

#include <freertos/FreeRTOS.h>

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

enum
{
	line_size = 64,
	line_amount = 8,
};

typedef enum
{
	ess_inactive,
	ess_esc_seen,
	ess_bracket_seen,
} escape_sequence_state_t;

typedef struct
{
	unsigned int size;
	unsigned int length;
	char data[line_size];
} line_t;

static_assert(sizeof(line_t) == 72);

typedef struct
{
	unsigned int size;
	unsigned int current;
	line_t line[line_amount];
} lines_t;

static_assert(sizeof(lines_t) == 584);

static bool inited = false;
static lines_t *lines;
static char hostname[16] = "<>";
static unsigned int console_stats_lines_received;
static unsigned int console_stats_bytes_received;
static unsigned int console_stats_bytes_received_error;
static unsigned int console_stats_lines_sent;
static unsigned int console_stats_bytes_sent;

static void prompt(unsigned int index)
{
	char prompt[32];

	snprintf(prompt, sizeof(prompt), "%s [%u]> ", hostname, index);
	write(1, prompt, strlen(prompt));
}

static void run_console(void *)
{
	static const char backspace_string[] = { 0x08, 0x20, 0x08 };
	static const char reprint_string[] = "^R\n";
	static const char interrupt_string[] = "^C\n";
	static const char newline_string[] = "\n";

	escape_sequence_state_t state;
	line_t *line;
	cli_buffer_t cli_buffer;
	char character;
	unsigned int ix;
	bool whitespace;

	prompt(lines->current);
	fsync(1);

	for(;;)
	{
		state = ess_inactive;
		line = &lines->line[lines->current];

		for(line->length = 0; line->length < line->size;)
		{
			if(read(0, &character, 1) != 1)
			{
				console_stats_bytes_received_error++;
				vTaskDelay(10 / portTICK_PERIOD_MS);
				continue;
			}

			console_stats_bytes_received++;

			switch(state)
			{
				case(ess_inactive):
				{
					if(character == 0x1b)
					{
						state = ess_esc_seen;
						continue;
					}

					break;
				}

				case(ess_esc_seen):
				{
					if(character == '[')
					{
						state = ess_bracket_seen;
						continue;
					}

					state = ess_inactive;
					break;
				}

				case(ess_bracket_seen):
				{
					switch(character)
					{
						case('A'):
						{
							for(ix = 0; ix < line->length; ix++)
								write(1, backspace_string, sizeof(backspace_string));

							if(lines->current > 0)
								lines->current--;
							else
								lines->current = lines->size - 1;

							line = &lines->line[lines->current];
							write(1, line->data, line->length);
							fsync(1);
							state = ess_inactive;
							continue;
						}

						case('B'):
						{
							for(ix = 0; ix < line->length; ix++)
								write(1, backspace_string, sizeof(backspace_string));

							if(lines->current < lines->size)
								lines->current++;
							else
								lines->current = 0;

							line = &lines->line[lines->current];
							write(1, line->data, line->length);
							fsync(1);
							state = ess_inactive;
							continue;
						}

						default:
						{
							break;
						}
					}

					state = ess_inactive;
					break;
				}
			}

			if((character == '\n') || (character == '\r'))
				break;

			if((character == /* ^H */ 0x08) || (character == /* DEL */ 0x7f))
			{
				if(line->length > 0)
				{
					line->length--;
					write(1, backspace_string, sizeof(backspace_string));
					fsync(1);
				}

				continue;
			}

			if(character == /* ^W */ 0x17)
			{
				for(whitespace = false; line->length > 0; line->length--)
				{
					if(whitespace && line->data[line->length - 1] != ' ')
						break;

					if(line->data[line->length - 1] == ' ')
						whitespace = true;

					write(1, backspace_string, sizeof(backspace_string));
					fsync(1);
				}

				continue;
			}

			if(character == /* ^U */ 0x15)
			{
				for(; line->length > 0; line->length--)
					write(1, backspace_string, sizeof(backspace_string));
				fsync(1);
				continue;
			}

			if(character == /* ^R */ 0x12)
			{
				write(1, reprint_string, sizeof(reprint_string));
				write(1, line->data, line->length);
				fsync(1);
				continue;
			}

			if(character == /* ^C */ 0x03)
			{
				write(1, interrupt_string, sizeof(interrupt_string));
				fsync(1);
				line->length = 0;
				break;
			}

			if((character < ' ') || (character > '~'))
				continue;

			write(1, &character, 1);
			fsync(1);
			line->data[line->length++] = character;
		}

		if((line->length == 2) && (line->data[0] == '!'))
		{
			if((line->data[1] >= '0') && (line->data[1] <= '7')) // FIXME
			{
				lines->current = line->data[1] - '0';
				line = &lines->line[lines->current];
			}
			else
			{
				if(line->data[1] == '!')
				{
					if(lines->current > 0)
						lines->current--;
					else
						lines->current = lines->size - 1;

					line = &lines->line[lines->current];
				}
			}
		}

		if(line->length)
		{
			cli_buffer.source = cli_source_console;
			cli_buffer.length = line->length;
			cli_buffer.data_from_malloc = 1;
			assert((cli_buffer.data = heap_caps_malloc(line->length ? line->length : line->length + 1, MALLOC_CAP_SPIRAM)));
			memcpy(cli_buffer.data, line->data, line->length);
			cli_receive_queue_push(&cli_buffer);

			if(lines->current < lines->size)
				lines->current++;
			else
				lines->current = 0;

			line = lines[lines->current].line;
			line->length = 0;

			write(1, newline_string, sizeof(newline_string));
			fsync(1);
		}
		else
		{
			write(1, newline_string, sizeof(newline_string));
			prompt(lines->current);
			fsync(1);
		}

		console_stats_lines_received++;
	}
}

void console_init()
{
	unsigned int ix;
	line_t *line;

	assert(!inited);

	if(!config_get_string("hostname", sizeof(hostname), hostname))
		strncpy(hostname, "esp32", sizeof(hostname));

	lines = (lines_t *)heap_caps_malloc(sizeof(lines_t) , MALLOC_CAP_SPIRAM);
	assert(lines);

	lines->size = line_amount;
	lines->current = 0;

	for(ix = 0; ix < lines->size; ix++)
	{
		line = &lines->line[ix];
		line->size = line_size;
		line->length = 0;
	}

	inited = true;

	if(xTaskCreatePinnedToCore(run_console, "console", 4096, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("console: xTaskCreatePinnedToNode run_console");
}

void console_write_line(const char *string)
{
	write(1, string, strlen(string));
	write(1, "\n", 1);
	fsync(1);
}

void console_send(const cli_buffer_t *cli_buffer)
{
	assert(inited);

	write(1, cli_buffer->data, cli_buffer->length);
	prompt(lines->current);
	fsync(1);

	console_stats_bytes_sent += cli_buffer->length;
	console_stats_lines_sent++;
}

void command_info_console(cli_command_call_t *call)
{
	unsigned int offset;

	assert(call->parameters->count == 0);

	offset = 0;

	offset += snprintf(call->result + offset, call->result_size - offset, "received:");
	offset += snprintf(call->result + offset, call->result_size - offset, "\n- lines: %u", console_stats_lines_received);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n- bytes: %u", console_stats_bytes_received);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n- errors: %u", console_stats_bytes_received_error);
	offset += snprintf(call->result + offset, call->result_size - offset, "\nsent:");
	offset += snprintf(call->result + offset, call->result_size - offset, "\n- lines: %u", console_stats_lines_sent);
	offset += snprintf(call->result + offset, call->result_size - offset, "\n- bytes: %u", console_stats_bytes_sent);
}
