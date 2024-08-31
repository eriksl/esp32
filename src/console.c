#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>

#include "string.h"
#include "cli.h"
#include "console.h"
#include "log.h"
#include "config.h"
#include "util.h"
#include "cli-command.h"

#include <freertos/FreeRTOS.h>
#include <driver/usb_serial_jtag.h>

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

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

static bool inited_1 = false;
static bool inited_2 = false;
static lines_t *lines;
static char hostname[16] = "<>";
static unsigned int console_stats_lines_received;
static unsigned int console_stats_bytes_received;
static unsigned int console_stats_bytes_received_error;
static unsigned int console_stats_lines_sent;
static unsigned int console_stats_bytes_sent;

static char read_console(void)
{
	char character;

	for(;;)
	{
		if(usb_serial_jtag_read_bytes(&character, 1, ~0) == 1)
			return(character);

		console_stats_bytes_received_error++;
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

static void write_console(unsigned int length, const char data[])
{
	unsigned int chunk, offset;

	offset = 0;

	for(offset = 0; length > 0; )
	{
		chunk = length;

		if(chunk > 128)
			chunk = 128;

		usb_serial_jtag_write_bytes(&data[offset], chunk, ~0);

		assert(length >= chunk);

		length -= chunk;
		offset += chunk;
	}
}

static void prompt()
{
	string_auto(prompt, 32);

	string_format(prompt, "%s [%u]> ", hostname, lines->current);
	write_console(string_length(prompt), string_cstr(prompt));
}

static void run_console(void *)
{
	static const char backspace_string[] = { 0x08, 0x20, 0x08 };
	static const char reprint_string[] = "^R\n";
	static const char history_string[] = "^@\n";
	static const char interrupt_string[] = "^C\n";
	static const char newline_string[] = "\n";

	assert(inited_2);

	escape_sequence_state_t state;
	line_t *line;
	cli_buffer_t cli_buffer;
	char character;
	unsigned int ix;
	bool whitespace;
	string_auto(tmp, 16);

	prompt();

	for(;;)
	{
		state = ess_inactive;
		line = &lines->line[lines->current];

		for(line->length = 0; line->length < line->size;)
		{
			character = read_console();
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
								write_console(sizeof(backspace_string), backspace_string);

							if(lines->current > 0)
								lines->current--;
							else
								lines->current = lines->size - 1;

							line = &lines->line[lines->current];
							write_console(line->length, line->data);
							state = ess_inactive;
							continue;
						}

						case('B'):
						{
							for(ix = 0; ix < line->length; ix++)
								write_console(sizeof(backspace_string), backspace_string);

							if((lines->current + 1) < lines->size)
								lines->current++;
							else
								lines->current = 0;

							line = &lines->line[lines->current];
							write_console(line->length, line->data);
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
					write_console(sizeof(backspace_string), backspace_string);
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

					write_console(sizeof(backspace_string), backspace_string);
				}

				continue;
			}

			if(character == /* ^U */ 0x15)
			{
				for(; line->length > 0; line->length--)
					write_console(sizeof(backspace_string), backspace_string);
				continue;
			}

			if(character == /* ^R */ 0x12)
			{
				write_console(sizeof(reprint_string), reprint_string);
				prompt();
				write_console(line->length, line->data);
				continue;
			}

			if(character == /* ^C */ 0x03)
			{
				write_console(sizeof(interrupt_string), interrupt_string);
				line->length = 0;
				break;
			}

			if(character == /* ^@ */ 0x00)
			{
				write_console(sizeof(history_string), history_string);

				for(ix = lines->current + 1; ix < lines->size; ix++)
				{
					string_format(tmp, "[%u] ", ix);
					write_console(string_length(tmp), string_cstr(tmp));
					write_console(lines->line[ix].length, lines->line[ix].data);
					write_console(sizeof(newline_string), newline_string);
				}

				for(ix = 0; ix < lines->current; ix++)
				{
					string_format(tmp, "[%u] ", ix);
					write_console(string_length(tmp), string_cstr(tmp));
					write_console(lines->line[ix].length, lines->line[ix].data);
					write_console(sizeof(newline_string), newline_string);
				}

				prompt();
				write_console(line->length, line->data);

				continue;
			}

			if((character < ' ') || (character > '~'))
				continue;

			write_console(1, &character);
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
			cli_buffer.data = string_new(line->length);
			string_assign_data(cli_buffer.data, line->length, (uint8_t *)line->data);

			cli_receive_queue_push(&cli_buffer);

			if((lines->current + 1) < lines->size)
				lines->current++;
			else
				lines->current = 0;

			line = lines[lines->current].line;
			line->length = 0;

			write_console(sizeof(newline_string), newline_string);
		}
		else
		{
			write_console(sizeof(newline_string), newline_string);
			prompt();
		}

		console_stats_lines_received++;
	}
}

void console_init_1()
{
	usb_serial_jtag_driver_config_t usb_serial_jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
	unsigned int ix;
	line_t *line;

	assert(!inited_1);

	lines = util_memory_alloc_spiram(sizeof(lines_t));

	lines->size = line_amount;
	lines->current = 0;

	for(ix = 0; ix < lines->size; ix++)
	{
		line = &lines->line[ix];
		line->size = line_size;
		line->length = 0;
	}

	usb_serial_jtag_config.rx_buffer_size = 128;
	usb_serial_jtag_config.tx_buffer_size = 128;
	util_abort_on_esp_err("usb_serial_jtag_driver_install", usb_serial_jtag_driver_install(&usb_serial_jtag_config));

	inited_1 = true;
}

void console_init_2()
{
	string_auto(hostname_in, 16);
	string_auto_init(hostname_key, "hostname");

	assert(inited_1);
	assert(!inited_2);

	if(config_get_string(hostname_key, hostname_in))
		string_to_cstr(hostname_in, sizeof(hostname), hostname);
	else
		strlcpy(hostname, "esp32", sizeof(hostname));

	inited_2 = true;

	if(xTaskCreatePinnedToCore(run_console, "console", 4096, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("console: xTaskCreatePinnedToNode run_console");
}

void console_write_line(const char *string)
{
	if(inited_1)
	{
		write_console(strlen(string), string);
		write_console(1, "\n");
	}
}

void console_send(const cli_buffer_t *cli_buffer)
{
	if(inited_1)
		write_console(string_length(cli_buffer->data), (const char *)string_data(cli_buffer->data));

	if(inited_2)
		prompt();

	console_stats_bytes_sent += string_length(cli_buffer->data);
	console_stats_lines_sent++;
}

void console_command_info(cli_command_call_t *call)
{
	assert(inited_2);
	assert(call->parameter_count == 0);

	string_format(call->result, "entered:");
	string_format_append(call->result, "\n- lines: %u", console_stats_lines_received);
	string_format_append(call->result, "\n- bytes: %u", console_stats_bytes_received);
	string_format_append(call->result, "\n- errors: %u", console_stats_bytes_received_error);
	string_format_append(call->result, "\nreplies:");
	string_format_append(call->result, "\n- lines: %u", console_stats_lines_sent);
	string_format_append(call->result, "\n- bytes: %u", console_stats_bytes_sent);
}
