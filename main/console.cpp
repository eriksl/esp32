#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>

#include "cli.h"
#include "log.h"
#include "config.h"
#include "util.h"
#include "cli-command.h"
#include "console.h"

#include <freertos/FreeRTOS.h>
#include <driver/usb_serial_jtag.h>

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#include <string>
#include <boost/format.hpp>

enum
{
	max_line_length = 64,
	lines_amount = 8,
	usb_uart_rx_buffer_size = 128,
	usb_uart_tx_buffer_size = 256,
	usb_uart_tx_timeout_ms = 100,
};

static_assert(usb_uart_rx_buffer_size > 64); // required by driver
static_assert(usb_uart_tx_buffer_size > 64); // required by driver
static_assert(pdMS_TO_TICKS(usb_uart_tx_timeout_ms) > 0);

typedef enum
{
	ess_inactive,
	ess_esc_seen,
	ess_bracket_seen,
} escape_sequence_state_t;

static bool inited_1 = false;
static bool inited_2 = false;
static std::string hostname;
static unsigned int current_line;
static std::string line[lines_amount];
static unsigned int console_stats_lines_received;
static unsigned int console_stats_bytes_received;
static unsigned int console_stats_bytes_received_error;
static unsigned int console_stats_lines_sent;
static unsigned int console_stats_bytes_sent;
static unsigned int console_stats_bytes_dropped;

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

static void write_console(const std::string &data)
{
	unsigned int chunk, offset, length;
	const char *raw_data;

	raw_data = data.data();
	length = data.length();
	offset = 0;

	while(length > 0)
	{
		chunk = length;

		if(chunk > usb_uart_tx_buffer_size)
			chunk = usb_uart_tx_buffer_size;

		if(!usb_serial_jtag_write_bytes(raw_data + offset, chunk, pdMS_TO_TICKS(usb_uart_tx_timeout_ms)))
		{
			console_stats_bytes_dropped += length - offset;
			break;
		}

		length -= chunk;
		offset += chunk;
	}
}

static void write_console(const char *data)
{
	std::string str = data;

	write_console(str);
}

static void write_console(char data)
{
	std::string str;

	str.assign(1, data);
	write_console(str);
}

static void prompt(void)
{
	write_console((boost::format("%s [%u]> ") % hostname % current_line).str());
}

static void run_console(void *)
{
	static constexpr char backspace_string[] = { 0x08, 0x20, 0x08, 0x00 };

	static constexpr char reprint_string[] = "^R\n";
	static constexpr char history_string[] = "^@\n";
	static constexpr char interrupt_string[] = "^C\n";
	static constexpr char newline_string[] = "\n";

	assert(inited_2);

	escape_sequence_state_t state;
	char character;
	unsigned int ix;
	bool whitespace;

	prompt();

	for(;;)
	{
		state = ess_inactive;

		line[current_line].clear();

		while(line[current_line].length() < max_line_length)
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
							for(ix = line[current_line].length(); ix > 0; ix--)
								write_console(backspace_string);

							line[current_line].clear();

							if(current_line > 0)
								current_line--;
							else
								current_line = lines_amount - 1;

							write_console(line[current_line]);
							state = ess_inactive;
							continue;
						}

						case('B'):
						{
							for(ix = line[current_line].length(); ix > 0; ix--)
								write_console(backspace_string);

							line[current_line].clear();

							if((current_line + 1) < lines_amount)
								current_line++;
							else
								current_line = 0;

							write_console(line[current_line]);
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
				if(line[current_line].length() > 0)
				{
					line[current_line].pop_back();
					write_console(backspace_string);
				}

				continue;
			}

			if(character == /* ^W */ 0x17)
			{
				for(whitespace = false; line[current_line].length() > 0; line[current_line].pop_back())
				{
					if(whitespace && (line[current_line].back() != ' '))
						break;

					if(line[current_line].back() == ' ')
						whitespace = true;

					write_console(backspace_string);
				}

				continue;
			}

			if(character == /* ^U */ 0x15)
			{
				for(ix = line[current_line].length(); ix > 0; ix--)
					write_console(backspace_string);

				line[current_line].clear();

				continue;
			}

			if(character == /* ^R */ 0x12)
			{
				write_console(reprint_string);
				prompt();
				write_console(line[current_line]);
				continue;
			}

			if(character == /* ^C */ 0x03)
			{
				write_console(interrupt_string);
				line[current_line].clear();
				break;
			}

			if(character == /* ^@ */ 0x00)
			{
				write_console(history_string);

				for(ix = current_line + 1; ix < lines_amount; ix++)
					write_console((boost::format("[%u] %s%s") % ix % line[ix] % newline_string).str());

				for(ix = 0; ix < current_line; ix++)
					write_console((boost::format("[%u] %s%s") % ix % line[ix] % newline_string).str());

				prompt();
				write_console(line[current_line]);

				continue;
			}

			if((character < ' ') || (character > '~'))
				continue;

			write_console(character);
			line[current_line].append(1, character);
		}

		if((line[current_line].length() == 2) && (line[current_line].at(0) == '!'))
		{
			if((line[current_line].at(1) >= '0') && (line[current_line].at(1) <= '7')) // FIXME
				current_line = line[current_line].at(1) - '0';
			else
			{
				if(line[current_line].at(1) == '!')
				{
					if(current_line > 0)
						current_line--;
					else
						current_line = lines_amount - 1;
				}
			}
		}

		if(line[current_line].length() > 0)
		{
			command_response_t *command_response = new command_response_t;

			command_response->source = cli_source_console;
			command_response->mtu = 32768;
			command_response->packetised = 0;
			command_response->packet = line[current_line];
			cli_receive_queue_push(command_response);
			command_response = nullptr;

			if((current_line + 1) < lines_amount)
				current_line++;
			else
				current_line = 0;

			line[current_line].clear();

			write_console(newline_string);
		}
		else
		{
			write_console(newline_string);
			prompt();
		}

		console_stats_lines_received++;
	}
}

void console_init_1(void)
{
	usb_serial_jtag_driver_config_t usb_serial_jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
	unsigned int ix;

	assert(!inited_1);

	current_line = 0;

	for(ix = 0; ix < lines_amount; ix++)
		line[ix].clear();

	usb_serial_jtag_config.rx_buffer_size = usb_uart_rx_buffer_size;
	usb_serial_jtag_config.tx_buffer_size = usb_uart_tx_buffer_size;
	util_abort_on_esp_err("usb_serial_jtag_driver_install", usb_serial_jtag_driver_install(&usb_serial_jtag_config));

	inited_1 = true;
}

void console_init_2(void) // FIXME
{
	assert(inited_1);
	assert(!inited_2);

	try
	{
		hostname = Config::get_string("hostname");
	}
	catch(transient_exception &)
	{
		hostname = "esp32";
	}

	inited_2 = true;

	if(xTaskCreatePinnedToCore(run_console, "console", 3 * 1024, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("console: xTaskCreatePinnedToNode run_console");
}

void console_write_line(const char *string)
{
	if(inited_1)
	{
		write_console(std::string(string));
		write_console("\n");
	}
}

void console_send(const command_response_t *command_response)
{
	if(!command_response)
		return;

	if(inited_1)
		write_console(command_response->packet);

	if(inited_2)
		prompt();

	console_stats_bytes_sent += command_response->packet.length();
	console_stats_lines_sent++;
}

void console_command_info(cli_command_call_t *call)
{
	assert(inited_2);
	assert(call->parameter_count == 0);

	call->result = "received:";
	call->result += (boost::format("\n- lines: %u") % console_stats_lines_received).str();
	call->result += (boost::format("\n- bytes: %u") % console_stats_bytes_received).str();
	call->result += (boost::format("\n- errors: %u") % console_stats_bytes_received_error).str();
	call->result += "\nsent:";
	call->result += (boost::format("\n- lines: %u") % console_stats_lines_sent).str();
	call->result += (boost::format("\n- bytes: %u") % console_stats_bytes_sent).str();
	call->result += (boost::format("\n- dropped: %u") % console_stats_bytes_dropped).str();
}
