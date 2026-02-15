#include "console.h"

#include "exception.h"
#include "util.h"
#include "log.h"
#include "command.h"

#include <esp_pthread.h>
#include <driver/usb_serial_jtag.h>

#include <thread>
#include <format>

Console *Console::singleton = nullptr;

Console::Console(Config &config_in) :
		config(config_in),
		current_line(0),
		running(false)
{
	esp_err_t rv;
	usb_serial_jtag_driver_config_t usb_serial_jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();

	if(this->singleton)
		throw(hard_exception("Console: already active"));

	usb_serial_jtag_config.rx_buffer_size = this->usb_uart_rx_buffer_size;
	usb_serial_jtag_config.tx_buffer_size = this->usb_uart_tx_buffer_size;

	if((rv = usb_serial_jtag_driver_install(&usb_serial_jtag_config)) != ESP_OK)
		throw(hard_exception("Console: error in usb_serial_jtag_driver_install"));

	try
	{
		this->hostname = this->config.get_string("hostname");
	}
	catch(transient_exception &)
	{
		this->hostname = "esp32";
	}

	this->singleton = this;
	this->command = nullptr;
}

void Console::set(Command *cmd)
{
	if(this->command)
		throw(hard_exception("Console::set(Command): already set"));

	this->command = cmd;
}

char Console::read_byte(void)
{
	char byte;

	for(;;)
	{
		if(usb_serial_jtag_read_bytes(&byte, 1, ~0) == 1)
			return(byte);

		this->stats["errors in receive"]++;
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

void Console::write_string(std::string_view data)
{
	unsigned int chunk, offset, length;
	const char *raw_data;

	raw_data = data.data();
	length = data.length();
	offset = 0;

	while(length > 0)
	{
		chunk = length;

		if(chunk > this->usb_uart_tx_buffer_size)
			chunk = this->usb_uart_tx_buffer_size;

		if(!usb_serial_jtag_write_bytes(raw_data + offset, chunk, pdMS_TO_TICKS(this->usb_uart_tx_timeout_ms)))
		{
			this->stats["errors in send"] += length - offset;
			break;
		}

		length -= chunk;
		offset += chunk;
	}
}

void Console::run_thread()
{
	static constexpr char backspace_string[] = { 0x08, 0x20, 0x08, 0x00 };
	static constexpr char reprint_string[] = "^R\n";
	static constexpr char history_string[] = "^@\n";
	static constexpr char interrupt_string[] = "^C\n";
	static constexpr char newline_string[] = "\n";

	escape_sequence_state_t state;
	char byte;
	unsigned int ix;
	bool whitespace;
	std::string *line;

	this->prompt();

	try
	{
		for(;;)
		{
			state = escape_sequence_state_t::ess_inactive;

			line = &lines[this->current_line];
			line->clear();

			while(line->length() < this->max_line_length)
			{
				byte = this->read_byte();
				this->stats["received bytes"]++;

				switch(state)
				{
					case(escape_sequence_state_t::ess_inactive):
					{
						if(byte == 0x1b)
						{
							state = escape_sequence_state_t::ess_esc_seen;
							continue;
						}

						break;
					}

					case(escape_sequence_state_t::ess_esc_seen):
					{
						if(byte == '[')
						{
							state = escape_sequence_state_t::ess_bracket_seen;
							continue;
						}

						state = escape_sequence_state_t::ess_inactive;
						break;
					}

					case(escape_sequence_state_t::ess_bracket_seen):
					{
						switch(byte)
						{
							case('A'):
							{
								for(ix = line->length(); ix > 0; ix--)
									this->write_string(backspace_string);

								if(this->current_line > 0)
									this->current_line--;
								else
									this->current_line = this->lines_amount - 1;

								line = &lines[this->current_line];
								this->write_string(*line);
								state = escape_sequence_state_t::ess_inactive;
								continue;
							}

							case('B'):
							{
								for(ix = line->length(); ix > 0; ix--)
									this->write_string(backspace_string);

								if((this->current_line + 1) < this->lines_amount)
									this->current_line++;
								else
									this->current_line = 0;

								line = &lines[this->current_line];
								this->write_string(*line);
								state = escape_sequence_state_t::ess_inactive;
								continue;
							}

							default:
							{
								break;
							}
						}

						state = escape_sequence_state_t::ess_inactive;
						break;
					}
				}

				if((byte == '\n') || (byte == '\r'))
					break;

				if((byte == /* ^H */ 0x08) || (byte == /* DEL */ 0x7f))
				{
					if(line->length() > 0)
					{
						line->pop_back();
						this->write_string(backspace_string);
					}

					continue;
				}

				if(byte == /* ^W */ 0x17)
				{
					for(whitespace = false; line->length() > 0; line->pop_back())
					{
						if(whitespace && (line->back() != ' '))
							break;

						if(line->back() == ' ')
							whitespace = true;

						this->write_string(backspace_string);
					}

					continue;
				}

				if(byte == /* ^U */ 0x15)
				{
					for(ix = line->length(); ix > 0; ix--)
						this->write_string(backspace_string);

					line->clear();

					continue;
				}

				if(byte == /* ^R */ 0x12)
				{
					this->write_string(reprint_string);
					this->prompt();
					this->write_string(*line);
					continue;
				}

				if(byte == /* ^C */ 0x03)
				{
					this->write_string(interrupt_string);
					line->clear();
					break;
				}

				if(byte == /* ^@ */ 0x00)
				{
					this->write_string(history_string);

					for(ix = this->current_line + 1; ix < this->lines_amount; ix++)
						this->write_string(std::format("[{:d}] {}\n", ix, this->lines[ix]));

					for(ix = 0; ix < this->current_line; ix++)
						this->write_string(std::format("[{:d}] {}\n", ix, this->lines[ix]));

					this->prompt();
					this->write_string(*line);

					continue;
				}

				if((byte < ' ') || (byte > '~'))
					continue;

				this->write_string(std::string(1, byte));
				line->append(1, byte);
			}

			if((line->length() == 2) && ((*line)[0] == '!'))
			{
				if(((*line)[1] >= '0') && ((*line)[1] <= '7')) // FIXME
					this->current_line = (*line)[1] - '0';
				else
				{
					if((*line)[1] == '!')
					{
						if(this->current_line > 0)
							current_line--;
						else
							this->current_line = this->lines_amount - 1;

						line = &lines[this->current_line];
					}
				}
			}

			if(line->length() > 0)
			{
				command_response_t *command_response = new command_response_t;

				command_response->source = cli_source_console;
				command_response->mtu = 32768;
				command_response->packetised = 0;
				command_response->packet = *line;
				this->command->receive_queue_push(command_response);
				command_response = nullptr;

				if((this->current_line + 1) < this->lines_amount)
					this->current_line++;
				else
					this->current_line = 0;

				line = &lines[this->current_line];
				line->clear();

				this->write_string(newline_string);
			}
			else
			{
				this->write_string(newline_string);
				this->prompt();
			}

			this->stats["received lines"]++;
		}
	}
	catch(const hard_exception &e)
	{
		Log::get().abort(std::format("console thread: hard exception: {}", e.what()).c_str());
	}
	catch(const transient_exception &e)
	{
		Log::get().abort(std::format("console thread: transient exception: {}", e.what()).c_str());
	}
	catch(...)
	{
		Log::get().abort("console thread: unknown exception");
	}
}

void Console::run_thread_wrapper(void *this_)
{
	Console *console;

	if(!this_)
		throw(hard_exception("Console::run_thread_wrapper: nullptr passed"));

	console = reinterpret_cast<Console *>(this_);

	console->run_thread();
}

void Console::run(void)
{
	esp_err_t rv;
	esp_pthread_cfg_t thread_config = esp_pthread_get_default_config();

	if(this->running)
		throw(hard_exception("Console::run: already running"));

	if(!this->command)
		throw(hard_exception("Console::run: command module not linked"));

	thread_config.thread_name = "console";
	thread_config.pin_to_core = 1;
	thread_config.stack_size = 3 * 1024;
	thread_config.prio = 1;
	thread_config.stack_alloc_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

	if((rv = esp_pthread_set_cfg(&thread_config)) != ESP_OK)
		throw(hard_exception(Log::get().esp_string_error(rv, "Console::run: esp_pthread_set_cfg")));

	std::thread new_thread(this->run_thread_wrapper, this);
	new_thread.detach();

	this->running = true;
}

void Console::prompt(void)
{
	this->write_string(std::format("{} [{:d}]> ", this->hostname, this->current_line));
}

Console &Console::get()
{
	if(!Console::singleton)
		throw(hard_exception("Console::get: not active"));

	return(*Console::singleton);
}

void Console::write(std::string_view string)
{
	if(Console::singleton)
	{
		this->write_string(string);
		this->write_string("\n");
	}
}

void Console::send(const command_response_t &command_response)
{
	this->write_string(command_response.packet);

	if(this->running)
		this->prompt();

	this->stats["sent bytes"] += command_response.packet.length();
	this->stats["sent lines"]++;
}

void Console::emergency_wall(std::string_view text_in)
{
	std::string text;

	text = text_in;
	text += "\n";

	usb_serial_jtag_driver_config_t usb_serial_jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
	usb_serial_jtag_config.rx_buffer_size = 128;
	usb_serial_jtag_config.tx_buffer_size = 128;
	usb_serial_jtag_driver_install(&usb_serial_jtag_config);

	usb_serial_jtag_write_bytes(text.c_str(), text.length(), -1);
}

void Console::info(std::string &dst)
{
	bool not_first = false;
	std::string key;

	for(const auto &one_stat : this->stats)
	{
		if(not_first)
			dst += "\n";

		not_first = true;

		key = one_stat.first + ":";

		dst += std::format("- {} {:d}", key, one_stat.second);
	}
}
