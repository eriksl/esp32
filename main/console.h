#pragma once

#include "config.h"
#include "command-response.h"

#include <map>
#include <array>
#include <string>

#include <freertos/FreeRTOS.h> // for vTaskDelay

class Command;

class Console final
{
	public:

		Console() = delete;
		Console(Console &) = delete;

		Console(Config &);
		void send(const command_response_t &);
		void write(std::string_view);
		void run();
		void info(std::string &dst);

		static void emergency_wall(std::string_view text);

		void set(Command *);
		static Console &get();

	private:

		enum class escape_sequence_state_t : unsigned int
		{
			ess_inactive,
			ess_esc_seen,
			ess_bracket_seen,
		};

		static constexpr unsigned int lines_amount = 8;
		static constexpr unsigned int max_line_length = 64;
		static constexpr unsigned int usb_uart_rx_buffer_size = 128;
		static constexpr unsigned int usb_uart_tx_buffer_size = 256;
		static constexpr unsigned int usb_uart_tx_timeout_ms = 100;

		static Console *singleton;

		std::map<std::string, int> stats;
		Config &config;
		Command *command;
		unsigned int current_line;
		bool running;
		std::string hostname;
		std::array<std::string, lines_amount> lines;

		char read_byte();
		void write_string(std::string_view);
		void prompt();
		void run_thread();
		static void run_thread_wrapper(void *);

		static_assert(usb_uart_rx_buffer_size > 64); // required by driver
		static_assert(usb_uart_tx_buffer_size > 64); // required by driver
		static_assert(pdMS_TO_TICKS(usb_uart_tx_timeout_ms) > 0);
};
