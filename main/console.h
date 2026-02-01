#pragma once

#include "config.h"
#include "cli-command.h"

#include <map>
#include <array>
#include <string>

class Console final
{
	public:

		Console() = delete;
		Console(Console &) = delete;

		Console(Config &);
		void send(const command_response_t &);
		void write(std::string_view);
		void run();
		const std::map<std::string, int>& statistics();

		static Console &get();

	private:

		static constexpr unsigned int lines_amount = 8;
		static constexpr unsigned int max_line_length = 64;
		static constexpr unsigned int usb_uart_rx_buffer_size = 128;
		static constexpr unsigned int usb_uart_tx_buffer_size = 256;
		static constexpr unsigned int usb_uart_tx_timeout_ms = 100;

		typedef enum : unsigned int
		{
			ess_inactive,
			ess_esc_seen,
			ess_bracket_seen,
		} escape_sequence_state_t;

		std::map<std::string, int> stats;
		Config &config;
		unsigned int current_line;
		bool running;
		std::string hostname;
		std::array<std::string, lines_amount> lines;

		char read_byte();
		void write_string(std::string_view);
		void prompt();
		void run_thread();
		static void run_thread_wrapper(void *this_);
};
