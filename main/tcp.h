#pragma once

#include "log.h"
#include "command-response.h"

#include <string>
#include <map>

class Command;

class TCP final
{
	public:

		explicit TCP() = delete;
		explicit TCP(const TCP &) = delete;
		explicit TCP(Log &);

		TCP &get();
		void set(Command *);
		void run();
		void send(const command_response_t *);
		void info(std::string &);

	private:

		static constexpr int mtu = 16 * 1024; // emperically determined

		static TCP *singleton;
		Log &log;
		Command *command;
		int socket_fd;
		std::map<std::string, int> stats;
		bool running;

		[[noreturn]] static void thread_wrapper(void *);
		[[noreturn]] void thread_runner();
};
