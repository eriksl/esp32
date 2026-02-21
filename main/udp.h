#pragma once

#include "log.h"
#include "command-response.h"

#include <string>
#include <map>

class Command;

class UDP final
{
	public:

		explicit UDP() = delete;
		explicit UDP(const UDP &) = delete;
		explicit UDP(Log &);

		UDP &get();
		void set(Command *);
		void run();
		void send(const command_response_t *);
		void info(std::string &);

	private:

		static constexpr int mtu = 16 * 1024; // emperically derived

		static UDP *singleton;
		Log &log;
		Command *command;
		int socket_fd;
		std::map<std::string, int> stats;
		bool running;

		[[noreturn]] static void thread_wrapper(void *);
		[[noreturn]] void thread_runner();
};
