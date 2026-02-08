#pragma once

#include "log.h"

#include <cstdint>
#include <string>
#include <array>

class System final
{
	public:

		explicit System() = delete;
		explicit System(Log &);
		explicit System(System &) = delete;

		System& get();

		void identify(std::string &out, int mtu);
		void info(std::string &out);
		void memory(std::string &out);
		void partitions(std::string &dst, int partition = -1);
		void process_list(std::string &out, int core = -1);
		bool process_kill(const std::string &name);

		int get_initial_free_heap();
		int get_initial_free_spiram();
		int get_initial_free_internal();
		int get_initial_free_total();
		int get_initial_free_rtcram();

	private:

		static constexpr unsigned int task_id_size = 48;

		typedef struct
		{
			unsigned int task_id;
			std::int64_t previous_runtime;
		} task_info_cache_t;

		static System *singleton;
		Log &log;

		int initial_free_heap;
		int initial_free_spiram;
		int initial_free_internal;
		int initial_free_total;
		int initial_free_rtcram;

		std::array<task_info_cache_t, task_id_size> task_info_cache;
};
