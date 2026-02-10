#pragma once

#include "console.h"
#include "util.h"

#include <cstdint>
#include <string>

#include <freertos/FreeRTOS.h> // for QueueHandle_t and SemaphoreHandle_t

#include <time.h>

#include <esp_err.h>
#include <sdkconfig.h>

class Log final
{
	public:

		Log() = delete;
		Log(const Log &) = delete;
		Log(Console &, Util &);

		void log(std::string_view);
		void log_esperr(esp_err_t, std::string_view);
		void log_errno(int /*errno*/, std::string_view);
		Log &operator << (std::string_view in)
		{
			Log::log(in);
			return(*this);
		}
		void warn_on_esp_err(std::string_view what, unsigned int rv);
		void abort_on_esp_err(std::string_view what, int rv);
		void abort(std::string_view what);
		std::string esp_string_error(esp_err_t e, std::string_view message);

		void clear();
		void setmonitor(bool);
		bool getmonitor();
		QueueHandle_t get_display_queue();
		void get_entry(int entry, time_t &, std::string &);
		void info(std::string &);
		void command_log(std::string &, int);

		static Log &get();

	private:

		static constexpr unsigned int log_buffer_size = 8192 - 32 - CONFIG_ULP_COPROC_RESERVE_MEM;
		static constexpr unsigned int log_buffer_entries = 62;
		static constexpr unsigned int log_buffer_data_size = 120;
		static constexpr unsigned int log_buffer_magic_word = 0x4afbcafe;

		struct log_entry_t
		{
			time_t timestamp;
			char data[log_buffer_data_size];
		};

		typedef struct
		{
			std::uint32_t magic_word;
			std::uint32_t random_salt;
			std::uint32_t magic_word_salted;
			int entries;
			int in;
			int out;
			log_entry_t entry[log_buffer_entries];
		} log_t;

		static Log *singleton;

		static char rtc_slow_memory[log_buffer_size];

		Console &console;
		Util &util;

		bool monitor;
		log_t *log_buffer;
		QueueHandle_t display_queue;
		SemaphoreHandle_t data_mutex;

		static_assert(sizeof(log_entry_t) == 128);
		static_assert(sizeof(log_t) == 7960);
		static_assert(sizeof(log_t) < log_buffer_size);

		void data_mutex_take(void)
		{
			xSemaphoreTake(data_mutex, portMAX_DELAY);
		}

		void data_mutex_give(void)
		{
			xSemaphoreGive(data_mutex);
		}

		static int idf_logging_function(const char *fmt, va_list ap);

		void signal_display(int item);
};
