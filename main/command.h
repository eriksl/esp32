#pragma once

#include "config.h"
#include "console.h"
#include "ledpixel.h"
#include "ledpwm.h"
#include "notify.h"
#include "log.h"
#include "system.h"
#include "pdm.h"
#include "mcpwm.h"
#include "fs.h"
#include "bt.h"
#include "exception.h"
#include "cli-command.h"

#include <string>

class Command final
{
	public:

		static constexpr int parameters_size = 16;

#if 0
		enum cli_parameter_type_description_t
		{
			cli_parameter_none = 0,
			cli_parameter_unsigned_int,
			cli_parameter_signed_int,
			cli_parameter_float,
			cli_parameter_string,
			cli_parameter_string_raw,
			cli_parameter_size,
		};

		struct cli_parameter_t
		{
			cli_parameter_type_description_t type:4;
			unsigned int has_value:1;

			union
			{
				unsigned int	unsigned_int;
				int				signed_int;
				float			fp;
			};

			std::string str;
		};

		struct cli_command_call_t
		{
			cli_source_t		source;
			unsigned int		mtu;
			unsigned int		parameter_count;
			cli_parameter_t		parameters[parameters_size];
			std::string			oob;
			std::string			result;
			std::string			result_oob;
		};
#endif

		static void config_info(cli_command_call_t *);
		static void config_set_int(cli_command_call_t *);
		static void config_set_string(cli_command_call_t *);
		static void config_erase(cli_command_call_t *);
		static void config_dump(cli_command_call_t *);
		static void config_show(cli_command_call_t *);
		static void console_info(cli_command_call_t *);
		static void ledpixel_info(cli_command_call_t *);
		static void ledpwm_info(cli_command_call_t *);
		static void notify_info(cli_command_call_t *);
		static void log_info(cli_command_call_t *);
		static void log_log(cli_command_call_t *);
		static void log_clear(cli_command_call_t *);
		static void log_monitor(cli_command_call_t *);
		static void system_info(cli_command_call_t *);
		static void system_identify(cli_command_call_t *);
		static void system_partitions(cli_command_call_t *);
		static void system_memory(cli_command_call_t *);
		static void system_process_list(cli_command_call_t *);
		static void system_process_stop(cli_command_call_t *);
		static void util_info(cli_command_call_t *);
		static void util_timezone(cli_command_call_t *);
		static void pdm_info(cli_command_call_t *);
		static void mcpwm_info(cli_command_call_t *);
		static void fs_list(cli_command_call_t *);
		static void fs_format(cli_command_call_t *);
		static void fs_read(cli_command_call_t *);
		static void fs_write(cli_command_call_t *);
		static void fs_erase(cli_command_call_t *);
		static void fs_rename(cli_command_call_t *);
		static void fs_truncate(cli_command_call_t *);
		static void fs_checksum(cli_command_call_t *);
		static void fs_info(cli_command_call_t *);
		static void command_help(cli_command_call_t *);
		static void hostname(cli_command_call_t *);
		[[noreturn]] static void reset(cli_command_call_t *);
		static void write(cli_command_call_t *);
		static void info(cli_command_call_t *);
		static void bluetooth_info(cli_command_call_t *);
		static void bluetooth_key(cli_command_call_t *);
		static void ota_start(cli_command_call_t *);
		static void ota_write(cli_command_call_t *);
		static void ota_finish(cli_command_call_t *);
		static void ota_commit(cli_command_call_t *);
		static void ota_confirm(cli_command_call_t *);
		static void wlan_client_config(cli_command_call_t *);
		static void wlan_info(cli_command_call_t *);
		static void wlan_ipv6_static(cli_command_call_t *);
		static void display_brightness(cli_command_call_t *);
		static void display_configure(cli_command_call_t *);
		static void display_erase(cli_command_call_t *);
		static void display_info(cli_command_call_t *);
		static void display_page_add_text(cli_command_call_t *);
		static void display_page_add_image(cli_command_call_t *);
		static void display_page_remove(cli_command_call_t *);
		static void i2c_info(cli_command_call_t *);
		static void i2c_speed(cli_command_call_t *);
		static void sensor_dump(cli_command_call_t *);
		static void sensor_info(cli_command_call_t *);
		static void sensor_json(cli_command_call_t *);
		static void sensor_stats(cli_command_call_t *);
		static void io_dump(cli_command_call_t *);
		static void io_read(cli_command_call_t *);
		static void io_stats(cli_command_call_t *);
		static void io_write(cli_command_call_t *);
		static void alias(cli_command_call_t *);
		static void run(cli_command_call_t *);
		static void udp_info(cli_command_call_t *);
		static void tcp_info(cli_command_call_t *);

		Command(Config &, Console &, Ledpixel &, LedPWM &, Notify &, Log &, System &, Util &, PDM &, MCPWM &, FS &, BT &);
		Command() = delete;
		Command(const Command &) = delete;

		static Command &get();

		void run();
		void receive_queue_push(command_response_t *);

	private:

		static constexpr int receive_queue_size = 8;
		static constexpr int send_queue_size = 8;

		typedef void(cli_command_function_t)(cli_command_call_t *);

		struct cli_unsigned_int_description_t
		{
			unsigned int lower_bound;
			unsigned int upper_bound;
		};

		struct cli_signed_int_description_t
		{
			int lower_bound;
			int upper_bound;
		};

		struct cli_float_description_t
		{
			float lower_bound;
			float upper_bound;
		};

		struct cli_string_description_t
		{
			unsigned int lower_length_bound;
			unsigned int upper_length_bound;
		};

		struct cli_parameter_description_t
		{
			cli_parameter_type_description_t type:4;

			unsigned int base:5;
			unsigned int value_required:1;
			unsigned int lower_bound_required:1;
			unsigned int upper_bound_required:1;

			const char *description;

			union
			{
				cli_unsigned_int_description_t	unsigned_int;
				cli_signed_int_description_t	signed_int;
				cli_float_description_t			fp;
				cli_string_description_t		string;
			};
		};

		struct cli_parameters_description_t
		{
			unsigned int count;
			cli_parameter_description_t entries[parameters_size];
		};

		struct cli_command_t
		{
			const char *name;
			const char *alias;
			const char *help;
			cli_command_function_t *function;
			cli_parameters_description_t parameters_description;
		};

		static const std::map<cli_parameter_type_description_t, std::string> parameter_type_to_string;
		static const cli_command_t cli_commands[];

		//FIXME
		static int cli_stats_commands_received;
		static int cli_stats_commands_received_packet;
		static int cli_stats_commands_received_raw;
		static int cli_stats_replies_sent;
		static int cli_stats_replies_sent_packet;
		static int cli_stats_replies_sent_raw;

		static Command *singleton;
		static Config *config_;
		static Console *console_;
		static Ledpixel *ledpixel_;
		static LedPWM *ledpwm_;
		static Notify *notify_;
		static Log *log_;
		static System *system_;
		static Util *util_;
		static PDM *pdm_;
		static MCPWM *mcpwm_;
		static FS *fs_;
		static BT *bt_;

		typedef std::map<std::string, std::string> string_string_map;
		string_string_map aliases;

		Config &config;
		Console &console;
		Ledpixel &ledpixel;
		LedPWM &ledpwm;
		Notify &notify;
		Log &log;
		System &system;
		Util &util;
		PDM &pdm;
		MCPWM &mcpwm;
		FS &fs;
		BT &bt;

		QueueHandle_t receive_queue_handle;
		QueueHandle_t send_queue_handle;
		bool running;

		std::string make_exception_text(std::string_view fn, std::string_view message1, std::string_view message2);
		void help(std::string out, const std::string &filter = "");
		command_response_t *receive_queue_pop();
		void send_queue_push(command_response_t *command_response);
		command_response_t *send_queue_pop();
		[[noreturn]] static void run_receive_queue_wrapper(void *);
		[[noreturn]] void run_receive_queue();
		[[noreturn]] static void run_send_queue_wrapper(void *);
		[[noreturn]] void run_send_queue();
		void alias_command(cli_command_call_t *call);
		void alias_expand(std::string &data) const;
};
