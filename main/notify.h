#pragma once

#include <string>

class Notify final
{
	public:

		typedef enum : unsigned int
		{
			notify_none = 0,
			notify_sys_booting,
			notify_sys_booting_finished,
			notify_net_associating,
			notify_net_associating_finished,
			notify_net_ipv4_acquired,
			notify_net_ipv6_ll_active,
			notify_net_ipv6_static_active,
			notify_net_ipv6_slaac_acquired,
			notify_net_ap_mode_init,
			notify_net_ap_mode_idle,
			notify_net_ap_mode_associated,
			notify_error,
			notify_size = notify_error,
		} notify_t;

		explicit Notify();
		explicit Notify(Notify &) = delete;

		static Notify &get();

		void run();
		void notify(notify_t);
		void info(std::string &);

	private:

		static constexpr unsigned int phase_size = 4;

		struct phase_t
		{
			unsigned int duty_shift;
			unsigned int time_ms;
			struct
			{
				unsigned int r;
				unsigned int g;
				unsigned int b;
			} colour;
		};

		struct notification_info_t
		{
			phase_t phase[phase_size];
		};

		static const notification_info_t notification_info[notify_size];
		static Notify *singleton;

		bool running;
		bool using_ledpixel;
		bool using_ledpwm;
		notify_t current_notification;
		int current_phase;

		static void run_thread_wrapper(void *);
		void run_thread();
};
