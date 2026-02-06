#pragma once

#include <string>

class Notify final
{
	public:

		enum class Notification : unsigned int
		{
			none = 0,
			sys_booting,
			sys_booting_finished,
			net_associating,
			net_associating_finished,
			net_ipv4_acquired,
			net_ipv6_ll_active,
			net_ipv6_static_active,
			net_ipv6_slaac_acquired,
			net_ap_mode_init,
			net_ap_mode_idle,
			net_ap_mode_associated,
		};

		explicit Notify();
		explicit Notify(Notify &) = delete;

		static Notify &get();

		void run();
		void notify(Notification);
		void info(std::string &);

	private:

		static constexpr unsigned int phase_size = 4;
		static constexpr unsigned int notification_size = 12;

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
			Notification notification;
			phase_t phase[phase_size];
		};

		static const notification_info_t notification_info[notification_size];
		static Notify *singleton;

		bool running;
		bool using_ledpixel;
		bool using_ledpwm;
		Notification current_notification;
		int current_phase;

		static void run_thread_wrapper(void *);
		void run_thread();
};
