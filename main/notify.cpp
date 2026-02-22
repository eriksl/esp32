#include "notify.h"

#include "ledpixel.h"
#include "ledpwm.h"
#include "exception.h"
#include "util.h"
#include "log.h"

#include "format"
#include "thread"

#include "magic_enum/magic_enum.hpp"

#include <esp_pthread.h>

const Notify::notification_info_t Notify::notification_info[Notify::notification_size] =
{
	{
		.notification = Notify::Notification::none,
		.phase =
		{
			{ .duty_shift =  0, .time_ms =    0, .colour = { 0x00, 0x00, 0x00 }},
			{ .duty_shift =  0, .time_ms =    0, .colour = { 0x00, 0x00, 0x00 }},
			{ .duty_shift =  0, .time_ms =    0, .colour = { 0x00, 0x00, 0x00 }},
			{ .duty_shift =  0, .time_ms =    0, .colour = { 0x00, 0x00, 0x00 }},
		},
	},
	{
		.notification = Notify::Notification::sys_booting,
		.phase =
		{
			{ .duty_shift = 14, .time_ms =   50, .colour = { 0xff, 0x00, 0x00 }},
			{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0x00, 0x00 }},
			{ .duty_shift = 14, .time_ms =   50, .colour = { 0xff, 0x00, 0x00 }},
			{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0x00, 0x00 }},
		},
	},
	{
		.notification = Notify::Notification::sys_booting_finished,
		.phase =
		{
			{ .duty_shift = 14, .time_ms =  300, .colour = { 0xff, 0x00, 0x00 }},
			{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0x00, 0x00 }},
			{ .duty_shift = 14, .time_ms =  300, .colour = { 0xff, 0x00, 0x00 }},
			{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0x00, 0x00 }},
		},
	},
	{
		.notification = Notify::Notification::net_associating,
		.phase =
		{
			{ .duty_shift =  9, .time_ms =   50, .colour = { 0x00, 0x00, 0xff }},
			{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0x00, 0x00 }},
			{ .duty_shift =  9, .time_ms =   50, .colour = { 0x00, 0x00, 0xff }},
			{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0x00, 0x00 }},
		},
	},
	{
		.notification = Notify::Notification::net_associating_finished,
		.phase =
		{
			{ .duty_shift =  9, .time_ms =  300, .colour = { 0x00, 0x00, 0xff }},
			{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0x00, 0x00 }},
			{ .duty_shift =  9, .time_ms =  300, .colour = { 0x00, 0x00, 0xff }},
			{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0x00, 0x00 }},
		},
	},
	{
		.notification = Notify::Notification::net_ipv4_acquired,
		.phase =
		{
			{ .duty_shift =  9, .time_ms = 1000, .colour = { 0x01, 0x01, 0x00 }},
			{ .duty_shift =  0, .time_ms = 1000, .colour = { 0x00, 0x00, 0x00 }},
			{ .duty_shift =  9, .time_ms = 1000, .colour = { 0x01, 0x01, 0x00 }},
			{ .duty_shift =  0, .time_ms = 1000, .colour = { 0x00, 0x00, 0x00 }},
		},
	},
	{
		.notification = Notify::Notification::net_ipv6_ll_active,
		.phase =
		{
			{ .duty_shift =  5, .time_ms =   50, .colour = { 0x00, 0x01, 0x00 }},
			{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0x00, 0x00 }},
			{ .duty_shift =  5, .time_ms =   50, .colour = { 0x00, 0x01, 0x00 }},
			{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0x00, 0x00 }},
		},
	},
	{
		.notification = Notify::Notification::net_ipv6_static_active,
		.phase =
		{
			{ .duty_shift =  5, .time_ms =  300, .colour = { 0x00, 0x01, 0x00 }},
			{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0x00, 0x00 }},
			{ .duty_shift =  5, .time_ms =  300, .colour = { 0x00, 0x01, 0x00 }},
			{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0x00, 0x00 }},
		},
	},
	{
		.notification = Notify::Notification::net_ipv6_slaac_acquired,
		.phase =
		{
			{ .duty_shift =  5, .time_ms = 1000, .colour = { 0x00, 0x01, 0x00 }},
			{ .duty_shift =  0, .time_ms = 1000, .colour = { 0x00, 0x00, 0x00 }},
			{ .duty_shift =  5, .time_ms = 1000, .colour = { 0x00, 0x01, 0x00 }},
			{ .duty_shift =  0, .time_ms = 1000, .colour = { 0x00, 0x00, 0x00 }},
		},
	},
	{
		.notification = Notify::Notification::net_ap_mode_init,
		.phase =
		{
			{ .duty_shift = 14, .time_ms =  100, .colour = { 0xff, 0x00, 0xff }},
			{ .duty_shift = 12, .time_ms =  100, .colour = { 0x00, 0x00, 0x00 }},
			{ .duty_shift = 14, .time_ms =  100, .colour = { 0xff, 0xff, 0xff }},
			{ .duty_shift = 12, .time_ms =  100, .colour = { 0x00, 0x00, 0xff }},
		},
	},
	{
		.notification = Notify::Notification::net_ap_mode_idle,
		.phase =
		{
			{ .duty_shift = 14, .time_ms =  500, .colour = { 0xff, 0x00, 0xff }},
			{ .duty_shift = 12, .time_ms =  500, .colour = { 0x00, 0x00, 0x00 }},
			{ .duty_shift = 14, .time_ms =  500, .colour = { 0xff, 0xff, 0xff }},
			{ .duty_shift = 12, .time_ms =  500, .colour = { 0x00, 0x00, 0x00 }},
		},
	},
	{
		.notification = Notify::Notification::net_ap_mode_associated,
		.phase =
		{
			{ .duty_shift = 14, .time_ms = 1200, .colour = { 0xff, 0x00, 0xff }},
			{ .duty_shift = 12, .time_ms = 1200, .colour = { 0x00, 0x00, 0x00 }},
			{ .duty_shift = 14, .time_ms = 1200, .colour = { 0xff, 0xff, 0xff }},
			{ .duty_shift = 12, .time_ms = 1200, .colour = { 0x00, 0x00, 0x00 }},
		},
	},
};

Notify *Notify::singleton = nullptr;

Notify::Notify() :
	running(false),
	current_notification(Notification::none),
	current_phase(-1)
{
	static_assert(magic_enum::enum_count<Notification>() == this->notification_size);

	if(this->singleton)
		throw(hard_exception("Notify: already active"));

#if (CONFIG_BSP_LEDPIXEL0 >= 0)
	this->using_ledpixel = true;
#else
	this->using_ledpixel = false;
#endif

#if (CONFIG_BSP_LEDPWM0 >= 0)
	this->using_ledpwm = true;
#else
	this->using_ledpwm = false;
#endif

	if(this->using_ledpixel)
	{
		try
		{
			Ledpixel::get().open(Ledpixel::Channel::channel_0_notify, "notification LED");
		}
		catch(const e32if_exception &e)
		{
			throw(hard_exception(std::format("Notify: Ledpixel.open: {}", e.what())));
		}
	}

	if(this->using_ledpwm)
	{
		try
		{
			LedPWM::get().open(LedPWM::Channel::channel_14bit_5khz_notify, "notification LED");
		}
		catch(const e32if_exception &e)
		{
			throw(hard_exception(std::format("Notify: LedPWM.open: {}", e.what())));
		}
	}

	this->singleton = this;
}

void Notify::run()
{
	esp_err_t rv;
	esp_pthread_cfg_t thread_config = esp_pthread_get_default_config();

	if(this->running)
		throw(hard_exception("Notify::run: already running"));

	if(!this->using_ledpwm && !this->using_ledpixel)
		return;

	thread_config.thread_name = "notify";
	thread_config.pin_to_core = 1;
	thread_config.stack_size = 1500;
	thread_config.prio = 1;
	thread_config.stack_alloc_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

	if((rv = esp_pthread_set_cfg(&thread_config)) != ESP_OK)
		throw(hard_exception("Notify::run: esp_pthread_set_cfg"));

	std::thread new_thread([this]() { this->thread_runner(); });
	new_thread.detach();

	this->running = true;
}

void Notify::thread_runner()
{
	const notification_info_t *info_ptr;
	const phase_t *phase_ptr;
	int sleep_ms = 0;

	try
	{
		for(;;)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));

			if(!magic_enum::enum_contains<Notification>(this->current_notification))
			{
				sleep_ms = 100;
				continue;
			}

			this->current_phase++;

			if((this->current_phase < 0) || (this->current_phase >= this->phase_size))
				this->current_phase = 0;

			info_ptr = &this->notification_info[magic_enum::enum_integer(this->current_notification)];

			assert(info_ptr->notification == this->current_notification);

			phase_ptr = &info_ptr->phase[this->current_phase];

			if(this->using_ledpixel)
			{
				Ledpixel::get().set(Ledpixel::Channel::channel_0_notify, 0, phase_ptr->colour.r, phase_ptr->colour.g, phase_ptr->colour.b);
				Ledpixel::get().flush(Ledpixel::Channel::channel_0_notify);
			}

			if(this->using_ledpwm)
				LedPWM::get().set(LedPWM::Channel::channel_14bit_5khz_notify, (1UL << phase_ptr->duty_shift) - 1);

			sleep_ms = phase_ptr->time_ms ? : 100;
		}
	}
	catch(const hard_exception &e)
	{
		Console::emergency_wall(std::format("notify thread: hard exception: {}", e.what()).c_str());
		::abort();
	}
	catch(const transient_exception &e)
	{
		Console::emergency_wall(std::format("notify thread: transient exception: {}", e.what()).c_str());
		::abort();
	}
	catch(...)
	{
		Console::emergency_wall("notify thread: unknown exception");
		::abort();
	}
}

void Notify::notify(Notification notification)
{
	if(!magic_enum::enum_contains<Notification>(notification))
		throw(hard_exception("Notify::notify:: invalid notification"));

	if(notification == Notification::none)
		return;

	this->current_notification = notification;
	this->current_phase = -1;
}

Notify &Notify::get()
{
	if(!Notify::singleton)
		throw(hard_exception("Notify::get: not active"));

	return(*Notify::singleton);
}

void Notify::info(std::string &dst)
{
	const notification_info_t *info_ptr;

	info_ptr = &this->notification_info[magic_enum::enum_integer(this->current_notification)];

	assert(info_ptr->notification == this->current_notification);

	dst += std::format("ledpixel enabled: {}\n", this->using_ledpixel ? "yes" : "no");
	dst += std::format("ledpwm   enabled: {}\n", this->using_ledpwm   ? "yes" : "no");
	dst += std::format("thread running: {}\n", this->running ? "yes" : "no");
	dst += std::format("current notification: {:d}: {}\n", magic_enum::enum_integer(this->current_notification), magic_enum::enum_name(this->current_notification));
	dst += std::format("- duty: {:d}\n", (1UL << info_ptr->phase[this->current_phase].duty_shift) - 1);
	dst += std::format("- sleep time: {:d} ms\n", info_ptr->phase[this->current_phase].time_ms);
	dst += std::format("- red   component: {:#04x}\n", info_ptr->phase[this->current_phase].colour.r);
	dst += std::format("- green component: {:#04x}\n", info_ptr->phase[this->current_phase].colour.g);
	dst += std::format("- blue  component: {:#04x}\n", info_ptr->phase[this->current_phase].colour.b);
}
