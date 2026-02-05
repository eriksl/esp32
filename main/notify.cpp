#include "notify.h"

#include "ledpixel.h"
#include "ledpwm.h"
#include "exception.h"
#include "util.h"

#include "format"
#include "thread"

#include <esp_pthread.h>

const Notify::notification_info_t Notify::notification_info[Notify::notify_size] =
{
	[notify_none] = {{
		{ .duty_shift =  0, .time_ms =    0, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift =  0, .time_ms =    0, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift =  0, .time_ms =    0, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift =  0, .time_ms =    0, .colour = { 0x00, 0x00, 0x00 }},
	}},
	[notify_sys_booting] = {{
		{ .duty_shift = 14, .time_ms =   50, .colour = { 0xff, 0x00, 0x00 }},
		{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift = 14, .time_ms =   50, .colour = { 0xff, 0x00, 0x00 }},
		{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0x00, 0x00 }},
	}},
	[notify_sys_booting_finished] = {{
		{ .duty_shift = 14, .time_ms =  300, .colour = { 0xff, 0x00, 0x00 }},
		{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift = 14, .time_ms =  300, .colour = { 0xff, 0x00, 0x00 }},
		{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0x00, 0x00 }},
	}},
	[notify_net_associating] = {{
		{ .duty_shift =  9, .time_ms =   50, .colour = { 0x00, 0x00, 0xff }},
		{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift =  9, .time_ms =   50, .colour = { 0x00, 0x00, 0xff }},
		{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0x00, 0x00 }},
	}},
	[notify_net_associating_finished] = {{
		{ .duty_shift =  9, .time_ms =  300, .colour = { 0x00, 0x00, 0xff }},
		{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift =  9, .time_ms =  300, .colour = { 0x00, 0x00, 0xff }},
		{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0x00, 0x00 }},
	}},
	[notify_net_ipv4_acquired] = {{
		{ .duty_shift =  9, .time_ms = 1000, .colour = { 0x01, 0x01, 0x00 }},
		{ .duty_shift =  0, .time_ms = 1000, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift =  9, .time_ms = 1000, .colour = { 0x01, 0x01, 0x00 }},
		{ .duty_shift =  0, .time_ms = 1000, .colour = { 0x00, 0x00, 0x00 }},
	}},
	[notify_net_ipv6_ll_active] = {{
		{ .duty_shift =  5, .time_ms =   50, .colour = { 0x00, 0x01, 0x00 }},
		{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift =  5, .time_ms =   50, .colour = { 0x00, 0x01, 0x00 }},
		{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0x00, 0x00 }},
	}},
	[notify_net_ipv6_static_active] = {{
		{ .duty_shift =  5, .time_ms =  300, .colour = { 0x00, 0x01, 0x00 }},
		{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift =  5, .time_ms =  300, .colour = { 0x00, 0x01, 0x00 }},
		{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0x00, 0x00 }},
	}},
	[notify_net_ipv6_slaac_acquired] = {{
		{ .duty_shift =  5, .time_ms = 1000, .colour = { 0x00, 0x01, 0x00 }},
		{ .duty_shift =  0, .time_ms = 1000, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift =  5, .time_ms = 1000, .colour = { 0x00, 0x01, 0x00 }},
		{ .duty_shift =  0, .time_ms = 1000, .colour = { 0x00, 0x00, 0x00 }},
	}},
	[notify_net_ap_mode_init] = {{
		{ .duty_shift = 14, .time_ms =  100, .colour = { 0xff, 0x00, 0xff }},
		{ .duty_shift = 12, .time_ms =  100, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift = 14, .time_ms =  100, .colour = { 0xff, 0xff, 0xff }},
		{ .duty_shift = 12, .time_ms =  100, .colour = { 0x00, 0x00, 0xff }},
	}},
	[notify_net_ap_mode_idle] = {{
		{ .duty_shift = 14, .time_ms =  500, .colour = { 0xff, 0x00, 0xff }},
		{ .duty_shift = 12, .time_ms =  500, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift = 14, .time_ms =  500, .colour = { 0xff, 0xff, 0xff }},
		{ .duty_shift = 12, .time_ms =  500, .colour = { 0x00, 0x00, 0x00 }},
	}},
	[notify_net_ap_mode_associated] = {{
		{ .duty_shift = 14, .time_ms = 1200, .colour = { 0xff, 0x00, 0xff }},
		{ .duty_shift = 12, .time_ms = 1200, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift = 14, .time_ms = 1200, .colour = { 0xff, 0xff, 0xff }},
		{ .duty_shift = 12, .time_ms = 1200, .colour = { 0x00, 0x00, 0x00 }},
	}},
};

Notify *Notify::singleton = nullptr;

Notify::Notify() :
	running(false),
	current_notification(notify_none),
	current_phase(-1)
{
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
			LedPWM::get().open(LedPWM::lpt_14bit_5khz_notify, "notification LED");
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
		throw(hard_exception(util_esp_string_error(rv, "Notify::run: esp_pthread_set_cfg")));

	std::thread new_thread(this->run_thread_wrapper, this);
	new_thread.detach();

	this->running = true;
}

void __attribute__((noreturn)) Notify::run_thread_wrapper(void *this_)
{
	Notify *notify;

	assert(this_);

	notify = reinterpret_cast<Notify *>(this_);

	notify->run_thread();
}

void __attribute__((noreturn)) Notify::run_thread()
{
	const notification_info_t *info_ptr;
	const phase_t *phase_ptr;
	int sleep_ms = 0;

	for(;;)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));

		if(this->current_notification >= notify_size)
		{
			sleep_ms = 100;
			continue;
		}

		this->current_phase++;

		if((this->current_phase < 0) || (this->current_phase >= this->phase_size))
			this->current_phase = 0;

		info_ptr = &this->notification_info[this->current_notification];
		phase_ptr = &info_ptr->phase[this->current_phase];

		if(this->using_ledpixel)
		{
			Ledpixel::get().set(Ledpixel::Channel::channel_0_notify, 0, phase_ptr->colour.r, phase_ptr->colour.g, phase_ptr->colour.b);
			Ledpixel::get().flush(Ledpixel::Channel::channel_0_notify);
		}

		if(this->using_ledpwm)
			LedPWM::get().set(LedPWM::lpt_14bit_5khz_notify, (1UL << phase_ptr->duty_shift) - 1);

		sleep_ms = phase_ptr->time_ms ? : 100;
	}
}

void Notify::notify(notify_t notification)
{
	if(notification >= notify_size)
		throw(hard_exception("Notify::notify: notification out of bounds"));

	if(notification == notify_none)
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
	dst += std::format("ledpixel enabled: {}\n", this->using_ledpixel ? "yes" : "no");
	dst += std::format("ledpwm   enabled: {}\n", this->using_ledpwm   ? "yes" : "no");
	dst += std::format("thread running: {}\n", this->running ? "yes" : "no");
	dst += std::format("current notification: {:d}\n", static_cast<unsigned int>(this->current_notification));
	dst += std::format("- duty: {:d}\n", (1UL << this->notification_info[this->current_notification].phase[this->current_phase].duty_shift) - 1);
	dst += std::format("- sleep time: {:d} ms\n", (this->notification_info[this->current_notification].phase[this->current_phase].time_ms));
	dst += std::format("- red   component: {:#04x}\n", this->notification_info[this->current_notification].phase[this->current_phase].colour.r);
	dst += std::format("- green component: {:#04x}\n", this->notification_info[this->current_notification].phase[this->current_phase].colour.g);
	dst += std::format("- blue  component: {:#04x}\n", this->notification_info[this->current_notification].phase[this->current_phase].colour.b);
}
