#include <stdint.h>
#include <stdbool.h>
#include <sdkconfig.h>

#include "notify.h"
#include "info.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

enum
{
	phase_size = 4
};

typedef struct
{
	unsigned int r;
	unsigned int g;
	unsigned int b;
} rgb_t;

typedef struct
{
	unsigned int duty_shift;
	unsigned int time_ms;
	rgb_t colour;
} phase_t;

typedef struct
{
	phase_t phase[phase_size];
} notification_info_t;

static const notification_info_t notification_info[notify_size] =
{
	[notify_none] = {{
		{ .duty_shift =  0, .time_ms =    0, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift =  0, .time_ms =    0, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift =  0, .time_ms =    0, .colour = { 0x00, 0x00, 0x00 }},
		{ .duty_shift =  0, .time_ms =    0, .colour = { 0x00, 0x00, 0x00 }},
	}},
	[notify_sys_booting] = {{
		{ .duty_shift = 14, .time_ms =   50, .colour = { 0xff, 0x00, 0x00 }},
		{ .duty_shift =  0, .time_ms =   50, .colour = { 0xff, 0x00, 0x00 }},
		{ .duty_shift = 14, .time_ms =   50, .colour = { 0xff, 0x00, 0x00 }},
		{ .duty_shift =  0, .time_ms =   50, .colour = { 0xff, 0x00, 0x00 }},
	}},
	[notify_sys_booting_finished] = {{
		{ .duty_shift = 14, .time_ms =  300, .colour = { 0xff, 0x00, 0x00 }},
		{ .duty_shift =  0, .time_ms =  300, .colour = { 0xff, 0x00, 0x00 }},
		{ .duty_shift = 14, .time_ms =  300, .colour = { 0xff, 0x00, 0x00 }},
		{ .duty_shift =  0, .time_ms =  300, .colour = { 0xff, 0x00, 0x00 }},
	}},
	[notify_net_associating] = {{
		{ .duty_shift =  9, .time_ms =   50, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  9, .time_ms =   50, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0xff, 0x00 }},
	}},
	[notify_net_associating_finished] = {{
		{ .duty_shift =  9, .time_ms =  300, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  9, .time_ms =  300, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0xff, 0x00 }},
	}},
	[notify_net_ipv4_acquired] = {{
		{ .duty_shift =  9, .time_ms = 1000, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  0, .time_ms = 1000, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  9, .time_ms = 1000, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  0, .time_ms = 1000, .colour = { 0x00, 0xff, 0x00 }},
	}},
	[notify_net_ipv6_ll_active] = {{
		{ .duty_shift =  5, .time_ms =   50, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  5, .time_ms =   50, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  0, .time_ms =   50, .colour = { 0x00, 0xff, 0x00 }},
	}},
	[notify_net_ipv6_static_active] = {{
		{ .duty_shift =  5, .time_ms =  300, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  5, .time_ms =  300, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  0, .time_ms =  300, .colour = { 0x00, 0xff, 0x00 }},
	}},
	[notify_net_ipv6_slaac_acquired] = {{
		{ .duty_shift =  5, .time_ms = 1000, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  0, .time_ms = 1000, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  5, .time_ms = 1000, .colour = { 0x00, 0xff, 0x00 }},
		{ .duty_shift =  0, .time_ms = 1000, .colour = { 0x00, 0xff, 0x00 }},
	}},
	[notify_net_ap_mode_init] = {{
		{ .duty_shift = 14, .time_ms =  100, .colour = { 0x00, 0x00, 0xff }},
		{ .duty_shift = 12, .time_ms =  100, .colour = { 0x00, 0x00, 0xff }},
		{ .duty_shift = 14, .time_ms =  100, .colour = { 0x00, 0x00, 0xff }},
		{ .duty_shift = 12, .time_ms =  100, .colour = { 0x00, 0x00, 0xff }},
	}},
	[notify_net_ap_mode_idle] = {{
		{ .duty_shift = 14, .time_ms =  500, .colour = { 0x00, 0x00, 0xff }},
		{ .duty_shift = 12, .time_ms =  500, .colour = { 0x00, 0x00, 0xff }},
		{ .duty_shift = 14, .time_ms =  500, .colour = { 0x00, 0x00, 0xff }},
		{ .duty_shift = 12, .time_ms =  500, .colour = { 0x00, 0x00, 0xff }},
	}},
	[notify_net_ap_mode_associated] = {{
		{ .duty_shift = 14, .time_ms = 1200, .colour = { 0x00, 0x00, 0xff }},
		{ .duty_shift = 12, .time_ms = 1200, .colour = { 0x00, 0x00, 0xff }},
		{ .duty_shift = 14, .time_ms = 1200, .colour = { 0x00, 0x00, 0xff }},
		{ .duty_shift = 12, .time_ms = 1200, .colour = { 0x00, 0x00, 0xff }},
	}},
};

static bool inited = false;
static notify_t current_notification;
static int current_phase;
static TimerHandle_t phase_timer;

#if defined(CONFIG_BSP_LED_HAVE_LEDPIXEL)
#include "ledpixel.h"
static ledpixel_t ledpixel;
#endif
#if defined(CONFIG_BSP_LED_HAVE_LED)
#include "pwm-led.h"
static pwm_led_t led_pwm_channel;
#endif

static void timer_handler(struct tmrTimerControl *)
{
	if(!inited || (current_notification >= notify_size))
		return;

	current_phase++;

	if((current_phase < 0) || (current_phase >= phase_size))
		current_phase = 0;

#if defined(CONFIG_BSP_LED_HAVE_LED) || defined(CONFIG_BSP_LED_HAVE_LEDPIXEL)
	const notification_info_t *info_ptr;
	const phase_t *phase_ptr;

	info_ptr = &notification_info[current_notification];
	phase_ptr = &info_ptr->phase[current_phase];
#endif

#if defined(CONFIG_BSP_LED_HAVE_LEDPIXEL)
	ledpixel_set(ledpixel, 0,
			phase_ptr->colour.r,
			phase_ptr->colour.g,
			phase_ptr->colour.b);
	ledpixel_flush(ledpixel);
#endif

#if defined(CONFIG_BSP_LED_HAVE_LED)
	pwm_led_channel_set(led_pwm_channel, (1UL << phase_ptr->duty_shift) - 1);
#endif

#if defined(CONFIG_BSP_LED_HAVE_LED) || defined(CONFIG_BSP_LED_HAVE_LEDPIXEL)
	if(phase_ptr->time_ms)
		if(!xTimerChangePeriod(phase_timer, pdMS_TO_TICKS(phase_ptr->time_ms), pdMS_TO_TICKS(1000))) // this will start the timer as well
			stat_notify_timer_failed++;
#endif
}

bool notify_init(void)
{
	if(inited)
		return(false);

#if defined(CONFIG_BSP_LED_HAVE_LEDPIXEL)
	if(!(ledpixel = ledpixel_new(1, CONFIG_BSP_LED_GPIO)))
		return(false);

	if(!ledpixel_set(ledpixel, 0, 0x00, 0x00, 0x00))
		return(false);

	if(!ledpixel_flush(ledpixel))
		return(false);
#endif

#if defined(CONFIG_BSP_LED_HAVE_LED)
	led_pwm_channel = pwm_led_channel_new(CONFIG_BSP_LED_GPIO, plt_14bit_5khz, "notification LED");
#endif

	current_phase = -1;
	current_notification = notify_none;
	inited = true;

	if(!(phase_timer = xTimerCreate("notify-phase", 1, pdFALSE, (void *)0, timer_handler)))
		return(false);

	return(true);
}

bool notify(notify_t notification)
{
	if(!inited || (notification >= notify_size))
		return(false);

	if(notification == notify_none)
		return(true);

	current_notification = notification;
	current_phase = -1;

#if defined(CONFIG_BSP_LED_HAVE_LED) || defined(CONFIG_BSP_LED_HAVE_LEDPIXEL)
	xTimerChangePeriod(phase_timer, pdMS_TO_TICKS(100), pdMS_TO_TICKS(1000)); // this will start the timer as well
#endif
	return(true);
}
