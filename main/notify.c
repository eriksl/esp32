#include <stdint.h>
#include <stdbool.h>
#include <sdkconfig.h>

#include "notify.h"

#if defined(CONFIG_BSP_LED_HAVE_LEDPIXEL) || defined(CONFIG_BSP_LED_HAVE_LED)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

enum
{
	duty_dim = 1UL << 6,
	duty_full = 1UL << 14,
};

typedef enum
{
	phase_system = 0,
	phase_net,
	phase_blank,
	phase_error,
	phase_size = phase_error,
} phase_t;

typedef struct
{
	unsigned int r;
	unsigned int g;
	unsigned int b;
} rgb_t;

typedef struct
{
	bool enabled;
	bool on;
	bool blinking;
	bool fast;
	bool dim;
} led_t;

typedef struct
{
	phase_t phase;
	rgb_t colour;
	led_t led;
} notification_info_t;

static const notification_info_t notification_info[notify_size] =
{
	[notify_sys_booting] =					{	phase_system,	{ 0xff, 0x00, 0x00 },	{ true,  true,  true,  true,  false, }},
	[notify_sys_booting_finished] =			{	phase_system,	{ 0x00, 0x01, 0x00 },	{ true,  true,  false, false, false, }},
	[notify_net_init] =						{	phase_net,		{ 0xff, 0x00, 0x00 },	{ false, false, false, false, false, }},
	[notify_net_associating] =				{	phase_net,		{ 0x05, 0x00, 0x05 },	{ false, false, false, false, false, }},
	[notify_net_associating_finished] =		{	phase_net,		{ 0x05, 0x00, 0x05 },	{ false, false, false, false, false, }},
	[notify_net_ipv4_acquired] =			{	phase_net,		{ 0x00, 0x05, 0x00 },	{ true,  true,  true,  false, false, }},
	[notify_net_ipv6_ll_active] =			{	phase_net,		{ 0x01, 0x00, 0x00 },	{ true,  true,  true,  true,  true,  }},
	[notify_net_ipv6_slaac_acquired] =		{	phase_net,		{ 0x00, 0x00, 0x01 },	{ true,  true,  true,  false, true,  }},
	[notify_net_ipv6_static_active] =		{	phase_net,		{ 0x00, 0x01, 0x00 },	{ false, false, false, false, false, }},
	[notify_net_ap_mode_init] =				{	phase_net,		{ 0xff, 0x40, 0x40 },	{ false, false, false, false, false, }},
	[notify_net_ap_mode_idle] =				{	phase_net,		{ 0xff, 0x88, 0x88 },	{ false, false, false, false, false, }},
	[notify_net_ap_mode_associated] =		{	phase_net,		{ 0xff, 0xff, 0xff },	{ false, false, false, false, false, }},
};

static bool inited = false;
#endif

#if defined(CONFIG_BSP_LED_HAVE_LEDPIXEL)
#include "ledpixel.h"

static ledpixel_t ledpixel;
static TimerHandle_t phase_timer;
static rgb_t phase_colour[phase_size];
static phase_t phase;

static void ledpixel_timer_handler(struct tmrTimerControl *)
{
	if(!inited)
		return;

	ledpixel_set(ledpixel, 0, phase_colour[phase].g, phase_colour[phase].r, phase_colour[phase].b);
	ledpixel_flush(ledpixel);

	xTimerStart(phase_timer, 0);

	phase = (phase + 1) % phase_size;
}

bool notify_init(void)
{
	if(inited)
		return(false);

	if(!(ledpixel = ledpixel_new(1, CONFIG_BSP_LED_GPIO)))
		return(false);

	for(phase = 0; phase < phase_size; phase++)
	{
		phase_colour[phase].r = 0;
		phase_colour[phase].g = 0;
		phase_colour[phase].b = 0;
	}

	if(!ledpixel_set(ledpixel, 0, 0x00, 0x00, 0x00))
		return(false);

	if(!ledpixel_flush(ledpixel))
		return(false);

	phase_timer = xTimerCreate("notify-phase", pdMS_TO_TICKS(500), pdFALSE, (void *)0, ledpixel_timer_handler);

	if(!phase_timer)
		return(false);

	phase = 0;
	inited = true;
	xTimerStart(phase_timer, 0);

	return(true);
}

bool notify(notify_t notification)
{
	const notification_info_t *info;

	if(!inited || (notification >= notify_size))
		return(false);

	info = &notification_info[notification];

	phase_colour[info->phase] = info->colour;

	return(true);
}
#else
#if defined(CONFIG_BSP_LED_HAVE_LED)

#include "pwm-led.h"

static TimerHandle_t state_timer;
static bool led_blinking;
static bool led_blinking_fast;
static unsigned int led_dim_duty;
static unsigned int led_counter;
static int led_pwm_channel;

static void led_timer_handler(struct tmrTimerControl *)
{
	if(!inited || !led_blinking)
		return;

	if((led_counter % (led_blinking_fast ? 1 : 50)) <= 25)
		pwm_led_channel_set(led_pwm_channel, led_dim_duty);
	else
		pwm_led_channel_set(led_pwm_channel, 0);

	led_counter++;

	xTimerStart(state_timer, 0);
}

bool notify_init(void)
{
	led_blinking = false;
	led_blinking_fast = false;
	led_dim_duty = duty_full;
	led_counter = 0;

	if((led_pwm_channel = pwm_led_channel_new(CONFIG_BSP_LED_GPIO, plt_14bit_5khz)) < 0)
		return(false);

	if(!(state_timer = xTimerCreate("notify-gpio", pdMS_TO_TICKS(50), pdFALSE, (void *)0, led_timer_handler)))
		return(false);

	inited = true;

	return(true);
}

bool notify(notify_t notification)
{
	const notification_info_t *info;

	if(!inited || (notification >= notify_size))
		return(false);

	info = &notification_info[notification];

	if(!info->led.enabled)
		return(true);

	led_dim_duty = info->led.dim ? duty_dim : duty_full;

	if(info->led.blinking)
	{
		led_blinking = true;
		led_blinking_fast = info->led.fast;

		xTimerStart(state_timer, 0);
	}
	else
	{
		led_blinking = false;

		if(!pwm_led_channel_set(led_pwm_channel, info->led.on ? led_dim_duty : 0))
			return(false);
	}

	return(true);
}
#else
bool notify_init(void)
{
	return(true);
}
bool notify(notify_t notification)
{
	return(true);
}
#endif
#endif
