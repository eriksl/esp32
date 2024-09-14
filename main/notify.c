#include <stdint.h>
#include <stdbool.h>
#include <sdkconfig.h>

#include "notify.h"

#include "string.h"
#include "log.h"
#include "util.h"

#if defined(CONFIG_BSP_LED_HAVE_LEDPIXEL) || defined(CONFIG_BSP_LED_HAVE_LED)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
} led_t;

typedef struct
{
	phase_t phase;
	rgb_t colour;
	led_t led;
} notification_info_t;

static const notification_info_t notification_info[notify_size] =
{
	[notify_sys_booting] =					{	phase_system,	{ 0xff, 0x00, 0x00 },	{ true,  true,  true,  true }},
	[notify_sys_booting_finished] =			{	phase_system,	{ 0x00, 0x01, 0x00 },	{ true,  true,  false, false }},
	[notify_net_init] =						{	phase_net,		{ 0xff, 0x00, 0x00 },	{ false, false, false, false }},
	[notify_net_associating] =				{	phase_net,		{ 0x05, 0x00, 0x05 },	{ false, false, false, false }},
	[notify_net_associating_finished] =		{	phase_net,		{ 0x05, 0x00, 0x05 },	{ false, false, false, false }},
	[notify_net_ipv4_acquired] =			{	phase_net,		{ 0x00, 0x05, 0x00 },	{ true,  true,  true,  false }},
	[notify_net_ipv6_ll_active] =			{	phase_net,		{ 0x01, 0x00, 0x00 },	{ false, false, false, false }},
	[notify_net_ipv6_slaac_acquired] =		{	phase_net,		{ 0x00, 0x00, 0x01 },	{ false, false, false, false }},
	[notify_net_ipv6_static_active] =		{	phase_net,		{ 0x00, 0x01, 0x00 },	{ false, false, false, false }},
	[notify_net_ap_mode_init] =				{	phase_net,		{ 0xff, 0x40, 0x40 },	{ false, false, false, false }},
	[notify_net_ap_mode_idle] =				{	phase_net,		{ 0xff, 0x88, 0x88 },	{ false, false, false, false }},
	[notify_net_ap_mode_associated] =		{	phase_net,		{ 0xff, 0xff, 0xff },	{ false, false, false, false }},
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
	assert(inited);

	ledpixel_set(ledpixel, 0, phase_colour[phase].g, phase_colour[phase].r, phase_colour[phase].b);
	ledpixel_flush(ledpixel);

	xTimerStart(phase_timer, 0);

	phase = (phase + 1) % phase_size;
}

void notify_init(void)
{
	assert(!inited);

	ledpixel = ledpixel_new(1, CONFIG_BSP_LED_GPIO);

	for(phase = 0; phase < phase_size; phase++)
	{
		phase_colour[phase].r = 0;
		phase_colour[phase].g = 0;
		phase_colour[phase].b = 0;
	}

	ledpixel_set(ledpixel, 0, 0x00, 0x00, 0x00);
	ledpixel_flush(ledpixel);

	phase = 0;
	inited = true;

	phase_timer = xTimerCreate("notify-phase", pdMS_TO_TICKS(500), pdFALSE, (void *)0, ledpixel_timer_handler);
	assert(phase_timer);

	xTimerStart(phase_timer, 0);
}

void notify(notify_t notification)
{
	const notification_info_t *info;

	assert(notification < notify_size);

	info = &notification_info[notification];

	phase_colour[info->phase] = info->colour;
}
#else
#if defined(CONFIG_BSP_LED_HAVE_LED)

#include <driver/gpio.h>

static TimerHandle_t state_timer_slow;
static TimerHandle_t state_timer_fast;
static bool led_blinking = false;
static unsigned int led_counter;

static void led_timer_handler_slow(struct tmrTimerControl *)
{
	if(led_blinking)
	{
		gpio_set_level(CONFIG_BSP_LED_GPIO, (led_counter++) & 0x01);
		xTimerStart(state_timer_slow, 0);
	}
}

static void led_timer_handler_fast(struct tmrTimerControl *)
{
	if(led_blinking)
	{
		gpio_set_level(CONFIG_BSP_LED_GPIO, (led_counter++) & 0x01);
		xTimerStart(state_timer_fast, 0);
	}
}

void notify_init(void)
{
	gpio_config_t gpio_pin_config =
	{
		.pin_bit_mask = (1ULL << CONFIG_BSP_LED_GPIO),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	util_abort_on_esp_err("gpio_config", gpio_config(&gpio_pin_config));

	led_blinking = false;
	led_counter = 0;

	state_timer_slow = xTimerCreate("notify-slow", pdMS_TO_TICKS(500), pdFALSE, (void *)0, led_timer_handler_slow);
	assert(state_timer_slow);
	state_timer_fast = xTimerCreate("notify-fast", pdMS_TO_TICKS(50), pdFALSE, (void *)0, led_timer_handler_fast);
	assert(state_timer_fast);

	inited = true;
}

void notify(notify_t notification)
{
	const notification_info_t *info;

	assert(notification < notify_size);

	info = &notification_info[notification];

	if(info->led.enabled)
	{
		if(info->led.blinking)
		{
			led_blinking = true;

			if(info->led.fast)
				xTimerStart(state_timer_fast, 0);
			else
				xTimerStart(state_timer_slow, 0);
		}
		else
		{
			led_blinking = false;

			gpio_set_level(CONFIG_BSP_LED_GPIO, info->led.on ? 1 : 0);
		}
	}
}
#else
void notify_init(void)
{
}
void notify(unsigned int new_state, unsigned int r, unsigned g, unsigned int b)
{
}
#endif
#endif
