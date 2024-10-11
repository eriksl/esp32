#include <stdint.h>
#include <stdbool.h>

#include <driver/ledc.h>
#include <soc/soc.h>

#include "string.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "pwm-led.h"

enum
{
	led_pwm_timers = 4,
	led_pwm_channels = 8,
};

typedef struct
{
	unsigned int resolution_bits;
	unsigned int frequency_hz;
} led_timer_t;

static_assert((int)led_pwm_timers == (int)LEDC_TIMER_MAX);
static_assert((int)led_pwm_channels == (int)LEDC_CHANNEL_MAX);
static_assert((int)plt_size < (int)LEDC_TIMER_MAX);

static bool inited = false;
static unsigned int channels_size;

static const led_timer_t timers[plt_size] =
{
	[plt_14bit_5khz] =
	{
		.resolution_bits = 14,
		.frequency_hz = 4882, // APB_CLK_FREQ / (1UL << 14), max
	},
	[plt_14bit_120hz] =
	{
		.resolution_bits = 14,
		.frequency_hz = 120,
	}
};

static pwm_led_type_t channel_to_timer_map[led_pwm_channels];

bool pwm_led_init(void)
{
	unsigned int ix;
	pwm_led_type_t timer;
	ledc_timer_config_t timer_config;

	if(inited)
		return(false);

	channels_size = 0;

	if(ledc_fade_func_install(0) != ESP_OK)
		return(false);

	for(ix = 0; ix < led_pwm_channels; ix++)
		channel_to_timer_map[ix] = plt_error;

	for(timer = plt_start; timer < plt_size; timer++)
	{
		timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
		timer_config.duty_resolution = timers[(unsigned int)timer].resolution_bits;
		timer_config.timer_num = (unsigned int)timer;
		timer_config.freq_hz = timers[(unsigned int)timer].frequency_hz;
		timer_config.clk_cfg = LEDC_USE_APB_CLK;
		timer_config.deconfigure = false;

		if(ledc_timer_config(&timer_config) != ESP_OK)
			return(false);
	}

	inited = true;

	return(true);
}

int pwm_led_channel_new(unsigned int gpio, pwm_led_type_t type)
{
	unsigned int rv;
	ledc_channel_config_t channel_config;

	if(!inited || (channels_size >= led_pwm_channels) || (type >= plt_size))
		return(-1);

	channel_config.gpio_num = gpio;
	channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
	channel_config.channel = channels_size;
	channel_config.intr_type = LEDC_INTR_DISABLE;
	channel_config.timer_sel = (unsigned int)type;
	channel_config.duty = 0;
	channel_config.hpoint = 0;
	channel_config.flags.output_invert = 0;

	if(ledc_channel_config(&channel_config) != ESP_OK)
		return(-1);

	channel_to_timer_map[channels_size] = type;

	rv = channels_size;
	channels_size++;

	return(rv);
}

bool pwm_led_channel_set(unsigned int channel, unsigned int duty)
{
	pwm_led_type_t timer;

	if(!inited || (channel >= channels_size))
		return(false);

	timer = channel_to_timer_map[channel];

	if(timer >= plt_size)
		return(false);

	if(duty >= (1UL << timers[timer].resolution_bits))
		duty = (1UL << timers[timer].resolution_bits) - 1;

	return(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, channel, duty, 0) == ESP_OK);
}

void command_pwm_led_info(cli_command_call_t *call)
{
	pwm_led_type_t timer;
	unsigned int ix;

	if(!inited)
		return;

	string_assign_cstr(call->result, "PWM LED INFO:");
	string_format_append(call->result, "\n- timers hardware available: %u, in use: %u", (unsigned int)led_pwm_timers, (unsigned int)plt_size);
	string_format_append(call->result, "\n- channels hardware available: %u, in use: %u", (unsigned int)led_pwm_channels, (unsigned int)channels_size);
	string_append_cstr(call->result, "\nCHANNELS:");

	for(ix = 0; ix < channels_size; ix++)
	{
		timer = channel_to_timer_map[ix];

		string_format_append(call->result, "\n- channel %u: timer: %u, resolution: %u, frequency: %u",
					ix, (unsigned int)timer, timers[timer].resolution_bits, timers[timer].frequency_hz);
	}
}
