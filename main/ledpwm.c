#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <driver/ledc.h>
#include <soc/soc.h>

#include "string.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "pwm-led.h"

enum
{
	led_pwm_timers_size = 4,
	led_pwm_channels_size = 8,
	led_pwm_channel_name_size = 32,
};

_Static_assert((int)led_pwm_timers_size == (int)LEDC_TIMER_MAX);
_Static_assert((int)led_pwm_channels_size == (int)LEDC_CHANNEL_MAX);
_Static_assert((int)plt_size < (int)LEDC_TIMER_MAX);

typedef struct
{
	char *name;
	ledc_channel_config_t config;
} _pwm_led_t;

static bool inited = false;
static unsigned int channels_size;
static ledc_timer_config_t *timer_configs;
static _pwm_led_t *channels;

void pwm_led_init(void)
{
	unsigned int channel;

	assert(!inited);

	timer_configs = (ledc_timer_config_t *)util_memory_alloc_spiram(sizeof(ledc_timer_config_t[led_pwm_timers_size]));
	assert(timer_configs);

	timer_configs[plt_14bit_5khz].speed_mode = LEDC_LOW_SPEED_MODE;
	timer_configs[plt_14bit_5khz].duty_resolution = 14;
	timer_configs[plt_14bit_5khz].timer_num = plt_14bit_5khz;
	timer_configs[plt_14bit_5khz].freq_hz = 4882; // APB_CLK_FREQ / (1UL << 14), max
	timer_configs[plt_14bit_5khz].clk_cfg = LEDC_USE_APB_CLK;
	timer_configs[plt_14bit_5khz].deconfigure = false;

	timer_configs[plt_14bit_120hz].speed_mode = LEDC_LOW_SPEED_MODE;
	timer_configs[plt_14bit_120hz].duty_resolution = 14;
	timer_configs[plt_14bit_120hz].timer_num = plt_14bit_120hz;
	timer_configs[plt_14bit_120hz].freq_hz = 120;
	timer_configs[plt_14bit_120hz].clk_cfg = LEDC_USE_APB_CLK;
	timer_configs[plt_14bit_120hz].deconfigure = false;

	util_abort_on_esp_err("ledc_timer_config", ledc_timer_config(&timer_configs[plt_14bit_5khz]));
	util_abort_on_esp_err("ledc_timer_config", ledc_timer_config(&timer_configs[plt_14bit_120hz]));

	channels = (_pwm_led_t *)util_memory_alloc_spiram(sizeof(_pwm_led_t[led_pwm_channels_size]));

	for(channel = 0; channel < led_pwm_channels_size; channel++)
	{
		channels[channel].name = (char *)util_memory_alloc_spiram(led_pwm_channel_name_size);
		channels[channel].config.gpio_num = -1;
		channels[channel].config.speed_mode = LEDC_LOW_SPEED_MODE;
		channels[channel].config.channel = channel;
		channels[channel].config.intr_type = LEDC_INTR_DISABLE;
		channels[channel].config.timer_sel = plt_error;
		channels[channel].config.duty = 0;
		channels[channel].config.hpoint = 0;
		channels[channel].config.flags.output_invert = 0;
	}

	channels_size = 0;

	util_abort_on_esp_err("ledc_fade_func_install", ledc_fade_func_install(0));

	inited = true;
}

pwm_led_t pwm_led_channel_new(unsigned int gpio, pwm_led_type_t type, const char *name)
{
	_pwm_led_t *channel;

	assert(inited);
	assert(channels_size < led_pwm_channels_size);
	assert(type < plt_size);

	channel = &channels[channels_size];

	strlcpy(channel->name, name, led_pwm_channel_name_size);
	channel->config.gpio_num = gpio;
	channel->config.channel = channels_size;
	channel->config.timer_sel = (unsigned int)type;

	util_abort_on_esp_err("ledc_channel_config", ledc_channel_config(&channel->config));

	channels_size++;

	return((pwm_led_t)channel);
}

void pwm_led_channel_set(pwm_led_t channel, unsigned int duty)
{
	ledc_timer_config_t *timer_ptr;
	_pwm_led_t *_channel;

	assert(inited);
	assert(channel);

	_channel = (_pwm_led_t *)channel;

	assert(_channel->config.timer_sel < (unsigned int)plt_size);

	timer_ptr = &timer_configs[_channel->config.timer_sel];

	if(duty >= (1UL << timer_ptr->duty_resolution))
		duty = (1UL << timer_ptr->duty_resolution) - 1;

	util_abort_on_esp_err("ledc_set_duty_and_update", ledc_set_duty_and_update(_channel->config.speed_mode, _channel->config.channel, duty, 0));
}

unsigned int pwm_led_channel_get(const const_pwm_led_t channel)
{
	const _pwm_led_t *_channel;

	assert(inited);
	assert(channel);

	_channel = (const _pwm_led_t *)channel;

	assert(_channel->config.timer_sel < (unsigned int)plt_size);

	return(ledc_get_duty(_channel->config.speed_mode, _channel->config.channel));
}

void command_pwm_led_info(cli_command_call_t *call)
{
	unsigned int channel;
	_pwm_led_t *_channel;

	assert(inited);
	assert(call);
	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "PWM LED INFO:");
	string_format_append(call->result, "\n- timers hardware available: %u, in use: %u", (unsigned int)led_pwm_timers_size, (unsigned int)plt_size);
	string_format_append(call->result, "\n- channels hardware available: %u, in use: %u", (unsigned int)led_pwm_channels_size, (unsigned int)channels_size);
	string_append_cstr(call->result, "\nCHANNELS:");

	for(channel = 0; channel < channels_size; channel++)
	{
		_channel = &channels[channel];

		assert(_channel->config.channel == channel);

		string_format_append(call->result, "\n- channel [%u] %*s timer: %u, resolution: %2u bits, frequency: %4u Hz, duty: %5u, hpoint: %u",
					channel,
					0 - led_pwm_channel_name_size, _channel->name,
					(unsigned int)_channel->config.timer_sel,
					(unsigned int)timer_configs[_channel->config.timer_sel].duty_resolution,
					(unsigned int)timer_configs[_channel->config.timer_sel].freq_hz,
					(unsigned int)ledc_get_duty(_channel->config.speed_mode, channel),
					(unsigned int)ledc_get_hpoint(_channel->config.speed_mode, channel));
	}
}
