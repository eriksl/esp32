#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "string.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "pdm.h"

#include "driver/sdm.h"
#include "driver/gpio.h"

enum
{
	pdm_sample_frequency = 80000000 / 1,
	pdm_sample_width = 8,
	pdm_channels_size = 8,
	pdm_channel_name_size = 32,
};

typedef struct
{
	char *name;
	unsigned int channel;
	unsigned int density;
	sdm_channel_handle_t handle;
	gpio_config_t gpio_config;
	sdm_config_t sdm_config;
} _pdm_t;

static bool inited = false;
static unsigned int channels_size;
static _pdm_t *channels;

void pdm_init(void)
{
	unsigned int channel;
	_pdm_t *channel_ptr;

	assert(!inited);

	channels = (_pdm_t *)util_memory_alloc_spiram(sizeof(_pdm_t[pdm_channels_size]));

	for(channel = 0; channel < pdm_channels_size; channel++)
	{
		channel_ptr = &channels[channel];

		channel_ptr->name = (char *)util_memory_alloc_spiram(pdm_channel_name_size);
		channel_ptr->channel = channel;
		channel_ptr->density = 0;
		channel_ptr->handle = (sdm_channel_handle_t)0;

		channel_ptr->gpio_config.pin_bit_mask = 0;
		channel_ptr->gpio_config.mode = GPIO_MODE_OUTPUT;
		channel_ptr->gpio_config.pull_up_en = GPIO_PULLUP_DISABLE;
		channel_ptr->gpio_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
		channel_ptr->gpio_config.intr_type = GPIO_INTR_DISABLE;

		channel_ptr->sdm_config.gpio_num = -1;
		channel_ptr->sdm_config.clk_src = SDM_CLK_SRC_DEFAULT;
		channel_ptr->sdm_config.sample_rate_hz =  pdm_sample_frequency,
		channel_ptr->sdm_config.flags.invert_out = 0;
		channel_ptr->sdm_config.flags.io_loop_back = 0;
	}

	channels_size = 0;

	inited = true;
}

pdm_t pdm_channel_new(unsigned int gpio, const char *name)
{
	_pdm_t *_channel;

	assert(inited);
	assert(channels_size < pdm_channels_size);

	_channel = &channels[channels_size];

	strlcpy(_channel->name, name, pdm_channel_name_size);
	_channel->density = 0;
	_channel->gpio_config.pin_bit_mask = (1ULL << gpio);
	_channel->sdm_config.gpio_num = gpio;

	util_abort_on_esp_err("gpio_reset_pin", gpio_reset_pin(gpio));
	util_abort_on_esp_err("gpio_config", gpio_config(&_channel->gpio_config));
	util_abort_on_esp_err("sdm_new_channel", sdm_new_channel(&_channel->sdm_config, &_channel->handle));
	util_abort_on_esp_err("sdm_channel_enable", sdm_channel_enable(_channel->handle));
	util_abort_on_esp_err("sdm_channel_set_pulse_density", sdm_channel_set_pulse_density(_channel->handle, _channel->density - 128));

	channels_size++;

	return((pdm_t)_channel);
}

void pdm_channel_set(pdm_t channel, unsigned int density)
{
	_pdm_t *_channel;

	assert(inited);
	assert(channel);

	_channel = (_pdm_t *)channel;

	assert(_channel->handle);

	if(density > 255)
		density = 255;

	_channel->density = density;

	util_abort_on_esp_err("sdm_channel_set_pulse_density", sdm_channel_set_pulse_density(_channel->handle, _channel->density - 128));
}

unsigned int pdm_channel_get(const const_pdm_t channel)
{
	const _pdm_t *_channel;

	assert(inited);
	assert(channel);

	_channel = (const _pdm_t *)channel;

	return(_channel->density);
}

void command_pdm_info(cli_command_call_t *call)
{
	unsigned int channel;
	_pdm_t *_channel;

	assert(inited);
	assert(call);
	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "PDM INFO:");
	string_format_append(call->result, "\n- channels hardware available: %u, in use: %u", (unsigned int)pdm_channels_size, channels_size);
	string_append_cstr(call->result, "\nCHANNELS:");

	for(channel = 0; channel < channels_size; channel++)
	{
		_channel = &channels[channel];

		assert(_channel->channel == channel);

		string_format_append(call->result, "\n- channel [%u] %*s resolution: %u bits, frequency: %5u kHz, density: %3u",
					channel, 0 - pdm_channel_name_size, _channel->name, (unsigned int)pdm_sample_width,
					(unsigned int)(_channel->sdm_config.sample_rate_hz / 1000), _channel->density);
	}
}
