#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "pdm.h"

#include <string>
#include <boost/format.hpp>

#include "driver/sdm.h"
#include "driver/gpio.h"

enum
{
	pdm_sample_frequency = 80000000 / 256,
};

typedef struct
{
	const char *owner;
	unsigned int density;
	sdm_channel_handle_t handle;
	struct
	{
		unsigned int available:1;
		unsigned int open:1;
	};
} channel_t;

typedef struct
{
	int gpio;
} handle_to_gpio_t;

static const handle_to_gpio_t handle_to_gpio[pdm_size] =
{
	[pdm_8bit_150khz_0] = { .gpio = CONFIG_BSP_PDM0 },
	[pdm_8bit_150khz_1] = { .gpio = CONFIG_BSP_PDM1 },
	[pdm_8bit_150khz_2] = { .gpio = CONFIG_BSP_PDM2 },
	[pdm_8bit_150khz_3] = { .gpio = CONFIG_BSP_PDM3 },
};

static bool inited = false;
static channel_t *channels;

void pdm_init(void)
{
	gpio_config_t gpio_pin_config =
	{
		.pin_bit_mask = 0,
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	sdm_config_t sdm_config =
	{
		.gpio_num = -1,
		.clk_src = SDM_CLK_SRC_DEFAULT,
		.sample_rate_hz =  pdm_sample_frequency,
		.flags =
		{
			.invert_out = 0,
			.io_loop_back = 0,
		},
	};

	pdm_t handle;
	channel_t *channel;
	const handle_to_gpio_t *handle_to_gpio_ptr;

	assert(!inited);

	channels = new channel_t[pdm_size];

	for(handle = pdm_first; handle < pdm_size; handle = static_cast<pdm_t>(handle + 1))
	{
		handle_to_gpio_ptr = &handle_to_gpio[handle];
		channel = &channels[handle];

		channel->owner = (const char *)0;
		channel->density = 0;
		channel->handle = (sdm_channel_handle_t)0;
		channel->available = 0;
		channel->open = 0;

		if(handle_to_gpio_ptr->gpio >= 0)
		{
			gpio_pin_config.pin_bit_mask = (1ULL << handle_to_gpio_ptr->gpio);
			Log::get().abort_on_esp_err("gpio_reset_pin", gpio_reset_pin(static_cast<gpio_num_t>(handle_to_gpio_ptr->gpio)));
			Log::get().abort_on_esp_err("gpio_config", gpio_config(&gpio_pin_config));

			sdm_config.gpio_num = handle_to_gpio_ptr->gpio;
			Log::get().abort_on_esp_err("sdm_new_channel", sdm_new_channel(&sdm_config, &channel->handle));
			Log::get().abort_on_esp_err("sdm_channel_enable", sdm_channel_enable(channel->handle));
			Log::get().abort_on_esp_err("sdm_channel_set_pulse_density", sdm_channel_set_pulse_density(channel->handle, channel->density - 128));

			channel->available = 1;
		}
	}

	inited = true;
}

bool pdm_channel_open(pdm_t handle, const char *owner)
{
	channel_t *channel;

	assert(inited);
	assert(handle < pdm_size);

	channel = &channels[handle];

	if(!channel->available)
		return(false);

	if(channel->open)
		return(true);

	channel->open = 1;
	channel->owner = owner;

	return(true);
}

void pdm_channel_set(pdm_t handle, unsigned int density)
{
	channel_t *channel;

	assert(inited);
	assert(handle < pdm_size);

	channel = &channels[handle];

	assert(channel->open);

	if(density > 255)
		density = 255;

	channel->density = density;

	Log::get().abort_on_esp_err("sdm_channel_set_pulse_density", sdm_channel_set_pulse_density(channel->handle, (int)channel->density - 128));
}

unsigned int pdm_channel_get(pdm_t handle)
{
	const channel_t *channel;

	assert(inited);
	assert(handle < pdm_size);

	channel = &channels[handle];

	return(channel->density);
}

void command_pdm_info(cli_command_call_t *call)
{
	pdm_t handle;
	channel_t *channel;

	assert(inited);
	assert(call);
	assert(call->parameter_count == 0);

	call->result = "PDM INFO:";
	call->result += (boost::format("\n- channels available: %u") % pdm_size).str();
	call->result += "\nchannels:";

	for(handle = pdm_first; handle < pdm_size; handle = static_cast<pdm_t>(handle + 1))
	{
		channel = &channels[handle];

		if(channel->available)
		{
			assert(!channel->open || channel->owner);

			call->result += (boost::format("\n- channel %u: 8 bits @ 150 kHz, gpio %2d is %s density: %3u, owned by: %s") %
					handle %
					handle_to_gpio[handle].gpio %
					(channel->open ? "open" : "not open") %
					channel->density %
					(channel->open ? channel->owner : "<none>")).str();
		}
	}
}
