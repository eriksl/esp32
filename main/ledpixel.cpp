#include <stdint.h>
#include <stdbool.h>
#include <sdkconfig.h>

extern "C"
{
#include "string.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"
}

#include "ledpixel.h"

#include <led_strip.h>
#include <led_strip_types.h>

typedef struct
{
	unsigned int r;
	unsigned int g;
	unsigned int b;
} rgb_t;

typedef struct
{
	led_strip_handle_t handle;
	int gpio;
	const char *owner;
	struct
	{
		unsigned int available:1;
		unsigned int open:1;
	};
	rgb_t rgbvalue[ledpixel_leds_size];
} channel_t;

typedef struct
{
	lp_t handle;
	int gpio;
} handle_to_gpio_t;

static const handle_to_gpio_t handle_to_gpio[lp_size] =
{
	[lp_0_notify] =	{ .handle = lp_0_notify,	.gpio = CONFIG_BSP_LEDPIXEL0 },
	[lp_1] =		{ .handle = lp_1,			.gpio = CONFIG_BSP_LEDPIXEL1 },
	[lp_2] =		{ .handle = lp_2,			.gpio = CONFIG_BSP_LEDPIXEL2 },
	[lp_3] =		{ .handle = lp_3,			.gpio = CONFIG_BSP_LEDPIXEL3 },
};

static bool inited = false;
static channel_t *channels;

void ledpixel_init(void)
{
	led_strip_config_t led_strip_config =
	{
		.strip_gpio_num = -1,
		.max_leds = ledpixel_leds_size,
		.led_model = LED_MODEL_WS2812,
		.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
		.flags =
		{
			.invert_out = 0,
		},
	};
	led_strip_rmt_config_t rmt_config =
	{
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.resolution_hz = 0,
		.mem_block_symbols = 0,
		.flags =
		{
			.with_dma = 0,
		},
	};

	lp_t handle;
	channel_t *channel;
	const handle_to_gpio_t *handle_to_gpio_ptr;

	assert(!inited);

	channels = (channel_t *)util_memory_alloc_spiram(sizeof(channel_t[lp_size]));
	assert(channels);

	for(handle = lp_first; handle < lp_size; handle = static_cast<lp_t>(handle + 1))
	{
		handle_to_gpio_ptr = &handle_to_gpio[handle];
		assert(handle_to_gpio_ptr->handle == handle);

		channel = &channels[handle];
		channel->handle = (led_strip_handle_t)0;
		channel->owner = (const char *)0;
		channel->available = 0;
		channel->open = 0;

		if(handle_to_gpio_ptr->gpio >= 0)
		{
			channel->gpio = handle_to_gpio_ptr->gpio;
			led_strip_config.strip_gpio_num = channel->gpio;
			util_abort_on_esp_err("led_strip_new_rmt_device", led_strip_new_rmt_device(&led_strip_config, &rmt_config, &channel->handle));
			util_abort_on_esp_err("led_strip_clear", led_strip_clear(channel->handle));
			util_abort_on_esp_err("led_strip_flush", led_strip_refresh(channel->handle));
			channel->available = 1;
		}
	}

	inited = true;
}

bool ledpixel_open(lp_t handle, const char *owner)
{
	channel_t *channel;
	unsigned int ix;

	assert(inited);
	assert(handle < lp_size);

	channel = &channels[handle];

	if(!channel->available)
		return(false);

	if(channel->open)
		return(true);

	channel->owner = owner;
	channel->open = 1;

	util_abort_on_esp_err("led_strip_clear", led_strip_clear(channel->handle));
	util_abort_on_esp_err("led_strip_flush", led_strip_refresh(channel->handle));

	for(ix = 0; ix < ledpixel_leds_size; ix++)
	{
		channel->rgbvalue[ix].r = 0;
		channel->rgbvalue[ix].g = 0;
		channel->rgbvalue[ix].b = 0;
	}

	return(true);
}

void ledpixel_set(lp_t handle, unsigned int index, unsigned int red, unsigned int green, unsigned int blue)
{
	channel_t *channel;
	rgb_t *rgb;

	assert(inited);
	assert(handle < lp_size);
	assert(index < ledpixel_leds_size);

	channel = &channels[handle];

	assert(channel->open);

	rgb = &channel->rgbvalue[index];

	rgb->r = red;
	rgb->g = green;
	rgb->b = blue;

	if(handle == lp_0_notify)
		util_abort_on_esp_err("led_strip_set_pixel", led_strip_set_pixel(channel->handle, index, rgb->g, rgb->r, rgb->b));
	else
		util_abort_on_esp_err("led_strip_set_pixel", led_strip_set_pixel(channel->handle, index, rgb->r, rgb->g, rgb->b));
}

void ledpixel_get(lp_t handle, unsigned int index, unsigned int *red, unsigned int *green, unsigned int *blue)
{
	const channel_t *channel;
	const rgb_t *rgb;

	assert(inited);
	assert(handle < lp_size);
	assert(index < ledpixel_leds_size);

	channel = &channels[handle];

	assert(channel->open);

	rgb = &channel->rgbvalue[index];

	*red = rgb->r;
	*green = rgb->g;
	*blue = rgb->b;
}

void ledpixel_flush(lp_t handle)
{
	channel_t *channel;

	assert(inited);
	assert(handle < lp_size);

	channel = &channels[handle];

	assert(channel->open);

	util_abort_on_esp_err("led_strip_refresh", led_strip_refresh(channel->handle));
}

void command_ledpixel_info(cli_command_call_t *call)
{
	lp_t handle;
	const channel_t *channel;
	const rgb_t *rgb;
	unsigned int ix;

	assert(inited);
	assert(call);
	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "LEDPIXEL INFO:");
	string_format_append(call->result, "\n- channels available: %u", (unsigned int)lp_size);
	string_append_cstr(call->result, "\nchannels:");

	for(handle = lp_first; handle < lp_size; handle = static_cast<lp_t>(handle + 1))
	{
		channel = &channels[handle];

		if(channel->available)
		{
			assert(!channel->open || channel->owner);

			string_format_append(call->result, "\n- channel %u: gpio %2d is %s, owned by: %s\n   rgbvalue:",
					(unsigned int)handle, channel->gpio,
					channel->open ? "open" : "not open",
					channel->open ? channel->owner : "<none>");

			for(ix = 0; ix < ledpixel_leds_size; ix++)
			{
				rgb = &channel->rgbvalue[ix];
				string_format_append(call->result, " (R:0x%02x,G:0x%02x,B:0x%02x)", rgb->r, rgb->g, rgb->b);
			}
		}
		else
			string_format_append(call->result, "\n- channel %d is unavailable", handle);

	}
}
