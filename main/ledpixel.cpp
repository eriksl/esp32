#include "log.h"
#include "util.h"
#include "exception.h"
#include "ledpixel.h"

#include <sdkconfig.h>

#include <led_strip_types.h>

#include <format>

Ledpixel *Ledpixel::singleton = nullptr;

const Ledpixel::handle_to_gpio_t Ledpixel::handle_to_gpio[Ledpixel::lp_size] =
{
	[Ledpixel::lp_0_notify] =	{ .handle = Ledpixel::lp_0_notify,	.gpio = CONFIG_BSP_LEDPIXEL0,	.pos { .r = 0, .g = 1, .b = 2 }},
	[Ledpixel::lp_1] =			{ .handle = Ledpixel::lp_1,			.gpio = CONFIG_BSP_LEDPIXEL1,	.pos { .r = 1, .g = 0, .b = 2 }},
	[Ledpixel::lp_2] =			{ .handle = Ledpixel::lp_2,			.gpio = CONFIG_BSP_LEDPIXEL2,	.pos { .r = 1, .g = 0, .b = 2 }},
	[Ledpixel::lp_3] =			{ .handle = Ledpixel::lp_3,			.gpio = CONFIG_BSP_LEDPIXEL3,	.pos { .r = 1, .g = 0, .b = 2 }},
};

Ledpixel::Ledpixel()
{
	led_strip_config_t led_strip_config =
	{
		.strip_gpio_num = -1,
		.max_leds = this->ledpixel_leds_size,
		.led_model = LED_MODEL_WS2812,
		.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
		.flags =
		{
			.invert_out = 0,
		},
	};
	static constexpr led_strip_rmt_config_t rmt_config =
	{
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.resolution_hz = 0,
		.mem_block_symbols = 0,
		.flags =
		{
			.with_dma = 0,
		},
	};

	esp_err_t rv;
	lp_t handle;
	channel_t *channel;
	const handle_to_gpio_t *handle_to_gpio_ptr;

	if(this->singleton)
		throw(hard_exception("Ledpixel: already active"));

	for(handle = lp_first; handle < this->lp_size; handle = static_cast<lp_t>(handle + 1))
	{
		handle_to_gpio_ptr = &this->handle_to_gpio[handle];
		assert(handle_to_gpio_ptr->handle == handle);

		channel = &channels[handle];
		channel->handle = nullptr;
		channel->owner = "";
		channel->available = 0;
		channel->open = 0;

		if(handle_to_gpio_ptr->gpio >= 0)
		{
			channel->gpio = handle_to_gpio_ptr->gpio;
			led_strip_config.strip_gpio_num = channel->gpio;
			led_strip_config.color_component_format.format.r_pos = handle_to_gpio_ptr->pos.r;
			led_strip_config.color_component_format.format.g_pos = handle_to_gpio_ptr->pos.g;
			led_strip_config.color_component_format.format.b_pos = handle_to_gpio_ptr->pos.b;
			led_strip_config.color_component_format.format.reserved = 0;
			led_strip_config.color_component_format.format.num_components = 3;

			if((rv = led_strip_new_rmt_device(&led_strip_config, &rmt_config, &channel->handle)) != ESP_OK)
				throw(hard_exception(util_esp_string_error(rv, "Ledpixel: led_strip_new_rmt_device")));

			if((rv = led_strip_clear(channel->handle)) != ESP_OK)
				throw(hard_exception(util_esp_string_error(rv, "Ledpixel: led_strip_clear")));

			if((rv = led_strip_refresh(channel->handle)) != ESP_OK)
				throw(hard_exception(util_esp_string_error(rv, "Ledpixel: led_strip_flush")));

			channel->available = 1;
		}
	}

	this->singleton = this;
}

Ledpixel& Ledpixel::get()
{
	if(!Ledpixel::singleton)
		throw(hard_exception("Ledpixel: not active"));

	return(*Ledpixel::singleton);
}

void Ledpixel::open(lp_t handle, std::string_view owner)
{
	esp_err_t rv;
	channel_t *channel;
	unsigned int ix;

	assert(handle < this->lp_size);

	channel = &channels[handle];

	if(!channel->available)
		throw(transient_exception("Ledpixel::open: channel unavailable"));

	if(channel->open)
		throw(transient_exception("Ledpixel::open: channel in use"));

	channel->owner = owner;
	channel->open = 1;

	if((rv = led_strip_clear(channel->handle)) != ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "Ledpixel::open: led_strip_clear")));

	if((rv = led_strip_refresh(channel->handle)) != ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "Ledpixel::open: led_strip_flush")));

	for(ix = 0; ix < this->ledpixel_leds_size; ix++)
	{
		channel->rgbvalue[ix].r = 0;
		channel->rgbvalue[ix].g = 0;
		channel->rgbvalue[ix].b = 0;
	}
}

void Ledpixel::set(lp_t handle, int index, int red, int green, int blue)
{
	esp_err_t rv;
	channel_t *channel;
	rgb_t *rgb;

	assert(handle < this->lp_size);
	assert((index >= 0) && (index < this->ledpixel_leds_size));

	channel = &channels[handle];

	if(!channel->open)
		throw(transient_exception("Ledpixel::set: channel not open"));

	rgb = &channel->rgbvalue[index];

	rgb->r = red;
	rgb->g = green;
	rgb->b = blue;

	if((rv = led_strip_set_pixel(channel->handle, index, rgb->r, rgb->g, rgb->b)) != ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "Ledpixel::set: led_strip_set_pixel")));
}

void Ledpixel::get(lp_t handle, int index, int &red, int &green, int &blue)
{
	const channel_t *channel;
	const rgb_t *rgb;

	assert(handle < this->lp_size);
	assert((index >= 0) && (index < this->ledpixel_leds_size));

	channel = &channels[handle];

	if(!channel->open)
		throw(transient_exception("Ledpixel::get: channel not open"));

	rgb = &channel->rgbvalue[index];

	red = rgb->r;
	green = rgb->g;
	blue = rgb->b;
}

void Ledpixel::flush(lp_t handle)
{
	esp_err_t rv;
	channel_t *channel;

	assert((handle >= 0) && (handle < this->lp_size));

	channel = &this->channels[handle];

	if(!channel->open)
		throw(transient_exception("Ledpixel::flush: channel not open"));

	if((rv = led_strip_refresh(channel->handle)) == ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "Ledpixel::flush: led_strip_refresh")));
}

void Ledpixel::info(std::string &dst)
{
	lp_t handle;
	const channel_t *channel;
	const rgb_t *rgb;
	unsigned int ix;

	dst += std::format("- channels available: {:d}", static_cast<unsigned int>(this->lp_size));
	dst += "\nchannels:";

	for(handle = lp_first; handle < this->lp_size; handle = static_cast<lp_t>(handle + 1))
	{
		channel = &channels[handle];

		if(channel->available)
		{
			if(!channel->open)
				throw(hard_exception("Ledpixel::dump: channel not open"));

			if(channel->owner.empty())
				throw(hard_exception("Ledpixel::dump: channel has no owner"));

			dst += std::format("\n- channel {:d}: gpio {:2d} is {}, owned by: {}\n   rgbvalues:",
					static_cast<unsigned int>(handle),
					channel->gpio,
					channel->open ? "open" : "not open",
					channel->open ? channel->owner : "<none>");

			for(ix = 0; ix < this->ledpixel_leds_size; ix++)
			{
				rgb = &channel->rgbvalue[ix];
				dst += std::format(" (R:{:#04x},G:{:#04x},B:{:#04x})", rgb->r, rgb->g, rgb->b);
			}
		}
		else
			dst += std::format("\n- channel {:d}: unavailable", static_cast<unsigned int>(handle));

	}
}
