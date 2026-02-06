#include "log.h"
#include "util.h"
#include "exception.h"
#include "ledpixel.h"

#include <sdkconfig.h>

#include <led_strip_types.h>

#include <format>

Ledpixel *Ledpixel::singleton = nullptr;

const Ledpixel::channel_to_gpio_t Ledpixel::channel_to_gpio[Ledpixel::channels_size] =
{
	{ .channel = Ledpixel::Channel::channel_0_notify,	.gpio = CONFIG_BSP_LEDPIXEL0,	.pos { .r = 0, .g = 1, .b = 2 }},
	{ .channel = Ledpixel::Channel::channel_1,			.gpio = CONFIG_BSP_LEDPIXEL1,	.pos { .r = 1, .g = 0, .b = 2 }},
	{ .channel = Ledpixel::Channel::channel_2,			.gpio = CONFIG_BSP_LEDPIXEL2,	.pos { .r = 1, .g = 0, .b = 2 }},
	{ .channel = Ledpixel::Channel::channel_3,			.gpio = CONFIG_BSP_LEDPIXEL3,	.pos { .r = 1, .g = 0, .b = 2 }},
};

Ledpixel::Ledpixel()
{
	led_strip_config_t led_strip_config =
	{
		.strip_gpio_num = -1,
		.max_leds = this->leds_size,
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
	Channel channel;
	handle_t *handle;
	const channel_to_gpio_t *channel_to_gpio_ptr;

	if(this->singleton)
		throw(hard_exception("Ledpixel: already active"));

	for(channel = this->Channel::first; static_cast<unsigned int>(channel) < this->channels_size; channel = static_cast<Channel>(static_cast<unsigned int>(channel) + 1))
	{
		channel_to_gpio_ptr = &this->channel_to_gpio[static_cast<unsigned int>(channel)];

		assert(channel_to_gpio_ptr->channel == channel);

		handle = &handles[static_cast<unsigned int>(channel)];
		handle->handle = nullptr;
		handle->owner = "";
		handle->available = 0;
		handle->open = 0;

		if(channel_to_gpio_ptr->gpio >= 0)
		{
			handle->gpio = channel_to_gpio_ptr->gpio;
			led_strip_config.strip_gpio_num = handle->gpio;
			led_strip_config.color_component_format.format.r_pos = channel_to_gpio_ptr->pos.r;
			led_strip_config.color_component_format.format.g_pos = channel_to_gpio_ptr->pos.g;
			led_strip_config.color_component_format.format.b_pos = channel_to_gpio_ptr->pos.b;
			led_strip_config.color_component_format.format.reserved = 0;
			led_strip_config.color_component_format.format.num_components = 3;

			if((rv = led_strip_new_rmt_device(&led_strip_config, &rmt_config, &handle->handle)) != ESP_OK)
				throw(hard_exception(util_esp_string_error(rv, "Ledpixel: led_strip_new_rmt_device")));

			if((rv = led_strip_clear(handle->handle)) != ESP_OK)
				throw(hard_exception(util_esp_string_error(rv, "Ledpixel: led_strip_clear")));

			if((rv = led_strip_refresh(handle->handle)) != ESP_OK)
				throw(hard_exception(util_esp_string_error(rv, "Ledpixel: led_strip_flush")));

			handle->available = 1;
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

void Ledpixel::open(Channel channel, std::string_view owner)
{
	esp_err_t rv;
	handle_t *handle;
	unsigned int ix;

	assert(static_cast<unsigned int>(channel) < this->channels_size);

	handle = &handles[static_cast<unsigned int>(channel)];

	if(!handle->available)
		throw(transient_exception("Ledpixel::open: channel unavailable"));

	if(handle->open)
		throw(transient_exception("Ledpixel::open: channel in use"));

	handle->owner = owner;
	handle->open = 1;

	if((rv = led_strip_clear(handle->handle)) != ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "Ledpixel::open: led_strip_clear")));

	if((rv = led_strip_refresh(handle->handle)) != ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "Ledpixel::open: led_strip_flush")));

	for(ix = 0; ix < this->leds_size; ix++)
	{
		handle->rgbvalue[ix].r = 0;
		handle->rgbvalue[ix].g = 0;
		handle->rgbvalue[ix].b = 0;
	}
}

void Ledpixel::set(Channel channel, int led, int red, int green, int blue)
{
	esp_err_t rv;
	handle_t *handle;
	rgb_t *rgb;

	assert(static_cast<unsigned int>(channel) < this->channels_size);
	assert((led >= 0) && (led < this->leds_size));

	handle = &handles[static_cast<unsigned int>(channel)];

	if(!handle->open)
		throw(transient_exception("Ledpixel::set: channel not open"));

	rgb = &handle->rgbvalue[led];

	rgb->r = red;
	rgb->g = green;
	rgb->b = blue;

	if((rv = led_strip_set_pixel(handle->handle, led, rgb->r, rgb->g, rgb->b)) != ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "Ledpixel::set: led_strip_set_pixel")));
}

void Ledpixel::get(Channel channel, int led, int &red, int &green, int &blue)
{
	const handle_t *handle;
	const rgb_t *rgb;

	assert(static_cast<unsigned int>(channel) < this->channels_size);
	assert((led >= 0) && (led < this->leds_size));

	handle = &handles[static_cast<unsigned int>(channel)];

	if(!handle->open)
		throw(transient_exception("Ledpixel::get: channel not open"));

	rgb = &handle->rgbvalue[led];

	red = rgb->r;
	green = rgb->g;
	blue = rgb->b;
}

void Ledpixel::flush(Channel channel)
{
	esp_err_t rv;
	handle_t *handle;

	assert((static_cast<unsigned int>(channel) >= 0) && (static_cast<unsigned int>(channel) < this->channels_size));

	handle = &this->handles[static_cast<unsigned int>(channel)];

	if(!handle->open)
		throw(transient_exception("Ledpixel::flush: channel not open"));

	if((rv = led_strip_refresh(handle->handle)) == ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "Ledpixel::flush: led_strip_refresh")));
}

void Ledpixel::info(std::string &dst)
{
	Channel channel;
	const handle_t *handle;
	const rgb_t *rgb;
	unsigned int ix;

	dst += std::format("- channels available: {:d}", static_cast<unsigned int>(this->channels_size));
	dst += "\nchannels:";

	for(channel = this->Channel::first; static_cast<unsigned int>(channel) < this->channels_size; channel = static_cast<Channel>(static_cast<unsigned int>(channel) + 1))
	{
		handle = &handles[static_cast<unsigned int>(channel)];

		if(handle->available)
		{
			if(!handle->open)
				throw(hard_exception("Ledpixel::dump: channel not open"));

			if(handle->owner.empty())
				throw(hard_exception("Ledpixel::dump: channel has no owner"));

			dst += std::format("\n- channel {:d}: gpio {:2d} is {}, owned by: {}\n   rgbvalues:",
					static_cast<unsigned int>(channel),
					handle->gpio,
					handle->open ? "open" : "not open",
					handle->open ? handle->owner : "<none>");

			for(ix = 0; ix < this->leds_size; ix++)
			{
				rgb = &handle->rgbvalue[ix];
				dst += std::format(" (R:{:#04x},G:{:#04x},B:{:#04x})", rgb->r, rgb->g, rgb->b);
			}
		}
		else
			dst += std::format("\n- channel {:d}: unavailable", static_cast<unsigned int>(channel));

	}
}
