#include "ledpwm.h"

#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "exception.h"

#include <driver/ledc.h>

#include <format>

#include "magic_enum/magic_enum.hpp"

LedPWM *LedPWM::singleton = nullptr;

const LedPWM::channel_to_gpio_t LedPWM::channel_to_gpio[LedPWM::channels_size] =
{
	{	.channel = LedPWM::Channel::channel_14bit_5khz_notify,		.gpio = CONFIG_BSP_LEDPWM0, .timer = LedPWM::timer_t::timer_5khz,	.frequency = 5000 },
	{	.channel = LedPWM::Channel::channel_14bit_5khz_lcd_spi_2,	.gpio = CONFIG_BSP_LEDPWM1, .timer = LedPWM::timer_t::timer_5khz,	.frequency = 5000 },
	{	.channel = LedPWM::Channel::channel_14bit_5khz_lcd_spi_3,	.gpio = CONFIG_BSP_LEDPWM2, .timer = LedPWM::timer_t::timer_5khz,	.frequency = 5000 },
	{	.channel = LedPWM::Channel::channel_14bit_120hz,			.gpio = CONFIG_BSP_LEDPWM3, .timer = LedPWM::timer_t::timer_120hz,	.frequency =  120 },
};

LedPWM::LedPWM()
{
	ledc_timer_config_t timer_config =
	{
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = static_cast<ledc_timer_bit_t>(14),
		.timer_num = static_cast<ledc_timer_t>(-1),
		.freq_hz = 0,
		.clk_cfg = LEDC_USE_APB_CLK,
		.deconfigure = false,
	};
	ledc_channel_config_t channel_config =
	{
		.gpio_num = -1,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = static_cast<ledc_channel_t>(-1),
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = static_cast<ledc_timer_t>(-1),
		.duty = 0,
		.hpoint = 0,
		.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
		.flags =
		{
			.output_invert = 0,
		},
	};

	esp_err_t rv;
	const channel_to_gpio_t *channel_to_gpio_ptr;
	handle_t *handle;

	static_assert(magic_enum::enum_count<Channel>() == channels_size);
	static_assert(magic_enum::enum_count<Channel>() < LEDC_CHANNEL_MAX);
	static_assert(magic_enum::enum_count<timer_t>() < LEDC_TIMER_MAX);

	if(this->singleton)
		throw(hard_exception("LedPWM: already active"));

	timer_config.freq_hz = 4882; // APB_CLK_FREQ / (1UL << 14), max
	timer_config.timer_num = static_cast<ledc_timer_t>(magic_enum::enum_integer(timer_t::timer_5khz));

	if((rv = ledc_timer_config(&timer_config)) != ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "LedPWM: ledc_timer_config 4882")));

	timer_config.freq_hz = 120;
	timer_config.timer_num = static_cast<ledc_timer_t>(magic_enum::enum_integer(timer_t::timer_120hz));
	if((rv = ledc_timer_config(&timer_config)) != ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "LedPWM: ledc_timer_config 120")));

	for(const auto &channel : magic_enum::enum_values<Channel>())
	{
		channel_to_gpio_ptr = &this->channel_to_gpio[magic_enum::enum_integer(channel)];

		assert(channel_to_gpio_ptr->channel == channel);

		handle = &this->handles[magic_enum::enum_integer(channel)];
		handle->timer = channel_to_gpio_ptr->timer;
		handle->channel = channel;
		handle->frequency = channel_to_gpio_ptr->frequency;
		handle->gpio = channel_to_gpio_ptr->gpio;
		handle->owner = "";
		handle->available = 0;
		handle->open = 0;

		if(channel_to_gpio_ptr->gpio >= 0)
		{
			channel_config.channel = static_cast<ledc_channel_t>(magic_enum::enum_integer(channel));
			channel_config.gpio_num = channel_to_gpio_ptr->gpio;
			channel_config.timer_sel = static_cast<ledc_timer_t>(magic_enum::enum_integer(channel_to_gpio_ptr->timer));

			if((rv = ledc_channel_config(&channel_config)) != ESP_OK)
				throw(hard_exception(util_esp_string_error(rv, "LedPWM: ledc_channel_config")));

			handle->available = 1;
		}
	}

	if((rv = ledc_fade_func_install(0)) != ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "LedPWM: ledc_fade_func_install")));

	this->singleton = this;
}

LedPWM& LedPWM::get()
{
	if(!LedPWM::singleton)
		throw(hard_exception("LedPWM::get: not active"));

	return(*LedPWM::singleton);
}

void LedPWM::open(Channel channel, std::string_view owner)
{
	esp_err_t rv;
	handle_t *handle;

	if(!magic_enum::enum_contains<Channel>(channel))
		throw(hard_exception("Ledpixel::open: invalid channel"));

	handle = &this->handles[magic_enum::enum_integer(channel)];

	if(!handle->available)
		throw(transient_exception("LedPWM::open: channel unavailable"));

	if(handle->open)
		throw(transient_exception("LedPWM::open: channel in use"));

	handle->open = 1;
	handle->owner = owner;

	if((rv = ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(magic_enum::enum_integer(channel)), 0, 0)) != ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "ledc_set_duty_and_update")));
}

void LedPWM::set(Channel channel, int duty)
{
	esp_err_t rv;
	handle_t *handle;

	if(!magic_enum::enum_contains<Channel>(channel))
		throw(hard_exception("Ledpixel::open: invalid channel"));

	handle = &this->handles[magic_enum::enum_integer(channel)];

	if(!handle->open)
		throw(transient_exception("LedPWM::set: channel not open"));

	if(duty >= this->max_duty)
		duty = this->max_duty - 1;
	else
		if(duty < 0)
			duty = 0;

	if((rv = ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(magic_enum::enum_integer(channel)), duty, 0)) != ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "ledc_set_duty_and_update")));
}

int LedPWM::get(Channel channel)
{
	const handle_t *handle;

	if(!magic_enum::enum_contains<Channel>(channel))
		throw(hard_exception("Ledpixel::open: invalid channel"));

	handle = &this->handles[magic_enum::enum_integer(channel)];

	if(!handle->open)
		throw(transient_exception("LedPWM::get: channel not open"));

	return(ledc_get_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(magic_enum::enum_integer(channel))));
}

void LedPWM::info(std::string &dst)
{
	handle_t *handle;
	std::string channel_name;
	std::string timer_name;

	dst += std::format("- channels available: {:d}", magic_enum::enum_count<Channel>());
	dst += "\nchannels:";

	for(const auto &entry : magic_enum::enum_entries<Channel>())
	{
		handle = &this->handles[magic_enum::enum_integer(entry.first)];

		if(handle->available)
		{
			channel_name = entry.second;
			channel_name += ",";

			timer_name = magic_enum::enum_name(handle->timer);
			timer_name += ",";

			dst += std::format("\n- channel {:d}: {:29s} 14 bit @ {:4d} Hz, timer {:d}: {:12s} gpio {:2d} is {} open, duty: {:5d}, owned by: {}",
					magic_enum::enum_integer(entry.first),
					channel_name,
					handle->frequency,
					magic_enum::enum_integer(handle->timer),
					timer_name,
					handle->gpio,
					handle->open ? "   " : "not",
					ledc_get_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(magic_enum::enum_integer(entry.first))),
					handle->open ? handle->owner : "<none>");
		}
		else
			dst = std::format("\n- channel {:d}: {}: unavailable",
					magic_enum::enum_integer(entry.first),
					channel_name);
	}
}
