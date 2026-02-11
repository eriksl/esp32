#include "pdm.h"

#include "exception.h"

#include "driver/gpio.h"

#include <format>

const std::map<PDM::Channel, int> PDM::channel_to_gpio
{
	{ PDM::Channel::channel_8bit_150khz_0, CONFIG_BSP_PDM0 },
	{ PDM::Channel::channel_8bit_150khz_1, CONFIG_BSP_PDM1 },
	{ PDM::Channel::channel_8bit_150khz_2, CONFIG_BSP_PDM2 },
	{ PDM::Channel::channel_8bit_150khz_3, CONFIG_BSP_PDM3 },
};

PDM *PDM::singleton = nullptr;

PDM::PDM(Log &log_in) : log(log_in)
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

	handle_t *handle;
	int gpio;
	esp_err_t rv;

	if(this->singleton)
		throw(hard_exception("PDM::PDM already active"));

	static_assert(magic_enum::enum_count<Channel>() == channels_size);

	try
	{
		for(const auto &channel : magic_enum::enum_values<Channel>())
		{
			handle = &this->handles[channel];

			handle->owner = "";
			handle->density = 0;
			handle->handle = nullptr;
			handle->available = 0;
			handle->open = 0;

			gpio = this->channel_to_gpio.at(channel);

			if(gpio >= 0)
			{
				handle->available = 1;

				gpio_pin_config.pin_bit_mask = (1ULL << gpio);

				if((rv = gpio_reset_pin(static_cast<gpio_num_t>(gpio))) != ESP_OK)
					throw(hard_exception(log.esp_string_error(rv, "gpio_reset_pin")));

				if((rv = gpio_config(&gpio_pin_config)) != ESP_OK)
					throw(hard_exception(log.esp_string_error(rv, "gpio_pin_config")));

				sdm_config.gpio_num = gpio;

				if((rv = sdm_new_channel(&sdm_config, &handle->handle)) != ESP_OK)
					throw(hard_exception(log.esp_string_error(rv, "sdm_config")));

				if((rv = sdm_channel_enable(handle->handle)) != ESP_OK)
					throw(hard_exception(log.esp_string_error(rv, "sdm_channel_enable")));

				if((rv = sdm_channel_set_pulse_density(handle->handle, handle->density - 128)) != ESP_OK)
					throw(hard_exception(log.esp_string_error(rv, "sdm_channel_set_pulse_density")));
			}
		}
	}
	catch(const hard_exception &e)
	{
		throw(hard_exception(std::format("PDM::PDM: {}", e.what())));
	}

	this->singleton = this;
}

PDM& PDM::get()
{
	if(!PDM::singleton)
		throw(hard_exception("PDM::get: not active"));

	return(*PDM::singleton);
}

void PDM::open(Channel channel, std::string_view owner)
{
	handle_t *handle;

	if(!this->singleton)
		throw(hard_exception("PDM::open not active"));

	if(!magic_enum::enum_contains<Channel>(channel))
		throw(hard_exception("PDM::open: invalid channel"));

	handle = &this->handles[channel];

	if(!handle->available)
		throw(transient_exception("PDM::open: channel unavailable"));

	if(handle->open)
		throw(transient_exception("PDM::open: channel in use"));

	handle->open = 1;
	handle->owner = owner;
}

void PDM::set(Channel channel, int density)
{
	esp_err_t rv;
	handle_t *handle;

	if(!this->singleton)
		throw(hard_exception("PDM::set not active"));

	if(!magic_enum::enum_contains<Channel>(channel))
		throw(hard_exception("PDM::set: invalid channel"));

	handle = &handles[channel];

	if(!handle->open)
		throw(transient_exception("PDM::set: channel not open"));

	if((density < 0) || (density > 255))
		throw(hard_exception(std::format("PDM::set: value {:d} out of range", density)));

	handle->density = density;

	if((rv = sdm_channel_set_pulse_density(handle->handle, handle->density - 128)) != ESP_OK)
		throw(hard_exception(log.esp_string_error(rv, "sdm_channel_set_pulse_density")));
}

int PDM::get(Channel channel)
{
	const handle_t *handle;

	if(!this->singleton)
		throw(hard_exception("PDM::get not active"));

	if(!magic_enum::enum_contains<Channel>(channel))
		throw(hard_exception("PDM::get: invalid channel"));

	handle = &this->handles[channel];

	return(handle->density);
}

void PDM::info(std::string &out)
{
	handle_t *handle;

	if(!this->singleton)
		throw(hard_exception("PDM::info: not active"));

	out += std::format("- channels available: {:d}",  channels_size);
	out += "\nchannels:";

	for(const auto &entry : magic_enum::enum_entries<Channel>())
	{
		handle = &handles[entry.first];

		if(handle->available)
		{
			out += std::format("\n- channel {:d}: {}: 8 bits @ 150 kHz, gpio {:2d} is {} density: {:3d}, owned by: {}",
					magic_enum::enum_integer(entry.first),
					entry.second,
					channel_to_gpio.at(entry.first),
					(handle->open ? "open" : "not open"),
					handle->density,
					(handle->open ? handle->owner : "<none>"));
		}
	}
}
