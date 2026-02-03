#include "ledpwm.h"

#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "exception.h"

#include <driver/ledc.h>
#include <soc/soc.h>

#include <format>

LedPWM *LedPWM::singleton = nullptr;

const LedPWM::handle_to_gpio_t LedPWM::handle_to_gpio[LedPWM::lpt_size] =
{
	[LedPWM::lpt_14bit_5khz_notify]	=		{ .gpio = CONFIG_BSP_LEDPWM0, .timer = LedPWM::timer_5khz,	.frequency = 5000 },
	[LedPWM::lpt_14bit_5khz_lcd_spi_2] 	=	{ .gpio = CONFIG_BSP_LEDPWM1, .timer = LedPWM::timer_5khz,	.frequency = 5000 },
	[LedPWM::lpt_14bit_5khz_lcd_spi_3] =	{ .gpio = CONFIG_BSP_LEDPWM2, .timer = LedPWM::timer_5khz,	.frequency = 5000 },
	[LedPWM::lpt_14bit_120hz] =				{ .gpio = CONFIG_BSP_LEDPWM3, .timer = LedPWM::timer_120hz,	.frequency =  120 },
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
	ledpwm_t handle;
	const handle_to_gpio_t *handle_to_gpio_ptr;
	channel_t *channel;

	static_assert(static_cast<unsigned int>(this->lpt_size) < static_cast<unsigned int>(LEDC_CHANNEL_MAX));
	static_assert(static_cast<unsigned int>(this->timer_size) < static_cast<unsigned int>(LEDC_TIMER_MAX));

	if(this->singleton)
		throw(hard_exception("LedPWM: already active"));

	timer_config.freq_hz = 4882; // APB_CLK_FREQ / (1UL << 14), max
	timer_config.timer_num = static_cast<ledc_timer_t>(this->timer_5khz);
	if((rv = ledc_timer_config(&timer_config)) != ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "LedPWM: ledc_timer_config 4882")));

	timer_config.freq_hz = 120;
	timer_config.timer_num = static_cast<ledc_timer_t>(this->timer_120hz);
	if((rv = ledc_timer_config(&timer_config)) != ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "LedPWM: ledc_timer_config 120")));

	for(handle = this->lpt_first; handle < this->lpt_size; handle = static_cast<ledpwm_t>(handle + 1))
	{
		handle_to_gpio_ptr = &this->handle_to_gpio[handle];
		channel = &this->channels[handle];

		channel->timer = handle_to_gpio_ptr->timer;
		channel->handle = handle;
		channel->frequency = handle_to_gpio_ptr->frequency;
		channel->gpio = handle_to_gpio_ptr->gpio;
		channel->owner = "";
		channel->available = 0;
		channel->open = 0;

		if(handle_to_gpio_ptr->gpio >= 0)
		{
			channel_config.channel = static_cast<ledc_channel_t>(handle);
			channel_config.gpio_num = handle_to_gpio_ptr->gpio;
			channel_config.timer_sel = static_cast<ledc_timer_t>(handle_to_gpio_ptr->timer);

			if((rv = ledc_channel_config(&channel_config)) != ESP_OK)
				throw(hard_exception(util_esp_string_error(rv, "LedPWM: ledc_channel_config")));

			channel->available = 1;
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

void LedPWM::open(ledpwm_t handle, std::string_view owner)
{
	esp_err_t rv;
	channel_t *channel;

	assert(handle < this->lpt_size);

	channel = &this->channels[handle];

	if(!channel->available)
		throw(transient_exception("LedPWM::open: channel unavailable"));

	if(channel->open)
		throw(transient_exception("LedPWM::open: channel in use"));

	channel->open = 1;
	channel->owner = owner;

	if((rv = ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(handle), 0, 0)) != ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "ledc_set_duty_and_update")));
}

void LedPWM::set(ledpwm_t handle, int duty)
{
	esp_err_t rv;
	channel_t *channel;

	assert(handle < this->lpt_size);

	channel = &this->channels[handle];

	if(!channel->open)
		throw(transient_exception("LedPWM::set: channel not open"));

	if(duty >= this->ledpwm_max_duty)
		duty = this->ledpwm_max_duty - 1;
	else
		if(duty < 0)
			duty = 0;

	if((rv = ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(handle), duty, 0)) != ESP_OK)
		throw(hard_exception(util_esp_string_error(rv, "ledc_set_duty_and_update")));
}

int LedPWM::get(ledpwm_t handle)
{
	const channel_t *channel;

	assert(handle < this->lpt_size);

	channel = &this->channels[handle];

	if(!channel->open)
		throw(transient_exception("LedPWM::get: channel not open"));

	return(ledc_get_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(handle)));
}

void LedPWM::info(std::string &dst)
{
	ledpwm_t handle;
	channel_t *channel;

	dst += std::format("- channels available: {:d}", static_cast<int>(this->lpt_size));
	dst += "\nchannels:";

	for(handle = this->lpt_first; handle < this->lpt_size; handle = static_cast<ledpwm_t>(handle + 1))
	{
		channel = &this->channels[handle];

		if(channel->available)
		{
			assert(!channel->open || !channel->owner.empty());

			dst += std::format("\n- channel {:d}: 14 bit @ {:4d} Hz, timer {:d}, gpio {:2d} is {} open, duty: {:5d}, owned by: {}",
					static_cast<int>(handle),
					channel->frequency,
					channel->timer,
					channel->gpio,
					channel->open ? "   " : "not",
					ledc_get_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(handle)),
					channel->open ? channel->owner : "<none>");
		}
		else
			dst = std::format("\n- channel {:d} is unavailable", static_cast<int>(handle));

	}
}
