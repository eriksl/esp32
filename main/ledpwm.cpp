#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <driver/ledc.h>
#include <soc/soc.h>

#include "string.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "ledpwm.h"

#include <string>
#include <boost/format.hpp>

enum
{
	ledpwm_resolution = 14,
	ledpwm_max_duty = (1U << ledpwm_resolution) - 1,
};

enum
{
	timer_5khz = 0,
	timer_120hz,
	timer_size,
};

static_assert((unsigned int)lpt_size < LEDC_CHANNEL_MAX);
static_assert((unsigned int)timer_size < LEDC_TIMER_MAX);

typedef struct
{
	int gpio;
	unsigned int timer;
	unsigned int handle;
	unsigned int frequency;
	const char *owner;
	struct
	{
		unsigned int available;
		unsigned int open;
	};
} channel_t;

typedef struct
{
	int gpio;
	unsigned int timer;
	unsigned int frequency;
} handle_to_gpio_t;

static const handle_to_gpio_t handle_to_gpio[lpt_size] =
{
	[lpt_14bit_5khz_notify]	=		{ .gpio = CONFIG_BSP_LEDPWM0, .timer = timer_5khz,	.frequency = 5000 },
	[lpt_14bit_5khz_lcd_spi_2] 	=	{ .gpio = CONFIG_BSP_LEDPWM1, .timer = timer_5khz,	.frequency = 5000 },
	[lpt_14bit_5khz_lcd_spi_3] =	{ .gpio = CONFIG_BSP_LEDPWM2, .timer = timer_5khz,	.frequency = 5000 },
	[lpt_14bit_120hz] =				{ .gpio = CONFIG_BSP_LEDPWM3, .timer = timer_120hz,	.frequency =  120 },
};

static bool inited = false;
static channel_t *channels;

void ledpwm_init(void)
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
	ledpwm_t handle;
	const handle_to_gpio_t *handle_to_gpio_ptr;
	channel_t *channel;

	assert(!inited);

	timer_config.freq_hz = 4882; // APB_CLK_FREQ / (1UL << 14), max
	timer_config.timer_num = static_cast<ledc_timer_t>(timer_5khz);
	util_abort_on_esp_err("ledc_timer_config", ledc_timer_config(&timer_config));

	timer_config.freq_hz = 120;
	timer_config.timer_num = static_cast<ledc_timer_t>(timer_120hz);
	util_abort_on_esp_err("ledc_timer_config", ledc_timer_config(&timer_config));

	channels = new channel_t[lpt_size];

	for(handle = lpt_first; handle < lpt_size; handle = static_cast<ledpwm_t>(handle + 1))
	{
		handle_to_gpio_ptr = &handle_to_gpio[handle];
		channel = &channels[handle];

		channel->timer = handle_to_gpio_ptr->timer;
		channel->handle = handle;
		channel->frequency = handle_to_gpio_ptr->frequency;
		channel->gpio = handle_to_gpio_ptr->gpio;
		channel->owner = (const char *)0;
		channel->available = 0;
		channel->open = 0;

		if(handle_to_gpio_ptr->gpio >= 0)
		{
			channel_config.channel = static_cast<ledc_channel_t>(handle);
			channel_config.gpio_num = handle_to_gpio_ptr->gpio;
			channel_config.timer_sel = static_cast<ledc_timer_t>(handle_to_gpio_ptr->timer);

			util_abort_on_esp_err("ledc_channel_config", ledc_channel_config(&channel_config));
			channel->available = 1;
		}
	}

	util_abort_on_esp_err("ledc_fade_func_install", ledc_fade_func_install(0));

	inited = true;
}

bool ledpwm_open(ledpwm_t handle, const char *owner)
{
	channel_t *channel;

	assert(inited);
	assert(handle < lpt_size);

	channel = &channels[handle];

	if(!channel->available)
		return(false);

	if(channel->open)
		return(true);

	channel->open = 1;
	channel->owner = owner;

	util_abort_on_esp_err("ledc_set_duty_and_update", ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(handle), 0, 0));

	return(true);
}

void ledpwm_set(ledpwm_t handle, unsigned int duty)
{
	channel_t *channel;

	assert(inited);
	assert(handle < lpt_size);

	channel = &channels[handle];

	assert(channel->open);

	if(duty >= ledpwm_max_duty)
		duty = ledpwm_max_duty - 1;

	util_abort_on_esp_err("ledc_set_duty_and_update", ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(handle), duty, 0));
}

unsigned int ledpwm_get(ledpwm_t handle)
{
	const channel_t *channel;

	assert(inited);
	assert(handle < lpt_size);

	channel = &channels[handle];

	assert(channel->open);

	return(ledc_get_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(handle)));
}

void command_ledpwm_info(cli_command_call_t *call)
{
	ledpwm_t handle;
	channel_t *channel;

	assert(inited);
	assert(call);
	assert(call->parameter_count == 0);

	call->result = "LED-PWM INFO:";
	call->result += (boost::format("\n- channels available: %u") % lpt_size).str();
	call->result += "\nchannels:";

	for(handle = lpt_first; handle < lpt_size; handle = static_cast<ledpwm_t>(handle + 1))
	{
		channel = &channels[handle];

		if(channel->available)
		{
			assert(!channel->open || channel->owner);

			call->result += (boost::format("\n- channel %u: 14 bits @ %4u Hz, timer %u, gpio %2d is %s duty: %5u, owned by: %s") %
					handle %
					channel->frequency %
					channel->timer %
					channel->gpio %
					(channel->open ? "open" : "not open") %
					ledc_get_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(handle)) %
					(channel->open ? channel->owner : "<none>")).str();
		}
		else
			call->result = (boost::format("\n- channel %d is unavailable") % handle).str();

	}
}
