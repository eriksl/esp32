#include <stdint.h>
#include <stdbool.h>

#include "log.h"

#include "string.h"
#include "util.h"
#include "cli-command.h"
#include "mcpwm.h"

#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_types.h"
#include "soc/soc.h"

#include <string>
#include <boost/format.hpp>

enum
{
	base_clock = 160000000,
	timer_frequency_150hz = base_clock / 16,
	timer_frequency_2400hz = base_clock,
	timer_resolution = 16,
	timer_ticks = (1U << timer_resolution) - 1,
};

static_assert(mpt_size <= (SOC_MCPWM_GROUPS * SOC_MCPWM_TIMERS_PER_GROUP));

typedef struct
{
	mcpwm_cmpr_handle_t handle;
} tree_comparator_t;

typedef struct
{
	mcpwm_gen_handle_t handle;
} tree_generator_t;

typedef struct
{
	tree_comparator_t comparator;
	tree_generator_t generator;
	mcpwm_oper_handle_t handle;
} tree_operator_t;

typedef struct
{
	mcpwm_timer_handle_t handle;
	tree_operator_t operator_;
} tree_timer_t;

typedef struct
{
	tree_timer_t timer[2];
} tree_group_t;

typedef struct
{
	tree_group_t group[2];
} mcpwm_tree_t;

typedef struct __attribute__((aligned(sizeof(uint32_t))))
{
	int gpio;
	unsigned int group;
	unsigned int timer;
	unsigned int frequency;
	const char *owner;
	unsigned int duty;
	const tree_comparator_t *comparator;
	struct
	{
		unsigned int available;
		unsigned int open;
	};
} channel_t;

typedef struct
{
	int gpio;
	unsigned int group;
	unsigned int timer;
	unsigned int timer_frequency;
	unsigned int pwm_frequency;
} handle_to_group_timer_t;

static const handle_to_group_timer_t handle_to_group_timer[mpt_size] =
{
	[mpt_16bit_150hz_0] =	{ .gpio = CONFIG_BSP_MCPWM0, .group = 0, .timer = 0, .timer_frequency = timer_frequency_150hz,  .pwm_frequency = 150 },
	[mpt_16bit_150hz_1] =	{ .gpio = CONFIG_BSP_MCPWM1, .group = 0, .timer = 1, .timer_frequency = timer_frequency_150hz,  .pwm_frequency = 150 },
	[mpt_16bit_2400hz_0] =	{ .gpio = CONFIG_BSP_MCPWM2, .group = 1, .timer = 0, .timer_frequency = timer_frequency_2400hz, .pwm_frequency = 2400 },
	[mpt_16bit_2400hz_1] =	{ .gpio = CONFIG_BSP_MCPWM3, .group = 1, .timer = 1, .timer_frequency = timer_frequency_2400hz, .pwm_frequency = 2400 },
};

static bool inited = false;
static channel_t *channels;
static mcpwm_tree_t *tree;

static const tree_comparator_t *setup(const handle_to_group_timer_t *mpt_ptr)
{
	const mcpwm_timer_config_t timer_config =
	{
		.group_id = static_cast<int>(mpt_ptr->group),
		.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
		.resolution_hz = mpt_ptr->timer_frequency,
		.count_mode = MCPWM_TIMER_COUNT_MODE_UP,
		.period_ticks = timer_ticks,
		.intr_priority = 0,
		.flags =
		{
			.update_period_on_empty = 0,
			.update_period_on_sync = 0,
			.allow_pd = 0,
		},
	};
	const mcpwm_operator_config_t operator_config =
	{
		.group_id = static_cast<int>(mpt_ptr->group),
		.intr_priority = 0,
		.flags = {},
	};
	const mcpwm_generator_config_t generator_config =
	{
		.gen_gpio_num = mpt_ptr->gpio,
		.flags =
		{
			.invert_pwm = 0,
			.io_loop_back = 0,
			.io_od_mode = 0,
			.pull_up = 0,
			.pull_down = 0,
		},
	};
	const mcpwm_comparator_config_t comparator_config =
	{
		.intr_priority = 0,
		.flags =
		{
			.update_cmp_on_tez = 1,
			.update_cmp_on_tep = 0,
			.update_cmp_on_sync = 0,
		},
	};

	tree_group_t *group = &tree->group[mpt_ptr->group];
	tree_timer_t *timer = &group->timer[mpt_ptr->timer];
	tree_operator_t *operator_ = &timer->operator_;
	tree_comparator_t *comparator = &operator_->comparator;
	tree_generator_t *generator = &operator_->generator;

	timer->handle = (mcpwm_timer_handle_t)0;
	operator_->handle = (mcpwm_oper_handle_t)0;
	comparator->handle = (mcpwm_cmpr_handle_t)0;
	generator->handle = (mcpwm_gen_handle_t)0;

	if(mpt_ptr->gpio < 0)
		return((const tree_comparator_t *)0);

	util_abort_on_esp_err("mcpwm_new_timer", mcpwm_new_timer(&timer_config, &timer->handle));
	util_abort_on_esp_err("mcpwm_new_operator", mcpwm_new_operator(&operator_config, &operator_->handle));
	util_abort_on_esp_err("mcpwm_operator_connect_timer", mcpwm_operator_connect_timer(operator_->handle, timer->handle));
	util_abort_on_esp_err("mcpwm_new_comparator", mcpwm_new_comparator(operator_->handle, &comparator_config, &comparator->handle));
	util_abort_on_esp_err("mcpwm_new_generator", mcpwm_new_generator(operator_->handle, &generator_config, &generator->handle));

	util_abort_on_esp_err("mcpwm_generator_set_action_on_timer_event",
			mcpwm_generator_set_action_on_timer_event(generator->handle, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));

	util_abort_on_esp_err("mcpwm_generator_set_action_on_compare_event",
			mcpwm_generator_set_action_on_compare_event(generator->handle, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator->handle, MCPWM_GEN_ACTION_LOW)));

	util_abort_on_esp_err("mcpwm_comparator_set_compare_value", mcpwm_comparator_set_compare_value(comparator->handle, 0));
	util_abort_on_esp_err("mcpwm_timer_enable", mcpwm_timer_enable(timer->handle));
	util_abort_on_esp_err("mcpwm_timer_start_stop", mcpwm_timer_start_stop(timer->handle, MCPWM_TIMER_START_NO_STOP));

	return(comparator);
}

void mcpwm_init(void)
{
	mcpwm_t handle;
	channel_t *channel;
	const handle_to_group_timer_t *handle_to_group_timer_ptr;

	assert(!inited);

	tree = new mcpwm_tree_t;
	channels = new channel_t[mpt_size];

	for(handle = mpt_first; handle < mpt_size; handle = static_cast<mcpwm_t>(handle + 1))
	{
		handle_to_group_timer_ptr = &handle_to_group_timer[handle];
		channel = &channels[handle];

		channel->owner = (const char *)0;
		channel->group = handle_to_group_timer_ptr->group;
		channel->timer = handle_to_group_timer_ptr->timer;
		channel->frequency = handle_to_group_timer_ptr->pwm_frequency;
		channel->gpio = handle_to_group_timer_ptr->gpio;
		channel->duty = 0;
		channel->open = 0;
		channel->available = !!(channel->comparator = setup(handle_to_group_timer_ptr));
	}

	inited = true;
}

bool mcpwm_open(mcpwm_t handle, const char *owner)
{
	channel_t *channel;

	assert(inited);
	assert(handle < mpt_size);

	channel = &channels[handle];

	if(!channel->available)
		return(false);

	if(channel->open)
		return(true);

	assert(channel->comparator);

	channel->open = 1;
	channel->duty = 0;
	channel->owner = owner;

	util_abort_on_esp_err("mcpwm_comparator_set_compare_value", mcpwm_comparator_set_compare_value(channel->comparator->handle, channel->duty));

	return(true);
}

void mcpwm_set(mcpwm_t handle, unsigned int duty)
{
	channel_t *channel;

	assert(inited);
	assert(handle < mpt_size);

	channel = &channels[handle];

	assert(channel->open);
	assert(channel->comparator);

	channel->duty = duty <= static_cast<unsigned int>(timer_ticks) ? duty : static_cast<unsigned int>(timer_ticks);

	util_abort_on_esp_err("mcpwm_comparator_set_compare_value", mcpwm_comparator_set_compare_value(channel->comparator->handle, channel->duty));
}

unsigned int mcpwm_get(mcpwm_t handle)
{
	const channel_t *channel;

	assert(inited);
	assert(handle < mpt_size);

	channel = &channels[handle];

	return(channel->duty);
}

void command_mcpwm_info(cli_command_call_t *call)
{
	mcpwm_t handle;
	channel_t *channel;

	assert(inited);
	assert(call);
	assert(call->parameter_count == 0);

	call->result = "MC-PWM INFO:";
	call->result += (boost::format("\n- channels available: %u") % mpt_size).str();
	call->result += "\nchannels:";

	for(handle = mpt_first; handle < mpt_size; handle = static_cast<mcpwm_t>(handle + 1))
	{
		channel = &channels[handle];

		if(channel->available)
			call->result += (boost::format("\n- channel %u: 16 bits @ %4u Hz, group %u, timer %u, gpio %2d is %s duty: %5u, owned by %s") %
					handle %
					channel->frequency %
					channel->group %
					channel->timer %
					channel->gpio %
					(channel->open ? "open" : "not open") %
					channel->duty %
					channel->owner).str();
		else
			call->result += (boost::format("\n- channel %d is unavailable") % handle).str();
	}
}
