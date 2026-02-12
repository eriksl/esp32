#include "mcpwm.h"

#include "log.h"
#include "exception.h"

#include <format>

#include "magic_enum/magic_enum.hpp"

const std::map<MCPWM::Channel, MCPWM::channel_to_group_timer_t> MCPWM::channel_to_group_timer
{
	{ MCPWM::Channel::channel_16bit_150hz_0,	{ .gpio = CONFIG_BSP_MCPWM0, .group = 0, .timer = 0, .timer_frequency = timer_frequency_150hz,  .pwm_frequency = 150 }},
	{ MCPWM::Channel::channel_16bit_150hz_1,	{ .gpio = CONFIG_BSP_MCPWM1, .group = 0, .timer = 1, .timer_frequency = timer_frequency_150hz,  .pwm_frequency = 150 }},
	{ MCPWM::Channel::channel_16bit_2400hz_0,	{ .gpio = CONFIG_BSP_MCPWM2, .group = 1, .timer = 0, .timer_frequency = timer_frequency_2400hz, .pwm_frequency = 2400 }},
	{ MCPWM::Channel::channel_16bit_2400hz_1,	{ .gpio = CONFIG_BSP_MCPWM3, .group = 1, .timer = 1, .timer_frequency = timer_frequency_2400hz, .pwm_frequency = 2400 }},
};

MCPWM *MCPWM::singleton = nullptr;

const MCPWM::tree_comparator_t* MCPWM::setup(const channel_to_group_timer_t *timer_ptr)
{
	const mcpwm_timer_config_t timer_config =
	{
		.group_id = static_cast<int>(timer_ptr->group),
		.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
		.resolution_hz = timer_ptr->timer_frequency,
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
		.group_id = static_cast<int>(timer_ptr->group),
		.intr_priority = 0,
		.flags = {},
	};
	const mcpwm_generator_config_t generator_config =
	{
		.gen_gpio_num = timer_ptr->gpio,
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
	esp_err_t rv;

	static_assert(this->channels_size <= (SOC_MCPWM_GROUPS * SOC_MCPWM_TIMERS_PER_GROUP));

	tree_group_t *group = &tree.group[timer_ptr->group];
	tree_timer_t *timer = &group->timer[timer_ptr->timer];
	tree_operator_t *operator_ = &timer->operator_;
	tree_comparator_t *comparator = &operator_->comparator;
	tree_generator_t *generator = &operator_->generator;

	timer->handle = nullptr;
	operator_->handle = nullptr;
	comparator->handle = nullptr;
	generator->handle = nullptr;

	if(timer_ptr->gpio < 0)
		return(nullptr);

	if((rv = mcpwm_new_timer(&timer_config, &timer->handle)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "MCPWM::setup: mcpwm_new_timer")));

	if((rv = mcpwm_new_operator(&operator_config, &operator_->handle)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "MCPWM::setup: mcpwm_new_operator")));

	if((rv = mcpwm_operator_connect_timer(operator_->handle, timer->handle)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "MCPWM::setup: mcpwm_operator_connect_timer")));

	if((rv = mcpwm_new_comparator(operator_->handle, &comparator_config, &comparator->handle)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "MCPWM::setup: mcpwm_new_comparator")));

	if((rv = mcpwm_new_generator(operator_->handle, &generator_config, &generator->handle)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "MCPWM::setup: mcpwm_new_generator")));

	if((rv = mcpwm_generator_set_action_on_timer_event(generator->handle, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH))) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "MCPWM::setup: mcpwm_generator_set_action_on_timer_event")));

	if((rv = mcpwm_generator_set_action_on_compare_event(generator->handle, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator->handle, MCPWM_GEN_ACTION_LOW))) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "MCPWM::setup: mcpwm_generator_set_action_on_compare_event")));

	if((rv = mcpwm_comparator_set_compare_value(comparator->handle, 0)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "MCPWM::setup: mcpwm_comparator_set_compare_value")));

	if((rv = mcpwm_timer_enable(timer->handle)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "MCPWM::setup: mcpwm_timer_enable")));

	if((rv = mcpwm_timer_start_stop(timer->handle, MCPWM_TIMER_START_NO_STOP)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "MCPWM::setup: mcpwm_timer_start_stop")));

	return(comparator);
}

MCPWM::MCPWM(Log &log_in) : log(log_in)
{
	handle_t *handle;
	const channel_to_group_timer_t *channel_to_group_timer_ptr;

	if(this->singleton)
		throw(hard_exception("MCPWM: already active"));

	for(const auto &channel : magic_enum::enum_values<Channel>())
	{
		channel_to_group_timer_ptr = &channel_to_group_timer.at(channel);
		handle = &handles[channel];

		handle->owner = "";
		handle->group = channel_to_group_timer_ptr->group;
		handle->timer = channel_to_group_timer_ptr->timer;
		handle->frequency = channel_to_group_timer_ptr->pwm_frequency;
		handle->gpio = channel_to_group_timer_ptr->gpio;
		handle->duty = 0;
		handle->open = 0;
		handle->available = !!(handle->comparator = setup(channel_to_group_timer_ptr));
	}

	this->singleton = this;
}

MCPWM& MCPWM::get()
{
	if(!MCPWM::singleton)
		throw(hard_exception("MCPWM::get: not active"));

	return(*MCPWM::singleton);
}

void MCPWM::open(Channel channel, std::string_view owner)
{
	esp_err_t rv;
	handle_t *handle;

	if(!this->singleton)
		throw(hard_exception("MCPWM::open: not active"));

	if(!magic_enum::enum_contains<Channel>(channel))
		throw(hard_exception("MCPWM::open: channel out of range"));

	handle = &handles[channel];

	if(!handle->available)
		throw(hard_exception("MCPWM::open: channel unavailable"));

	if(handle->open)
		throw(transient_exception("MCPWM::open: channel already open"));

	if(!handle->comparator)
		throw(hard_exception("MCPWM::open: channel has no comparator"));

	handle->duty = 0;

	if((rv = mcpwm_comparator_set_compare_value(handle->comparator->handle, handle->duty)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "MCPWM::open: mcpwm_comparator_set_compare_value")));

	handle->open = 1;
	handle->owner = owner;
}

void MCPWM::set(Channel channel, int duty)
{
	esp_err_t rv;
	handle_t *handle;

	if(!this->singleton)
		throw(hard_exception("MCPWM::set: not active"));

	if(!magic_enum::enum_contains<Channel>(channel))
		throw(hard_exception("MCPWM::set: channel out of range"));

	handle = &handles[channel];

	if(!handle->open)
		throw(transient_exception("MCPWM::set: channel not open"));

	if(!handle->comparator)
		throw(hard_exception("MCPWM::set: channel has no comparator"));

	if(duty >= timer_ticks)
		throw(hard_exception(std::format("MCPWM::set: value {:d} out of range", duty)));

	if((rv = mcpwm_comparator_set_compare_value(handle->comparator->handle, handle->duty)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "MCPWM::set: mcpwm_comparator_set_compare_value")));

	handle->duty = duty;
}

int MCPWM::get(Channel channel)
{
	const handle_t *handle;

	if(!this->singleton)
		throw(hard_exception("MCPWM::get: not active"));

	if(!magic_enum::enum_contains<Channel>(channel))
		throw(hard_exception("MCPWM::get: channel out of range"));

	handle = &handles[channel];

	return(handle->duty);
}

void MCPWM::info(std::string &out)
{
	const handle_t *handle;

	out += std::format("- channels available: {:d}", channels_size);
	out += "\nchannels:";

	for(auto const &channel : magic_enum::enum_values<Channel>())
	{
		handle = &handles[channel];

		if(handle->available)
			out += std::format("\n- channel {:d}: 16 bits @ {:4d} Hz, group {:d}, timer {:d}, gpio {:2d} is {} duty: {:5d}, owned by {}",
					magic_enum::enum_integer(channel),
					handle->frequency,
					handle->group,
					handle->timer,
					handle->gpio,
					handle->open ? "open" : "not open",
					handle->duty,
					handle->owner);
		else
			out += std::format("\n- channel {:d} is unavailable", magic_enum::enum_integer(channel));
	}
}
