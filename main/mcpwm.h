#pragma once

#include "log.h"

#include <string>
#include <map>

#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_types.h"
#include "soc/soc.h"

class MCPWM final
{
	public:

		enum class Channel
		{
			channel_16bit_150hz_0 = 0,
			channel_16bit_150hz_1,
			channel_16bit_2400hz_0,
			channel_16bit_2400hz_1,
		};

		MCPWM() = delete;
		MCPWM(const MCPWM &) = delete;
		MCPWM(Log &);

		static MCPWM &get();

		void open(Channel channel, std::string_view owner);
		void set(Channel channel, int duty);
		int get(Channel channel);
		void info(std::string &);

	private:

		static constexpr int channels_size = 4;
		static constexpr int timers = 2;
		static constexpr int groups = 2;
		static constexpr int base_clock = 160000000;
		static constexpr int timer_frequency_150hz = base_clock / 16;
		static constexpr int timer_frequency_2400hz = base_clock;
		static constexpr int timer_resolution = 16;
		static constexpr int timer_ticks = (1U << timer_resolution) - 1;

		struct tree_comparator_t
		{
			mcpwm_cmpr_handle_t handle;
		};

		struct tree_generator_t
		{
			mcpwm_gen_handle_t handle;
		};

		struct tree_operator_t
		{
			tree_comparator_t comparator;
			tree_generator_t generator;
			mcpwm_oper_handle_t handle;
		};

		struct tree_timer_t
		{
			mcpwm_timer_handle_t handle;
			tree_operator_t operator_;
		};

		struct tree_group_t
		{
			tree_timer_t timer[timers];
		};

		struct mcpwm_tree_t
		{
			tree_group_t group[groups];
		};

		struct __attribute__((aligned(sizeof(uint32_t)))) handle_t
		{
			int gpio;
			unsigned int group;
			unsigned int timer;
			unsigned int frequency;
			std::string owner;
			unsigned int duty;
			const tree_comparator_t *comparator;
			struct
			{
				unsigned int available:1;
				unsigned int open:1;
			};
		};

		struct channel_to_group_timer_t
		{
			int gpio;
			unsigned int group;
			unsigned int timer;
			unsigned int timer_frequency;
			unsigned int pwm_frequency;
		};

		static const std::map<Channel, channel_to_group_timer_t> channel_to_group_timer;

		static MCPWM *singleton;
		Log &log;

		std::map<Channel, handle_t> handles;
		mcpwm_tree_t tree;

		const tree_comparator_t *setup(const channel_to_group_timer_t *);
};
