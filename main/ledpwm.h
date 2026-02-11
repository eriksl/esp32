#pragma once

#include <string>

#include "magic_enum/magic_enum.hpp"
#include "magic_enum/magic_enum_containers.hpp"

class LedPWM final
{
	public:

		static constexpr unsigned int channels_size = 4;

		enum class Channel : unsigned int
		{
			channel_14bit_5khz_notify = 0,
			channel_14bit_5khz_lcd_spi_2,
			channel_14bit_5khz_lcd_spi_3,
			channel_14bit_120hz,
		};

		explicit LedPWM();
		explicit LedPWM(const LedPWM &) = delete;

		static LedPWM& get();

		void open(Channel channel, std::string_view owner);
		void set(Channel channel, int duty);
		int get(Channel channel);
		void info(std::string &dst);

	private:

		static constexpr unsigned int resolution = 14;
		static constexpr unsigned int max_duty = (1U << resolution) - 1;

		enum class timer_t : unsigned int
		{
			timer_5khz,
			timer_120hz,
		};

		struct handle_t
		{
			int gpio;
			timer_t timer;
			Channel channel;
			unsigned int frequency;
			std::string owner;
			struct
			{
				unsigned int available;
				unsigned int open;
			};
		};

		struct channel_to_gpio_t
		{
			Channel channel;
			int gpio;
			timer_t timer;
			unsigned int frequency;
		};

		static LedPWM *singleton;

		static const LedPWM::channel_to_gpio_t channel_to_gpio[channels_size];
		magic_enum::containers::array<Channel, handle_t> handles;
};
