#pragma once

#include <led_strip.h>

#include <string>
#include <array>

class Ledpixel final
{
	public:

		static constexpr unsigned int leds_size = 4;
		static constexpr unsigned int channels_size = 4;

		enum class Channel : unsigned int
		{
			channel_0_notify = 0,
			channel_1,
			channel_2,
			channel_3,
			first = channel_0_notify,
			last = channel_3,
		};

		static_assert(static_cast<unsigned int>(Channel::last) == (channels_size - 1));

		Ledpixel();
		Ledpixel(const Ledpixel &) = delete;

		void open(Channel channel, std::string_view owner);
		void set(Channel channel, int led, int red, int green, int blue);
		void get(Channel channel, int led, int &red, int &green, int &blue);
		void flush(Channel channel);
		void info(std::string &dst);

		static Ledpixel& get();

	private:

		struct channel_to_gpio_t
		{
			Channel channel;
			int gpio;
			struct
			{
				int r;
				int g;
				int b;
			} pos;
		};

		struct rgb_t
		{
			int r;
			int g;
			int b;
		};

		struct handle_t
		{
			led_strip_handle_t handle;
			int gpio;
			std::string owner;
			struct
			{
				unsigned int available:1;
				unsigned int open:1;
			};
			rgb_t rgbvalue[leds_size];
		};

		static Ledpixel *singleton;

		static const channel_to_gpio_t channel_to_gpio[channels_size];
		std::array<handle_t, channels_size> handles;
};
