#pragma once

#include <led_strip.h>

#include <string>
#include <array>

class Ledpixel final
{
	public:

		enum
		{
			ledpixel_leds_size = 4
		};

		typedef enum
		{
			lp_0_notify = 0,
			lp_first = lp_0_notify,
			lp_1,
			lp_2,
			lp_3,
			lp_size,
			lp_error = lp_size,
		} lp_t;

		Ledpixel();
		Ledpixel(const Ledpixel &) = delete;

		void open(lp_t channel, std::string_view owner);
		void set(lp_t channel, int instance, int red, int green, int blue);
		void get(lp_t channel, int instance, int &red, int &green, int &blue);
		void flush(lp_t channel);
		void info(std::string &dst);

		static Ledpixel& get();

	private:

		typedef struct
		{
			lp_t handle;
			int gpio;
			struct
			{
				int r;
				int g;
				int b;
			} pos;
		} handle_to_gpio_t;

		typedef struct
		{
			int r;
			int g;
			int b;
		} rgb_t;

		typedef struct
		{
			led_strip_handle_t handle;
			int gpio;
			std::string owner;
			struct
			{
				unsigned int available:1;
				unsigned int open:1;
			};
			rgb_t rgbvalue[ledpixel_leds_size];
		} channel_t;

		static Ledpixel *singleton;

		static const handle_to_gpio_t handle_to_gpio[lp_size];
		std::array<channel_t, lp_size> channels;
};
