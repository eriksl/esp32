#pragma once

#include <string>
#include <array>

class LedPWM final
{
	public:

		typedef enum : unsigned int
		{
			lpt_14bit_5khz_notify = 0,
			lpt_first = lpt_14bit_5khz_notify,
			lpt_14bit_5khz_lcd_spi_2,
			lpt_14bit_5khz_lcd_spi_3,
			lpt_14bit_120hz,
			lpt_size,
			lpt_error = lpt_size,
		} ledpwm_t;

		LedPWM();
		LedPWM(const LedPWM &) = delete;

		static LedPWM& get();

		void open(ledpwm_t handle, std::string_view owner);
		void set(ledpwm_t handle, int duty);
		int get(ledpwm_t handle);
		void info(std::string &dst);

	private:

		enum : unsigned int
		{
			ledpwm_resolution = 14,
			ledpwm_max_duty = (1U << ledpwm_resolution) - 1,
		};

		enum : unsigned int
		{
			timer_5khz = 0,
			timer_120hz,
			timer_size,
		};

		typedef struct
		{
			int gpio;
			unsigned int timer;
			unsigned int handle;
			unsigned int frequency;
			std::string owner;
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

		static const LedPWM::handle_to_gpio_t handle_to_gpio[lpt_size];

		static LedPWM *singleton;

		std::array<channel_t, lpt_size> channels;
};
