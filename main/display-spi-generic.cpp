#include "ledpwm.h"

#include "display.h"
#include "display-module.h"
#include "display-spi.h"
#include "display-spi-generic.h"

#include "exception.h"

#include <driver/gpio.h>

#include <string>
#include <format>
#include <thread>
#include <chrono>

enum
{
	cmd_swreset =	0x01,
	cmd_sleepout = 	0x11,
	cmd_noron =		0x13,
	cmd_invoff =	0x20,
	cmd_invon =		0x21,
	cmd_dispoff =	0x28,
	cmd_dispon =	0x29,
	cmd_caset =		0x2a,
	cmd_raset =		0x2b,
	cmd_ramwr =		0x2c,
	cmd_madctl =	0x36,
	cmd_colmod =	0x3a,

	madctl_my =		(1 << 7),
	madctl_mx = 	(1 << 6),
	madctl_mv =		(1 << 5),
	madctl_ml =		(1 << 4),
	madctl_bgr =	(1 << 3),
	madctl_mh =		(1 << 2),

	colmod_18bpp = 	0b01100110,

	spi_normal_speed = 40'000'000,
};

DisplayModuleGenericSPI::DisplayModuleGenericSPI(Config& config_in, Log& log_in, Util& util_in, SPI& spi_in, LedPWM& ledpwm_in,
			int module_index_in, int x_size_in, int y_size_in, bool flip_in, bool invert_in, bool rotate_in)
		:
			DisplayModuleSPI(config_in, log_in, util_in, spi_in, ledpwm_in, module_index_in, x_size_in, y_size_in, flip_in, invert_in, rotate_in)
{
	esp_err_t rv;
	int madctl;

	gpio_config_t gpio_pin_config =
	{
		.pin_bit_mask = 0,
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	switch(module_index_in)
	{
		case(0):
		{
			gpio_pin_config.pin_bit_mask = (1ULL << CONFIG_BSP_SPI2_DISPLAY_DC);
			this->dc_gpio = CONFIG_BSP_SPI2_DISPLAY_DC;
			this->ledpwm_channel = LedPWM::Channel::channel_14bit_5khz_lcd_spi_2;
			break;
		}

		case(1):
		{
			gpio_pin_config.pin_bit_mask = (1ULL << CONFIG_BSP_SPI3_DISPLAY_DC);
			this->dc_gpio = CONFIG_BSP_SPI3_DISPLAY_DC;
			this->ledpwm_channel = LedPWM::Channel::channel_14bit_5khz_lcd_spi_3;
			break;
		}

		default:
		{
			throw(hard_exception("DisplayModuleGenericSPI: invalid dc or bl arguments"));
			break;
		}
	}

	this->speed(spi_normal_speed);

	if((rv = gpio_config(&gpio_pin_config)) != ESP_OK)
		throw(transient_exception(this->log.esp_string_error(rv, "DisplayModuleSPIGeneric: gpio_config")));

	LedPWM::get().open(ledpwm_channel, "backlight generic SPI LCD");

	this->send_command(cmd_dispoff);
	this->send_command(cmd_swreset);

	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	this->send_command(cmd_sleepout);

	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	this->send_command(cmd_noron);
	this->send_command(cmd_dispon);

	this->send_command_data_1b(cmd_colmod, colmod_18bpp);
	this->send_command(this->invert ? cmd_invon : cmd_invoff);

	madctl = madctl_bgr;

	if(this->rotate)
		madctl |= madctl_mv;

	if(!this->flip)
		madctl |= madctl_my | madctl_mx;

	send_command_data_1b(cmd_madctl, madctl);
}

void DisplayModuleGenericSPI::send_command(unsigned char cmd)
{
	gpio_set_level(static_cast<gpio_num_t>(this->dc_gpio), 0);

	this->transfer({
			.send =
			{
				.command =
				{
					.bits = 8,
					.data = cmd,
				},
				.address = {}
			},
			.receive = {},
	});

	gpio_set_level(static_cast<gpio_num_t>(this->dc_gpio), 1);
}

void DisplayModuleGenericSPI::send_command_data_1b(unsigned char cmd, unsigned char data)
{
	this->send_command(cmd);

	this->transfer({
			.send =
			{
				.command =
				{
					.bits = 8,
					.data = data,
				},
				.address = {}
			},
			.receive = {},
	});
}

void DisplayModuleGenericSPI::send_command_data_2w(unsigned char cmd, unsigned int word1, unsigned int word2)
{
	unsigned int data;

	data = ((word1 & 0x0000ffff) << 16) | ((word2 & 0x0000ffff) << 0);

	this->send_command(cmd);

	this->transfer({
			.send =
			{
				.command = {},
				.address =
				{
					.bits = 32,
					.data = data,
				}
			},
			.receive = {},
	});
}

void DisplayModuleGenericSPI::set_window(int from_x, int from_y, int to_x, int to_y)
{
	this->send_command_data_2w(cmd_caset, from_x, to_x);
	this->send_command_data_2w(cmd_raset, from_y, to_y);
}

void DisplayModuleGenericSPI::box(int r, int g, int b, int from_x, int from_y, int to_x, int to_y)
{
	int pixel, pixels;
	SPI::data_t rgb{static_cast<unsigned char>(r), static_cast<unsigned char>(g), static_cast<unsigned char>(b)};

	this->set_window(from_x, from_y, to_x, to_y);
	this->send_command(cmd_ramwr);

	pixels = (to_x - from_x + 1) * (to_y - from_y + 1);

	if(pixels <= 0)
		return;

	for(pixel = 0; pixel < pixels; pixel++)
		this->push(rgb);

	this->flush();
}

std::string DisplayModuleGenericSPI::_name()
{
	return(std::format("Generic SPI, DC at GPIO {:d}, backlight at channel {:d}", this->dc_gpio, static_cast<int>(this->ledpwm_channel)));
}

void DisplayModuleGenericSPI::_brightness(int brightness)
{
	if(brightness == 100)
		this->ledpwm.set(this->ledpwm_channel, 1UL << 14);
	else
		this->ledpwm.set(this->ledpwm_channel, ((1UL << 14) * brightness) / 100);
}

void DisplayModuleGenericSPI::_clear(int r, int g, int b)
{
	this->box(r, g, b, 0, 0, this->display_x_size() - 1, this->display_y_size() - 1);
}

void DisplayModuleGenericSPI::_box(const Display::box_rgb_args_t& args)
{
	this->box(args.r, args.g, args.b, args.geometry.from_x, args.geometry.from_y, args.geometry.to_x, args.geometry.to_y);
}

void DisplayModuleGenericSPI::_set_window(const Display::geometry_t& geometry)
{
	this->set_window(geometry.from_x, geometry.from_y, geometry.to_x, geometry.to_y);
}

void DisplayModuleGenericSPI::_plot(int length, const Display::rgb_t* pixels)
{
	int current;

	this->send_command(cmd_ramwr);

	for(current = 0; current < length; current++)
		this->push(SPI::data_t{pixels[current].r, pixels[current].g, pixels[current].b});

	this->flush();
}

void DisplayModuleGenericSPI::_set_active_layer(int)
{
}

void DisplayModuleGenericSPI::_show_layer(int)
{
}
