#include "display.h"
#include "display-module.h"
#include "display-spi.h"
#include "display-ra8875.h"
#include "exception.h"

#include "log.h"

#include <format>
#include <thread>
#include <chrono>

static constexpr const unsigned char rs_data =	0b00000000;
static constexpr const unsigned char rs_cmd =	0b10000000;
static constexpr const unsigned char rs_write =	0b00000000;
static constexpr const unsigned char rs_read =	0b01000000;

static constexpr const unsigned int spi_speed_initial =  1'000'000;
static constexpr const unsigned int spi_speed_read =    10'000'000;
static constexpr const unsigned int spi_speed_normal =  40'000'000;

enum
{
	reg_pwrr =		0x01,
	reg_mrwc =		0x02,
	reg_pcsr =		0x04,
	reg_sysr =		0x10,
	reg_hdwr =		0x14,
	reg_hndftr =	0x15,
	reg_hndr =		0x16,
	reg_hstr =		0x17,
	reg_hpwr =		0x18,
	reg_vdhr0 =		0x19,
	reg_vdhr1 =		0x1a,
	reg_vndr0 =		0x1b,
	reg_vndr1 =		0x1c,
	reg_vstr0 =		0x1d,
	reg_vstr1 =		0x1e,
	reg_vpwr =		0x1f,
	reg_dpcr =		0x20,
	reg_hsaw0 =		0x30,
	reg_hsaw1 =		0x31,
	reg_vsaw0 =		0x32,
	reg_vsaw1 =		0x33,
	reg_heaw0 =		0x34,
	reg_heaw1 =		0x35,
	reg_veaw0 =		0x36,
	reg_veaw1 =		0x37,
	reg_mwcr0 =		0x40,
	reg_mwcr1 =		0x41,
	reg_curh0 =		0x46,
	reg_curh1 =		0x47,
	reg_curv0 =		0x48,
	reg_curv1 =		0x49,
	reg_ltpr0 =		0x52,
	reg_ltpr1 =		0x53,
	reg_bgcr0 =		0x60,
	reg_bgcr1 =		0x61,
	reg_bgcr2 =		0x62,
	reg_fgcr0 =		0x63,
	reg_fgcr1 =		0x64,
	reg_fgcr2 =		0x65,
	reg_pll_c1 =	0x88,
	reg_pll_c2 =	0x89,
	reg_p1cr =		0x8a,
	reg_p1dcr =		0x8b,
	reg_mclr =		0x8e,

	reg_pllc1_plldivm_div_1 =				0b000000,
	reg_pllc1_plldivn_bitpos =				0x00,
	reg_pllc1_plldivn_mask =				0x1f,
	reg_pllc1_plldivn =						19,
	reg_pllc1_value =						reg_pllc1_plldivm_div_1 | ((reg_pllc1_plldivn & reg_pllc1_plldivn_mask) << reg_pllc1_plldivn_bitpos),

	reg_pllc2_plldivk_div_bitpos =			0x00,
	reg_pllc2_plldivk_div_mask =			0x07,
	reg_pllc2_plldivk =						0x02,
	reg_pllc2_value =						(reg_pllc2_plldivk & reg_pllc2_plldivk_div_mask) << reg_pllc2_plldivk_div_bitpos,

	reg_sysr_color_depth_16 =				0b00001000,
	reg_sysr_if_8bit =						0b00000000,

	reg_pcsr_sample_falling_edge =			0b10000000,
	reg_pcsr_clock_period_system =			0b00000000,
	reg_pcsr_clock_period_system_by_2 =		0b00000001,
	reg_pcsr_clock_period_system_by_4 =		0b00000010,
	reg_pcsr_clock_period_system_by_8 =		0b00000011,

	reg_hndftr_de_polarity_active_high =	0b00000000,
	reg_hpwr_hsync_polarity_active_low =	0b00000000,
	reg_vpwr_vsync_polarity_active_low =	0b00000000,

	reg_p1cr_pwm1_enable =					0b10000000,
	reg_p1cr_pwm1_disable =					0b00000000,
	reg_p1cr_function_pwm1 =				0b00000000,
	reg_p1cr_clock_ratio_1 =				0b00000000,
	reg_p1cr_clock_ratio_2 =				0b00000001,
	reg_p1cr_clock_ratio_4 =				0b00000010,
	reg_p1cr_clock_ratio_8 =				0b00000011,
	reg_p1cr_clock_ratio_16 =				0b00000100,
	reg_p1cr_clock_ratio_32 =				0b00000101,
	reg_p1cr_clock_ratio_64 =				0b00000110,
	reg_p1cr_clock_ratio_128 =				0b00000111,
	reg_p1cr_clock_ratio_256 =				0b00001000,
	reg_p1cr_clock_ratio_512 =				0b00001001,
	reg_p1cr_clock_ratio_1024 =				0b00001010,
	reg_p1cr_clock_ratio_2048 =				0b00001011,
	reg_p1cr_clock_ratio_4096 =				0b00001100,
	reg_p1cr_clock_ratio_8192 =				0b00001101,
	reg_p1cr_clock_ratio_16384 =			0b00001110,
	reg_p1cr_clock_ratio_32768 =			0b00001111,

	reg_pwrr_display_enable =				0b10000000,
	reg_pwrr_display_disable =				0b00000000,
	reg_pwrr_display_sleep_mode_enable =	0b00000010,
	reg_pwrr_display_sleep_mode_disable =	0b00000000,
	reg_pwrr_display_reset_start =			0b00000001,
	reg_pwrr_display_reset_complete =		0b00000000,

	reg_mwcr1_graphic_cursor_disable =		0b00000000,
	reg_mwcr1_graphic_cursor_enable =		0b10000000,
	reg_mwcr1_graphic_cursor_select_0 =		0b00000000,
	reg_mwcr1_graphic_cursor_select_1 =		0b00010000,
	reg_mwcr1_graphic_cursor_select_2 =		0b00100000,
	reg_mwcr1_graphic_cursor_select_3 =		0b00110000,
	reg_mwcr1_graphic_cursor_select_4 =		0b01000000,
	reg_mwcr1_graphic_cursor_select_5 =		0b01010000,
	reg_mwcr1_graphic_cursor_select_6 =		0b01100000,
	reg_mwcr1_graphic_cursor_select_7 =		0b01110000,
	reg_mwcr1_write_destination_layer =		0b00000000,
	reg_mwcr1_write_destination_cgram =		0b00000100,
	reg_mwcr1_write_destination_cursor =	0b00001000,
	reg_mwcr1_write_destination_pattern =	0b00001100,
	reg_mwcr1_write_destination_layer_0 =	0b00000000,
	reg_mwcr1_write_destination_layer_1 =	0b00000001,

	reg_mclr_memory_clear_start =			0b10000000,
	reg_mclr_memory_clear_complete =		0b00000000,
	reg_mclr_memory_area_active_window =	0b01000000,
	reg_mclr_memory_area_full_window =		0b00000000,

	reg_mwcr0_mode_text =					0b10000000,
	reg_mwcr0_mode_graphic =				0b00000000,
	reg_mwcr0_cursor_visible =				0b01000000,
	reg_mwcr0_cursor_invisible =			0b00000000,
	reg_mwcr0_cursor_blink =				0b00100000,
	reg_mwcr0_cursor_steady =				0b00000000,
	reg_mwcr0_memory_write_direction_lrtd =	0b00000000,
	reg_mwcr0_memory_write_direction_rltd =	0b00000100,
	reg_mwcr0_memory_write_direction_tdlr =	0b00001000,
	reg_mwcr0_memory_write_direction_dtlr =	0b00001100,
	reg_mwcr0_memory_write_autoincr_en =	0b00000000,
	reg_mwcr0_memory_write_autoincr_dis =	0b00000010,
	reg_mwcr0_memory_read_autoincr_en =		0b00000000,
	reg_mwcr0_memory_read_autoincr_dis =	0b00000001,

	reg_ltpr0_scroll_both =					0b00000000,
	reg_ltpr0_scroll_layer_1 =				0b01000000,
	reg_ltpr0_scroll_layer_2 =				0b10000000,
	reg_ltpr0_scroll_buffer =				0b11000000,
	reg_ltpr0_floatwin_transparency_dis =	0b00000000,
	reg_ltpr0_floatwin_transparency_en =	0b00100000,
	reg_ltpr0_visible_layer_1 =				0b00000000,
	reg_ltpr0_visible_layer_2 =				0b00000001,
	reg_ltpr0_visible_layer_lighten =		0b00000010,
	reg_ltpr0_visible_layer_transparent =	0b00000011,
	reg_ltpr0_visible_layer_or =			0b00000100,
	reg_ltpr0_visible_layer_and =			0b00000101,
	reg_ltpr0_visible_layer_floatwin =		0b00000110,
	reg_ltpr0_visible_layer_unused =		0b00000111,

	reg_ltpr1_transparency_layer_2_8_8 =	0b00000000,
	reg_ltpr1_transparency_layer_2_7_8 =	0b00010000,
	reg_ltpr1_transparency_layer_2_6_8 =	0b00100000,
	reg_ltpr1_transparency_layer_2_5_8 =	0b00110000,
	reg_ltpr1_transparency_layer_2_4_8 =	0b01000000,
	reg_ltpr1_transparency_layer_2_3_8 =	0b01010000,
	reg_ltpr1_transparency_layer_2_2_8 =	0b01100000,
	reg_ltpr1_transparency_layer_2_1_8 =	0b01110000,
	reg_ltpr1_transparency_layer_2_0_8 =	0b10000000,
	reg_ltpr1_transparency_layer_1_8_8 =	0b00000000,
	reg_ltpr1_transparency_layer_1_7_8 =	0b00000001,
	reg_ltpr1_transparency_layer_1_6_8 =	0b00000010,
	reg_ltpr1_transparency_layer_1_5_8 =	0b00000011,
	reg_ltpr1_transparency_layer_1_4_8 =	0b00000100,
	reg_ltpr1_transparency_layer_1_3_8 =	0b00000101,
	reg_ltpr1_transparency_layer_1_2_8 =	0b00000110,
	reg_ltpr1_transparency_layer_1_1_8 =	0b00000111,
	reg_ltpr1_transparency_layer_1_0_8 =	0b00001000,

	reg_dpcr_one_layer =					0b00000000,
	reg_dpcr_two_layer =					0b10000000,
	reg_dpcr_hor_scan_ltor =				0b00000000,
	reg_dpcr_hor_scan_rtol =				0b00001000,
	reg_dpcr_vert_scan_ltor =				0b00000000,
	reg_dpcr_vert_scan_rtol =				0b00000100,

	horizontal_blanking = 38,
	horizontal_blanking_fine = 4,
	horizontal_sync_start = 16,
	horizontal_sync_length = 32,
	vertical_blanking = 14,
	vertical_sync_start = 6,
	vertical_sync_length = 2,
};

DisplayModuleRA8875::DisplayModuleRA8875(Config& config_in, Log& log_in, Util& util_in, SPI& spi_in, LedPWM& ledpwm_in,
			int module_index_in, int x_size_in, int y_size_in, bool flip_in, bool invert_in, bool rotate_in, bool blinvert_in)
		:
			DisplayModuleSPI(config_in, log_in, util_in, spi_in, ledpwm_in, module_index_in, x_size_in, y_size_in, flip_in, invert_in, rotate_in, blinvert_in), active_layer(0)
{
	int rv;

	this->speed(spi_speed_initial);

	this->write_register(reg_pll_c1, reg_pllc1_value);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	this->write_register(reg_pll_c2, reg_pllc2_value);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	this->write_register(reg_pcsr, reg_pcsr_sample_falling_edge | reg_pcsr_clock_period_system_by_8);

	// status register

	if((rv = this->read_status()) != 0)
		this->log << std::format("ra8875: unexpected status: {:d}/{:x}/{:b}", rv, rv, rv);

	this->speed(spi_speed_normal);

	// interface

	this->write_register(reg_sysr, reg_sysr_color_depth_16 | reg_sysr_if_8bit);

	// horizontal

	this->write_register(reg_hdwr, (this->x_size / 8) - 1);
	this->write_register(reg_hndftr, reg_hndftr_de_polarity_active_high | (horizontal_blanking_fine / 2));
	this->write_register(reg_hndr, (horizontal_blanking / 8) - 1);
	this->write_register(reg_hstr, (horizontal_sync_start / 8) - 1);
	this->write_register(reg_hpwr, reg_hpwr_hsync_polarity_active_low | ((horizontal_sync_length / 8) - 1));

	// vertical

	this->write_register(reg_vdhr0, ((this->y_size >> 0) & 0xff) + 1);
	this->write_register(reg_vdhr1, ((this->y_size >> 8) & 0x01) + 0);
	this->write_register(reg_vndr0, ((vertical_blanking >> 0) & 0xff) + 1);
	this->write_register(reg_vndr1, ((vertical_blanking >> 8) & 0x01) + 0);
	this->write_register(reg_vstr0, ((vertical_sync_start >> 0) & 0xff) + 1);
	this->write_register(reg_vstr1, ((vertical_sync_start >> 8) & 0x01) + 0);
	this->write_register(reg_vpwr, reg_vpwr_vsync_polarity_active_low | (vertical_sync_length - 1));

	// PWM

	this->write_register(reg_p1cr, reg_p1cr_pwm1_enable | reg_p1cr_function_pwm1 | reg_p1cr_clock_ratio_2048); // 114 Hz refresh
	this->write_register(reg_pwrr, reg_pwrr_display_enable | reg_pwrr_display_sleep_mode_disable | reg_pwrr_display_reset_complete);

	// MISC

	this->write_register(reg_dpcr, reg_dpcr_two_layer | reg_dpcr_hor_scan_ltor | reg_dpcr_vert_scan_ltor);
	this->write_register(reg_mwcr0, reg_mwcr0_mode_graphic | reg_mwcr0_cursor_invisible | reg_mwcr0_cursor_steady |
			reg_mwcr0_memory_write_direction_lrtd | reg_mwcr0_memory_write_autoincr_en | reg_mwcr0_memory_read_autoincr_en);

	this->_brightness(100);

	this->show_layer(1);
	this->set_active_layer(1);
	this->_clear(0x00, 0x00, 0x00);

	this->show_layer(0);
	this->set_active_layer(0);
	this->_clear(0x00, 0x00, 0x00);

	this->set_fgcolour(0xff, 0xff, 0xff);
	this->set_bgcolour(0x00, 0x00, 0x00);
}

void DisplayModuleRA8875::write_register(unsigned char reg, unsigned char reg_data)
{
	uint64_t data = ((rs_write | rs_cmd) << 24) | ((reg & 0xff) << 16) | ((rs_write | rs_data) << 8) | (reg_data & 0xff);

	this->transfer({
			.send =
			{
				.command = {},
				.address =
				{
					.bits = 32,
					.data = data,
				},
			},
			.receive = {},
	});
}

int DisplayModuleRA8875::read_status()
{
	uint16_t data = rs_read | rs_cmd;
	SPI::data_t in;

	this->transfer({
			.send =
			{
				.command =
				{
					.bits = 8,
					.data = data,
				},
				.address = {},
			},
			.receive =
			{
				.length = 1,
				.data = &in,
			},
	});

	return(in[0]);
}

void DisplayModuleRA8875::_set_active_layer(int layer)
{
	if((layer != 0) && (layer != 1))
		throw(hard_exception("DisplayModuleRA8875::activate_layer: invalid argument"));

	this->write_register(reg_mwcr1, reg_mwcr1_graphic_cursor_disable | reg_mwcr1_write_destination_layer | (layer ? 0x01 : 0x00));
}

void DisplayModuleRA8875::_show_layer(int layer)
{
	unsigned int value = reg_ltpr0_scroll_layer_1 | reg_ltpr0_floatwin_transparency_dis;

	if((layer != 0) && (layer != 1))
		throw(hard_exception("DisplayModuleRA8875::show_layer: invalid argument"));

	value |= (layer == 0) ? reg_ltpr0_visible_layer_1 : reg_ltpr0_visible_layer_2;

	this->write_register(reg_ltpr1, reg_ltpr1_transparency_layer_2_8_8 | reg_ltpr1_transparency_layer_1_8_8);
	this->write_register(reg_ltpr0, value);
}

void DisplayModuleRA8875::set_fgcolour(unsigned int r, unsigned int g, unsigned int b)
{
	write_register(reg_fgcr0, (r & 0b11111000) >> 3);
	write_register(reg_fgcr1, (g & 0b11111100) >> 2);
	write_register(reg_fgcr2, (b & 0b11111000) >> 3);
}

void DisplayModuleRA8875::set_bgcolour(unsigned int r, unsigned int g, unsigned int b)
{
	write_register(reg_bgcr0, (r & 0b11111000) >> 3);
	write_register(reg_bgcr1, (g & 0b11111100) >> 2);
	write_register(reg_bgcr2, (b & 0b11111000) >> 3);
}

std::string DisplayModuleRA8875::_name()
{
	return("RAiO RA8875");
}

void DisplayModuleRA8875::_brightness(int brightness)
{
	static const constexpr unsigned int p1cr = reg_p1cr_function_pwm1 | reg_p1cr_clock_ratio_2048;
	static const constexpr unsigned int pwrr = reg_pwrr_display_sleep_mode_disable | reg_pwrr_display_reset_complete;

	if((brightness < 0) || (brightness > 100))
		throw(transient_exception("DisplayModuleRA8875: brightness out of range"));

	if(brightness == 0)
	{
		this->write_register(reg_p1dcr, 0);
		this->write_register(reg_p1cr, p1cr | reg_p1cr_pwm1_disable);
		this->write_register(reg_pwrr, pwrr | reg_pwrr_display_disable);
	}
	else
	{
		if(brightness == 100)
			brightness = 255;
		else
			brightness = brightness * 2550 / 1000;

		this->write_register(reg_p1dcr, brightness);
		this->write_register(reg_p1cr, p1cr | reg_p1cr_pwm1_enable);
		this->write_register(reg_pwrr, pwrr | reg_pwrr_display_enable);
	}
}

void DisplayModuleRA8875::_clear(int r, int g, int b)
{
	this->_box({ .r = r, .g = g, .b = b, .geometry = { .from_x = 0, .from_y = 0, .to_x = this->x_size - 1, .to_y = this->y_size - 1}});
}

void DisplayModuleRA8875::_box(const Display::box_rgb_args_t& args)
{
	int status, saved_speed, cycles;

	this->set_window(args.geometry);
	this->set_bgcolour(args.r, args.g, args.b);
	this->write_register(reg_mclr, reg_mclr_memory_clear_start | reg_mclr_memory_area_active_window);
	saved_speed = this->speed();
	this->speed(spi_speed_read);

	for(cycles = 0; cycles < 128; cycles++)
	{
		status = this->read_status();

		if((status & (1 << 7)) == 0)
			break;
	}

	this->speed(saved_speed);
}

void DisplayModuleRA8875::_set_window(const Display::geometry_t& geo)
{
	this->write_register(reg_hsaw0, (geo.from_x >> 0) & 0xff);
	this->write_register(reg_hsaw1, (geo.from_x >> 8) & 0x03);
	this->write_register(reg_vsaw0, (geo.from_y >> 0) & 0xff);
	this->write_register(reg_vsaw1, (geo.from_y >> 8) & 0x01);
	this->write_register(reg_heaw0, (geo.to_x >> 0) & 0xff);
	this->write_register(reg_heaw1, (geo.to_x >> 8) & 0x03);
	this->write_register(reg_veaw0, (geo.to_y >> 0) & 0xff);
	this->write_register(reg_veaw1, (geo.to_y >> 8) & 0x01);

	this->write_register(reg_curh1, (geo.from_x & 0xff00) >> 8);
	this->write_register(reg_curh0, (geo.from_x & 0x00ff) >> 0);
	this->write_register(reg_curv1, (geo.from_y & 0xff00) >> 8);
	this->write_register(reg_curv0, (geo.from_y & 0x00ff) >> 0);
}

void DisplayModuleRA8875::_plot(int length, const Display::rgb_t* pixels)
{
	int current;
	unsigned int rgb16, r5, g6, b5;
	unsigned char b1, b2;

	this->set_leader(SPI::data_t{rs_write | rs_cmd, reg_mrwc, rs_write | rs_data});

	for(current = 0; current < length; current++)
	{
		r5 = (pixels[current].r & 0b11111000) >> 3;
		g6 = (pixels[current].g & 0b11111100) >> 2;
		b5 = (pixels[current].b & 0b11111000) >> 3;

		rgb16 = ((r5 << 11) | (g6 << 5) | (b5 << 0));

		b1 = ((rgb16 & 0xff00) >> 8) & 0xff;
		b2 = ((rgb16 & 0x00ff) >> 0) & 0xff;

		this->push(SPI::data_t{b1, b2});
	}

	this->flush();
	this->clear_leader();
}
