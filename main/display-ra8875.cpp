#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <driver/spi_master.h>
#include <driver/gpio.h>

#include "log.h"
#include "util.h"
#include "display-ra8875.h"
#include "ledpwm.h"
#include "sdkconfig.h"

union rgb16_t
{
	struct __attribute__((packed))
	{
		unsigned int b:5;
		unsigned int g:6;
		unsigned int r:5;
	};
	struct __attribute__((packed))
	{
		unsigned int second:8;
		unsigned int first:8;
	};
};

static_assert(sizeof(rgb16_t) == 2);

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
	reg_becr0 =		0x50,
	reg_becr1 =		0x51,
	reg_hsbe0 =		0x54,
	reg_hsbe1 =		0x55,
	reg_vsbe0 =		0x56,
	reg_vsbe1 =		0x57,
	reg_hdbe0 =		0x58,
	reg_hdbe1 =		0x59,
	reg_vdbe0 =		0x5a,
	reg_vdbe1 =		0x5b,
	reg_bewr0 =		0x5c,
	reg_bewr1 =		0x5d,
	reg_behr0 =		0x5e,
	reg_behr1 =		0x5f,
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
	reg_pllc1_plldivm_div_2 =				0b100000,
	reg_pllc1_plldivn_bitpos =				0x00,
	reg_pllc1_plldivn_mask =				0x1f,
	reg_pllc1_plldivn =						0x13,
	reg_pllc1_value =						reg_pllc1_plldivm_div_1 | ((reg_pllc1_plldivn & reg_pllc1_plldivn_mask) << reg_pllc1_plldivn_bitpos),

	reg_pllc2_plldivk_div_bitpos =			0x00,
	reg_pllc2_plldivk_div_mask =			0x07,
	reg_pllc2_plldivk =						0x02,
	reg_pllc2_value =						(reg_pllc2_plldivk & reg_pllc2_plldivk_div_mask) << reg_pllc2_plldivk_div_bitpos,

	reg_sysr_color_depth_8 =				0b00000000,
	reg_sysr_color_depth_16 =				0b00001000,
	reg_sysr_if_8bit =						0b00000000,
	reg_sysr_if_16bit =						0b00000010,

	reg_pcsr_sample_rising_edge =			0b00000000,
	reg_pcsr_sample_falling_edge =			0b10000000,
	reg_pcsr_clock_period_system =			0b00000000,
	reg_pcsr_clock_period_system_by_2 =		0b00000001,
	reg_pcsr_clock_period_system_by_4 =		0b00000010,
	reg_pcsr_clock_period_system_by_8 =		0b00000011,

	reg_hndftr_de_polarity_active_high =	0b00000000,
	reg_hndftr_de_polarity_active_low =		0b10000000,

	reg_hpwr_hsync_polarity_active_low =	0b00000000,
	reg_hpwr_hsync_polarity_active_high =	0b10000000,

	reg_vpwr_vsync_polarity_active_low =	0b00000000,
	reg_vpwr_vsync_polarity_active_high =	0b10000000,

	reg_p1cr_pwm1_enable =					0b10000000,
	reg_p1cr_pwm1_disable =					0b00000000,
	reg_p1cr_disable_level_low =			0b00000000,
	reg_p1cr_disable_level_high =			0b01000000,
	reg_p1cr_function_pwm1 =				0b00000000,
	reg_p1cr_function_fixed =				0b00010000,
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

	reg_vsbe1_source_layer_0 =				0b00000000,
	reg_vsbe1_source_layer_1 =				0b10000000,

	reg_vdbe1_destination_layer_0 =			0b00000000,
	reg_vdbe1_destination_layer_1 =			0b10000000,

	reg_dpcr_one_layer =					0b00000000,
	reg_dpcr_two_layer =					0b10000000,
	reg_dpcr_hor_scan_ltor =				0b00000000,
	reg_dpcr_hor_scan_rtol =				0b00001000,
	reg_dpcr_vert_scan_ltor =				0b00000000,
	reg_dpcr_vert_scan_rtol =				0b00000100,

	reg_becr0_idle =						0b00000000,
	reg_becr0_busy =						0b10000000,
	reg_becr0_src_block =					0b00000000,
	reg_becr0_src_lineair =					0b01000000,
	reg_becr0_dst_block =					0b00000000,
	reg_becr0_dst_lineair =					0b00100000,

	reg_becr1_rop_code_write =				0b00000000,
	reg_becr1_rop_code_read =				0b00000001,
	reg_becr1_rop_code_move_pos =			0b00000010,
	reg_becr1_rop_code_move_neg =			0b00000011,
	reg_becr1_rop_code_write_transp =		0b00000100,
	reg_becr1_rop_code_move_transp =		0b00000101,
	reg_becr1_rop_code_fill_pattern =		0b00000110,
	reg_becr1_rop_code_fill_pattern_transp=	0b00000111,
	reg_becr1_rop_code_expand_colour =		0b00001000,
	reg_becr1_rop_code_expand_colour_transp=0b00001001,
	reg_becr1_rop_code_move_expand_colour=	0b00001010,
	reg_becr1_rop_code_move_expand_transp=	0b00001011,
	reg_becr1_rop_code_fill =				0b00001100,

	reg_becr1_rop_expand_col_bit_start_0 =	0b01110000,

	reg_becr1_rop_func_black =				0b00000000,
	reg_becr1_rop_func_ns_and_nd =			0b00010000,
	reg_becr1_rop_func_ns_and_d =			0b00100000,
	reg_becr1_rop_func_ns =					0b00110000,
	reg_becr1_rop_func_s_and_nd =			0b01000000,
	reg_becr1_rop_func_nd =					0b01010000,
	reg_becr1_rop_func_s_xor_d =			0b01100000,
	reg_becr1_rop_func_ns_or_nd =			0b01110000,
	reg_becr1_rop_func_s_and_d =			0b10000000,
	reg_becr1_rop_func_n_s_xor_d =			0b10010000,
	reg_becr1_rop_func_d =					0b10100000,
	reg_becr1_rop_func_ns_or_d =			0b10110000,
	reg_becr1_rop_func_s =					0b11000000,
	reg_becr1_rop_func_s_or_nd =			0b11010000,
	reg_becr1_rop_func_s_or_d =				0b11100000,

	horizontal_blanking = 38,
	horizontal_blanking_fine = 4,
	horizontal_sync_start = 16,
	horizontal_sync_length = 32,
	vertical_blanking = 14,
	vertical_sync_start = 6,
	vertical_sync_length = 2,

	spi_speed_initial = 1000000,
	spi_speed_normal = 40000000,
};

enum : unsigned int
{
	rs_data =	0b00000000,
	rs_cmd =	0b10000000,
	rs_write =	0b00000000,
	rs_read =	0b01000000,
};

typedef struct
{
	spi_host_device_t esp_host;
	unsigned int cs;
	unsigned int sck;
	unsigned int mosi;
	unsigned int miso;
	LedPWM::Channel bl;
} spi_signal_t;

typedef struct
{
	spi_signal_t spi2;
	spi_signal_t spi3;
} spi_host_signal_t;

static const spi_host_signal_t spi_host_signal =
{
	.spi2 =
	{
		.esp_host =	SPI2_HOST,
		.cs =		10,									/* IOMUX, fixed */
		.sck =		12,									/* IOMUX, fixed */
		.mosi =		11,									/* IOMUX, fixed */
		.miso =		13,									/* IOMUX, fixed */
		.bl =		LedPWM::Channel::channel_14bit_5khz_lcd_spi_2,
	},
	.spi3 =
	{
		.esp_host =	SPI3_HOST,
		.cs =		CONFIG_BSP_SPI3_DISPLAY_CS,
		.sck =		CONFIG_BSP_SPI3_SCK,
		.mosi =		CONFIG_BSP_SPI3_MOSI,
		.miso =		CONFIG_BSP_SPI3_MISO,
		.bl =		LedPWM::Channel::channel_14bit_5khz_lcd_spi_3,
	},
};

static bool inited = false;
static spi_device_handle_t spi_device_handle;
static const spi_signal_t *spi_signal;
static SemaphoreHandle_t spi_mutex;
static unsigned int x_size, y_size;

static uint8_t *pixel_buffer = nullptr;
static unsigned int pixel_buffer_size = 0;
static unsigned int pixel_buffer_length = 0;

static inline void spi_mutex_take(void)
{
	assert(xSemaphoreTake(spi_mutex, portMAX_DELAY));
}

static inline void spi_mutex_give(void)
{
	assert(xSemaphoreGive(spi_mutex));
}

static void rgb24_to_rgb16(const display_rgb_t &rgb24, unsigned int &r16, unsigned int &g16, unsigned int &b16)
{
	r16 = (rgb24.r & 0b11111000) >> 3;
	g16 = (rgb24.g & 0b11111100) >> 2;
	b16 = (rgb24.b & 0b11111000) >> 3;
}

static void rgb24_to_rgb16(const display_rgb_t &rgb24, rgb16_t &rgb16)
{
	unsigned int r16, g16, b16;

	rgb24_to_rgb16(rgb24, r16, g16, b16);

	rgb16.r = r16;
	rgb16.g = g16;
	rgb16.b = b16;
}

static void write_register(unsigned int reg, unsigned int length, const uint8_t *data)
{
	spi_transaction_t transaction;
	uint8_t reg_data;

	assert(inited);
	assert(spi_device_handle);
	assert(data || (length == 0));

	spi_mutex_take();

	memset(&transaction, 0, sizeof(transaction));

	reg_data = static_cast<uint8_t>(reg);

	transaction.cmd = rs_write | rs_cmd;
	transaction.addr = 0;
	transaction.length = 8;
	transaction.tx_buffer = &reg_data;
	transaction.rxlength = 0;
	transaction.rx_buffer = nullptr;

	util_abort_on_esp_err("spi_device_transmit 1a", spi_device_transmit(spi_device_handle, &transaction));

	transaction.cmd = rs_write | rs_data;
	transaction.addr = 0;
	transaction.length = length * 8;
	transaction.tx_buffer = data;
	transaction.rxlength = 0;
	transaction.rx_buffer = nullptr;

	util_abort_on_esp_err("spi_device_transmit 1b", spi_device_transmit(spi_device_handle, &transaction));

	spi_mutex_give();
}

static void write_register_1(unsigned int reg, unsigned int data)
{
	uint8_t data_byte = static_cast<uint8_t>(data);

	write_register(reg, 1, &data_byte);
}

static void fgcolour_set(unsigned int r, unsigned int g, unsigned int b)
{
	display_rgb_t rgb24;

	rgb24.r = r;
	rgb24.g = g;
	rgb24.b = b;

	rgb24_to_rgb16(rgb24, r, g, b);

	write_register_1(reg_fgcr0, r);
	write_register_1(reg_fgcr1, g);
	write_register_1(reg_fgcr2, b);
}

static void bgcolour_set(unsigned int r, unsigned int g, unsigned int b)
{
	display_rgb_t rgb24;

	rgb24.r = r;
	rgb24.g = g;
	rgb24.b = b;

	rgb24_to_rgb16(rgb24, r, g, b);

	write_register_1(reg_bgcr0, r);
	write_register_1(reg_bgcr1, g);
	write_register_1(reg_bgcr2, b);
}

static void set_window(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1)
{
	assert(x0 < x_size);
	assert(x1 < x_size);
	assert(y0 < y_size);
	assert(y1 < y_size);

	write_register_1(reg_hsaw0, (x0 >> 0) & 0xff);
	write_register_1(reg_hsaw1, (x0 >> 8) & 0x03);
	write_register_1(reg_vsaw0, (y0 >> 0) & 0xff);
	write_register_1(reg_vsaw1, (y0 >> 8) & 0x01);
	write_register_1(reg_heaw0, (x1 >> 0) & 0xff);
	write_register_1(reg_heaw1, (x1 >> 8) & 0x03);
	write_register_1(reg_veaw0, (y1 >> 0) & 0xff);
	write_register_1(reg_veaw1, (y1 >> 8) & 0x01);
}

static void set_cursor(unsigned int x, unsigned int y)
{
	assert(x < x_size);
	assert(y < y_size);

	write_register_1(reg_curh1, (x & 0xff00) >> 8);
	write_register_1(reg_curh0, (x & 0x00ff) >> 0);
	write_register_1(reg_curv1, (y & 0xff00) >> 8);
	write_register_1(reg_curv0, (y & 0x00ff) >> 0);
}

static void box(unsigned int r, unsigned int g, unsigned int b, unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y)
{
	set_window(from_x, from_y, to_x, to_y);
	bgcolour_set(r, g, b);
	write_register_1(reg_mclr, reg_mclr_memory_clear_start | reg_mclr_memory_area_active_window);
	util_sleep(50);
	set_window(0, 0, x_size - 1, y_size - 1);
}

static void show_layer(unsigned int layer)
{
	unsigned int value = reg_ltpr0_scroll_layer_1 | reg_ltpr0_floatwin_transparency_dis;

	assert((layer == 0) || (layer == 1));

	value |= (layer == 0) ? reg_ltpr0_visible_layer_1 : reg_ltpr0_visible_layer_2;

	write_register_1(reg_ltpr1, reg_ltpr1_transparency_layer_2_8_8 | reg_ltpr1_transparency_layer_1_8_8);
	write_register_1(reg_ltpr0, value);
}

static void set_layer(unsigned int layer)
{
	assert((layer == 0) || (layer == 1));

	write_register_1(reg_mwcr1, reg_mwcr1_graphic_cursor_disable | reg_mwcr1_write_destination_layer | (layer ? 0x01 : 0x00));
}

static void pixel_buffer_flush()
{
	unsigned int length;

	assert(inited);
	assert(pixel_buffer);
	assert(pixel_buffer_size > 0);
	assert(spi_device_handle);

	length = pixel_buffer_length;
	pixel_buffer_length = 0;

	assert(length <= pixel_buffer_size);

	if(length > 0)
		write_register(reg_mrwc, length, pixel_buffer);
}

static void pixel_buffer_write(const display_rgb_t &pixel)
{
	rgb16_t rgb16;

	assert(inited);
	assert(pixel_buffer);
	assert(pixel_buffer_size > 0);
	assert(spi_device_handle);

	if((pixel_buffer_length + 2) >= pixel_buffer_size)
		pixel_buffer_flush();

	rgb24_to_rgb16(pixel, rgb16);

	pixel_buffer[pixel_buffer_length++] = rgb16.first;
	pixel_buffer[pixel_buffer_length++] = rgb16.second;
}

void display_ra8875_box(display_colour_t colour, unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y)
{
	const display_rgb_t *rgb;

	assert(inited);

	if(colour >= dc_size)
		return;

	rgb = &display_colour_map[colour];

	box(rgb->r, rgb->g, rgb->b, from_x, from_y, to_x, to_y);
}

void display_ra8875_clear(display_colour_t bg)
{
	display_ra8875_box(bg, 0, 0, x_size - 1, y_size - 1);
}

void display_ra8875_write(const font_t *font, display_colour_t fg_colour, display_colour_t bg_colour,
		unsigned int from_x, unsigned int from_y,
		unsigned int to_x, unsigned int to_y,
		const std::deque<uint32_t> &unicode_line)
{
	std::deque<uint32_t>::const_iterator unicode_it;
	const font_glyph_t *glyph;
	int col, row, bit;
	unsigned current_glyph;
	unsigned int ix;
	const display_rgb_t *fg_rgb;
	const display_rgb_t *bg_rgb;

	assert(inited);
	assert(pixel_buffer);
	assert(pixel_buffer_size > 0);

	assert(from_x <= to_x);
	assert(from_y <= to_y);

	if((from_x >= x_size) || (from_y >= y_size))
		return;

	if(to_x >= x_size)
		to_x = x_size - 1;

	if(to_y >= y_size)
		to_y = y_size - 1;

	assert(fg_colour < dc_size);
	assert(bg_colour < dc_size);

	fg_rgb = &display_colour_map[fg_colour];
	bg_rgb = &display_colour_map[bg_colour];

	set_window(from_x, from_y, to_x, to_y);
	set_cursor(from_x, from_y);

	col = from_x;

	for(unicode_it = unicode_line.begin(); unicode_it != unicode_line.end(); unicode_it++)
	{
		if((*unicode_it >= 0xf800) && (*unicode_it < 0xf808)) // abuse private use unicode codepoints for foreground colours
		{
			ix = (*unicode_it - 0xf800);

			if(ix >= dc_size)
				log_format("display-spi-generic: foreground colour out of range: %u", ix);
			else
				fg_rgb = &display_colour_map[ix];
		}
		else
		{
			if((*unicode_it >= 0xf808) && (*unicode_it < 0xf810)) // abuse private use unicode codepoints for background colours
			{
				ix = (*unicode_it - 0xf808);

				if(ix >= dc_size)
					log_format("display-spi-generic: background colour out of range: %u", ix);
				else
					bg_rgb = &display_colour_map[ix];
			}
			else
			{
				if(*unicode_it < font_basic_glyphs_size)
					glyph = &font->basic_glyph[*unicode_it];
				else
				{
					for(current_glyph = 0; current_glyph < font->extra_glyphs; current_glyph++)
					{
						glyph = &font->extra_glyph[current_glyph];

						if(glyph->codepoint == *unicode_it)
							break;
					}

					if(current_glyph >= font->extra_glyphs)
						glyph = (font_glyph_t *)0;
				}

				if(glyph)
				{
					for(bit = 0; bit < font->net.width; bit++)
					{
						for(row = 0; row < (to_y - from_y + 1); row++)
						{
							if(row < font->net.height)
							{
								if(glyph->row[row] & (1 << bit))
									pixel_buffer_write(*fg_rgb);
								else
									pixel_buffer_write(*bg_rgb);
							}
							else
								pixel_buffer_write(*bg_rgb);
						}

						if(++col > to_x)
							goto finished;
					}
				}
			}
		}
	}

	for(; col < (to_x + 1); col++)
		for(row = 0; row < (to_y - from_y + 1); row++)
			pixel_buffer_write(*bg_rgb);

finished:
	pixel_buffer_flush();
}

void display_ra8875_plot_line(unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int rgb_pixels_length, const display_rgb_t *pixels)
{
	unsigned int current;
	display_rgb_t bg;

	assert(inited);
	assert(pixel_buffer);
	assert(pixel_buffer_size > 0);

	bg = display_colour_map[dc_blue];

	if((from_x >= x_size) || (from_y >= y_size))
		return;

	if(to_x >= x_size)
		to_x = x_size - 1;

	set_window(from_x, from_y, to_x, from_y);
	set_cursor(from_x, from_y);

	if((to_x - from_x) < rgb_pixels_length)
		rgb_pixels_length = to_x - from_x + 1;

	for(current = 0; current < rgb_pixels_length; current++)
		pixel_buffer_write(pixels[current]);

	for(; current < (to_x - from_x); current++)
		pixel_buffer_write(bg);

	pixel_buffer_flush();
}

bool display_ra8875_init(const display_init_parameters_t *parameters)
{
	size_t max_transaction_length;

	switch(parameters->interface_index)
	{
		case(0):
		{
			spi_signal = &spi_host_signal.spi2;
			break;
		}

		case(1):
		{
			spi_signal = &spi_host_signal.spi3;
			break;
		}

		default:
		{
			log_format("init display ra8875: unknown spi interface %d, use 0 for SPI2 or 1 for SPI3", parameters->interface_index);
			return(false);
		}
	}

	if((parameters->x_size < 0) || (parameters->y_size < 0))
	{
		log("init display ra8875: display dimensions required");
		return(false);
	}

	x_size = parameters->x_size;
	y_size = parameters->y_size;

	spi_bus_config_t bus_config =
	{
		.mosi_io_num = static_cast<int>(spi_signal->mosi),
		.miso_io_num = static_cast<int>(spi_signal->miso),
		.sclk_io_num = static_cast<int>(spi_signal->sck),
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.data4_io_num = -1,
		.data5_io_num = -1,
		.data6_io_num = -1,
		.data7_io_num = -1,
		.data_io_default_level = false,
		.max_transfer_sz = 0,
		.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI,
		.isr_cpu_id = ESP_INTR_CPU_AFFINITY_1,
		.intr_flags = 0,
	};

	spi_device_interface_config_t device =
	{
		.command_bits = 8,
		.address_bits = 0,
		.dummy_bits = 0,
		.mode = 0,
		.clock_source = SPI_CLK_SRC_DEFAULT,
		.duty_cycle_pos = 0,
		.cs_ena_pretrans = 0,
		.cs_ena_posttrans = 0,
		.clock_speed_hz = spi_speed_initial,
		.input_delay_ns = 0,
		.sample_point = SPI_SAMPLING_POINT_PHASE_0,
		.spics_io_num = static_cast<int>(spi_signal->cs),
		.flags = 0,
		.queue_size = 1,
		.pre_cb = nullptr,
		.post_cb = nullptr,
	};

	spi_mutex = xSemaphoreCreateMutex();
	assert(spi_mutex);

	util_abort_on_esp_err("spi_bus_initialize", spi_bus_initialize(spi_signal->esp_host, &bus_config, SPI_DMA_CH_AUTO));
	util_abort_on_esp_err("spi_bus_add_device 1", spi_bus_add_device(spi_signal->esp_host, &device, &spi_device_handle));

	util_abort_on_esp_err("spi_bus_get_max_transaction_len", spi_bus_get_max_transaction_len(spi_signal->esp_host, &max_transaction_length));

	pixel_buffer_size = max_transaction_length;
	pixel_buffer_length = 0;
	pixel_buffer = static_cast<uint8_t *>(heap_caps_malloc(pixel_buffer_size, MALLOC_CAP_DMA));

	inited = true;

	// PLL

	write_register_1(reg_pll_c1, reg_pllc1_value);
	util_sleep(50);
	write_register_1(reg_pll_c2, reg_pllc2_value);
	util_sleep(50);

	util_abort_on_esp_err("spi_bus_remove_device", spi_bus_remove_device(spi_device_handle));
	device.clock_speed_hz = spi_speed_normal;
	util_abort_on_esp_err("spi_bus_add_device 2", spi_bus_add_device(spi_signal->esp_host, &device, &spi_device_handle));

	// interface

	write_register_1(reg_sysr, reg_sysr_color_depth_16 | reg_sysr_if_8bit);
	write_register_1(reg_pcsr, reg_pcsr_sample_falling_edge | reg_pcsr_clock_period_system_by_8);

	// horizontal

	write_register_1(reg_hdwr, (x_size / 8) - 1);
	write_register_1(reg_hndftr, reg_hndftr_de_polarity_active_high | (horizontal_blanking_fine / 2));
	write_register_1(reg_hndr, (horizontal_blanking / 8) - 1);
	write_register_1(reg_hstr, (horizontal_sync_start / 8) - 1);
	write_register_1(reg_hpwr, reg_hpwr_hsync_polarity_active_low | ((horizontal_sync_length / 8) - 1));

	// vertical

	write_register_1(reg_vdhr0, ((y_size >> 0) & 0xff) + 1);
	write_register_1(reg_vdhr1, ((y_size >> 8) & 0x01) + 0);
	write_register_1(reg_vndr0, ((vertical_blanking >> 0) & 0xff) + 1);
	write_register_1(reg_vndr1, ((vertical_blanking >> 8) & 0x01) + 0);
	write_register_1(reg_vstr0, ((vertical_sync_start >> 0) & 0xff) + 1);
	write_register_1(reg_vstr1, ((vertical_sync_start >> 8) & 0x01) + 0);
	write_register_1(reg_vpwr, reg_vpwr_vsync_polarity_active_low | (vertical_sync_length - 1));

	// PWM

	write_register_1(reg_p1cr, reg_p1cr_pwm1_enable | reg_p1cr_function_pwm1 | reg_p1cr_clock_ratio_2048); // 114 Hz refresh
	write_register_1(reg_pwrr, reg_pwrr_display_enable | reg_pwrr_display_sleep_mode_disable | reg_pwrr_display_reset_complete);

	// MISC

	write_register_1(reg_dpcr, reg_dpcr_two_layer | reg_dpcr_hor_scan_ltor | reg_dpcr_vert_scan_ltor);

	write_register_1(reg_mwcr0, reg_mwcr0_mode_graphic | reg_mwcr0_cursor_invisible | reg_mwcr0_cursor_steady |
			reg_mwcr0_memory_write_direction_tdlr | reg_mwcr0_memory_write_autoincr_en | reg_mwcr0_memory_read_autoincr_en);

	display_ra8875_bright(100);

	show_layer(1);
	set_layer(1);
	box(0x00, 0x00, 0x00, 0, 0, x_size - 1, y_size - 1);

	show_layer(0);
	set_layer(0);
	box(0x00, 0x00, 0x00, 0, 0, x_size - 1, y_size - 1);

	bgcolour_set(0x00, 0x00, 0x00);
	fgcolour_set(0xff, 0xff, 0xff);

	return(true);
}

void display_ra8875_bright(unsigned int brightness)
{
	static constexpr unsigned int p1cr = reg_p1cr_function_pwm1 | reg_p1cr_clock_ratio_2048;
	static constexpr unsigned int pwrr = reg_pwrr_display_sleep_mode_disable | reg_pwrr_display_reset_complete;

	assert(brightness <= 100);

	if(brightness == 0)
	{
		write_register_1(reg_p1dcr, 0);
		write_register_1(reg_p1cr, p1cr | reg_p1cr_pwm1_disable);
		write_register_1(reg_pwrr, pwrr | reg_pwrr_display_disable);
	}
	else
	{
		if(brightness == 100)
			brightness = 255;
		else
			brightness = brightness * 2550 / 1000;

		write_register_1(reg_p1dcr, brightness);
		write_register_1(reg_p1cr, p1cr | reg_p1cr_pwm1_enable);
		write_register_1(reg_pwrr, pwrr | reg_pwrr_display_enable);
	}
}

void display_ra8875_set_layer(unsigned int layer)
{
	set_layer(layer);
}

void display_ra8875_show_layer(unsigned int layer)
{
	show_layer(layer);
}
