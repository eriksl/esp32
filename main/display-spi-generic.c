#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <driver/sdm.h>

#include "string.h"
#include "log.h"
#include "util.h"
#include "display-spi-generic.h"
#include "sdkconfig.h"

enum
{
	cmd_swreset =			0x01,
	cmd_sleepout = 			0x11,
	cmd_noron =				0x13,
	cmd_invoff =			0x20,
	cmd_invon =				0x21,
	cmd_dispoff =			0x28,
	cmd_dispon =			0x29,
	cmd_caset =				0x2a,
	cmd_raset =				0x2b,
	cmd_ramwr =				0x2c,
	cmd_madctl =			0x36,
	cmd_colmod =			0x3a,

	madctl_my =		(1 << 7),
	madctl_mx = 	(1 << 6),
	madctl_mv =		(1 << 5),
	madctl_ml =		(1 << 4),
	madctl_bgr =	(1 << 3),
	madctl_mh =		(1 << 2),

	colmod_18bpp = 	0b01100110,
};

typedef struct
{
	unsigned int cs;
	unsigned int sck;
	unsigned int mosi;
	unsigned int miso;
	unsigned int dc;
	unsigned int bl;
	unsigned int esp_host;
} spi_signal_t;

typedef struct
{
	spi_signal_t spi2;
	spi_signal_t spi3;
} spi_host_signal_t;

typedef struct
{
	unsigned int gpio;
	unsigned int level;
} callback_data_t;

static const spi_host_signal_t spi_host_signal =
{
	.spi2 =
	{
		.esp_host =	SPI2_HOST,
		.cs =		10,									/* IOMUX, fixed */
		.sck =		12,									/* IOMUX, fixed */
		.mosi =		11,									/* IOMUX, fixed */
		.miso =		13,									/* IOMUX, fixed */
		.dc =		CONFIG_BSP_SPI2_DISPLAY_DC,
		.bl =		CONFIG_BSP_SPI2_DISPLAY_BACKLIGHT,
	},
	.spi3 =
	{
		.esp_host =	SPI3_HOST,
		.cs =		CONFIG_BSP_SPI3_DISPLAY_CS,
		.sck =		CONFIG_BSP_SPI3_SCK,
		.mosi =		CONFIG_BSP_SPI3_MOSI,
		.miso =		CONFIG_BSP_SPI3_MISO,
		.dc =		CONFIG_BSP_SPI3_DISPLAY_DC,
		.bl =		CONFIG_BSP_SPI3_DISPLAY_BACKLIGHT,
	},
};

static bool inited = false;
static sdm_channel_handle_t sdm_channel_handle;
static spi_device_handle_t spi_device_handle;
static const spi_signal_t *spi_signal;
static callback_data_t callback_data_gpio_on;
static callback_data_t callback_data_gpio_off;
static SemaphoreHandle_t spi_mutex;
static unsigned int spi_pending;
static unsigned int x_size, y_size;
static unsigned int x_mirror, y_mirror;
static unsigned int rotate;
static unsigned int invert;
static unsigned int madctl;

static inline void spi_mutex_take(void)
{
	assert(xSemaphoreTake(spi_mutex, portMAX_DELAY));
}

static inline void spi_mutex_give(void)
{
	assert(xSemaphoreGive(spi_mutex));
}

static void send_command_data(bool send_cmd, unsigned int cmd, unsigned int length, const uint8_t *data)
{
	static spi_transaction_ext_t transaction_base;
	static spi_transaction_t *transaction = &transaction_base.base;
	static spi_transaction_ext_t *transaction_ext = &transaction_base;
	static spi_transaction_t *transaction_result;

	spi_mutex_take();

	assert(inited);
	assert(spi_device_handle);
	assert(send_cmd || (cmd == 0));
	assert((length == 0) || (data != (const uint8_t *)0));

	for(; spi_pending > 0; spi_pending--)
		util_abort_on_esp_err("spi_device_get_trans_result", spi_device_get_trans_result(spi_device_handle, &transaction_result, portMAX_DELAY));

	if(cmd)
	{
		memset(transaction_ext, 0, sizeof(*transaction_ext));

		transaction->flags = SPI_TRANS_VARIABLE_CMD;
		transaction->cmd = cmd;
		transaction->addr = 0;
		transaction->length = 0;
		transaction->rxlength = 0;
		transaction->tx_buffer = (const void *)0;
		transaction->rx_buffer = (void *)0;
		transaction->user = &callback_data_gpio_off;
		transaction_ext->command_bits = 8;
		transaction_ext->address_bits = 0;
		transaction_ext->dummy_bits = 0;

		util_abort_on_esp_err("spi_device_transmit", spi_device_transmit(spi_device_handle, transaction));
	}

	if(length > 0)
	{
		memset(transaction_ext, 0, sizeof(*transaction_ext));

		transaction->flags = SPI_TRANS_VARIABLE_CMD;
		transaction->cmd = 0;
		transaction->addr = 0;
		transaction->length = length * 8;
		transaction->rxlength = 0;
		transaction->tx_buffer = data;
		transaction->rx_buffer = (void *)0;
		transaction->user = &callback_data_gpio_on;
		transaction_ext->command_bits = 0;
		transaction_ext->address_bits = 0;
		transaction_ext->dummy_bits = 0;

		util_abort_on_esp_err("spi_device_transmit", spi_device_transmit(spi_device_handle, transaction));
	}

	spi_mutex_give();
}

static void send_command(unsigned int cmd)
{
	send_command_data(true, cmd, 0, 0);
}

static void send_command_data_1(unsigned int cmd, unsigned int data)
{
	uint8_t bytes[1];

	bytes[0] = data & 0xff;

	send_command_data(true, cmd, 1, bytes);
}

static void send_command_data_2_16(unsigned int cmd, unsigned int data_1, unsigned int data_2)
{
	uint8_t bytes[4];

	bytes[0] = (data_1 & 0xff00) >> 8;
	bytes[1] = (data_1 & 0x00ff) >> 0;
	bytes[2] = (data_2 & 0xff00) >> 8;
	bytes[3] = (data_2 & 0x00ff) >> 0;

	send_command_data(true, cmd, 4, bytes);
}

static void set_window(unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y)
{
	if(from_x >= x_size)
		from_x = x_size - 1;

	if(to_x >= x_size)
		to_x = x_size - 1;

	if(from_y >= y_size)
		from_y = y_size - 1;

	if(to_y >= y_size)
		to_y = y_size - 1;

	send_command_data_2_16(cmd_caset, from_y, to_y);
	send_command_data_2_16(cmd_raset, from_x, to_x);
	send_command(cmd_ramwr);
}

static unsigned int pixel_buffer_size = 0;
static unsigned int pixel_buffer_rgb_length = 0;
static unsigned int pixel_buffer_rgb_size = 0;
static display_rgb_t *pixel_buffer_rgb = (display_rgb_t *)0;
static uint8_t *pixel_buffer = (uint8_t *)0;

static void pixel_buffer_flush(int length, bool wait)
{
	static spi_transaction_ext_t transaction_base;
	static spi_transaction_t *transaction = &transaction_base.base;
	static spi_transaction_ext_t *transaction_ext = &transaction_base;
	static spi_transaction_t *transaction_result;

	assert(inited);
	assert(spi_device_handle);

	spi_mutex_take();

	if(length < 0)
	{
		length = pixel_buffer_rgb_length;
		pixel_buffer_rgb_length = 0;
	}

	assert(length <= pixel_buffer_rgb_size);

	if(length > 0)
	{
		memset(transaction_ext, 0, sizeof(*transaction_ext));

		transaction->flags = SPI_TRANS_VARIABLE_CMD;
		transaction->cmd = 0;
		transaction->addr = 0;
		transaction->length = length * 8 * sizeof(display_rgb_t);
		transaction->rxlength = 0;
		transaction->tx_buffer = pixel_buffer;
		transaction->rx_buffer = (void *)0;
		transaction->user = &callback_data_gpio_on;
		transaction_ext->command_bits = 0;
		transaction_ext->address_bits = 0;
		transaction_ext->dummy_bits = 0;

		for(; spi_pending > 0; spi_pending--)
			util_abort_on_esp_err("spi_device_get_trans_result", spi_device_get_trans_result(spi_device_handle, &transaction_result, portMAX_DELAY));

		util_abort_on_esp_err("spi_device_queue_trans", spi_device_queue_trans(spi_device_handle, transaction, portMAX_DELAY));
		spi_pending++;

		if(wait)
			for(; spi_pending > 0; spi_pending--)
				util_abort_on_esp_err("spi_device_get_trans_result", spi_device_get_trans_result(spi_device_handle, &transaction_result, portMAX_DELAY));
	}

	spi_mutex_give();
}

static void pixel_buffer_write(unsigned int length_rgb, const display_rgb_t *pixels_rgb, bool wait)
{
	assert(inited);
	assert(pixel_buffer);
	assert(pixel_buffer_size > 0);
	assert(pixel_buffer_rgb);
	assert(pixel_buffer_rgb_size > 0);
	assert(length_rgb < pixel_buffer_rgb_size);

	if((pixel_buffer_rgb_length + length_rgb) >= pixel_buffer_rgb_size)
		pixel_buffer_flush(-1, wait);

	memcpy(&pixel_buffer_rgb[pixel_buffer_rgb_length], pixels_rgb, length_rgb * sizeof(display_rgb_t));
	pixel_buffer_rgb_length += length_rgb;
}

static void box(unsigned int r, unsigned int g, unsigned int b, unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y)
{
	unsigned int pixel, pixels, chunk;
	unsigned int length_x, length_y;

	assert(inited);
	assert(pixel_buffer);
	assert(pixel_buffer_size > 0);
	assert(pixel_buffer_rgb);
	assert(pixel_buffer_rgb_size > 0);
	assert(to_x >= from_x);
	assert(to_y >= from_y);

	length_x = to_x - from_x + 1;
	length_y = to_y - from_y + 1;
	pixels = (length_x * length_y);

	for(pixel = 0; pixel < pixel_buffer_rgb_size; pixel++)
	{
		pixel_buffer_rgb[pixel].r = r;
		pixel_buffer_rgb[pixel].g = g;
		pixel_buffer_rgb[pixel].b = b;
	}

	set_window(from_x, from_y, to_x, to_y);

	chunk = 0;

	for(pixel = 0; pixel < pixels; pixel += chunk)
	{
		chunk = pixel_buffer_rgb_size;

		if(chunk > (pixels - pixel))
			chunk = pixels - pixel;

		pixel_buffer_flush(chunk, true);
	}
}

void display_spi_generic_box(display_colour_t colour, unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y)
{
	const display_rgb_t *rgb;

	assert(inited);

	if(colour >= dc_size)
		return;

	rgb = &display_colour_map[colour];

	box(rgb->r, rgb->g, rgb->b, from_x, from_y, to_x, to_y);
}

void display_spi_generic_clear(display_colour_t bg)
{
	display_spi_generic_box(bg, 0, 0, x_size - 1, y_size - 1);
}

void display_spi_generic_write(const font_t *font, display_colour_t fg_colour, display_colour_t bg_colour,
		unsigned int from_x, unsigned int from_y,
		unsigned int to_x, unsigned int to_y,
		unsigned int unicode_line_length, const uint32_t *unicode_line)
{
	unsigned int current_unicode_entry;
	uint32_t code;
	const font_glyph_t *glyph;
	int col, row, bit;
	unsigned current_glyph;
	unsigned int ix;
	display_rgb_t fg_rgb;
	display_rgb_t bg_rgb;

	assert(inited);
	assert(pixel_buffer);
	assert(pixel_buffer_size > 0);
	assert(pixel_buffer_rgb);
	assert(pixel_buffer_rgb_size > 0);

	assert(from_x <= to_x);
	assert(from_y <= to_y);

	if((from_x >= x_size) || (from_y >= y_size))
		return;

	assert(fg_colour < dc_size);
	assert(bg_colour < dc_size);

	fg_rgb = display_colour_map[fg_colour];
	bg_rgb = display_colour_map[bg_colour];

	set_window(from_x, from_y, to_x, to_y);

	col = from_x;

	for(current_unicode_entry = 0; current_unicode_entry < unicode_line_length; current_unicode_entry++)
	{
		code = unicode_line[current_unicode_entry];

		if((code >= 0xf800) && (code < 0xf808)) // abuse private use unicode codepoints for foreground colours
		{
			ix = (code - 0xf800);

			if(ix >= dc_size)
				log_format("display-spi-generic: foreground colour out of range: %u", ix);
			else
				fg_rgb = display_colour_map[ix];
		}
		else
		{
			if((code >= 0xf808) && (code < 0xf810)) // abuse private use unicode codepoints for background colours
			{
				ix = (code - 0xf808);

				if(ix >= dc_size)
					log_format("display-spi-generic: background colour out of range: %u", ix);
				else
					bg_rgb = display_colour_map[ix];
			}
			else
			{
				if(code < font_basic_glyphs_size)
					glyph = &font->basic_glyph[code];
				else
				{
					for(current_glyph = 0; current_glyph < font->extra_glyphs; current_glyph++)
					{
						glyph = &font->extra_glyph[current_glyph];

						if(glyph->codepoint == code)
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
									pixel_buffer_write(1, &fg_rgb, true);
								else
									pixel_buffer_write(1, &bg_rgb, true);
							}
							else
								pixel_buffer_write(1, &bg_rgb, true);
						}

						if(++col > to_x)
							goto finished;
					}
				}
			}
		}
	}

	for(; col <= to_x; col++)
		for(row = 0; row < (to_y - from_y + 1); row++)
			pixel_buffer_write(1, &bg_rgb, true);

finished:
	pixel_buffer_flush(-1, true);
}

void display_spi_generic_plot_line(unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int rgb_pixels_length, const display_rgb_t *pixels)
{
	unsigned int current, chunk;
	static const display_rgb_t * const bg = &display_colour_map[dc_blue];

	assert(inited);
	assert(pixel_buffer);
	assert(pixel_buffer_size > 0);
	assert(pixel_buffer_rgb);
	assert(pixel_buffer_rgb_size > 0);

	if((from_x >= x_size) || (from_y >= y_size))
		return;

	if(to_x >= x_size)
		to_x = x_size - 1;

	set_window(from_x, from_y, to_x, from_y);

	if((to_x - from_x) < rgb_pixels_length)
		rgb_pixels_length = to_x - from_x + 1;

	chunk = 0;

	for(current = 0; current < rgb_pixels_length; current += chunk)
	{
		chunk = rgb_pixels_length - current;

		if(chunk > pixel_buffer_rgb_size)
			chunk = pixel_buffer_rgb_size;

		pixel_buffer_write(chunk, &pixels[current], true);
	}

	for(; current < (to_x - from_x); current++)
		pixel_buffer_write(1, bg, true);

	pixel_buffer_flush(-1, false);
}

void pre_callback(spi_transaction_t *transaction)
{
	assert(inited);
	assert(transaction);
	assert(transaction->user);

	const callback_data_t *callback_data = (const callback_data_t *)transaction->user;

	gpio_set_level(callback_data->gpio, callback_data->level);
}

bool display_spi_generic_init(const display_init_parameters_t *parameters)
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
			log_format("init display-lcd-spi: unknown spi interface %u, use 0 for SPI2 or 1 for SPI3", parameters->interface_index);
			return(false);
		}
	}

	if((parameters->x_size < 0) || (parameters->y_size < 0))
	{
		log("init display-lcd-spi: display dimensions required");
		return(false);
	}

	x_size = parameters->x_size;
	y_size = parameters->y_size;

	x_mirror = 0;
	y_mirror = 0;
	rotate = 0;
	invert = 0;

	if(parameters->x_mirror >= 0)
		y_mirror = parameters->x_mirror;

	if(parameters->y_mirror >= 0)
		x_mirror = parameters->y_mirror;

	if(parameters->rotate >= 0)
		rotate = parameters->rotate;

	if(parameters->invert >= 0)
		invert = parameters->invert;

	callback_data_gpio_on.gpio = spi_signal->dc;
	callback_data_gpio_on.level = 1;
	callback_data_gpio_off.gpio = spi_signal->dc;
	callback_data_gpio_off.level = 0;

	gpio_config_t gpio_pin_config =
	{
		.pin_bit_mask = (1ULL << spi_signal->dc) | (1ULL << spi_signal->bl),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	sdm_config_t pdm_config =
	{
		.gpio_num = spi_signal->bl,
		.clk_src = SDM_CLK_SRC_DEFAULT,
		.sample_rate_hz = 80000000 / 1,
		.flags.invert_out = 0,
		.flags.io_loop_back = 0,
	};

	spi_bus_config_t bus_config =
	{
		.sclk_io_num = spi_signal->sck,
		.mosi_io_num = spi_signal->mosi,
		.miso_io_num = spi_signal->miso,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 0,
		.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI,
	};

	spi_device_interface_config_t device =
	{
		.command_bits = 0,
		.address_bits = 0,
		.dummy_bits = 0,
		.mode = 0,
		.clock_source = SPI_CLK_SRC_DEFAULT,
		.duty_cycle_pos = 0,
		.cs_ena_pretrans = 0,
		.cs_ena_posttrans = 0,
		.clock_speed_hz = 40000000,
		.input_delay_ns = 0,
		.spics_io_num = spi_signal->cs,
		.flags = 0,
		.queue_size = 1,
		.pre_cb = pre_callback,
		.post_cb = (transaction_cb_t)0,
	};

	spi_mutex = xSemaphoreCreateMutex();
	assert(spi_mutex);
	spi_pending = 0;

	util_abort_on_esp_err("gpio_config", gpio_config(&gpio_pin_config));

	util_abort_on_esp_err("sdm_new_channel", sdm_new_channel(&pdm_config, &sdm_channel_handle));
	util_abort_on_esp_err("sdm_channel_enable", sdm_channel_enable(sdm_channel_handle));
	util_abort_on_esp_err("sdm_channel_set_pulse_density", sdm_channel_set_pulse_density(sdm_channel_handle, 127));

	util_abort_on_esp_err("spi_bus_initialize", spi_bus_initialize(spi_signal->esp_host, &bus_config, SPI_DMA_CH_AUTO));
	util_abort_on_esp_err("spi_bus_add_device", spi_bus_add_device(spi_signal->esp_host, &device, &spi_device_handle));

	util_abort_on_esp_err("spi_bus_get_max_transaction_len", spi_bus_get_max_transaction_len(spi_signal->esp_host, &max_transaction_length));
	pixel_buffer_size = max_transaction_length;
	pixel_buffer = util_memory_alloc_dma(pixel_buffer_size);
	pixel_buffer_rgb = (display_rgb_t *)pixel_buffer;
	pixel_buffer_rgb_size = pixel_buffer_size / sizeof(display_rgb_t);
	pixel_buffer_rgb_length = 0;

	inited = true;

	send_command(cmd_dispoff);
	send_command(cmd_swreset);
	util_sleep(200);
	send_command(cmd_sleepout);
	util_sleep(200);
	send_command(cmd_noron);
	send_command(cmd_dispon);
	send_command_data_1(cmd_colmod, colmod_18bpp);
	send_command(invert ? cmd_invon : cmd_invoff);

	madctl = madctl_bgr;

	if(rotate)
		madctl |= madctl_mv;

	if(!x_mirror)
		madctl |= madctl_my;

	if(!y_mirror)
		madctl |= madctl_mx;

	send_command_data_1(cmd_madctl, madctl);

	return(true);
}

static const int perc_to_sdm[][2] =
{
	{	0,		-128	},
	{	25,		-108	},
	{	50,		-96		},
	{	75,		-88		},
	{	100,	127		},
};

void display_spi_generic_bright(unsigned int percentage)
{
	unsigned int ix;

	if(percentage <= 100)
	{
		for(ix = 0; ix < 16; ix++)
			if(percentage <= perc_to_sdm[ix][0])
				break;

		util_abort_on_esp_err("sdm_channel_set_pulse_density", sdm_channel_set_pulse_density(sdm_channel_handle, perc_to_sdm[ix][1]));
	}
}

