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
	cmd_nop =				0x00,
	cmd_swreset =			0x01,
	cmd_rddinfo =			0x04,
	cmd_rddstatus =			0x09,
	cmd_rddsigmode =		0x0e,
	cmd_rddpm =				0x0a,
	cmd_rdmadctl =			0x0b,
	cmd_rddsdr =			0x0f,
	cmd_sleepin =			0x10,
	cmd_sleepout = 			0x11,
	cmd_noron =				0x13,
	cmd_invoff =			0x20,
	cmd_invon =				0x21,
	cmd_allpixelsoff =		0x22,
	cmd_allpixelson =		0x23,
	cmd_gamma_set =			0x26,
	cmd_dispoff =			0x28,
	cmd_dispon =			0x29,
	cmd_caset =				0x2a,
	cmd_raset =				0x2b,
	cmd_ramwr =				0x2c,
	cmd_ramrd =				0x2e,
	cmd_madctl =			0x36,
	cmd_idmoff =			0x38,
	cmd_idmon =				0x39,
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
static unsigned int x_size, x_offset;
static unsigned int y_size, y_offset;
static unsigned int x_mirror, y_mirror;
static unsigned int rotate;
static unsigned int invert;
static unsigned int madctl;

static void send_command_data(bool send_cmd, unsigned int cmd, unsigned int length, const uint8_t *data)
{
	spi_transaction_ext_t transaction_base;
	spi_transaction_t *transaction = &transaction_base.base;
	spi_transaction_ext_t *transaction_ext = &transaction_base;

	assert(inited);
	assert(spi_device_handle);
	assert(send_cmd || (cmd == 0));
	assert((length == 0) || (data != (const uint8_t *)0));

	spi_device_acquire_bus(spi_device_handle, portMAX_DELAY);

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

		if(length > 0)
			transaction->flags |= SPI_TRANS_CS_KEEP_ACTIVE;

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

	spi_device_release_bus(spi_device_handle);
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

static void box(unsigned int r, unsigned int g, unsigned int b, unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y)
{
	unsigned int pixel, pixels, chunk;
	unsigned int length_x, length_y;
	display_rgb_t *pixel_buffer_rgb;
	unsigned int pixel_buffer_rgb_size;

	assert(inited);
	assert(display_pixel_buffer);
	assert(display_pixel_buffer_size > 0);
	assert(to_x > from_x);
	assert(to_y > from_y);

	pixel_buffer_rgb = (display_rgb_t *)display_pixel_buffer;
	pixel_buffer_rgb_size = display_pixel_buffer_size / sizeof(display_rgb_t);

	length_x = to_x - from_x + 1;
	length_y = to_y - from_y + 1;
	pixels = (length_x * length_y);

	for(pixel = 0; pixel < pixel_buffer_rgb_size; pixel++)
	{
		pixel_buffer_rgb[pixel].r = r;
		pixel_buffer_rgb[pixel].g = g;
		pixel_buffer_rgb[pixel].b = b;
	}

	send_command_data_2_16(cmd_caset, from_x + x_offset, to_x + x_offset);
	send_command_data_2_16(cmd_raset, from_y + y_offset, to_y + y_offset);
	send_command(cmd_ramwr);

	chunk = 0;

	for(pixel = 0; pixel < pixels; pixel += chunk)
	{
		chunk = pixel_buffer_rgb_size;

		if(chunk > (pixels - pixel))
			chunk = pixels - pixel;

		send_command_data(false, 0, chunk * sizeof(display_rgb_t), display_pixel_buffer);
	}
}

void display_spi_generic_clear(void)
{
	assert(inited);

	box(0x00, 0x00, 0x00, 0, 0, x_size - 1, y_size - 1);
}

void pre_callback(spi_transaction_t *transaction)
{
	assert(inited);
	assert(transaction);
	assert(transaction->user);

	const callback_data_t *callback_data = (const callback_data_t *)transaction->user;

	gpio_set_level(callback_data->gpio, callback_data->level);
}

bool display_spi_generic_init(const display_init_parameters_t *parameters, unsigned int *buffer_size)
{
	int freq_khz;
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

	x_offset = 0;
	y_offset = 0;
	x_mirror = 0;
	y_mirror = 0;
	rotate = 0;
	invert = 0;

	if(parameters->x_offset >= 0)
		x_offset = parameters->x_offset;

	if(parameters->y_offset >= 0)
		y_offset = parameters->y_offset;

	if(parameters->x_mirror >= 0)
		x_mirror = parameters->x_mirror;

	if(parameters->y_mirror >= 0)
		y_mirror = parameters->y_mirror;

	if(parameters->rotate >= 0)
		rotate = parameters->rotate;

	if(parameters->invert >= 0)
		invert = parameters->invert;

	log_format("- host interface input: %u", parameters->interface_index);
	log_format("- cs: %u", spi_signal->cs);
	log_format("- sck: %u", spi_signal->sck);
	log_format("- mosi: %u", spi_signal->mosi);
	log_format("- miso: %u", spi_signal->miso);
	log_format("- dc: %u", spi_signal->dc);
	log_format("- bl: %u", spi_signal->bl);
	log_format("- esp_host: %u", spi_signal->esp_host);
	log_format("- size: %u x %u", x_size, y_size);
	log_format("- offset: %u x %u", x_offset, y_offset);
	log_format("- mirror: %u x %u", x_mirror, y_mirror);
	log_format("- rotate: %u", rotate);
	log_format("- invert: %u", invert);

	callback_data_gpio_on.gpio = spi_signal->dc;
	callback_data_gpio_on.level = 1;
	callback_data_gpio_off.gpio = spi_signal->dc;
	callback_data_gpio_off.level = 0;

	log_format("- D/C gpio on: %u -> %u", callback_data_gpio_on.gpio, callback_data_gpio_on.level);
	log_format("- D/C gpio off: %u -> %u", callback_data_gpio_off.gpio, callback_data_gpio_off.level);

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

	util_abort_on_esp_err("gpio_config", gpio_config(&gpio_pin_config));
	util_abort_on_esp_err("spi_bus_initialize", spi_bus_initialize(spi_signal->esp_host, &bus_config, SPI_DMA_CH_AUTO));
	util_abort_on_esp_err("spi_bus_add_device", spi_bus_add_device(spi_signal->esp_host, &device, &spi_device_handle));
	util_abort_on_esp_err("spi_device_get_actual_freq", spi_device_get_actual_freq(spi_device_handle, &freq_khz));
	log_format("lcd-spi: actual bus frequency: %d MHz", freq_khz / 1000);

	util_abort_on_esp_err("spi_bus_get_max_transaction_len", spi_bus_get_max_transaction_len(spi_signal->esp_host, &max_transaction_length));
	log_format("lcd-spi: max transaction size: %u", max_transaction_length);
	*buffer_size = max_transaction_length;

	util_abort_on_esp_err("sdm_new_channel", sdm_new_channel(&pdm_config, &sdm_channel_handle));
	util_abort_on_esp_err("sdm_channel_enable", sdm_channel_enable(sdm_channel_handle));
	util_abort_on_esp_err("sdm_channel_set_pulse_density", sdm_channel_set_pulse_density(sdm_channel_handle, 127));

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
		madctl |= madctl_mx;

	if(!y_mirror)
		madctl |= madctl_my;

	send_command_data_1(cmd_madctl, madctl);

	return(true);
}

static const int perc_to_sdm[][2] =
{
	{	0,		-128	},
	{	25,		-108	},
	{	50,		-96		},
	{	75,		-80		},
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

static inline void send_pixel(const display_rgb_t *pixel)
{
	static unsigned int ix = 0;

	display_rgb_t *pixel_buffer_rgb;
	unsigned int pixel_buffer_rgb_size;

	pixel_buffer_rgb = (display_rgb_t *)display_pixel_buffer;
	pixel_buffer_rgb_size = display_pixel_buffer_size / sizeof(display_rgb_t);

	if(((ix + 1) >= pixel_buffer_rgb_size)|| !pixel)
	{
		send_command_data(false, 0, ix * sizeof(display_rgb_t), display_pixel_buffer);
		ix = 0;
	}

	if(pixel)
		pixel_buffer_rgb[ix++] = *pixel;
}

void display_spi_generic_write(const font_t *font, unsigned int cursor_row, unsigned int unicode_line_length, const uint32_t *unicode_line)
{
	int col, row, top, bottom, bit;
	unsigned int current_unicode_entry;
	uint32_t code;
	const font_glyph_t *glyph;
	unsigned current_glyph;
	unsigned int ix;
	display_rgb_t foreground_pixel = { 0xff, 0xff, 0xff };
	display_rgb_t background_pixel = { 0x00, 0x00, 0x00 };

	assert(display_pixel_buffer);
	assert(display_pixel_buffer_size > 0);

	top = font->net.height * cursor_row;
	bottom = top + font->net.height - 1;

	if((top < 0) || (bottom < 0) || (top > x_size) || (bottom > x_size))
		return;

	send_command_data_2_16(cmd_caset, top + x_offset, bottom + x_offset);
	send_command_data_2_16(cmd_raset,   0 + y_offset, y_size + y_offset);
	send_command(cmd_ramwr);

	col = 0;

	for(current_unicode_entry = 0; current_unicode_entry < unicode_line_length; current_unicode_entry++)
	{
		code = unicode_line[current_unicode_entry];

		if((code >= 0xf800) && (code < 0xf808)) // abuse private use unicode codepoints for foreground colours
		{
			ix = (code - 0xf800);

			if(ix >= display_colours)
				log_format("display-spi-generic: foreground colour out of range: %u", ix);
			else
				foreground_pixel = display_colour_map[ix];
		}
		else
		{
			if((code >= 0xf808) && (code < 0xf810)) // abuse private use unicode codepoints for background colours
			{
				ix = (code - 0xf808);

				if(ix >= display_colours)
					log_format("display-spi-generic: background colour out of range: %u", ix);
				else
					background_pixel = display_colour_map[ix];
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

				if(glyph && (col < y_size))
				{
					for(bit = 0; bit < font->net.width; bit++)
					{
						for(row = 0; row < font->net.height; row++)
						{
							if(glyph->row[row] & (1 << bit))
								send_pixel(&foreground_pixel);
							else
								send_pixel(&background_pixel);
						}

						col++;
					}
				}
			}
		}
	}

	for(; col < y_size; col++)
		for(row = 0; row < font->net.height; row++)
			send_pixel(&background_pixel);

	send_pixel((const display_rgb_t *)0); // flush
}