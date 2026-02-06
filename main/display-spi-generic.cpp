#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <driver/spi_master.h>
#include <driver/gpio.h>

#include "log.h"
#include "util.h"
#include "ledpwm.h"
#include "sdkconfig.h"

#include "display-spi-generic.h"

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
};

typedef struct
{
	unsigned int esp_host;
	unsigned int cs;
	unsigned int sck;
	unsigned int mosi;
	unsigned int miso;
	unsigned int dc;
	LedPWM::Channel bl;
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
		.bl =		LedPWM::Channel::channel_14bit_5khz_lcd_spi_2,
	},
	.spi3 =
	{
		.esp_host =	SPI3_HOST,
		.cs =		CONFIG_BSP_SPI3_DISPLAY_CS,
		.sck =		CONFIG_BSP_SPI3_SCK,
		.mosi =		CONFIG_BSP_SPI3_MOSI,
		.miso =		CONFIG_BSP_SPI3_MISO,
		.dc =		CONFIG_BSP_SPI3_DISPLAY_DC,
		.bl =		LedPWM::Channel::channel_14bit_5khz_lcd_spi_3,
	},
};

static bool inited = false;
static spi_device_handle_t spi_device_handle;
static const spi_signal_t *spi_signal;
static callback_data_t callback_data_gpio_on;
static callback_data_t callback_data_gpio_off;
static SemaphoreHandle_t spi_mutex;
static unsigned int spi_pending;
static LedPWM::Channel ledpwm_channel;
static unsigned int x_size, y_size;
static unsigned int flip;
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
	send_command_data(true, cmd, 0, nullptr);
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
		const std::deque<uint32_t> &unicode_line)
{
	std::deque<uint32_t>::const_iterator unicode_it;
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

	for(unicode_it = unicode_line.begin(); unicode_it != unicode_line.end(); unicode_it++)
	{
		if((*unicode_it >= 0xf800) && (*unicode_it < 0xf808)) // abuse private use unicode codepoints for foreground colours
		{
			ix = (*unicode_it - 0xf800);

			if(ix >= dc_size)
				log_format("display-spi-generic: foreground colour out of range: %u", ix);
			else
				fg_rgb = display_colour_map[ix];
		}
		else
		{
			if((*unicode_it >= 0xf808) && (*unicode_it < 0xf810)) // abuse private use unicode codepoints for background colours
			{
				ix = (*unicode_it - 0xf808);

				if(ix >= dc_size)
					log_format("display-spi-generic: background colour out of range: %u", ix);
				else
					bg_rgb = display_colour_map[ix];
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

static void pre_callback(spi_transaction_t *transaction)
{
	assert(inited);
	assert(transaction);
	assert(transaction->user);

	const callback_data_t *callback_data = (const callback_data_t *)transaction->user;

	gpio_set_level(static_cast<gpio_num_t>(callback_data->gpio), callback_data->level);
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
			log_format("init display-lcd-spi: unknown spi interface %d, use 0 for SPI2 or 1 for SPI3", parameters->interface_index);
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

	flip = 0;
	invert = 0;
	rotate = 0;

	if(parameters->flip >= 0)
		flip = parameters->flip;

	if(parameters->invert >= 0)
		invert = parameters->invert;

	if(parameters->rotate >= 0)
		rotate = parameters->rotate;

	callback_data_gpio_on.gpio = spi_signal->dc;
	callback_data_gpio_on.level = 1;
	callback_data_gpio_off.gpio = spi_signal->dc;
	callback_data_gpio_off.level = 0;

	gpio_config_t gpio_pin_config =
	{
		.pin_bit_mask = (1ULL << spi_signal->dc),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

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
		.isr_cpu_id = static_cast<esp_intr_cpu_affinity_t>(1),
		.intr_flags = 0,
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
		.sample_point = static_cast<spi_sampling_point_t>(0),
		.spics_io_num = static_cast<int>(spi_signal->cs),
		.flags = 0,
		.queue_size = 1,
		.pre_cb = pre_callback,
		.post_cb = (transaction_cb_t)0,
	};

	spi_mutex = xSemaphoreCreateMutex();
	assert(spi_mutex);
	spi_pending = 0;

	util_abort_on_esp_err("gpio_config", gpio_config(&gpio_pin_config));

	util_abort_on_esp_err("spi_bus_initialize", spi_bus_initialize(static_cast<spi_host_device_t>(spi_signal->esp_host), &bus_config, SPI_DMA_CH_AUTO));
	util_abort_on_esp_err("spi_bus_add_device", spi_bus_add_device(static_cast<spi_host_device_t>(spi_signal->esp_host), &device, &spi_device_handle));

	util_abort_on_esp_err("spi_bus_get_max_transaction_len", spi_bus_get_max_transaction_len(static_cast<spi_host_device_t>(spi_signal->esp_host), &max_transaction_length));
	pixel_buffer_size = max_transaction_length;
	pixel_buffer = static_cast<uint8_t *>(heap_caps_malloc(pixel_buffer_size, MALLOC_CAP_DMA));
	pixel_buffer_rgb = (display_rgb_t *)pixel_buffer;
	pixel_buffer_rgb_size = pixel_buffer_size / sizeof(display_rgb_t);
	pixel_buffer_rgb_length = 0;

	ledpwm_channel = spi_signal->bl;
	LedPWM::get().open(ledpwm_channel, "backlight generic SPI LCD");

	inited = true;

	send_command(cmd_dispoff);
	send_command(cmd_swreset);
	util_sleep(400);
	send_command(cmd_sleepout);
	util_sleep(100);
	send_command(cmd_noron);
	send_command(cmd_dispon);
	send_command_data_1(cmd_colmod, colmod_18bpp);
	send_command(invert ? cmd_invon : cmd_invoff);

	madctl = madctl_bgr;

	if(rotate)
		madctl |= madctl_mv;

	if(!flip)
		madctl |= madctl_my | madctl_mx;

	send_command_data_1(cmd_madctl, madctl);

	return(true);
}

void display_spi_generic_bright(unsigned int percentage)
{
	assert(percentage <= 100);

	LedPWM::get().set(ledpwm_channel, ((1UL << 14) * (100 - percentage)) / 100);
}
