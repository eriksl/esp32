#include <stdint.h>
#include <stdbool.h>

#include "string.h"
#include "cli-command.h"
#include "ledpixel.h"
#include "log.h"
#include "util.h"

#include <led_strip.h>

typedef struct
{
	led_strip_handle_t		handle;
	led_strip_config_t 		config;
	led_strip_rmt_config_t 	rmt_config;
} _ledpixel_t;

static bool inited = false;

void ledpixel_init(void)
{
	assert(!inited);

	inited = true;
}

ledpixel_t ledpixel_new(unsigned int max_leds, unsigned int gpio)
{
	_ledpixel_t *_ledpixel = util_memory_alloc_spiram(sizeof(_ledpixel_t));

	_ledpixel->config.strip_gpio_num = gpio;
	_ledpixel->config.max_leds = max_leds;
	_ledpixel->config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
	_ledpixel->config.led_model = LED_MODEL_WS2812;
	_ledpixel->config.flags.invert_out = 0;

	_ledpixel->rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
	_ledpixel->rmt_config.resolution_hz = 0;
	_ledpixel->rmt_config.mem_block_symbols = 0;
	_ledpixel->rmt_config.flags.with_dma = 1;

	util_abort_on_esp_err("led_strip_new_rmt_device", led_strip_new_rmt_device(&_ledpixel->config, &_ledpixel->rmt_config, &_ledpixel->handle));
	util_abort_on_esp_err("led_strip_clear", led_strip_clear(_ledpixel->handle));

	return((ledpixel_t)_ledpixel);
}

void ledpixel_set(ledpixel_t ledpixel, unsigned int index, unsigned int red, unsigned int green, unsigned int blue)
{
	_ledpixel_t *_ledpixel = (_ledpixel_t *)ledpixel;

	util_abort_on_esp_err("led_string_set_pixel", led_strip_set_pixel(_ledpixel->handle, index, red, green, blue));
}

void ledpixel_flush(ledpixel_t ledpixel)
{
	_ledpixel_t *_ledpixel = (_ledpixel_t *)ledpixel;

	util_abort_on_esp_err("led_strip_refresh", led_strip_refresh(_ledpixel->handle));
}
