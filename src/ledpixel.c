#include "ledpixel.h"

#include <led_strip.h>

#include <stdint.h>
#include <stdbool.h>

#include "util.h"

static bool inited = false;

void ledpixel_init(void)
{
	assert(!inited);

	inited = true;
}

void ledpixel_new(ledpixel_t *ledpixel, unsigned int max_leds, unsigned int gpio)
{
	ledpixel->config.strip_gpio_num = gpio;
	ledpixel->config.max_leds = max_leds;
	ledpixel->config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
	ledpixel->config.led_model = LED_MODEL_WS2812;
	ledpixel->config.flags.invert_out = 0;

	ledpixel->rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
	ledpixel->rmt_config.resolution_hz = 0;
	ledpixel->rmt_config.mem_block_symbols = 0;
	ledpixel->rmt_config.flags.with_dma = 1;

	util_abort_on_esp_err("led_strip_new_rmt_device", led_strip_new_rmt_device(&ledpixel->config, &ledpixel->rmt_config, &ledpixel->handle));
	util_abort_on_esp_err("led_strip_clear", led_strip_clear(ledpixel->handle));
}

void ledpixel_set(ledpixel_t *ledpixel, unsigned int index, unsigned int red, unsigned int green, unsigned int blue)
{
	util_abort_on_esp_err("led_string_set_pixel", led_strip_set_pixel(ledpixel->handle, index, red, green, blue));
}

void ledpixel_flush(ledpixel_t *ledpixel)
{
	util_abort_on_esp_err("led_strip_refresh", led_strip_refresh(ledpixel->handle));
}
