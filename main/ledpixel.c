#include <stdint.h>
#include <stdbool.h>
#include <sdkconfig.h>

#if defined(CONFIG_BSP_LED_HAVE_LEDPIXEL)

#include "ledpixel.h"
#include "string.h"
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

bool ledpixel_init(void)
{
	if(inited)
		return(false);

	inited = true;

	return(true);
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
	_ledpixel->rmt_config.flags.with_dma = 0;

	if(led_strip_new_rmt_device(&_ledpixel->config, &_ledpixel->rmt_config, &_ledpixel->handle) != ESP_OK)
		return((ledpixel_t)0);

	if(led_strip_clear(_ledpixel->handle) != ESP_OK)
		return((ledpixel_t)0);

	return((ledpixel_t)_ledpixel);
}

bool ledpixel_set(ledpixel_t ledpixel, unsigned int index, unsigned int red, unsigned int green, unsigned int blue)
{
	_ledpixel_t *_ledpixel = (_ledpixel_t *)ledpixel;

	if(!_ledpixel)
		return(false);

	return(led_strip_set_pixel(_ledpixel->handle, index, red, green, blue) == ESP_OK);
}

bool ledpixel_flush(ledpixel_t ledpixel)
{
	_ledpixel_t *_ledpixel = (_ledpixel_t *)ledpixel;

	if(!_ledpixel)
		return(false);

	return(led_strip_refresh(_ledpixel->handle) == ESP_OK);
}

#endif
