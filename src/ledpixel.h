#ifndef _ledpixel_h_
#define _ledpixel_h_

#include <led_strip.h>

typedef struct
{
	led_strip_handle_t		handle;
	led_strip_config_t 		config;
	led_strip_rmt_config_t 	rmt_config;
} ledpixel_t;

void ledpixel_new(ledpixel_t *ledpixel, unsigned int max_leds, unsigned int gpio);
void ledpixel_set(ledpixel_t *ledpixel, unsigned int index, unsigned int r, unsigned int g, unsigned int b);
void ledpixel_flush(ledpixel_t *ledpixel);

#endif
