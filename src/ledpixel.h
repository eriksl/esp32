#pragma once

typedef void ledpixel_t;

ledpixel_t *ledpixel_new(unsigned int max_leds, unsigned int gpio);
void ledpixel_set(ledpixel_t *ledpixel, unsigned int index, unsigned int r, unsigned int g, unsigned int b);
void ledpixel_flush(ledpixel_t *ledpixel);
