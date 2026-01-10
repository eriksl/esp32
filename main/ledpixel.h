#pragma once

enum
{
	ledpixel_leds_size = 4
};

typedef enum
{
	lp_0_notify = 0,
	lp_first = lp_0_notify,
	lp_1,
	lp_2,
	lp_3,
	lp_size,
	lp_error = lp_size,
} lp_t;

bool ledpixel_open(lp_t handle, const char *owner);
void ledpixel_set(lp_t handle, unsigned int index, unsigned int red, unsigned int green, unsigned int blue);
void ledpixel_get(lp_t handle, unsigned int index, unsigned int *red, unsigned int *green, unsigned int *blue);
void ledpixel_flush(lp_t handle);

void ledpixel_init(void);
