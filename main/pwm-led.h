#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
	plt_start = 0,
	plt_14bit_5khz = plt_start,
	plt_14bit_120hz,
	plt_size,
	plt_error = plt_size,
} pwm_led_type_t;

bool pwm_led_init(void);
int pwm_led_channel_new(unsigned int gpio, pwm_led_type_t type);
bool pwm_led_channel_set(unsigned int channel, unsigned int duty);
int pwm_led_channel_get(unsigned int channel);
