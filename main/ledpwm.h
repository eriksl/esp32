#pragma once

struct pwm_led_opaque_t {} __attribute__((aligned(sizeof(int))));

typedef struct pwm_led_opaque_t *pwm_led_t;
typedef const struct pwm_led_opaque_t *const_pwm_led_t;

typedef enum
{
	plt_first = 0,
	plt_14bit_5khz = plt_first,
	plt_14bit_120hz,
	plt_size,
	plt_error = plt_size,
} pwm_led_type_t;

void pwm_led_init(void);
pwm_led_t pwm_led_channel_new(unsigned int gpio, pwm_led_type_t type, const char *name);
void pwm_led_channel_set(pwm_led_t channel, unsigned int duty);
unsigned int pwm_led_channel_get(const const_pwm_led_t channel);
