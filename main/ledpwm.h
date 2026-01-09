#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
	lpt_14bit_5khz_notify = 0,
	lpt_first = lpt_14bit_5khz_notify,
	lpt_14bit_5khz_lcd_spi_2,
	lpt_14bit_5khz_lcd_spi_3,
	lpt_14bit_120hz,
	lpt_size,
	lpt_error = lpt_size,
} ledpwm_t;

bool ledpwm_open(ledpwm_t handle, const char *owner);
void ledpwm_set(ledpwm_t handle, unsigned int duty);
unsigned int ledpwm_get(ledpwm_t handle);

#ifdef __cplusplus
}
#endif

void ledpwm_init(void);
