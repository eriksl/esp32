#include <stdint.h>
#include <stdbool.h>
#include <sdkconfig.h>

#include "notify.h"

#if defined(CONFIG_BSP_LED_HAVE_LEDPIXEL)
#include "string.h"
#include "log.h"
#include "util.h"
#include "ledpixel.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

typedef struct
{
	unsigned int r;
	unsigned int g;
	unsigned int b;
} state_colour_t;

static bool inited = false;
static ledpixel_t ledpixel;
static state_colour_t state_colour[notify_state_size];
static unsigned int state;
static TimerHandle_t state_timer;

static void timer_handler(struct tmrTimerControl *)
{
	ledpixel_set(ledpixel, 0, state_colour[state].g, state_colour[state].r, state_colour[state].b);
	ledpixel_flush(ledpixel);

	xTimerStart(state_timer, 0);

	state = (state + 1) % notify_state_size;
}

void notify_init(void)
{
	unsigned int colour;

	assert(!inited);

	ledpixel = ledpixel_new(1, CONFIG_BSP_LED_GPIO);

	for(state = 0; state < notify_state_size; state++)
	{
		for(colour = 0; colour < 3; colour++)
		{
			state_colour[state].r = 0;
			state_colour[state].g = 0;
			state_colour[state].b = 0;
		}
	}

	state = 0;

	ledpixel_set(ledpixel, 0, 0x00, 0x00, 0x00);
	ledpixel_flush(ledpixel);

	inited = true;

	state_timer = xTimerCreate("notify-state", pdMS_TO_TICKS(500), pdFALSE, (void *)0, timer_handler);
	assert(state_timer);

	xTimerStart(state_timer, 0);
}

void notify(unsigned int new_state, unsigned int r, unsigned g, unsigned int b)
{
	assert(new_state < notify_state_size);

	state = new_state;

	state_colour[state].r = r;
	state_colour[state].g = g;
	state_colour[state].b = b;

	timer_handler((struct tmrTimerControl *)0);
}
#else
void notify_init(void)
{
}

void notify(unsigned int new_state, unsigned int r, unsigned g, unsigned int b)
{
}
#endif
