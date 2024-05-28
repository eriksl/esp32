#include <stdint.h>

#include "string.h"
#include "cli-command.h"
#include "main.h"
#include "ledpixel.h"
#include "log.h"
#include "util.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void run_main(void)
{
	ledpixel_t ledpixel;

	ledpixel = ledpixel_new(1, 47);
	ledpixel_set(ledpixel, 0, 0x00, 0x00, 0xff);
	ledpixel_flush(ledpixel);

	vTaskDelay(200 / portTICK_PERIOD_MS);

	ledpixel_set(ledpixel, 0, 0x00, 0xff, 0x00);
	ledpixel_flush(ledpixel);

	for(;;)
	{
		vTaskDelay(500 / portTICK_PERIOD_MS);
		ledpixel_set(ledpixel, 0, 0xff, 0x00, 0x00);
		ledpixel_flush(ledpixel);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		ledpixel_set(ledpixel, 0, 0x00, 0x00, 0xff);
		ledpixel_flush(ledpixel);
	}
}
