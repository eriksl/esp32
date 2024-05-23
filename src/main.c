#include "main.h"
#include "util.h"

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <led_strip.h>

static const led_strip_config_t led_string_config =
{
	.strip_gpio_num = 47,
	.max_leds = 1,
};

static const led_strip_rmt_config_t led_strip_rmt_config =
{
	.flags.with_dma = true,
};

static led_strip_handle_t led_strip_handle;

void run_main(void)
{
	util_abort_on_esp_err("led_strip_new_rmt_device", led_strip_new_rmt_device(&led_string_config, &led_strip_rmt_config, &led_strip_handle));
	led_strip_clear(led_strip_handle);

	led_strip_set_pixel(led_strip_handle, 0, 0x00, 0x00, 0xff);
	led_strip_refresh(led_strip_handle);

	vTaskDelay(200 / portTICK_PERIOD_MS);

	led_strip_set_pixel(led_strip_handle, 0, 0x00, 0xff, 0x00);
	led_strip_refresh(led_strip_handle);

	for(;;)
	{
		vTaskDelay(500 / portTICK_PERIOD_MS);
		led_strip_set_pixel(led_strip_handle, 0, 0xff, 0x00, 0x00);
		led_strip_refresh(led_strip_handle);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		led_strip_set_pixel(led_strip_handle, 0, 0x00, 0x00, 0xff);
		led_strip_refresh(led_strip_handle);
	}
}
