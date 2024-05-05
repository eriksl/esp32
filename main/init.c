#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs.h>
#include <nvs_flash.h>

#include "bt.h"
#include "cli.h"
#include "main.h"

void app_main(void)
{
	esp_err_t rv;

	ESP_LOGI("init", "stack 1: %d", uxTaskGetStackHighWaterMark(0));

	rv = nvs_flash_init();
	ESP_ERROR_CHECK(rv);

	if((rv == ESP_ERR_NVS_NO_FREE_PAGES) || (rv == ESP_ERR_NVS_NEW_VERSION_FOUND))
	{
		ESP_LOGE("init", "erase and reinit flash");
		ESP_ERROR_CHECK(nvs_flash_erase());
		ESP_ERROR_CHECK(nvs_flash_init());
	}

	cli_init();
	bt_init();

	ESP_LOGI("init", "stack 2: %d", uxTaskGetStackHighWaterMark(0));

	run_main();
}
