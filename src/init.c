#include "cli.h"
#include "bt.h"
#include "main.h"
#include "util.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs.h>
#include <nvs_flash.h>

void app_main(void)
{
	esp_err_t rv;

	rv = nvs_flash_init();
	ESP_ERROR_CHECK(rv);

	if((rv == ESP_ERR_NVS_NO_FREE_PAGES) || (rv == ESP_ERR_NVS_NEW_VERSION_FOUND))
	{
		ESP_LOGW("init", "erase and reinit flash");
		ESP_ERROR_CHECK(nvs_flash_erase());
		ESP_ERROR_CHECK(nvs_flash_init());
	}

	util_init();
	cli_init();
	bt_init();

	run_main();
}
