#include "cli.h"
#include "bt.h"
#include "main.h"
#include "log.h"
#include "console.h"
#include "util.h"

#include <esp_err.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs.h>
#include <nvs_flash.h>

void app_main(void)
{
	esp_err_t rv;

	rv = nvs_flash_init();

	if((rv == ESP_ERR_NVS_NO_FREE_PAGES) || (rv == ESP_ERR_NVS_NEW_VERSION_FOUND))
	{
		log("init: erase and reinit flash");
		util_abort_on_esp_err("nvs_flash_erase", nvs_flash_erase());
		util_abort_on_esp_err("nvs_flash_init", nvs_flash_init());
	}
	else
		util_abort_on_esp_err("nvs_flash_init", rv);

	log_init();
	util_init();
	cli_init();
	bt_init();
	console_init();

	run_main();
}
