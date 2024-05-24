#include "config.h"
#include "log.h"
#include "util.h"

#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>

#include <stdbool.h>

static bool inited = false;

void config_init(void)
{
	esp_err_t rv;

	assert(!inited);

	rv = nvs_flash_init();

	if((rv == ESP_ERR_NVS_NO_FREE_PAGES) || (rv == ESP_ERR_NVS_NEW_VERSION_FOUND))
	{
		log("init: erase and reinit flash");
		util_abort_on_esp_err("nvs_flash_erase", nvs_flash_erase());
		util_abort_on_esp_err("nvs_flash_init", nvs_flash_init());
	}
	else
		util_abort_on_esp_err("nvs_flash_init", rv);

	inited = true;
}
