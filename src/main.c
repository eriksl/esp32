#include "main.h"
#include "util.h"

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <esp_system.h>
#include <esp_log.h>
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
	ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_string_config, &led_strip_rmt_config, &led_strip_handle));
	led_strip_clear(led_strip_handle);

	led_strip_set_pixel(led_strip_handle, 0, 0x00, 0x00, 0xff);
	led_strip_refresh(led_strip_handle);

	esp_chip_info_t chip_info;
	uint32_t flash_size;
	esp_chip_info(&chip_info);
	printf("This is a %s chip with %d CPU core(s), %s%s%s%s, ",
		CONFIG_IDF_TARGET,
		chip_info.cores,
		(chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
		(chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
		(chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
		(chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

	unsigned major_rev = chip_info.revision / 100;
	unsigned minor_rev = chip_info.revision % 100;
	printf("silicon revision v%d.%d, ", major_rev, minor_rev);
	if(esp_flash_get_size(NULL, &flash_size) != ESP_OK)
	{
		printf("Get flash size failed");
		return;
	}

	printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
		(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	printf("Free heap size: %" PRIu32 " bytes\n", esp_get_free_heap_size());
	printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

	printf("heap exec: %u\n", heap_caps_get_free_size(MALLOC_CAP_EXEC) / 1024);
	printf("heap 32bit: %u\n", heap_caps_get_free_size(MALLOC_CAP_32BIT) / 1024);
	printf("heap 8bit: %u\n", heap_caps_get_free_size(MALLOC_CAP_8BIT) / 1024);
	printf("heap dma: %u\n", heap_caps_get_free_size(MALLOC_CAP_DMA) / 1024);
	printf("heap spiram: %u\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024);
	printf("heap internal: %u\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024);
	printf("heap default: %u\n", heap_caps_get_free_size(MALLOC_CAP_DEFAULT) / 1024);
	printf("heap iram 8 bit: %u\n", heap_caps_get_free_size(MALLOC_CAP_IRAM_8BIT) / 1024);
	printf("heap retention: %u\n", heap_caps_get_free_size(MALLOC_CAP_RETENTION) / 1024);
	printf("heap rtcram: %u\n", heap_caps_get_free_size(MALLOC_CAP_RTCRAM) / 1024);
	printf("heap tcm: %u\n", heap_caps_get_free_size(MALLOC_CAP_TCM) / 1024);

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
