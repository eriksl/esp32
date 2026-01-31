#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <esp_log.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
#include <sdkconfig.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "display.h"
#include "info.h"

#include <string>
#include <boost/format.hpp>

static bool inited = false;

unsigned int initial_free_heap;
unsigned int initial_free_spiram;
unsigned int initial_free_internal;
unsigned int initial_free_total;
unsigned int initial_free_rtcram;

unsigned int stat_notify_timer_failed;

void info_command_info(cli_command_call_t *call)
{
	const esp_app_desc_t *desc;
	esp_chip_info_t chip_info;
	uint32_t flash_size;

	assert(inited);
	assert(call->parameter_count == 0);

	if(!(desc = esp_app_get_description()))
	{
		call->result = "ERROR: esp_app_get_description failed";
		return;
	}

	esp_chip_info(&chip_info);

	call->result = (boost::format("SoC: %s with %d cores\nRF: %s%s%s%s") %
		CONFIG_IDF_TARGET %
		chip_info.cores %
		((chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "") %
		((chip_info.features & CHIP_FEATURE_BT) ? "BT" : "") %
		((chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "") %
		((chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "")).str();

	unsigned major_rev = chip_info.revision / 100;
	unsigned minor_rev = chip_info.revision % 100;
	call->result += (boost::format("\nRevision: %u.%u") % major_rev % minor_rev).str();

	esp_flash_get_size(NULL, &flash_size);

	call->result += (boost::format("\nFlash: %lu MB %s") %
		(flash_size / (1024 * 1024)) %
		((chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external")).str();

	call->result += (boost::format("\nBSP:\n- board name: %s\n- flash size: %d MB\n- SPI RAM size: %d MB") %
			CONFIG_BSP_BOARD_TYPE_NAME %
			(CONFIG_BSP_FLASH_SIZE / 1024) %
			(CONFIG_BSP_SPIRAM_SIZE / 1024)).str();
#if (CONFIG_BSP_LEDPIXEL0 >= 0)
	call->result += (boost::format("\n- LEDpixel at GPIO %d") % CONFIG_BSP_LEDPIXEL0).str();
#else
	call->result += "\n- no LEDpixel";
#endif
#if (CONFIG_BSP_LEDPWM0 >= 0)
	call->result += (boost::format("\n- status LED at GPIO %d") % CONFIG_BSP_LEDPWM0).str();
#else
	call->result += "\n- no status LED";
#endif

	call->result += (boost::format("\nfirmware\n- date: %s %s\n- build start: %s %s\n") %
			__DATE__ %
			__TIME__ %
			desc->date %
			desc->time).str();

	call->result += (boost::format("stats:\n- notify timer failed: %u") % stat_notify_timer_failed).str();
}

void info_command_info_board(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 0);

	call->result = (boost::format("firmware date: %s %s, ") % __DATE__ % __TIME__).str();
	call->result += (boost::format("transport mtu: %u, ") % call->mtu).str();
    call->result += (boost::format("display area: %ux%u") % display_image_x_size() % display_image_y_size()).str();
}

void info_command_info_partitions(cli_command_call_t *call)
{
	int rv;
	esp_partition_iterator_t partition_iterator;
	const esp_partition_t *partition;
	const char *type, *subtype;
	unsigned int index;
	const esp_partition_t *boot_partition, *running_partition;
	esp_ota_img_states_t ota_state;
	const char *ota_state_text;
	std::string sha256_hash;
	std::string sha256_hash_text;

	assert(inited);
	assert(call->parameter_count == 0);

	if(!(boot_partition = esp_ota_get_boot_partition()))
	{
		call->result = "ERROR: esp_ota_get_boot_partition failed";
		return;
	}

	if(!(running_partition = esp_ota_get_running_partition()))
	{
		call->result = "ERROR: esp_ota_get_running_partition failed";
		return;
	}

	if(!(partition_iterator = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, (const char *)0)))
	{
		call->result = "ERROR: esp_partition_find failed";
		return;
	}

	call->result = "Partitions:\n";

	for(index = 0; partition_iterator; index++, partition_iterator = esp_partition_next(partition_iterator))
	{
		if(!(partition = esp_partition_get(partition_iterator)))
		{
			call->result = "ERROR: esp_partition_get failed";
			return;
		}

		switch(partition->type)
		{
			case(ESP_PARTITION_TYPE_APP):
			{
				type = "app";
				break;
			}

			case(ESP_PARTITION_TYPE_DATA):
			{
				type = "data";
				break;
			}

			default:
			{
				type = "unknown";
				break;
			}
		}

		switch(partition->subtype)
		{
			case(ESP_PARTITION_SUBTYPE_APP_FACTORY):
			//case(ESP_PARTITION_SUBTYPE_DATA_OTA): // these are using the same value
			{
				if(partition->type == ESP_PARTITION_TYPE_APP)
					subtype = "factory";
				else
					subtype = "ota data";

				break;
			}

			case(ESP_PARTITION_SUBTYPE_APP_OTA_0):
			{
				subtype = "ota 0";
				break;
			}

			case(ESP_PARTITION_SUBTYPE_APP_OTA_1):
			{
				subtype = "ota 1";
				break;
			}

			case(ESP_PARTITION_SUBTYPE_DATA_NVS):
			{
				subtype = "nvs";
				break;
			}

			default:
			{
				subtype = "unknown";
				break;
			}
		}

		ota_state_text = "";

		if((partition->type == ESP_PARTITION_TYPE_APP) &&
			((partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) || (partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1)))
		{
			if(!esp_ota_get_state_partition(partition, &ota_state))
			{
				switch(ota_state)
				{
					case(ESP_OTA_IMG_NEW):				{ ota_state_text = "N"; break; }
					case(ESP_OTA_IMG_PENDING_VERIFY):	{ ota_state_text = "P"; break; }
					case(ESP_OTA_IMG_VALID):			{ ota_state_text = "V"; break; }
					case(ESP_OTA_IMG_INVALID):			{ ota_state_text = "I"; break; }
					case(ESP_OTA_IMG_ABORTED):			{ ota_state_text = "A"; break; }
					case(ESP_OTA_IMG_UNDEFINED):		{ ota_state_text = "U"; break; }
					default:							{ ota_state_text = "?"; break; }
				}
			}
			else
				ota_state_text = "X";
		}

		sha256_hash.resize(32);

		if((rv = esp_partition_get_sha256(partition, reinterpret_cast<uint8_t *>(sha256_hash.data()))))
			sha256_hash_text = "<invalid>";
		else
			sha256_hash_text = util_hash_to_string(sha256_hash);

		call->result += (boost::format("%s  %2u %1s%1s%1s %-8s %06lx %4lu %-7s %-8s %-64s") %
				((index > 0) ? "\n" : "") %
				index %
				ota_state_text %
				((partition->address == boot_partition->address) ? "b" : " ") %
				((partition->address == running_partition->address) ? "r" : " ") %
				partition->label %
				partition->address %
				(partition->size / 1024) %
				type %
				subtype %
				sha256_hash_text).str();
	}

	esp_partition_iterator_release(partition_iterator);
}

void info_command_info_memory(cli_command_call_t *call)
{
	unsigned int free_total;
	assert(inited);
	assert(call->parameter_count == 0);

	free_total = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);

	call->result = "MEMORY";
	call->result += "\namount kB:";
	call->result += (boost::format("\n- %-29s %5lu / %5u kB")	% "free heap total"				% (esp_get_free_heap_size() / 1024)	% (initial_free_heap / 1024)).str();
	call->result += (boost::format("\n- %-29s %5lu kB")			% "minimum free heap"			% (esp_get_minimum_free_heap_size() / 1024)).str();
	call->result += (boost::format("\n- %-29s %5u kB")			% "heap executable"				% (heap_caps_get_free_size(MALLOC_CAP_EXEC) / 1024)).str();
	call->result += (boost::format("\n- %-29s %5u kB")			% "heap 32 bit addressable"		% (heap_caps_get_free_size(MALLOC_CAP_32BIT) / 1024)).str();
	call->result += (boost::format("\n- %-29s %5u kB")			% "heap 8 bit addressable"		% (heap_caps_get_free_size(MALLOC_CAP_8BIT) / 1024)).str();
	call->result += (boost::format("\n- %-29s %5u kB")			% "heap DMA adressable"			% (heap_caps_get_free_size(MALLOC_CAP_DMA) / 1024)).str();
	call->result += (boost::format("\n- %-29s %5u / %5u kB")	% "heap SPI RAM"				% (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024)	% (initial_free_spiram / 1024)).str();
	call->result += (boost::format("\n- %-29s %5u / %5u kB")	% "heap internal RAM"			% (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024)	% (initial_free_internal / 1024)).str();
	call->result += (boost::format("\n- %-29s %5u / %5u kB")	% "heap default"				% (free_total / 1024)									% (initial_free_total / 1024)).str();
	call->result += (boost::format("\n- %-29s %5u kB")			% "heap IRAM 8 bit adressable"	% (heap_caps_get_free_size(MALLOC_CAP_IRAM_8BIT) / 1024)).str();
	call->result += (boost::format("\n- %-29s %5u kB")			% "heap retention"				% (heap_caps_get_free_size(MALLOC_CAP_RETENTION) / 1024)).str();
	call->result += (boost::format("\n- %-29s %5u / %5u kB")	% "heap RTC RAM"				% (heap_caps_get_free_size(MALLOC_CAP_RTCRAM) / 1024)	% (initial_free_rtcram / 1024)).str();
	call->result += (boost::format("\n- %-29s %5u kB")			% "heap TCM"					% (heap_caps_get_free_size(MALLOC_CAP_TCM) / 1024)).str();
	call->result += "\nusage:";
	call->result += (boost::format("\n- total: %6.3f MB")			% (initial_free_total / (1024.0 * 1024.0))).str();
	call->result += (boost::format("\n- used:  %6.3f MB %4.1f%%")	% ((initial_free_total - free_total) / (1024.0 * 1024.0)) % ((100.0 * (initial_free_total - free_total)) / initial_free_total)).str();
	call->result += (boost::format("\n- free:  %6.3f MB %4.1f%%")	% (free_total / (1024.0 * 1024.0)) % ((100.0 * free_total) / initial_free_total)).str();
}

void info_init(void)
{
	assert(!inited);

	initial_free_heap = esp_get_free_heap_size();
	initial_free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
	initial_free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
	initial_free_total = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
	initial_free_rtcram = heap_caps_get_free_size(MALLOC_CAP_RTCRAM);

	inited = true;
}
