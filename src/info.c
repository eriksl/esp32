#include <stdint.h>
#include <stdbool.h>

#include <esp_log.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_heap_caps.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "string.h"
#include "cli-command.h"
#include "info.h"
#include "log.h"
#include "util.h"

static bool inited = false;
static unsigned int initial_free_heap;
static unsigned int initial_free_spiram;
static unsigned int initial_free_internal;
static unsigned int initial_free_total;
static unsigned int initial_free_rtcram;

void info_command_info(cli_command_call_t *call)
{
	const esp_app_desc_t *desc;
	esp_chip_info_t chip_info;
	uint32_t flash_size;

	assert(inited);
	assert(call->parameter_count == 0);

	if(!(desc = esp_app_get_description()))
	{
		string_format(call->result, "ERROR: esp_app_get_description failed");
		return;
	}

	string_format(call->result,
			"> firmware\n"
			">   date: %s %s\n"
			">   build start: %s %s\n",
			__DATE__, __TIME__, desc->date, desc->time);

	esp_chip_info(&chip_info);

	string_format_append(call->result, "\nSoC: %s with %d cores\nRF: %s%s%s%s\n",
		CONFIG_IDF_TARGET,
		chip_info.cores,
		(chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
		(chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
		(chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
		(chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

	unsigned major_rev = chip_info.revision / 100;
	unsigned minor_rev = chip_info.revision % 100;
	string_format_append(call->result, "Revision: %d.%d\n", major_rev, minor_rev);

	esp_flash_get_size(NULL, &flash_size);

	string_format_append(call->result, "Flash: %lu MB %s\n", flash_size / (1024 * 1024),
		(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
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
	unsigned char sha256_hash[32];
	string_auto(sha256_hash_text, (sizeof(sha256_hash) * 2) + 1);

	assert(inited);
	assert(call->parameter_count == 0);

	if(!(boot_partition = esp_ota_get_boot_partition()))
	{
		string_format(call->result, "ERROR: esp_ota_get_boot_partition failed");
		return;
	}

	if(!(running_partition = esp_ota_get_running_partition()))
	{
		string_format(call->result, "ERROR: esp_ota_get_running_partition failed");
		return;
	}

	if(!(partition_iterator = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, (const char *)0)))
	{
		string_format(call->result, "ERROR: esp_partition_find failed");
		return;
	}

	string_format(call->result, "Partitions:\n");

	for(index = 0; partition_iterator; index++, partition_iterator = esp_partition_next(partition_iterator))
	{
		if(!(partition = esp_partition_get(partition_iterator)))
		{
			string_format(call->result, "ERROR: esp_partition_get failed");
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

		if((rv = esp_partition_get_sha256(partition, sha256_hash)))
			string_assign_cstr(sha256_hash_text, "<invalid>");
		else
			util_hash_to_string(sha256_hash_text, sizeof(sha256_hash), sha256_hash);

		string_format_append(call->result, "%s  %2u %1s%1s%1s %-8s %06lx %4lu %-7s %-8s %-64s",
				(index > 0) ? "\n" : "",
				index,
				ota_state_text,
				(partition->address == boot_partition->address) ? "b" : " ",
				(partition->address == running_partition->address) ? "r" : " ",
				partition->label, partition->address, partition->size / 1024, type, subtype, string_cstr(sha256_hash_text));
	}

	esp_partition_iterator_release(partition_iterator);
}

void info_command_info_memory(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 0);

	string_format(call->result, "MEMORY");
	string_format_append(call->result, "\n%-30s %-4s", "amount:", "kB");
	string_format_append(call->result, "\n- %-28s %4lu / %4u",	"free heap total",				esp_get_free_heap_size() / 1024, initial_free_heap  / 1024);
	string_format_append(call->result, "\n- %-28s %4lu",		"minimum free heap",			esp_get_minimum_free_heap_size() / 1024);
	string_format_append(call->result, "\n- %-28s %4u",  		"heap executable",				heap_caps_get_free_size(MALLOC_CAP_EXEC) / 1024);
	string_format_append(call->result, "\n- %-28s %4u",  		"heap 32 bit addressable",		heap_caps_get_free_size(MALLOC_CAP_32BIT) / 1024);
	string_format_append(call->result, "\n- %-28s %4u",  		"heap 8 bit addressable",		heap_caps_get_free_size(MALLOC_CAP_8BIT) / 1024);
	string_format_append(call->result, "\n- %-28s %4u",  		"heap DMA adressable",			heap_caps_get_free_size(MALLOC_CAP_DMA) / 1024);
	string_format_append(call->result, "\n- %-28s %4u / %4u",	"heap SPI RAM",					heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024, initial_free_spiram / 1024);
	string_format_append(call->result, "\n- %-28s %4u / %4u",	"heap internal RAM",			heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024, initial_free_internal / 1024);
	string_format_append(call->result, "\n- %-28s %4u / %4u",	"heap default",					heap_caps_get_free_size(MALLOC_CAP_DEFAULT) / 1024, initial_free_total / 1024);
	string_format_append(call->result, "\n- %-28s %4u",  		"heap IRAM 8 bit adressable",	heap_caps_get_free_size(MALLOC_CAP_IRAM_8BIT) / 1024);
	string_format_append(call->result, "\n- %-28s %4u",  		"heap retention",				heap_caps_get_free_size(MALLOC_CAP_RETENTION) / 1024);
	string_format_append(call->result, "\n- %-28s %4u / %4u",	"heap RTC RAM",					heap_caps_get_free_size(MALLOC_CAP_RTCRAM) / 1024, initial_free_rtcram / 1024);
	string_format_append(call->result, "\n- %-28s %4u",			"heap TCM",						heap_caps_get_free_size(MALLOC_CAP_TCM) / 1024);
	string_append_cstr(call->result, "\ntimings:");
	string_format_append(call->result, "\n- malloc min time: %llu microseconds", stat_util_time_malloc_min);
	string_format_append(call->result, "\n- malloc max time: %llu microseconds", stat_util_time_malloc_max);
}

void info_command_info_process(cli_command_call_t *call)
{
	unsigned int ix, processes;
	unsigned long runtime;
	TaskStatus_t *process_info;
	const TaskStatus_t *pip;
	const char *name;
	const char *state;

	assert(inited);
	assert(call->parameter_count == 0);

	processes = uxTaskGetNumberOfTasks();
	process_info = util_memory_alloc_spiram(sizeof(*process_info) * processes);
	assert(uxTaskGetSystemState(process_info, processes, &runtime) == processes);

	string_format_append(call->result, "processes: %u\n", processes);
	string_format(call->result, "  %2s  %-12s %-10s %-4s %-5s\n", "#", "name", "state", "prio", "stack");

	for(ix = 0; ix < processes; ix++)
	{
		pip = &process_info[ix];
		name = pip->pcTaskName ? pip->pcTaskName : "(null)";

		switch(pip->eCurrentState)
		{
			case(eRunning):
			{
				state = "running";
				break;
			}

			case(eReady):
			{
				state = "ready";
				break;
			}

			case(eBlocked):
			{
				state = "blocked";
				break;
			}

			case(eSuspended):
			{
				state = "suspended";
				break;
			}

			case(eDeleted):
			{
				state = "deleted";
				break;
			}

			default:
			{
				state = "invalid";
				break;
			}
		}

		string_format_append(call->result, "  %2u: %-12s %-10s %4u %5u\n",
				pip->xTaskNumber,
				name,
				state,
				pip->uxCurrentPriority,
				(unsigned int)pip->usStackHighWaterMark);
	}

	free(process_info);
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
