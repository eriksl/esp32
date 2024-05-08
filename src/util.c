#include "util.h"

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <mbedtls/base64.h>
#include <mbedtls/md5.h>

#include <esp_log.h>
#include <esp_check.h>

enum
{
	slots = 16,
};

typedef struct
{
	char tag[24];
	unsigned int current;
	TaskHandle_t task;
} stack_usage_t;

static bool inited;
static SemaphoreHandle_t stack_usage_semaphore;
static stack_usage_t stack_usage[slots] = {};

uint32_t util_md5_32(unsigned int length, const uint8_t *data)
{
	uint8_t hash[16];

	mbedtls_md5(data, length, hash);

#if 0
	ESP_LOGI("md5", "%u: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
			length,
			hash[0],
			hash[1],
			hash[2],
			hash[3],
			hash[4],
			hash[5],
			hash[6],
			hash[7],
			hash[8],
			hash[9],
			hash[10],
			hash[11],
			hash[12],
			hash[13],
			hash[14],
			hash[15]);
#endif

	return((hash[0] << 24) | (hash[1] << 16) | (hash[2] << 8) | (hash[3] << 0));
}

void util_stack_usage_update(const char *tag)
{
	unsigned int ix;
	stack_usage_t *sup;

	assert(tag);
	assert(*tag);
	assert(inited);

	if(xSemaphoreTake(stack_usage_semaphore, portTICK_PERIOD_MS * 10) == pdFALSE)
	{
		ESP_LOGW("util", "util_stack_usage_update: semaphore take timeout");
		return;
	}

	for(ix = 0; ix < slots; ix++)
	{
		sup = &stack_usage[ix];

		if(*sup->tag && !strcmp(sup->tag, tag))
			break;
	}

	if(!*sup->tag || (ix >= slots))
	{
		for(ix = 0; ix < slots; ix++)
		{
			sup = &stack_usage[ix];

			if(!*sup->tag)
				break;
		}

		if(ix >= slots)
		{
			ESP_LOGW("util", "util_stack_usage_update: no more free slots");
			goto error;
		}

		strncpy(sup->tag, tag, sizeof(sup->tag) - 1);
		sup->current = 0;
		sup->task = xTaskGetCurrentTaskHandle();
	}

	ix = uxTaskGetStackHighWaterMark(0);

	if((sup->current == 0) || (ix < sup->current))
		sup->current = ix;

error:
	xSemaphoreGive(stack_usage_semaphore);
}

void util_stack_usage_show(void)
{
	unsigned int ix;
	stack_usage_t *sup;

	assert(inited);

	xSemaphoreTake(stack_usage_semaphore, portMAX_DELAY);

	for(ix = 0; ix < slots; ix++)
	{
		sup = &stack_usage[ix];

		if(*sup->tag)
			ESP_LOGI("util", "stack usage[%2u]: %-12s %-24s %4u", ix, pcTaskGetName(sup->task), sup->tag, sup->current);
	}

	xSemaphoreGive(stack_usage_semaphore);
}

void util_init(void)
{
	assert(!inited);

	inited = true;

	assert((stack_usage_semaphore = xSemaphoreCreateMutex()));
}
