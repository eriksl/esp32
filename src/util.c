#include <stdint.h>
#include <stdbool.h>

#include "string.h"
#include "cli-command.h"
#include "log.h"
#include "util.h"

#include <freertos/FreeRTOS.h>

#include <mbedtls/base64.h>
#include <mbedtls/md5.h>

static bool inited;

uint32_t util_md5_32(unsigned int length, const uint8_t *data)
{
	uint8_t hash[16];

	mbedtls_md5(data, length, hash);

	return((hash[0] << 24) | (hash[1] << 16) | (hash[2] << 8) | (hash[3] << 0));
}

unsigned int util_partition_to_slot(const esp_partition_t *partition)
{
	unsigned int slot;

	assert(partition);
	assert(partition->type == ESP_PARTITION_TYPE_APP);

	if(partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0)
		slot = 0;
	else
		if(partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1)
			slot = 1;
		else
			util_abort("util_partition_to_slot: unknown OTA partition type");

	return(slot);
}

void util_init(void)
{
	assert(!inited);
	inited = true;
}
