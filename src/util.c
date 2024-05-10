#include "util.h"

#include <string.h>

#include <freertos/FreeRTOS.h>

#include <mbedtls/base64.h>
#include <mbedtls/md5.h>

#include <esp_log.h>
#include <esp_check.h>

static bool inited;

uint32_t util_md5_32(unsigned int length, const uint8_t *data)
{
	uint8_t hash[16];

	mbedtls_md5(data, length, hash);

	return((hash[0] << 24) | (hash[1] << 16) | (hash[2] << 8) | (hash[3] << 0));
}

void util_init(void)
{
	assert(!inited);
	inited = true;
}
