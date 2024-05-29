#pragma once

#include <esp_ota_ops.h>

#pragma GCC poison strcpy
#pragma GCC poison strcat
#pragma GCC poison strncpy
#pragma GCC poison strncat
#pragma GCC poison printf
#pragma GCC poison sprintf
#pragma GCC poison snprintf

void util_init(void);
uint32_t util_md5_32(unsigned int length, const uint8_t *data);
unsigned int util_partition_to_slot(const esp_partition_t *partition);

static inline void util_abort_on_esp_err(const char *what, int rv)
{
	if(rv)
	{
		log_format("abort: %s (%s) [0x%x]", what, esp_err_to_name(rv), rv);
		abort();
	}
}

static inline void util_warn_on_esp_err(const char *what, int rv)
{
	if(rv)
	{
		log_format("warning: %s (%s) [0x%x]", what, esp_err_to_name(rv), rv);
		abort();
	}
}

static inline void util_abort(const char *what)
{
	log_format("abort: %s", what);
	abort();
}
