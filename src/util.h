#ifndef _util_h_
#define _util_h_

#include <stdint.h>

#include <esp_err.h>
#include <esp_ota_ops.h>

#include "log.h"

void util_init(void);
uint32_t util_md5_32(unsigned int length, const uint8_t *data);
void util_hash_to_text(unsigned int hash_length, const uint8_t *hash_data, unsigned int length, char *data);
unsigned int util_partition_to_slot(const esp_partition_t *partition);

static inline void util_abort_on_esp_err(const char *what, int rv)
{
	if(rv)
	{
		log("abort: %s (%s) [0x%x]", what, esp_err_to_name(rv), rv);
		abort();
	}
}

static inline void util_warn_on_esp_err(const char *what, int rv)
{
	if(rv)
	{
		log("warning: %s (%s) [0x%x]", what, esp_err_to_name(rv), rv);
		abort();
	}
}

static inline void util_abort(const char *what)
{
	log("abort: %s", what);
	abort();
}

#endif
