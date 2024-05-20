#ifndef _util_h_
#define _util_h_

#include <esp_ota_ops.h>
#include <stdint.h>

void util_init(void);
uint32_t util_md5_32(unsigned int length, const uint8_t *data);
void util_hash_to_text(unsigned int hash_length, const uint8_t *hash_data, unsigned int length, char *data);
unsigned int util_partition_to_slot(const esp_partition_t *partition);

#endif
