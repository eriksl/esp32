#ifndef _util_h_
#define _util_h_

#include <stdint.h>

void util_init(void);
uint32_t util_md5_32(unsigned int length, const uint8_t *data);

#endif
