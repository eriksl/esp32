#ifndef _packet_h_
#define _packet_h_

// for packet_header_t
#define __esp32__
#define assert_size(type, size) _Static_assert(sizeof(type) == size, "sizeof(" #type ") != " #size)
#define assert_field(name, field, offset) _Static_assert(offsetof(name, field) == offset)
#define attr_packed __attribute__ ((__packed__))
#include "ota.h"
#undef assert_size
#undef assert_field
#undef attr_packed

bool packet_is_packet(unsigned int length, const void *buffer);
unsigned int packet_length(unsigned int length, const void *buffer);
void packet_decapsulate(cli_buffer_t *cli_buffer, char **data, unsigned int *oob_data_length, uint8_t **oob_data);
void packet_encapsulate(cli_buffer_t *cli_buffer, const char *data, unsigned int oob_data_length, const uint8_t *oob_data);

#endif
