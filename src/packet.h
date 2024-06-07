#pragma once

// for packet_header_t
#define __esp32__
#define assert_size(type, size) _Static_assert(sizeof(type) == size, "sizeof(" #type ") != " #size)
#define assert_field(name, field, offset) _Static_assert(offsetof(name, field) == offset)
#define attr_packed __attribute__ ((__packed__))
#include "ota.h"
#undef assert_size
#undef assert_field
#undef attr_packed

bool packet_is_packet(const string_t buffer);
unsigned int packet_length(const string_t buffer);
void packet_decapsulate(cli_buffer_t *cli_buffer, string_t *data, string_t *oob_data);
void packet_encapsulate(cli_buffer_t *cli_buffer, const string_t data, const string_t oob_data);
