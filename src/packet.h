#pragma once

bool packet_valid(const const_string_t data);
bool packet_complete(const const_string_t data);
void packet_encapsulate(cli_buffer_t *dst, const const_string_t data, const const_string_t oob_data) __attribute__((nonnull (1, 2)));
void packet_decapsulate(const cli_buffer_t *src, string_t *data, string_t *oob_data) __attribute__((nonnull (1, 2, 3)));
