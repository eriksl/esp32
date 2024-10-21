#pragma once

typedef enum
{
	io_id_pcf8574_26 = 0,
	io_id_first = io_id_pcf8574_26,
	io_id_pcf8574_3a,
	io_id_size,
	io_id_error = io_id_size,
} io_id_t;

typedef enum
{
	io_cap_input = 0,
	io_cap_first = io_cap_input,
	io_cap_output,
	io_cap_size,
	io_cap_error = io_cap_size,
} io_capabilities_t;

_Static_assert(io_cap_size < 32); // bitmap

void io_init(void);
bool io_info(string_t result, unsigned int io);
bool io_read(string_t result, unsigned int io, unsigned int pin, unsigned int *value);
bool io_write(string_t result, unsigned int io, unsigned int pin, unsigned int value);
bool io_pin_info(string_t result, unsigned int io, unsigned int pin);
