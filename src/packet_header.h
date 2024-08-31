#pragma once

#include <stdint.h>
#include <stddef.h>

enum
{
	packet_header_soh = 0x01,
	packet_header_version = 3,
	packet_header_id = 0x4afb,
};

typedef struct __attribute__((packed))
{
	uint8_t soh;
	uint8_t version;
	uint16_t id;
	uint16_t header_length;
	uint16_t payload_length;
	uint16_t oob_length;

	union
	{
		struct __attribute__((packed))
		{
			unsigned int spare_0:1;
			unsigned int spare_1:1;
			unsigned int spare_2:1;
			unsigned int spare_3:1;
			unsigned int spare_4:1;
			unsigned int spare_5:1;
			unsigned int spare_6:1;
			unsigned int spare_7:1;
			unsigned int spare_8:1;
			unsigned int spare_9:1;
			unsigned int spare_10:1;
			unsigned int spare_11:1;
			unsigned int spare_12:1;
			unsigned int spare_13:1;
			unsigned int spare_14:1;
			unsigned int spare_15:1;
		} flag;
		uint16_t flags;
	};
	uint16_t spare[2];
	uint32_t header_checksum;
	uint32_t packet_checksum;
	uint8_t data[];
} packet_header_t;

#define assert_field(name, field, offset) static_assert(offsetof(name, field) == offset)

assert_field(packet_header_t, soh, 0);
assert_field(packet_header_t, version, 1);
assert_field(packet_header_t, id, 2);
assert_field(packet_header_t, header_length, 4);
assert_field(packet_header_t, payload_length, 6);
assert_field(packet_header_t, oob_length, 8);
assert_field(packet_header_t, flag, 10);
assert_field(packet_header_t, flags, 10);
assert_field(packet_header_t, spare[0], 12);
assert_field(packet_header_t, spare[1], 14);
assert_field(packet_header_t, header_checksum, 16);
assert_field(packet_header_t, packet_checksum, 20);
assert_field(packet_header_t, data, 24);
static_assert(sizeof(packet_header_t) == 24);
static_assert((sizeof(packet_header_t) % 4) == 0);
