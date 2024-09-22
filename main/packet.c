#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "string.h"
#include "cli.h"
#include "packet.h"
#include "log.h"
#include "util.h"
#include "packet_header.h"

#include <freertos/FreeRTOS.h>

bool packet_valid(const const_string_t data)
{
	packet_header_t *packet_header = (packet_header_t *)string_data(data);

	return((string_length(data) >= sizeof(*packet_header)) &&
			(packet_header->soh == packet_header_soh) &&
			(packet_header->version == packet_header_version) &&
			(packet_header->id == packet_header_id));
}

bool packet_complete(const const_string_t data)
{
    const packet_header_t *packet_header = (packet_header_t *)string_data(data);

    return(string_length(data) >= (unsigned int)(packet_header->header_length + packet_header->payload_length + packet_header->oob_length));
}

void packet_encapsulate(cli_buffer_t *dst, const const_string_t data, const const_string_t oob_data)
{
	assert(dst);
	assert(!dst->data);
	assert(data);
	unsigned int oob_data_length;

	if(oob_data)
		oob_data_length = string_length(oob_data);
	else
		oob_data_length = 0;

	if(dst->packetised)
	{
		static const uint8_t crc32_padding_string[4] = { 0, 0, 0, 0 };

		uint32_t crc32;
		unsigned int checksummed, crc32_padding;
		packet_header_t packet_header;

		memset(&packet_header, 0, sizeof(packet_header));
		packet_header.soh = packet_header_soh;
		packet_header.version = packet_header_version;
		packet_header.id = packet_header_id;
		packet_header.header_length = sizeof(packet_header);
		packet_header.payload_length = string_length(data);
		packet_header.oob_length = oob_data_length;

		crc32 = util_crc32cksum_byte(0, (void *)0, 0);
		crc32 = util_crc32cksum_byte(crc32, &packet_header, offsetof(packet_header_t, header_checksum));
		packet_header.header_checksum = crc32;

		checksummed = 0;
		crc32 = util_crc32cksum_byte(0, (void *)0, 0);
		crc32 = util_crc32cksum_byte(crc32, &packet_header, offsetof(packet_header_t, packet_checksum));
		checksummed += offsetof(packet_header_t, packet_checksum);
		crc32 = util_crc32cksum_byte(crc32, string_data(data), string_length(data));
		checksummed += string_length(data);
		crc32 = util_crc32cksum_byte(crc32, string_data(data) /* this is correct, don't use NULL here */, oob_data_length);
		checksummed += oob_data_length;
		crc32_padding = (4 - (checksummed & 0x03)) & 0x03;
		crc32 = util_crc32cksum_byte(crc32, crc32_padding_string, crc32_padding);

		packet_header.packet_checksum = crc32;

		dst->data = string_new(sizeof(packet_header) + string_length(data) + oob_data_length);
		string_assign_data(dst->data, sizeof(packet_header), (const uint8_t *)&packet_header);
		string_append_string(dst->data, data);

		if(oob_data)
			string_append_string(dst->data, oob_data);
	}
	else
	{
		dst->data = string_new(string_length(data) + 1 /* \n */ + 1 /* '\0' */ + oob_data_length);

		string_assign_string(dst->data, data);

		if(string_at_back(dst->data) != '\n')
			string_append(dst->data, '\n');

		if(oob_data && oob_data_length)
		{
			string_append(dst->data, '\0');
			string_append_string(dst->data, oob_data);
		}
	}
}

void packet_decapsulate(const cli_buffer_t *src, string_t *data, string_t *oob_data)
{
	uint32_t our_checksum;
	unsigned checksummed, crc32_padding;
	static const uint8_t crc32_padding_string[4] = { 0, 0, 0, 0 };

	assert(src);
	assert(data);
	assert(oob_data);

	if(src->packetised)
	{
		const packet_header_t *packet_header = (packet_header_t *)string_data(src->data);

		if(packet_header->header_length != sizeof(*packet_header))
			log_format("decapsulate: invalid packet header length, expected: %u, received: %u", sizeof(*packet_header), (unsigned int)packet_header->header_length);

		if((unsigned int)(packet_header->header_length + packet_header->payload_length + packet_header->oob_length) != string_length(src->data))
			log_format("decapsulate: invalid packet length, expected: %u, received: %u",
					(packet_header->header_length + packet_header->payload_length + packet_header->oob_length), string_length(src->data));

		our_checksum = util_crc32cksum_byte(0, (void *)0, 0);
		our_checksum = util_crc32cksum_byte(our_checksum, packet_header, offsetof(packet_header_t, header_checksum));

		if(our_checksum != packet_header->header_checksum)
		{
			log_format("decapsulate: invalid header checksum, ours: 0x%x, theirs: 0x%x", (unsigned int)our_checksum, (unsigned int)packet_header->header_checksum);
			goto error;
		}

		*data = string_new(packet_header->payload_length);
		string_assign_data(*data, packet_header->payload_length, &packet_header->data[0]);

		*oob_data = string_new(packet_header->oob_length);
		string_assign_data(*oob_data, packet_header->oob_length, &packet_header->data[packet_header->payload_length]);

		checksummed = 0;
		our_checksum = util_crc32cksum_byte(0, (void *)0, 0);
		our_checksum = util_crc32cksum_byte(our_checksum, string_data(src->data), offsetof(packet_header_t, packet_checksum));
		checksummed += offsetof(packet_header_t, packet_checksum);

		our_checksum = util_crc32cksum_byte(our_checksum, string_data(*data), string_length(*data));
		checksummed += string_length(*data);

		our_checksum = util_crc32cksum_byte(our_checksum, string_data(*oob_data), string_length(*oob_data));
		checksummed += string_length(*oob_data);

		crc32_padding = (4 - (checksummed & 0x03)) & 0x03;
		our_checksum = util_crc32cksum_byte(our_checksum, crc32_padding_string, crc32_padding);

		if(our_checksum != packet_header->packet_checksum)
		{
			log_format("decapsulate: invalid packet checksum, ours: 0x%x, theirs: 0x%x", (unsigned int)our_checksum,  (unsigned int)packet_header->packet_checksum);
			goto error;
		}
	}
	else
	{
		unsigned int oob_length, oob_offset;
		const uint8_t *oob_offset_ptr;

		if((oob_offset_ptr = memchr(string_data(src->data), 0, string_length(src->data))))
		{
			oob_offset = oob_offset_ptr - string_data(src->data) + 1;
			oob_length = string_length(src->data) - oob_offset;

			if(oob_offset >= string_length(src->data))
			{
				log_format("decapsulate: invalid oob data, offset: %d, data length: %d", oob_offset, string_length(src->data));
				goto error;
			}

			//log_format("! oob_offset: %d", oob_offset);
			//log_format("! oob_length %d", oob_length);
			//log_format("! data length: %d", string_length(src->data));

			*data = string_new(oob_offset);
			string_assign_data(*data, oob_offset, &(string_data(src->data)[0]));
			*oob_data = string_new(oob_length);
			string_assign_data(*oob_data, oob_length, &(string_data(src->data)[oob_offset]));
		}
		else
		{
			*data = string_new(string_length(src->data));
			string_assign_data(*data, string_length(src->data), string_data(src->data));
			*oob_data = (string_t)0;
		}
	}

	return;

error:
	// FIXME: use string_const

	static const char *error_msg = "<error>";

	*data = string_new(strlen(error_msg));
	string_assign_cstr(*data, error_msg);
	*oob_data = (string_t)0;
}
