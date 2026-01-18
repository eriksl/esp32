#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "string.h"
#include "cli.h"
#include "log.h"
#include "util.h"

#include "packet_header.h"
#include "packet.h"

#include <string>
#include <boost/format.hpp>

#include <freertos/FreeRTOS.h>

bool packet_valid(const std::string &data)
{
	const packet_header_t *packet_header = reinterpret_cast<const packet_header_t *>(data.data());

	return((data.length() >= sizeof(*packet_header)) &&
			(packet_header->soh == packet_header_soh) &&
			(packet_header->version == packet_header_version) &&
			(packet_header->id == packet_header_id));
}

unsigned int packet_length(const std::string &data)
{
	const packet_header_t *packet_header = reinterpret_cast<const packet_header_t *>(data.data());

    return(packet_header->header_length + packet_header->payload_length + packet_header->oob_length);
}

bool packet_complete(const std::string &data)
{
    return(data.length() >= packet_length(data));
}

void packet_encapsulate(command_response_t *dst, const std::string &data, const std::string &oob_data)
{
	assert(dst);

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
		packet_header.payload_length = data.length();
		packet_header.oob_length = oob_data.length();

		crc32 = util_crc32(0, nullptr, 0);
		crc32 = util_crc32(crc32, reinterpret_cast<const uint8_t *>(&packet_header), offsetof(packet_header_t, header_checksum));
		packet_header.header_checksum = crc32;

		checksummed = 0;
		crc32 = util_crc32(0, nullptr, 0);
		crc32 = util_crc32(crc32, reinterpret_cast<const uint8_t *>(&packet_header), offsetof(packet_header_t, packet_checksum));
		checksummed += offsetof(packet_header_t, packet_checksum);

		crc32 = util_crc32(crc32, reinterpret_cast<const uint8_t *>(data.data()), data.length());
		checksummed += data.length();
		crc32 = util_crc32(crc32, reinterpret_cast<const uint8_t *>(oob_data.data()), oob_data.length());
		checksummed += oob_data.length();

		crc32_padding = (4 - (checksummed & 0x03)) & 0x03;
		crc32 = util_crc32(crc32, crc32_padding_string, crc32_padding);

		packet_header.packet_checksum = crc32;

		dst->packet.assign(reinterpret_cast<const char *>(&packet_header), sizeof(packet_header));
		dst->packet.append(data);
		dst->packet.append(oob_data);
	}
	else
	{
		dst->packet = data;

		if(!dst->packet.empty() && (dst->packet.back() != '\n'))
			dst->packet.append("\n");

		if(oob_data.length() > 0)
		{
			dst->packet.append(1, '\0');
			dst->packet.append(oob_data);
		}
	}
}

void packet_decapsulate(const command_response_t *src, std::string &data, std::string &oob_data)
{
	uint32_t our_checksum;
	unsigned checksummed, crc32_padding;
	static const uint8_t crc32_padding_string[4] = { 0, 0, 0, 0 };

	assert(src);

	if(src->packetised)
	{
		const packet_header_t *packet_header = reinterpret_cast<const packet_header_t *>(src->packet.data());

		if(packet_header->header_length != sizeof(*packet_header))
			log_format("decapsulate: invalid packet header length, expected: %u, received: %u", sizeof(*packet_header), (unsigned int)packet_header->header_length);

		if((packet_header->header_length + packet_header->payload_length + packet_header->oob_length) != src->packet.length())
			log_format("decapsulate: invalid packet length, expected: %d, received: %u",
					(packet_header->header_length + packet_header->payload_length + packet_header->oob_length), src->packet.length());

		our_checksum = util_crc32(0, nullptr, 0);
		our_checksum = util_crc32(our_checksum, reinterpret_cast<const uint8_t *>(packet_header), offsetof(packet_header_t, header_checksum));

		if(our_checksum != packet_header->header_checksum)
		{
			log_format("decapsulate: invalid header checksum, ours: 0x%x, theirs: 0x%x", (unsigned int)our_checksum, (unsigned int)packet_header->header_checksum);
			data = "<error>";
			oob_data = "";
			return;
		}

		data.assign(reinterpret_cast<const char *>(&packet_header->data[0]), packet_header->payload_length);
		oob_data.assign(reinterpret_cast<const char *>(&packet_header->data[packet_header->payload_length]), packet_header->oob_length);

		checksummed = 0;
		our_checksum = util_crc32(0, nullptr, 0);
		our_checksum = util_crc32(our_checksum, reinterpret_cast<const uint8_t *>(packet_header), offsetof(packet_header_t, packet_checksum));
		checksummed += offsetof(packet_header_t, packet_checksum);

		our_checksum = util_crc32(our_checksum, reinterpret_cast<const uint8_t *>(data.data()), data.length());
		checksummed += data.length();

		our_checksum = util_crc32(our_checksum, reinterpret_cast<const uint8_t *>(oob_data.data()), oob_data.length());
		checksummed += oob_data.length();

		crc32_padding = (4 - (checksummed & 0x03)) & 0x03;
		our_checksum = util_crc32(our_checksum, crc32_padding_string, crc32_padding);

		if(our_checksum != packet_header->packet_checksum)
		{
			log_format("decapsulate: invalid packet checksum, ours: 0x%x, theirs: 0x%x", (unsigned int)our_checksum, (unsigned int)packet_header->packet_checksum);
			data = "<error>";
			oob_data = "";
			return;
		}
	}
	else
	{
		unsigned int oob_offset;

		if((src->packet.length() > 1) && ((oob_offset = src->packet.find_first_of('\0')) != std::string::npos))
		{
			data = src->packet.substr(0, oob_offset);
			oob_data = src->packet.substr(oob_offset + 1);
		}
		else
			data = src->packet;
	}
}
