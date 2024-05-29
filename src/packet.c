#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "string.h"
#include "cli-command.h"
#include "cli.h"
#include "packet.h"
#include "log.h"
#include "util.h"

#include <freertos/FreeRTOS.h>

void packet_decapsulate(cli_buffer_t *cli_buffer, string_t *data, string_t *oob_data)
{
	string_auto_init(error_message, "<error");
	packet_header_t *packet;
	const uint8_t *data_pad;
	unsigned int data_length, data_pad_offset, oob_data_offset, oob_data_length;
	unsigned int our_checksum, their_checksum;

	assert(cli_buffer);
	assert(data);
	assert(oob_data);

	packet = (packet_header_t *)cli_buffer->data;

	if((cli_buffer->length >= sizeof(*packet)) && (packet->soh == packet_header_soh) && (packet->version == packet_header_version) && (packet->id == packet_header_id))
	{
		if(packet->length != cli_buffer->length)
		{
			log_format("packet: incomplete packet: %u / %u", cli_buffer->length, packet->length);
			goto error;
		}

		if((packet->data_offset != sizeof(*packet)) ||
				(packet->data_pad_offset < packet->data_offset) || (packet->data_pad_offset > packet->length) ||
				(packet->oob_data_offset < packet->data_pad_offset) || (packet->oob_data_offset > packet->length))
		{
			log("packet: invalid offset in packet header");
			goto error;
		}

		if(packet->flag.md5_32_provided)
		{
			their_checksum = packet->checksum;
			packet->checksum = 0;
			our_checksum = util_md5_32(cli_buffer->length, cli_buffer->data);

			if(our_checksum != their_checksum)
			{
				log_format("packet: invalid checksum: 0x%08x[%d] / 0x%08x", our_checksum, packet->length, their_checksum);
				goto error;
			}
		}

		data_length = packet->data_pad_offset - packet->data_offset;
		*data = string_new(data_length + 1);
		string_assign_data(*data, data_length, cli_buffer->data + packet->data_offset);

		if((oob_data_length = packet->length - packet->oob_data_offset))
		{
			*oob_data = string_new(oob_data_length + 1);
			string_assign_data(*oob_data, oob_data_length, &cli_buffer->data[packet->oob_data_offset]);
		}
		else
			*oob_data = (string_t)0;

		if(packet->flag.transaction_id_provided)
		{
			cli_buffer->transaction_id_valid = 1;
			cli_buffer->transaction_id = packet->transaction_id;
		}
		else
		{
			cli_buffer->transaction_id_valid = 0;
			cli_buffer->transaction_id = 0;
		}

		cli_buffer->broadcast_groups = packet->broadcast_groups;
		cli_buffer->checksum_requested = packet->flag.md5_32_requested ? 1 : 0;
		cli_buffer->packetised = 1;
	}
	else
	{
		if((data_pad = memchr(cli_buffer->data, 0, cli_buffer->length)))
		{
			if((data_pad_offset = data_pad - cli_buffer->data) >= cli_buffer->length)
			{
				log_format("packet: invalid data pad offset: %u/%u", data_pad_offset, cli_buffer->length);
				goto error;
			}

			if((oob_data_offset = ((data_pad_offset + 3 + 1) & ~0x03)) >= cli_buffer->length)
			{
				log_format("packet: invalid oob data offset: %u/%u", oob_data_offset, cli_buffer->length);
				goto error;
			}

			oob_data_length = cli_buffer->length - oob_data_offset;

			*data = string_new(data_pad_offset + 1);
			string_assign_data(*data, data_pad_offset, &cli_buffer->data[0]);

			*oob_data = string_new(oob_data_length + 1);
			string_assign_data(*oob_data, oob_data_length, &cli_buffer->data[oob_data_offset]);
		}
		else
		{
			*data = string_new(cli_buffer->length + 1);
			string_assign_data(*data, cli_buffer->length, &cli_buffer->data[0]);

			*oob_data = (string_t)0;
		}

		cli_buffer->transaction_id = 0;
		cli_buffer->broadcast_groups = 0;
		cli_buffer->checksum_requested = 0;
		cli_buffer->packetised = 0;
	}

	return;

error:
	string_assign_string(*data, error_message);
	*oob_data = (string_t)0;
}

void packet_encapsulate(cli_buffer_t *cli_buffer, const string_t data, const string_t oob_data)
{
	packet_header_t *packet;
	unsigned int data_length;
	unsigned int data_offset;
	unsigned int data_pad_offset;
	unsigned int oob_data_length;
	unsigned int oob_data_offset;

	assert(cli_buffer);
	assert(!cli_buffer->data);
	assert(data);

	if(oob_data)
		oob_data_length = string_length(oob_data);
	else
		oob_data_length = 0;

	data_length = string_length(data);

	if(cli_buffer->packetised)
	{
		data_offset = sizeof(*packet);
		data_pad_offset = data_offset + data_length + /* \n */ 1;
		oob_data_offset = oob_data_length ? (data_pad_offset + 3) & ~0x03 : data_pad_offset;
		assert(oob_data_offset >= data_pad_offset);

		cli_buffer->length = oob_data_offset + oob_data_length;
		cli_buffer->data_from_malloc = 1;
		cli_buffer->data = heap_caps_malloc(cli_buffer->length, MALLOC_CAP_SPIRAM);
		memcpy(&cli_buffer->data[data_offset], string_cstr(data), data_length);
		cli_buffer->data[data_offset + data_length] = '\n';
		memset(&cli_buffer->data[data_pad_offset], 0, oob_data_offset - data_pad_offset);

		if(oob_data)
			memcpy(&cli_buffer->data[oob_data_offset], string_data(oob_data), oob_data_length);

		packet = (packet_header_t *)cli_buffer->data;
		memset(packet, 0, sizeof(*packet));
		packet->soh = packet_header_soh;
		packet->version = packet_header_version;
		packet->id = packet_header_id;
		packet->length = cli_buffer->length;
		packet->data_offset = data_offset;
		packet->data_pad_offset = data_pad_offset;
		packet->oob_data_offset = oob_data_offset;
		packet->broadcast_groups = 0;
		packet->flag.md5_32_requested = 0;

		if(cli_buffer->transaction_id_valid)
		{
			packet->flag.transaction_id_provided = 1;
			packet->transaction_id = cli_buffer->transaction_id;
		}
		else
		{
			packet->flag.transaction_id_provided = 0;
			packet->transaction_id = 0;
		}

		if(cli_buffer->checksum_requested)
		{
			packet->flag.md5_32_provided = 1;
			packet->checksum = util_md5_32(cli_buffer->length, cli_buffer->data);
		}
		else
		{
			packet->flag.md5_32_provided = 0;
			packet->checksum = 0;
		}
	}
	else
	{
		data_pad_offset = data_length + /* \n */ 1;
		oob_data_offset = oob_data_length ? ((data_pad_offset + 3 + /* \0 */ 1) & ~0x03) : data_pad_offset;
		assert(oob_data_offset >= data_pad_offset);

		cli_buffer->length = oob_data_offset + oob_data_length;
		cli_buffer->data_from_malloc = 1;
		cli_buffer->data = heap_caps_malloc(cli_buffer->length, MALLOC_CAP_SPIRAM);
		assert(cli_buffer->data);
		memcpy(cli_buffer->data, string_cstr(data), data_length);
		cli_buffer->data[data_length] = '\n';

		if(oob_data)
		{
			memset(&cli_buffer->data[data_pad_offset], 0, oob_data_offset - data_pad_offset);
			memcpy(&cli_buffer->data[oob_data_offset], string_data(oob_data), oob_data_length);
		}
	}
}

bool packet_is_packet(unsigned int length, const void *buffer)
{
	packet_header_t *packet;

	if(length < sizeof(*packet))
		return(false);

	packet = (packet_header_t *)buffer;

	if((packet->soh != packet_header_soh) || (packet->version != packet_header_version) || (packet->id != packet_header_id))
		return(false);

	return(true);
}

unsigned int packet_length(unsigned int length, const void *buffer)
{
	packet_header_t *packet;

	if(!packet_is_packet(length, buffer))
		return(0);

	packet = (packet_header_t *)buffer;

	return(packet->length);
}
