#include "packet.h"
#include "util.h"

#include <string.h>

#include <freertos/FreeRTOS.h>

#include <esp_log.h>
#include <esp_check.h>

void packet_decapsulate(cli_buffer_t *cli_buffer, char **data, unsigned int *oob_data_length, uint8_t **oob_data)
{
	static const char *error = "<error>";
	packet_header_t *packet;
	unsigned int data_length;
	unsigned int our_checksum, their_checksum;

	// FIXME: broadcast groups
	// FIXME: transaction id
	// FIXME: incomplete buffers

	if(cli_buffer->length > sizeof(*packet))
	{
		packet = (packet_header_t *)cli_buffer->data;

		if((packet->soh == packet_header_soh) && (packet->version == packet_header_version) && (packet->id == packet_header_id))
		{
			if(packet->length != cli_buffer->length)
			{
				ESP_LOGW("packet", "packet incomplete: %u / %u", cli_buffer->length, packet->length);
				goto error;
			}

			if(packet->flag.md5_32_provided)
			{
				their_checksum = packet->checksum;
				packet->checksum = 0;
				our_checksum = util_md5_32(cli_buffer->length, cli_buffer->data);

				if(our_checksum != their_checksum)
				{
					ESP_LOGW("packet", "invalid checksum: 0x%08x[%d] / 0x%08x", our_checksum, packet->length, their_checksum);
					goto error;
				}
			}

			// FIXME check offsets

			data_length = packet->data_pad_offset - packet->data_offset;
			assert((*data = heap_caps_malloc(data_length + 1, MALLOC_CAP_SPIRAM)));
			memcpy(*data, cli_buffer->data + packet->data_offset, data_length);
			(*data)[data_length] = '\0';

			if((*oob_data_length = packet->length - packet->oob_data_offset))
			{
				assert((*oob_data = heap_caps_malloc(*oob_data_length, MALLOC_CAP_SPIRAM)));
				memcpy(*oob_data, cli_buffer->data + packet->oob_data_offset, *oob_data_length);
			}
			else
				*oob_data = (uint8_t *)0;

			cli_buffer->checksum_requested = packet->flag.md5_32_requested ? 1 : 0;
			cli_buffer->packetised = 1;

			ESP_LOGI("packet", "return valid packet");

			return;
		}
	}

	assert((*data = heap_caps_malloc(cli_buffer->length + 1, MALLOC_CAP_SPIRAM)));
	memcpy(*data, cli_buffer->data, cli_buffer->length);
	(*data)[cli_buffer->length] = '\0';
	*oob_data = (uint8_t *)0;
	*oob_data_length = 0;

	cli_buffer->checksum_requested = 0;
	cli_buffer->packetised = 0;

	return;

error: // FIXME
	assert((*data = heap_caps_malloc(sizeof(error), MALLOC_CAP_SPIRAM)));
	strcpy(*data, error);
	*oob_data = (uint8_t *)0;
	*oob_data_length = 0;
}

void packet_encapsulate(cli_buffer_t *cli_buffer, const char *data, unsigned int oob_data_length, const uint8_t *oob_data)
{
	packet_header_t *packet;
	unsigned int data_length;
	unsigned int data_offset;
	unsigned int oob_data_offset;

	assert(!cli_buffer->data);

	data_length = strlen((const char *)data);

	if(cli_buffer->packetised)
	{
		data_offset = sizeof(*packet);
		oob_data_offset = data_offset + ((data_length + 1 /* \n */ + 3) & ~0x03);

		cli_buffer->length = oob_data_offset + oob_data_length;
		cli_buffer->data_from_malloc = 1;
		cli_buffer->data = heap_caps_malloc(cli_buffer->length, MALLOC_CAP_SPIRAM);
		memcpy(&cli_buffer->data[data_offset], data, data_length);
		cli_buffer->data[data_offset + data_length] = '\n';
		memcpy(&cli_buffer->data[oob_data_offset], oob_data, oob_data_length);

		packet = (packet_header_t *)cli_buffer->data;
		memset(packet, 0, sizeof(*packet));
		packet->soh = packet_header_soh;
		packet->version = packet_header_version;
		packet->id = packet_header_id;
		packet->length = cli_buffer->length;
		packet->data_offset = data_offset;
		packet->data_pad_offset = data_offset + data_length + 1;
		packet->oob_data_offset = oob_data_offset;
		packet->broadcast_groups = 0;
		packet->flag.md5_32_requested = 0;
		packet->flag.md5_32_provided = 1;
		packet->flag.transaction_id_provided = 0;
		packet->transaction_id = 0;
		packet->checksum = util_md5_32(cli_buffer->length, cli_buffer->data);

		return;
	}

	cli_buffer->length = data_length + 1;
	cli_buffer->data_from_malloc = 1;
	cli_buffer->data = heap_caps_malloc(cli_buffer->length, MALLOC_CAP_SPIRAM);
	memcpy(cli_buffer->data, data, data_length);
	cli_buffer->data[data_length] = '\n';
}
