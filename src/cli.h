#pragma once

enum
{
	cli_buffer_magic_number_head = 0x1234abcd,
	cli_buffer_magic_number_tail  = 0xdcba4321,
};

typedef enum
{
	cli_source_none = 0,
	cli_source_bt,
	cli_source_console,
	cli_source_wlan_tcp,
	cli_source_wlan_udp,
} cli_source_t;

typedef struct
{
	unsigned int	magic_number_head;
	cli_source_t	source;
	unsigned int	length;
	uint8_t			*data;
	unsigned int	transaction_id;
	unsigned int	broadcast_groups;

	struct
	{
		unsigned int data_from_malloc:1;
		unsigned int packetised:1;
		unsigned int checksum_requested:1;
		unsigned int transaction_id_valid:1;
	};

	union
	{
		struct
		{
			unsigned int connection_handle;
			unsigned int attribute_handle;
		} bt;

		struct
		{
			struct
			{
				unsigned int sin6_length;
				char sin6_addr[32];
			} address;
		} ip;
	};
	unsigned int magic_number_tail;
} cli_buffer_t;

void cli_init(void);
void cli_receive_queue_push(cli_buffer_t *buffer);
