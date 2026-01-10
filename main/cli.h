#pragma once

enum
{
	cli_buffer_magic_number_head = 0x1234abcd,
	cli_buffer_magic_number_tail = 0xdcba4321,
};

typedef enum
{
	cli_source_none = 0,
	cli_source_bt,
	cli_source_console,
	cli_source_wlan_tcp,
	cli_source_wlan_udp,
	cli_source_script,
	cli_source_size,
} cli_source_t;

typedef struct
{
	unsigned int	magic_number_head;
	cli_source_t	source;
	unsigned int	mtu;
	string_t		data;

	struct
	{
		unsigned int packetised:1;
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

		struct
		{
			char name[16];
			void *task; // TaskHandle_t
		} script;
	};
	unsigned int magic_number_tail;
} cli_buffer_t;

void cli_receive_queue_push(cli_buffer_t *buffer);
void cli_init(void);
