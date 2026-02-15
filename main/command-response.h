#pragma once

#include <string>

enum cli_source_t
{
	cli_source_none = 0,
	cli_source_bt,
	cli_source_console,
	cli_source_wlan_tcp,
	cli_source_wlan_udp,
	cli_source_script,
	cli_source_size,
};

struct command_response_t
{
	cli_source_t source;
	unsigned int mtu;
	std::string packet;

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
};
