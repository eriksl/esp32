#ifndef _cli_h_
#define _cli_h_

#include <stdint.h>

typedef enum
{
	cli_source_none = 0,
	cli_source_bt,
	cli_source_station,
	cli_source_ap,
	cli_source_console,
} cli_source_t;

typedef struct
{
	cli_source_t	source;
	unsigned int	length;
	uint8_t			*data;
	unsigned int	data_from_malloc;

	union
	{
		struct
		{
			unsigned int connection_handle;
			unsigned int attribute_handle;
		} bt;
	};
} cli_buffer_t;

void cli_init(void);
void cli_receive_queue_push(const cli_buffer_t *buffer);

#endif
