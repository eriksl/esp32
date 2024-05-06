#ifndef _cli_h_
#define _cli_h_

#include <stdint.h>

void cli_init(void);
void cli_receive_from_bt_queue_push(unsigned int length, uint8_t *data);

#endif
