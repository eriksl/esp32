#ifndef _bt_h_
#define _bt_h_

#include "cli-command.h"
#include "cli.h"

int bt_init(void);
void bt_send(const cli_buffer_t *);
void command_info_bluetooth(cli_command_call_t *call);

#endif
