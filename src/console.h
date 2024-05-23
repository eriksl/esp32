#ifndef _console_h_
#define _console_h_

#include "cli-command.h"
#include "cli.h"

void command_info_console(cli_command_call_t *call);
void console_send(const cli_buffer_t *);
void console_write_line(const char *string);
void console_init();

#endif
