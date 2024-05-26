#pragma once

void console_command_info(cli_command_call_t *call);

void console_send(const cli_buffer_t *);
void console_write_line(const char *string);
void console_init();
