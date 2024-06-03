#pragma once

#define log(s) log_cstr(s)

void log_init(void);
void log_cstr(const char *string);
void log_format(const char *f, ...) __attribute__ ((format (printf, 1, 2)));

void log_command_info(cli_command_call_t *call);
void log_command_log(cli_command_call_t *call);
void log_command_log_clear(cli_command_call_t *call);
