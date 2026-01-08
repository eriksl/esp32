#pragma once

void console_init_1(void);
void console_init_2(void);

#ifdef __cplusplus
extern "C"
{
#endif
void console_send(const cli_buffer_t *);
void console_write_line(const char *string);
#ifdef __cplusplus
}
#endif
