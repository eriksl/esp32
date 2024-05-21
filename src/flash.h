#ifndef _flash_h_
#define _flash_h_

void command_flash_bench(cli_function_call_t *call);
void command_flash_checksum(cli_function_call_t *call);
void command_flash_info(cli_function_call_t *call);
void command_flash_read(cli_function_call_t *call);
void command_flash_write(cli_function_call_t *call);

#endif
