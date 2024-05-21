#ifndef _flash_h_
#define _flash_h_

void command_flash_bench(cli_command_call_t *call);
void command_flash_checksum(cli_command_call_t *call);
void command_flash_info(cli_command_call_t *call);
void command_flash_read(cli_command_call_t *call);
void command_flash_write(cli_command_call_t *call);

#endif
