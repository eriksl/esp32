#pragma once

void fs_init(void);

void fs_command_info(cli_command_call_t *call);
void fs_command_list(cli_command_call_t *call);
void fs_command_format(cli_command_call_t *call);
void fs_command_read(cli_command_call_t *call);
void fs_command_append(cli_command_call_t *call);
void fs_command_erase(cli_command_call_t *call);
void fs_command_checksum(cli_command_call_t *call);
