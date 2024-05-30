#pragma once

void fs_init(void);

void command_fs_info(cli_command_call_t *call);
void command_fs_ls(cli_command_call_t *call);
void command_fs_format(cli_command_call_t *call);
void command_fs_read(cli_command_call_t *call);
void command_fs_append(cli_command_call_t *call);
void command_fs_erase(cli_command_call_t *call);
void command_fs_checksum(cli_command_call_t *call);
