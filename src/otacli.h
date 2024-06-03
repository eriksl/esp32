#pragma once

void ota_command_start(cli_command_call_t *call);
void ota_command_write(cli_command_call_t *call);
void ota_command_finish(cli_command_call_t *call);
void ota_command_commit(cli_command_call_t *call);
void ota_command_confirm(cli_command_call_t *call);
