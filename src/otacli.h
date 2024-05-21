#ifndef _otacli_h_
#define _otacli_h_

void command_ota_start(cli_function_call_t *call);
void command_ota_write(cli_function_call_t *call);
void command_ota_finish(cli_function_call_t *call);
void command_ota_commit(cli_function_call_t *call);
void command_ota_confirm(cli_function_call_t *call);

#endif
