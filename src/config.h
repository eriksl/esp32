#pragma once

void config_init(void);
bool config_get_uint(const string_t key, uint32_t *value);
bool config_get_int(const string_t key, int32_t *value);
bool config_get_string(const string_t key, string_t string);
void config_set_uint(const string_t key, uint32_t value);
void config_set_int(const string_t key, int32_t value);
void config_set_string(const string_t key, string_t value);
bool config_erase(const string_t key);

void config_command_info(cli_command_call_t *call);
void config_command_dump(cli_command_call_t *call);
void config_command_show(cli_command_call_t *call);
void config_command_set_int(cli_command_call_t *call);
void config_command_set_uint(cli_command_call_t *call);
void config_command_set_string(cli_command_call_t *call);
void config_command_erase(cli_command_call_t *call);
