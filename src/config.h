#pragma once

void config_init(void);
bool config_get_uint(const char *key, uint32_t *value);
bool config_get_int(const char *key, int32_t *value);
bool config_get_string(const char *key, unsigned int string_size, char *string);

void config_set_uint(const char *key, uint32_t value);
void config_set_int(const char *key, int32_t value);
void config_set_string(const char *key, const char *value);

bool config_erase(const char *key);

void command_info_config(cli_command_call_t *call);
void command_config_dump(cli_command_call_t *call);
void command_config_show(cli_command_call_t *call);

void command_config_set_int(cli_command_call_t *call);
void command_config_set_uint(cli_command_call_t *call);
void command_config_set_string(cli_command_call_t *call);

void command_config_erase(cli_command_call_t *call);
