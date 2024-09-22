#pragma once

void config_init(void);

bool config_get_uint(const string_t key, uint32_t *value);
bool config_get_uint_cstr(const char *key, uint32_t *value);
bool config_get_int(const string_t key, int32_t *value);
bool config_get_int_cstr(const char *key, int32_t *value);
bool config_get_string(const string_t key, string_t string);
bool config_get_string_cstr(const char *key, string_t string);

void config_set_uint(const string_t key, uint32_t value);
void config_set_uint_cstr(const char *key, uint32_t value);
void config_set_int(const string_t key, int32_t value);
void config_set_int_cstr(const char *key, int32_t value);
void config_set_string(const string_t key, string_t value);
void config_set_string_cstr(const char *key, string_t value);

bool config_erase(const string_t key);
bool config_erase_cstr(const char *key);
bool config_erase_wildcard(const string_t key);
bool config_erase_wildcard_cstr(const char *key);
