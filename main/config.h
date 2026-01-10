#pragma once

#include <string>

void config_init(void);

// get

bool config_get_uint(const string_t key, uint32_t &value);
bool config_get_uint(const std::string &key, uint32_t &value);

bool config_get_int(const string_t key, int32_t &value);
bool config_get_int(const std::string &key, int32_t &value);

bool config_get_string(const string_t key, string_t string);
bool config_get_string(const std::string &key, std::string &string);

// set

void config_set_uint(const string_t key, uint32_t value);
void config_set_uint(const std::string &key, uint32_t value);

void config_set_int(const string_t key, int32_t value);
void config_set_int(const std::string &key, int32_t value);

void config_set_string(const string_t key, string_t value);
void config_set_string(const std::string &key, const std::string &value);

// erase

bool config_erase(const string_t key);
bool config_erase(const std::string &key);

bool config_erase_wildcard(const string_t key);
bool config_erase_wildcard(const std::string &key);
