#pragma once

#include <string>

bool packet_valid(const std::string &data);
bool packet_complete(const std::string &data);
void packet_encapsulate(command_response_t *dst, const std::string &data, const std::string &oob_data);
void packet_decapsulate(const command_response_t *src, std::string &data, std::string &oob_data);
