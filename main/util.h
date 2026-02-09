#pragma once

#include <string>

#include <time.h>

#include <esp_err.h>

std::string_view yesno(bool);

std::string util_time_to_string(const time_t &stamp);
std::string util_time_to_string(std::string_view format, const time_t &stamp);

void util_warn_on_esp_err(std::string_view what, unsigned int rv);
void util_abort_on_esp_err(std::string_view what, int rv);
void util_abort(std::string_view what);
std::string util_esp_string_error(esp_err_t e, const std::string &message);

void util_init(void);
