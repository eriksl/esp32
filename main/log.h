#pragma once

#include <sys/time.h>
#include <freertos/FreeRTOS.h>

#include <string>

#define log(s) do { log_cstr(s); } while(0)

void log_cstr(const char *string);
void log_errno(const char *string);
void log_cstr_errno(const char *string);

void log_format(const char *f, ...) __attribute__ ((format (printf, 1, 2)));
void log_format_errno(const char *f, ...) __attribute__ ((format (printf, 1, 2)));
void log_setmonitor(bool val);

QueueHandle_t log_get_display_queue(void);
void log_get_entry(unsigned int entry, time_t *stamp, std::string &dst);

void log_init(void);
