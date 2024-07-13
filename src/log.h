#pragma once

void log_init(void);

void _log_cstr(bool append_strerror, const char *string);
void _log_format(bool append_strerror, const char *f, ...) __attribute__ ((format (printf, 2, 3)));
#define log(s) do { _log_cstr(false, s); } while(0)
#define log_cstr(s) do { _log_cstr(false, s); } while(0)
#define log_errno(s) do { _log_cstr(true, s); } while(0)
#define log_cstr_errno(s) do { _log_cstr(true, s); } while(0)
#define log_format(f, ...) do { _log_format(false, f, __VA_ARGS__); } while(0)
#define log_format_errno(f, ...) do { _log_format(true, f, __VA_ARGS__); } while(0)
