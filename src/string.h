#pragma once

enum
{
	string_header_length = 20,
};

typedef struct string_opaque_t {} *string_t;

#define string_auto(_name, _length) \
	char _ ## _name ## _data[string_header_length + _length]; \
	string_t _name = (string_t)_ ## _name ## _data; \
	do { _string_auto(_name, _length); } while(0)

void _string_auto(string_t dst, unsigned int size);
string_t string_new(unsigned int length);
string_t string_const(const char *const_string);
string_t string_init(unsigned int size, const char *init_string);
void string_free(string_t string);
void string_clear(string_t dst);
unsigned int string_length(const string_t src);
void string_append_string(string_t dst, const string_t src);
void string_append_cstr(string_t dst, const char *src);
void string_assign_cstr(string_t dst, const char *cstr);
void string_format(string_t dst, const char *fmt, ...) __attribute__ ((format (printf, 2, 3)));;
void string_format_append(string_t dst, const char *fmt, ...) __attribute__ ((format (printf, 2, 3)));;;
char string_at(string_t dst, unsigned int offset);
void string_assign(string_t dst, unsigned int offset, char src);
void string_append(string_t dst, char src);
const char *string_cstr(const string_t src);
void string_to_cstr(const string_t src, unsigned int dst_size, char *dst);


