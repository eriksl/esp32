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

#define string_auto_init(_name, _string) \
	char _ ## _name ## _data[string_header_length + sizeof(_string)]; \
	string_t _name = (string_t)_ ## _name ## _data; \
	do { _string_auto(_name, sizeof(_string)); string_assign_cstr(_name, _string); } while(0)

void _string_auto(string_t dst, unsigned int size);
string_t string_new(unsigned int length);
string_t string_const(const char *const_string);
string_t string_init(unsigned int size, const char *init_string);
void string_free(string_t string);
void string_clear(string_t dst);
void string_fill(string_t dst, unsigned int length, char byte);
unsigned int string_length(const string_t src);
unsigned int string_size(const string_t src);
void string_append_string(string_t dst, const string_t src);
void string_append_cstr(string_t dst, const char *src);
void string_append(string_t dst, char src);
void string_assign_string(string_t dst, const string_t src);
void string_assign_cstr(string_t dst, const char *cstr);
void string_assign(string_t dst, unsigned int offset, char src);
void string_assign_data(string_t dst, unsigned int length, const uint8_t *data);
void string_format(string_t dst, const char *fmt, ...) __attribute__ ((format (printf, 2, 3)));;
void string_format_append(string_t dst, const char *fmt, ...) __attribute__ ((format (printf, 2, 3)));;;
char string_at(string_t dst, unsigned int offset);
const char *string_cstr(const string_t src);
char *string_cstr_nonconst(string_t src);
const uint8_t *string_data(const string_t src);
void string_to_cstr(const string_t src, unsigned int dst_size, char *dst);
string_t string_parse(const string_t src, unsigned int index);
bool string_equal_string(const string_t dst, const string_t src);
bool string_equal_cstr(const string_t dst, const char *src);
bool string_uint(const string_t src, unsigned int base, unsigned int *value);
bool string_int(const string_t src, unsigned int base, int *value);
bool string_float(const string_t src, float *value);
void string_hash(string_t dst, unsigned int hash_length, const uint8_t *hash);
void string_replace(string_t dst, unsigned int start_pos, unsigned int end_pos, char from, char to);
