#pragma once

enum
{
	string_header_length = 20,
};

struct string_opaque_t {} __attribute__((aligned(sizeof(int))));

typedef struct string_opaque_t *string_t;
typedef const struct string_opaque_t *const_string_t;

#define string_auto(_name, _length) \
	string_t _name = (string_t)__builtin_alloca_with_align(string_header_length + /* terminator */ 1 + _length, 8 * sizeof(int)); \
	_string_auto(_name, _length)

#define string_auto_init(_name, _string) \
	string_auto(_name, sizeof(_string)); \
	string_assign_cstr(_name, _string);

void _string_auto(string_t dst, unsigned int size);

#define string_new(l) _string_new(l, __FILE__, __LINE__)
string_t _string_new(unsigned int length, const char *file, unsigned int line);

#define string_dup(s) _string_dup(s, __FILE__, __LINE__)
string_t _string_dup(const const_string_t src, const char *file, unsigned int line);

#define string_new_from_mbuf(l) _string_new_from_mbuf(l, __FILE__, __LINE__)
string_t _string_new_from_mbuf(const void *mbuf, const char *file, unsigned int line);

#define string_free(s) _string_free(s, __FILE__, __LINE__)
void _string_free(string_t *string, const char *file, unsigned int line);

#define string_const(s) _string_const(s, __FILE__, __LINE__)
string_t _string_const(const char *const_string, const char *file, unsigned int line);

const_string_t string_empty_string(void);

void string_clear(string_t dst);
unsigned int string_length(const const_string_t src);
bool string_blank(const const_string_t src);
unsigned int string_length_utf8(const const_string_t src);
bool string_empty(const const_string_t src);
bool string_full(const const_string_t src);
unsigned int string_size(const const_string_t src);
void string_append_string(string_t dst, const const_string_t src);
void string_append_cstr(string_t dst, const char *src);
void string_append_data(string_t dst, unsigned int length, const uint8_t *data);
void string_append(string_t dst, char src);
unsigned int string_append_mbuf(string_t dst, const void *src);
void string_assign_string(string_t dst, const const_string_t src);
void string_assign_cstr(string_t dst, const char *cstr);
void string_assign(string_t dst, unsigned int offset, char src);
void string_assign_data(string_t dst, unsigned int length, const uint8_t *data);
unsigned int string_assign_mbuf(string_t dst, const void *src);
void string_cut(string_t dst, const const_string_t src, unsigned int from, unsigned int length);
void string_truncate(string_t dst, unsigned int length);
void string_format(string_t dst, const char *fmt, ...) __attribute__ ((format (printf, 2, 3)));;
void string_format_append(string_t dst, const char *fmt, ...) __attribute__ ((format (printf, 2, 3)));;;
char string_at(const const_string_t src, unsigned int offset);
char string_at_back(const const_string_t src);
char string_pop_back(string_t dst);
const char *string_cstr(const const_string_t src);
const uint8_t *string_data(const const_string_t src);
void string_to_cstr(const const_string_t src, unsigned int dst_size, char *dst);
string_t string_parse(const const_string_t src, unsigned int *offset);
bool string_equal_string(const const_string_t dst, const const_string_t src);
bool string_equal_cstr(const const_string_t dst, const char *src);
bool string_equal_data(const const_string_t dst, unsigned int length, const uint8_t *src);
bool string_uint(const const_string_t src, unsigned int base, unsigned int *value);
bool string_int(const const_string_t src, unsigned int base, int *value);
bool string_float(const const_string_t src, float *value);
void string_replace(string_t dst, unsigned int start_pos, unsigned int end_pos, char from, char to);
void string_tolower(string_t dst);
int string_read_fd(string_t dst, unsigned int fd, unsigned int length);
int string_recvfrom_fd(string_t dst, unsigned int fd, unsigned int *sockaddr_len, void *sockaddr);

void string_module_init(void);
