#include <stdint.h>
#include <string.h>

#include "string.h"
#include "cli-command.h"
#include "log.h"
#include "util.h"

#include <freertos/FreeRTOS.h>

typedef enum
{
	string_magic_word = 0x4afb0001,
	null_byte = 1,
} string_magic_word_t;

typedef struct
{
	string_magic_word_t magic_word;
	unsigned int size;
	unsigned int length;
	struct
	{
		unsigned int header_from_malloc:1;
		unsigned int header_const:1;
		unsigned int data_const:1;
		unsigned int data_from_malloc:1;
	};
	union
	{
		char *data;
		const char *const_data;
	};
	char data_start[];
} _string_t;

static_assert(string_header_length == sizeof(_string_t));

static void _string_init(_string_t *_dst, unsigned int size, bool header_const, bool header_from_malloc, bool data_const, bool data_from_malloc)
{
	assert(_dst);

	_dst->magic_word = string_magic_word;
	_dst->size = size;
	_dst->length = 0;
	_dst->header_const = header_const ? 1 : 0;
	_dst->header_from_malloc = header_from_malloc ? 1 : 0;
	_dst->data_const = data_const ? 1 : 0;
	_dst->data_from_malloc = data_from_malloc ? 1 : 0;
	_dst->data = (char *)0;
}

void _string_auto(string_t dst, unsigned int size)
{
	_string_t *_dst = (_string_t *)dst;

	assert(_dst);
	assert(size);

	_string_init(_dst, size, false, false, false, false);
	_dst->data = _dst->data_start;
	string_clear((string_t)_dst);
}

string_t string_new(unsigned int size)
{
	_string_t *_dst;

	assert(size);

	_dst = heap_caps_malloc(sizeof(_string_t) + size, MALLOC_CAP_SPIRAM);
	assert(_dst);

	_string_init(_dst, size, false, true, false, false);
	_dst->data = _dst->data_start;
	string_clear((string_t)_dst);

	return((string_t)_dst);
}

string_t string_const(const char *const_string)
{
	_string_t *_dst;
	unsigned int length;

	assert(const_string);

	length = strlen(const_string);

	_dst = heap_caps_malloc(sizeof(_string_t), MALLOC_CAP_SPIRAM);
	assert(_dst);

	_string_init(_dst, length + null_byte, false, true, true, false);
	_dst->length = length;
	_dst->const_data = const_string;

	return((string_t)_dst);
}

string_t string_init(unsigned int size, const char *init_string)
{
	_string_t *_dst;

	assert(size > strlen(init_string));

	_dst = heap_caps_malloc(sizeof(_string_t) + size, MALLOC_CAP_SPIRAM);
	assert(_dst);

	_string_init(_dst, size, false, true, false, false);
	_dst->data = _dst->data_start;

	string_assign_cstr((string_t)_dst, init_string);

	return((string_t)_dst);
}

void string_free(string_t string)
{
	_string_t *_dst = (_string_t *)string;

	assert(_dst);
	assert(_dst->magic_word == string_magic_word);
	assert(_dst->length < _dst->size);

	if(_dst->header_const)
		return;

	_dst->magic_word = 0xffffffff;
	_dst->size = 0;
	_dst->length = 0;

	if(_dst->data_from_malloc)
	{
		assert(!_dst->data_const);
		free(_dst->data);
		_dst->data = (char *)0;
	}

	if(_dst->header_from_malloc)
		free(_dst);
}

unsigned int string_length(const string_t src)
{
	_string_t *_src = (_string_t *)src;

	assert(_src);
	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);

	return(_src->length);
}

unsigned int string_size(const string_t src)
{
	_string_t *_src = (_string_t *)src;

	assert(_src);
	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);

	return(_src->size);
}

void string_clear(string_t dst)
{
	_string_t *_dst = (_string_t *)dst;

	assert(_dst);
	assert(_dst->magic_word == string_magic_word);
	assert(!_dst->header_const);
	assert(!_dst->data_const);
	assert(_dst->length < _dst->size);
	assert(_dst->size > 0);

	_dst->length = 0;
	_dst->data[_dst->length] = '\0';
}

void string_fill(string_t dst, unsigned int length, char byte)
{
	_string_t *_dst = (_string_t *)dst;

	assert(_dst);
	assert(_dst->magic_word == string_magic_word);
	assert(!_dst->header_const);
	assert(!_dst->data_const);
	assert(_dst->length < _dst->size);
	assert(_dst->size > 0);

	if((length + null_byte) > _dst->size)
		length = _dst->size - null_byte;

	memset(_dst->data, byte, length);

	_dst->length = length;
	_dst->data[_dst->length] = '\0';

	assert(_dst->length < _dst->size);
}

void string_append_string(string_t dst, const string_t src)
{
	_string_t *_src = (_string_t *)src;
	_string_t *_dst = (_string_t *)dst;

	assert(_src);
	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);

	assert(_dst);
	assert(_dst->magic_word == string_magic_word);
	assert(_dst->length < _dst->size);
	assert(_dst->size > 0);
	assert(!_dst->header_const);
	assert(!_dst->data_const);

	string_append_cstr(dst, string_cstr(src));
}

void string_append_cstr(string_t dst, const char *src)
{
	_string_t *_dst = (_string_t *)dst;
	unsigned int length;

	assert(_dst);
	assert(_dst->magic_word == string_magic_word);
	assert(_dst->length < _dst->size);
	assert(_dst->size > 0);
	assert(!_dst->header_const);
	assert(!_dst->data_const);

	if((_dst->length + null_byte) >= _dst->size)
		return;

	length = strlen(src);

	if((_dst->length + length + null_byte) > _dst->size)
		length = _dst->size - (_dst->length + null_byte);

	memcpy(_dst->data + _dst->length, src, length);
	_dst->length += length;
	_dst->data[_dst->length] = '\0';

	assert(_dst->length < _dst->size);
}

void string_assign_cstr(string_t dst, const char *cstr)
{
	string_clear(dst);
	string_append_cstr(dst, cstr);
}

void string_assign_data(string_t dst, unsigned int length, const uint8_t *src)
{
	_string_t *_dst = (_string_t *)dst;

	assert(_dst);
	assert(_dst->magic_word == string_magic_word);
	assert(_dst->length < _dst->size);
	assert(_dst->size > 0);
	assert(!_dst->header_const);
	assert(!_dst->data_const);

	if((_dst->length + null_byte) >= _dst->size)
		return;

	if((length + null_byte) > _dst->size)
		length = _dst->size - null_byte;

	memcpy(_dst->data, src, length);

	_dst->length = length;
	_dst->data[_dst->length] = '\0';

	assert(_dst->length < _dst->size);
}

static void _string_format(string_t dst, bool append, const char *fmt, va_list ap)
{
	_string_t *_dst = (_string_t *)dst;
	unsigned int size;
	int length;

	assert(_dst);
	assert(_dst->magic_word == string_magic_word);
	assert(!_dst->header_const);
	assert(!_dst->data_const);
	assert(_dst->length < _dst->size);
	assert(_dst->size > 0);

	if(!append)
		string_clear(dst);

	size = _dst->size - _dst->length;

	assert(size > 0);

	length = vsnprintf(_dst->data + _dst->length, size, fmt, ap);

	assert(length >= 0);

	if(length >= size)
		length = size - null_byte;

	_dst->length += length;

	assert(_dst->length < _dst->size);
	assert(_dst->data[_dst->length] == '\0');
}

void string_format_append(string_t dst, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	_string_format(dst, true, fmt, ap);
	va_end(ap);
}

void string_format(string_t dst, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	_string_format(dst, false, fmt, ap);
	va_end(ap);
}

const char *string_cstr(const string_t src)
{
	_string_t *_src = (_string_t *)src;

	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);
	assert(_src->data[_src->length] == '\0');

	return(_src->data);
}

char *string_cstr_nonconst(string_t src)
{
	_string_t *_src = (_string_t *)src;

	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);
	assert(_src->data[_src->length] == '\0');
	assert(!_src->header_const);
	assert(!_src->data_const);

	return(_src->data);
}

const uint8_t *string_data(const string_t src)
{
	_string_t *_src = (_string_t *)src;

	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);
	assert(_src->data[_src->length] == '\0');

	return((const uint8_t *)_src->data);
}

void string_to_cstr(const string_t src, unsigned int dst_size, char *dst)
{
	unsigned int length;
	_string_t *_src = (_string_t *)src;

	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);
	assert(_src->data[_src->length] == '\0');
	assert(dst_size > 0);

	if(((length = _src->length) + null_byte) > dst_size)
		length = dst_size - null_byte;

	memcpy(dst, _src->data, length);
	dst[length] = '\0';
}

char string_at(const string_t src, unsigned int offset)
{
	_string_t *_src = (_string_t *)src;

	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);

	if(offset >= _src->length)
		return('\0');

	return(_src->data[offset]);
}

void string_assign(string_t dst, unsigned int offset, char src)
{
	_string_t *_dst = (_string_t *)dst;

	assert(_dst->magic_word == string_magic_word);
	assert(_dst->length < _dst->size);
	assert(_dst->size > 0);
	assert(!_dst->header_const);
	assert(!_dst->data_const);

	if(offset >= _dst->length)
		return;

	_dst->data[offset] = src;
}

void string_append(string_t dst, char src)
{
	_string_t *_dst = (_string_t *)dst;

	assert(_dst->magic_word == string_magic_word);
	assert(_dst->length < _dst->size);
	assert(_dst->size > 0);
	assert(!_dst->header_const);
	assert(!_dst->data_const);

	if((_dst->length + null_byte) < _dst->size)
	{
		_dst->data[_dst->length++] = src;
		_dst->data[_dst->length] = '\0';
	}
}
