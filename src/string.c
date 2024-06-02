#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include "string.h"
#include "cli-command.h"
#include "stringcli.h"
#include "log.h"
#include "util.h"

#include <freertos/FreeRTOS.h>

//#define DEBUG 1

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

bool inited = false;
unsigned int allocated = 0;
unsigned int freed = 0;
unsigned int auto_called = 0;
unsigned int new_called = 0;
unsigned int const_called = 0;
unsigned int init_called = 0;

static_assert(string_header_length == sizeof(_string_t));

void string_module_init(void)
{
	assert(!inited);

	inited = true;
}

static void _string_init_header(_string_t *_dst, unsigned int size, bool header_const, bool header_from_malloc, bool data_const, bool data_from_malloc)
{
	assert(inited);
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

	assert(inited);
	assert(_dst);
	assert(size);

	auto_called++;

	_string_init_header(_dst, size, false, false, false, false);
	_dst->data = _dst->data_start;
	string_clear((string_t)_dst);
}

string_t _string_new(unsigned int size, const char *file, unsigned int line)
{
	_string_t *_dst;

	assert(inited);
	assert(size);

#ifdef DEBUG
	log_format("string_new: %u @ %s:%u", size, file, line);
#endif

	_dst = heap_caps_malloc(sizeof(_string_t) + size, MALLOC_CAP_SPIRAM);
	assert(_dst);

	_string_init_header(_dst, size, false, true, false, false);
	_dst->data = _dst->data_start;
	string_clear((string_t)_dst);

	new_called++;
	allocated++;

	return((string_t)_dst);
}

string_t _string_const(const char *const_string, const char *file, unsigned int line)
{
	_string_t *_dst;
	unsigned int length;

	assert(inited);
	assert(const_string);

#ifdef DEBUG
	log_format("string_const: \"%s\" @ %s:%u", const_string, file, line);
#endif

	length = strlen(const_string);

	_dst = heap_caps_malloc(sizeof(_string_t), MALLOC_CAP_SPIRAM);
	assert(_dst);

	_string_init_header(_dst, length + null_byte, false, true, true, false);
	_dst->length = length;
	_dst->const_data = const_string;

	const_called++;
	allocated++;

	return((string_t)_dst);
}

string_t _string_init(unsigned int size, const char *init_string, const char *file, unsigned int line)
{
	_string_t *_dst;

	assert(inited);
	assert(size > strlen(init_string));

#ifdef DEBUG
	log_format("string_init: \"%s\"[%u] @ %s:%u", init_string, size, file, line);
#endif

	_dst = heap_caps_malloc(sizeof(_string_t) + size, MALLOC_CAP_SPIRAM);
	assert(_dst);

	_string_init_header(_dst, size, false, true, false, false);
	_dst->data = _dst->data_start;

	string_assign_cstr((string_t)_dst, init_string);

	init_called++;
	allocated++;

	return((string_t)_dst);
}

void _string_free(string_t string, const char *file, unsigned int line)
{
	_string_t *_dst = (_string_t *)string;

	assert(inited);
	assert(_dst);
	assert(_dst->magic_word == string_magic_word);
	assert(_dst->length < _dst->size);

#ifdef DEBUG
	log_format("string_free: %u @ %s:%u", _dst->size, file, line);
#endif

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

	freed++;
}

unsigned int string_length(const string_t src)
{
	_string_t *_src = (_string_t *)src;

	assert(inited);
	assert(_src);
	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);

	return(_src->length);
}

void string_set_length(string_t dst, unsigned int length)
{
	_string_t *_dst = (_string_t *)dst;

	assert(inited);
	assert(_dst);
	assert(_dst->magic_word == string_magic_word);
	assert(_dst->size > 0);
	assert(!_dst->header_const);
	assert(!_dst->data_const);
	
	if(length > (_dst->size + null_byte))
		length = _dst->size - null_byte;

	_dst->length = length;
	_dst->data[length] = '\0';
}

unsigned int string_size(const string_t src)
{
	_string_t *_src = (_string_t *)src;

	assert(inited);
	assert(_src);
	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);

	return(_src->size);
}

void string_clear(string_t dst)
{
	_string_t *_dst = (_string_t *)dst;

	assert(inited);
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

	assert(inited);
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
	unsigned int length;
	_string_t *_src = (_string_t *)src;
	_string_t *_dst = (_string_t *)dst;

	assert(inited);

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

	if((_dst->length + null_byte) >= _dst->size)
		return;

	length = _src->length;

	if((_dst->length + length + null_byte) > _dst->size)
		length = _dst->size - (_dst->length + null_byte);

	memcpy(_dst->data + _dst->length, _src->data, length);
	_dst->length += length;
	_dst->data[_dst->length] = '\0';

	assert(_dst->length < _dst->size);
}

void string_append_cstr(string_t dst, const char *src)
{
	_string_t *_dst = (_string_t *)dst;
	unsigned int length;

	assert(inited);
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

void string_append(string_t dst, char src)
{
	_string_t *_dst = (_string_t *)dst;

	assert(inited);
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

void string_assign_string(string_t dst, const string_t src)
{
	string_clear(dst);
	string_append_string(dst, src);
}

void string_assign_cstr(string_t dst, const char *cstr)
{
	string_clear(dst);
	string_append_cstr(dst, cstr);
}

void string_assign_data(string_t dst, unsigned int length, const uint8_t *src)
{
	_string_t *_dst = (_string_t *)dst;

	assert(inited);
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

	assert(inited);
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

	assert(inited);
	assert(_src);
	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);
	assert(_src->data[_src->length] == '\0');

	return(_src->data);
}

char *string_cstr_nonconst(string_t src)
{
	_string_t *_src = (_string_t *)src;

	assert(inited);
	assert(_src);
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

	assert(inited);
	assert(_src);
	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);
	assert(_src->data[_src->length] == '\0');

	return((const uint8_t *)_src->data);
}

uint8_t *string_data_nonconst(string_t src)
{
	_string_t *_src = (_string_t *)src;

	assert(inited);
	assert(_src);
	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);
	assert(_src->data[_src->length] == '\0');
	assert(!_src->header_const);
	assert(!_src->data_const);

	return((uint8_t *)_src->data);
}

void string_to_cstr(const string_t src, unsigned int dst_size, char *dst)
{
	unsigned int length;
	_string_t *_src = (_string_t *)src;

	assert(inited);
	assert(_src);
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

	assert(inited);
	assert(_src);
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

	assert(inited);
	assert(_dst);
	assert(_dst->magic_word == string_magic_word);
	assert(_dst->length < _dst->size);
	assert(_dst->size > 0);
	assert(!_dst->header_const);
	assert(!_dst->data_const);

	if(offset >= _dst->length)
		return;

	_dst->data[offset] = src;
}

string_t string_parse(const string_t src, unsigned int index)
{
	unsigned int offset, current_index, start, end;
	_string_t *_src = (_string_t *)src;
	string_t dst;

	assert(inited);
	assert(_src);
	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);
	assert(_src->data[_src->length] == '\0');

	offset = 0;
	start = 0;
	end = 0;

	for(current_index = 0; (offset < _src->length) && (current_index <= index); current_index++)
	{
		for(; offset < _src->length; offset++)
			if((_src->data[offset] != ' ') &&
					(_src->data[offset] != '\t') &&
					(_src->data[offset] != '\n') &&
					(_src->data[offset] != '\r'))
				break;

		if(offset >= _src->length)
			return((string_t)0);

		start = offset;

		for(; offset < _src->length; offset++)
			if((_src->data[offset] == ' ') ||
					(_src->data[offset] == '\t') ||
					(_src->data[offset] == '\n') ||
					(_src->data[offset] == '\r'))
				break;

		end = offset;
	}

	if(current_index != (index + 1))
		return((string_t)0);

	dst = string_new(end - start + null_byte);
	string_assign_data(dst, end - start, (uint8_t *)&_src->data[start]);

	return(dst);
}

bool string_equal_string(const string_t dst, const string_t src)
{
	_string_t *_src = (_string_t *)src;
	_string_t *_dst = (_string_t *)dst;

	assert(inited);

	assert(_src);
	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);

	assert(_dst);
	assert(_dst->magic_word == string_magic_word);
	assert(_dst->length < _dst->size);
	assert(_dst->size > 0);

	if(_src->length != _dst->length)
		return(false);

	if(memcmp(_src->data, _dst->data, _src->length))
		return(false);

	return(true);
}

bool string_equal_cstr(const string_t dst, const char *src)
{
	unsigned int length;
	_string_t *_dst = (_string_t *)dst;

	assert(inited);
	assert(src);

	assert(_dst->magic_word == string_magic_word);
	assert(_dst->length < _dst->size);
	assert(_dst->size > 0);

	length = strlen(src);

	if(length != _dst->length)
		return(false);

	if(memcmp(_dst->data, src, length))
		return(false);

	return(true);
}

bool string_uint(const string_t src, unsigned int base, unsigned int *value)
{
	_string_t *_src = (_string_t *)src;
	char *endptr;

	assert(inited);
	assert(value);
	assert(_src);
	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);

	if(_src->length == 0)
	{
		*value = 0;
		return(false);
	}

	errno = 0;

	*value = strtoul(_src->data, &endptr, base);

	if(errno || !endptr || (*endptr != '\0'))
	{
		*value = 0;
		return(false);
	}

	return(true);
}

bool string_int(const string_t src, unsigned int base, int *value)
{
	_string_t *_src = (_string_t *)src;
	char *endptr;

	assert(inited);
	assert(value);
	assert(_src);
	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);

	if(_src->length == 0)
	{
		*value = 0;
		return(false);
	}

	errno = 0;

	*value = strtol(_src->data, &endptr, base);

	if(errno || !endptr || (*endptr != '\0'))
	{
		*value = 0;
		return(false);
	}

	return(true);
}

bool string_float(const string_t src, float *value)
{
	_string_t *_src = (_string_t *)src;
	char *endptr;

	assert(inited);
	assert(value);
	assert(_src);
	assert(_src->magic_word == string_magic_word);
	assert(_src->length < _src->size);
	assert(_src->size > 0);

	if(_src->length == 0)
	{
		*value = 0;
		return(false);
	}

	errno = 0;

	*value = strtof(_src->data, &endptr);

	if(errno || !endptr || (*endptr != '\0'))
	{
		*value = 0;
		return(false);
	}

	return(true);
}

void string_hash(string_t dst, unsigned int hash_size, const uint8_t *hash)
{
	unsigned int in, out, value;

	assert(inited);
	assert(dst);
	assert(hash);

	string_clear(dst);

	for(in = 0, out = 0; in < hash_size; out++)
	{
		if(out & 0x1)
		{
			value = (hash[in] & 0x0f) >> 0;
			in++;
		}
		else
			value = (hash[in] & 0xf0) >> 4;

		if(value >= 0xa)
			string_append(dst, (value - 10) + 'a');
		else
			string_append(dst, (value -  0) + '0');
	}
}

void string_replace(string_t dst, unsigned int start_pos, unsigned int end_pos, char from, char to)
{
	unsigned int ix;
	_string_t *_dst = (_string_t *)dst;

	assert(inited);
	assert(_dst);
	assert(_dst->magic_word == string_magic_word);
	assert(_dst->length < _dst->size);
	assert(_dst->size > 0);
	assert(_dst->data[_dst->length] == '\0');
	assert(!_dst->header_const);
	assert(!_dst->data_const);

	if(start_pos > _dst->length)
		return;

	if(_dst->length < 1)
		return;

	if(end_pos >= _dst->length)
		end_pos = _dst->length - 1;

	for(ix = start_pos; ix <= end_pos; ix++)
		if(_dst->data[ix] == from)
			_dst->data[ix] = to;
}

void command_info_string(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	string_format(call->result, "STRING\nstats:\n- allocate events: %u\n- free events: %u\n- active: %u", allocated, freed, allocated - freed);
	string_format_append(call->result, "\nmethods called:\n- auto: %u\n- new: %u\n- init: %u\n- const: %u", auto_called, new_called, init_called, const_called);
}
