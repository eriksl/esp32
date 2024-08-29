#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>

#include "string.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"

#include <freertos/FreeRTOS.h>

#include <unistd.h>
#include <host/ble_hs_mbuf.h>
#include <os/os_mbuf.h>

typedef enum
{
	string_magic_word = 0x4afb0001,
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

static bool inited = false;
static unsigned int allocated = 0;
static unsigned int freed = 0;
static unsigned int auto_called = 0;
static unsigned int new_called = 0;
static unsigned int const_called = 0;
static unsigned int init_called = 0;
static uint64_t string_parse_time_min = 0;
static uint64_t string_parse_time_max = 0;

static const _string_t empty_string =
{
	.magic_word = string_magic_word,
	.size = 1,
	.length = 0,
	.header_from_malloc = 0,
	.header_const = 1,
	.data_const = 1,
	.data_from_malloc = 0,
	.const_data = ""
};

static void _string_check_const(const _string_t *string)
{
	assert(inited);
	assert(string);
	assert(string->magic_word == string_magic_word);
	assert(string->length <= string->size);
	assert(string->data[string->length] == '\0');
}

static void _string_check(const _string_t *string)
{
	_string_check_const(string);

	assert(!string->header_const);
	assert(!string->data_const);
}

void string_module_init(void)
{
	assert(!inited);

	inited = true;
}

const_string_t string_empty_string(void)
{
	return((string_t)&empty_string);
}

static void _string_init_header(_string_t *_dst, unsigned int size, bool header_const, bool header_from_malloc, bool data_const, bool data_from_malloc)
{
	assert(inited);
	assert(_dst);

	_dst->magic_word = string_magic_word;
	_dst->size = size; // allocate one byte more than size, so data[size] can be \0
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

	auto_called++;

	_string_init_header(_dst, size, false, false, false, false);
	_dst->data = _dst->data_start;
	_dst->data[0] = '\0';
}

string_t _string_new(unsigned int size, const char *file, unsigned int line)
{
	_string_t *_dst;

	assert(inited);

	_dst = util_memory_alloc_spiram(sizeof(_string_t) + size + 1); // NOTE: length < size and [length + 1] must be able to contain \0

	assert(_dst);

	_string_init_header(_dst, size, false, true, false, false);
	_dst->data = _dst->data_start;
	_dst->data[0] = '\0';

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

	length = strlen(const_string);

	_dst = util_memory_alloc_spiram(sizeof(_string_t));

	assert(_dst);

	_string_init_header(_dst, length, false, true, true, false);
	_dst->length = length;
	_dst->const_data = const_string;

	const_called++;
	allocated++;

	return((string_t)_dst);
}

void _string_free(string_t *string, const char *file, unsigned int line)
{
	_string_t *_dst;

	assert(inited);
	assert(string);

	_dst = *(_string_t **)string;

	if(!_dst)
	{
		log_format("string_free(null) at %s:%d", file, line);
		return;
	}

	_string_check_const(_dst);

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

	*string = (string_t)0;
}

unsigned int string_length(const const_string_t src)
{
	_string_t *_src = (_string_t *)src;

	_string_check_const(_src);

	return(_src->length);
}

unsigned int string_size(const const_string_t src)
{
	_string_t *_src = (_string_t *)src;

	_string_check_const(_src);

	return(_src->size);
}

bool string_empty(const const_string_t src)
{
	return(string_length(src) == 0);
}

bool string_full(const const_string_t src)
{
	return(string_length(src) >= string_size(src));
}

void string_clear(string_t dst)
{
	_string_t *_dst = (_string_t *)dst;

	_string_check(_dst);

	_dst->length = 0;
	_dst->data[_dst->length] = '\0';
}

void string_append_data(string_t dst, unsigned int length, const uint8_t *src)
{
	_string_t *_dst = (_string_t *)dst;

	_string_check(_dst);

	if((_dst->length + length) > _dst->size)
		length = _dst->size - _dst->length;

	util_memcpy(&_dst->data[_dst->length], src, length);

	_dst->length += length;
	_dst->data[_dst->length] = '\0';

	assert(_dst->length <= _dst->size);
}

void string_append_string(string_t dst, const const_string_t src)
{
	_string_t *_src = (_string_t *)src;

	_string_check_const(_src);

	string_append_data(dst, string_length(src), string_data(src));
}

void string_append_cstr(string_t dst, const char *src)
{
	string_append_data(dst, strlen(src), (const uint8_t *)src);
}

void string_append(string_t dst, char src)
{
	string_append_data(dst, 1, (const uint8_t *)&src);
}

unsigned int string_append_mbuf(string_t dst, const void *src)
{
	unsigned int length;
	uint16_t om_length;
	_string_t *_dst = (_string_t *)dst;
	const struct os_mbuf *_src = (const struct os_mbuf *)src;

	_string_check(_dst);

	length = os_mbuf_len(_src);

	if((_dst->length + length) > _dst->size)
		length = _dst->size - _dst->length;

	ble_hs_mbuf_to_flat(_src, _dst->data + _dst->length, length, &om_length);

	assert(om_length == length);

	_dst->length += length;

	assert(_dst->length <= _dst->size);
	_dst->data[_dst->length] = '\0';

	return(length);
}

void string_assign_string(string_t dst, const const_string_t src)
{
	string_clear(dst);
	string_append_string(dst, src);
}

void string_assign_cstr(string_t dst, const char *cstr)
{
	string_clear(dst);
	string_append_cstr(dst, cstr);
}

void string_assign_data(string_t dst, unsigned int length, const uint8_t *data)
{
	string_clear(dst);
	string_append_data(dst, length, data);
}

void string_assign(string_t dst, unsigned int offset, char src)
{
	string_clear(dst);
	string_append(dst, src);
}

unsigned int string_assign_mbuf(string_t dst, const void *src)
{
	string_clear(dst);
	return(string_append_mbuf(dst, src));
}

static void _string_format(string_t dst, bool append, const char *fmt, va_list ap)
{
	_string_t *_dst = (_string_t *)dst;
	unsigned int size;
	int length;

	assert(fmt);
	assert(_dst->size >= _dst->length);

	_string_check(_dst);

	if(!append)
		string_clear(dst);

	size = _dst->size - _dst->length;

	length = vsnprintf(_dst->data + _dst->length, size, fmt, ap);

	assert(length >= 0);

	_dst->length += length;

	if(_dst->length > _dst->size)
		_dst->length = _dst->size;

	assert(_dst->length <= _dst->size);

	_dst->data[_dst->length] = '\0';
}

void string_format_append(string_t dst, const char *fmt, ...)
{
	va_list ap;

	assert(fmt);

	va_start(ap, fmt);
	_string_format(dst, true, fmt, ap);
	va_end(ap);
}

void string_format(string_t dst, const char *fmt, ...)
{
	va_list ap;

	assert(fmt);

	va_start(ap, fmt);
	_string_format(dst, false, fmt, ap);
	va_end(ap);
}

const char *string_cstr(const const_string_t src)
{
	_string_t *_src = (_string_t *)src;

	_string_check_const(_src);

	return(_src->const_data);
}

const uint8_t *string_data(const const_string_t src)
{
	return((const uint8_t *)string_cstr(src));
}

void string_to_cstr(const const_string_t src, unsigned int dst_size, char *dst)
{
	unsigned int length;
	_string_t *_src = (_string_t *)src;

	_string_check_const(_src);
	assert(dst_size > 0);

	length = _src->length;

	if(length > (dst_size - 1))
		length = dst_size - 1;

	util_memcpy(dst, _src->const_data, length);
	dst[length] = '\0';
}

char string_at(const const_string_t src, unsigned int offset)
{
	_string_t *_src = (_string_t *)src;

	_string_check_const(_src);

	if(offset >= _src->length)
		return('\0');

	return(_src->data[offset]);
}

char string_at_tail(const const_string_t src)
{
	unsigned int length;
	_string_t *_src = (_string_t *)src;

	_string_check_const(_src);

	length = string_length(src);

	if(length < 1)
		return('\0');

	return(string_at(src, length - 1));
}

void string_cut(string_t dst, const const_string_t src, unsigned int from, unsigned int length)
{
	const _string_t *_src = (const _string_t *)src;
	_string_t *_dst = (_string_t *)dst;

	_string_check_const(_src);
	_string_check(_dst);

	if((from + length) > _src->length)
		length = _src->length - from;

	if(length > _dst->size)
		length = _dst->size;

	util_memcpy(_dst->data, &_src->data[from], length);

	_dst->length = length;
	_dst->data[_dst->length] = '\0';
}

void string_truncate(string_t dst, unsigned int length)
{
	_string_t *_dst = (_string_t *)dst;

	_string_check(_dst);

	if(length >= _dst->length)
		return;

	_dst->length = length;
	_dst->data[_dst->length] = '\0';
}

string_t string_parse(const const_string_t src, unsigned int *offset)
{
	unsigned int start, length;
	_string_t *_src = (_string_t *)src;
	string_t dst;

	uint64_t time_start, time_spent;

	time_start = esp_timer_get_time();

	_string_check_const(_src);

	if(*offset >= _src->length)
	{
		dst = (string_t)0;
		goto error;
	}

	for(; *offset < _src->length; (*offset)++)
		if(_src->data[*offset] > ' ')
			break;

	start = *offset;

	for(length = 0; *offset < _src->length; (*offset)++, length++)
		if(_src->data[*offset] <= ' ')
			break;

	assert((start + length) <= _src->length);

	if(length == 0)
	{
		dst = (string_t)0;
		goto error;
	}

	dst = string_new(length);
	string_cut(dst, src, start, length);

error:
	time_spent = esp_timer_get_time() - time_start;

	if(string_parse_time_min == 0)
		string_parse_time_min = time_spent;
	else
		if(string_parse_time_min > time_spent)
			string_parse_time_min = time_spent;

	if(string_parse_time_max == 0)
		string_parse_time_max = time_spent;
	else
		if(string_parse_time_max < time_spent)
			string_parse_time_max = time_spent;

	return(dst);
}

bool string_equal_data(const const_string_t dst, unsigned int length, const uint8_t *src)
{
	_string_t *_dst = (_string_t *)dst;

	assert(src);
	_string_check_const(_dst);

	if(length != _dst->length)
		return(false);

	if(memcmp(_dst->const_data, src, length))
		return(false);

	return(true);
}

bool string_equal_cstr(const const_string_t dst, const char *src)
{
	assert(src);
	return(string_equal_data(dst, strlen(src), (const uint8_t *)src));
}

bool string_equal_string(const const_string_t dst, const const_string_t src)
{
	_string_t *_src = (_string_t *)src;
	_string_t *_dst = (_string_t *)dst;

	_string_check_const(_src);
	_string_check_const(_dst);

	return(string_equal_data(dst, string_length(src), string_data(src)));
}

bool string_uint(const const_string_t src, unsigned int base, unsigned int *value)
{
	_string_t *_src = (_string_t *)src;
	char *endptr;

	_string_check_const(_src);

	if(_src->length == 0)
	{
		*value = 0;
		return(false);
	}

	errno = 0;

	*value = strtoul(_src->const_data, &endptr, base);

	if(errno || !endptr || (*endptr != '\0'))
	{
		*value = 0;
		return(false);
	}

	return(true);
}

bool string_int(const const_string_t src, unsigned int base, int *value)
{
	_string_t *_src = (_string_t *)src;
	char *endptr;

	_string_check_const(_src);

	if(_src->length == 0)
	{
		*value = 0;
		return(false);
	}

	errno = 0;

	*value = strtol(_src->const_data, &endptr, base);

	if(errno || !endptr || (*endptr != '\0'))
	{
		*value = 0;
		return(false);
	}

	return(true);
}

bool string_float(const const_string_t src, float *value)
{
	_string_t *_src = (_string_t *)src;
	char *endptr;

	_string_check_const(_src);

	if(_src->length == 0)
	{
		*value = 0;
		return(false);
	}

	errno = 0;

	*value = strtof(_src->const_data, &endptr);

	if(errno || !endptr || (*endptr != '\0'))
	{
		*value = 0;
		return(false);
	}

	return(true);
}

void string_replace(string_t dst, unsigned int start_pos, unsigned int end_pos, char from, char to)
{
	unsigned int ix;
	_string_t *_dst = (_string_t *)dst;

	_string_check(_dst);

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

void string_tolower(string_t dst)
{
	unsigned int ix;
	_string_t *_dst = (_string_t *)dst;

	_string_check(_dst);

	for(ix = 0; ix <= _dst->length; ix++)
		if((_dst->data[ix] >= 'A') && (_dst->data[ix] <= 'Z'))
			_dst->data[ix] += 'a' - 'A';
}

int string_read_fd(string_t dst, unsigned int fd, unsigned int length)
{
	int rv;
	_string_t *_dst = (_string_t *)dst;

	_string_check(_dst);

	if(length > _dst->size)
		length = _dst->size;

	rv = read(fd, _dst->data, length);

	if(rv < 0)
		_dst->length = 0;
	else
		_dst->length = (unsigned int)rv;

	assert(_dst->length <= _dst->size);
	_dst->data[_dst->length] = '\0';

	return(rv);
}

int string_recvfrom_fd(string_t dst, unsigned int fd, unsigned int *sockaddr_len, void *sockaddr)
{
	unsigned int length;
	int rv;
	_string_t *_dst = (_string_t *)dst;

	_string_check(_dst);

	length = _dst->size - _dst->length;

	rv = recvfrom(fd, &_dst->data[_dst->length], length, 0, (struct sockaddr *)sockaddr, (socklen_t *)sockaddr_len);

	if(rv > 0)
		_dst->length += rv;

	assert(_dst->length <= _dst->size);
	_dst->data[_dst->length] = '\0';

	return(rv);
}

void string_command_info(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	string_format(call->result, "STRING");
	string_format_append(call->result, "\nstats:\n- allocate events: %u\n- free events: %u\n- active: %u", allocated, freed, allocated - freed);
	string_format_append(call->result, "\nmethods called:\n- auto: %u\n- new: %u\n- init: %u\n- const: %u", auto_called, new_called, init_called, const_called);
	string_format_append(call->result, "\ntimings:\n- string_parse min: %llu microseconds\n- string_parse max: %llu microseconds", string_parse_time_min, string_parse_time_max);
}
