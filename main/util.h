#pragma once

#include <stdint.h>
#include <sys/time.h>

#include "string.h"
#include "log.h"

#include <esp_ota_ops.h>
#include <esp_netif_ip_addr.h>

#include <string>

#if 0 // FIXME
#pragma GCC poison strcpy
#pragma GCC poison strcat
#pragma GCC poison strncpy
#pragma GCC poison strncat
#pragma GCC poison printf
#pragma GCC poison sprintf
#endif

extern uint64_t stat_util_time_malloc_min;
extern uint64_t stat_util_time_malloc_max;
extern uint64_t stat_util_time_memcpy_min;
extern uint64_t stat_util_time_memcpy_max;

typedef enum
{
	ipv6_address_loopback,
	ipv6_address_link_local,
	ipv6_address_multicast,
	ipv6_address_site_local,
	ipv6_address_ipv4_mapped,
	ipv6_address_unspecified,
	ipv6_address_global_slaac,
	ipv6_address_global_static,
	ipv6_address_other,
	ipv6_address_size,
} ipv6_address_type_t;

#define util_memory_alloc_spiram(amount) _util_memory_alloc_spiram(amount, __FILE__, __LINE__)
#define util_memory_alloc_dma(amount) _util_memory_alloc_dma(amount, __FILE__, __LINE__)
#define util_memcpy(to, from, length) do { _util_memcpy(to, from, length, __FILE__, __LINE__); } while(0)

void util_sleep(unsigned int msec);
unsigned int util_partition_to_slot(const esp_partition_t *partition);

std::string util_ipv4_addr_to_string(const uint32_t *in /* sockaddr_in->sin_addr.in_addr = uint32_t */);
std::string util_ipv6_addr_to_string(const uint8_t in[] /* sockaddr6_in->sin6_addr.in6_addr = uint8_t[16] */);
std::string util_ipv6_address_type_string(const uint8_t in[] /* sockaddr6_in->sin6_addr.s6_addr = char[16] */);
ipv6_address_type_t util_ipv6_address_type(const uint8_t in[] /* sockaddr6_in->sin6_addr.s6_addr = char[16] */);
std::string util_mac_addr_to_string(const uint8_t mac[6], bool invert);

std::string util_time_to_string(const time_t &stamp);
std::string util_time_to_string(std::string_view format, const time_t &stamp);

void util_hash_to_string(string_t dst, unsigned int hash_size, const uint8_t *hash);
void *_util_memory_alloc_spiram(unsigned int amount, const char *file, unsigned int line);
void *_util_memory_alloc_dma(unsigned int amount, const char *file, unsigned int line);
void _util_memcpy(void *to, const void *from, unsigned int length, const char *file, unsigned int line);
void util_hexdump_cstr(string_t dst, unsigned int src_length, const uint8_t *src);
void util_hexdump(string_t dst, const const_string_t src);

static inline void util_abort_on_esp_err(const char *what, unsigned int rv)
{
	if(rv)
	{
		log_setmonitor(true);
		log_format("abort: %s (%s) [0x%x]", what, esp_err_to_name(rv), rv);
		abort();
	}
}

static inline void util_warn_on_esp_err(const char *what, unsigned int rv)
{
	if(rv)
		log_format("warning: %s (%s) [0x%x]", what, esp_err_to_name(rv), rv);
}

static inline void util_abort(const char *what)
{
	log_setmonitor(true);
	log_format("abort: %s", what);
	abort();
}

void util_init(void);
