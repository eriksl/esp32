#pragma once

#include <stdint.h>
#include <sys/time.h>

#include <esp_ota_ops.h>
#include <esp_netif_ip_addr.h>

#pragma GCC poison strcpy
#pragma GCC poison strcat
#pragma GCC poison strncpy
#pragma GCC poison strncat
#pragma GCC poison printf
#pragma GCC poison sprintf
#pragma GCC poison snprintf

void util_init(void);
uint32_t util_md5_32(unsigned int length, const uint8_t *data);
unsigned int util_partition_to_slot(const esp_partition_t *partition);
void util_esp_ipv4_addr_to_string(string_t dst, const esp_ip4_addr_t *src);
void util_esp_ipv6_addr_to_string(string_t dst, const esp_ip6_addr_t *src);
void util_mac_addr_to_string(string_t dst, const uint8_t mac[6], bool inverse);
void util_time_to_string(string_t dst, const time_t *ticks);
void util_hash_to_string(string_t dst, unsigned int hash_size, const uint8_t *hash);

static inline void util_abort_on_esp_err(const char *what, int rv)
{
	if(rv)
	{
		log_format("abort: %s (%s) [0x%x]", what, esp_err_to_name(rv), rv);
		abort();
	}
}

static inline void util_warn_on_esp_err(const char *what, int rv)
{
	if(rv)
		log_format("warning: %s (%s) [0x%x]", what, esp_err_to_name(rv), rv);
}

static inline void util_abort(const char *what)
{
	log_format("abort: %s", what);
	abort();
}
