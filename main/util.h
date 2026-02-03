#pragma once

#include "log.h"

#include <stdint.h>
#include <esp_ota_ops.h>
#include <string>

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

void util_sleep(unsigned int msec);
unsigned int util_partition_to_slot(const esp_partition_t *partition);

std::string util_ipv4_addr_to_string(const uint32_t *in /* sockaddr_in->sin_addr.in_addr = uint32_t */);
std::string util_ipv6_addr_to_string(const uint8_t in[] /* sockaddr6_in->sin6_addr.in6_addr = uint8_t[16] */);
std::string util_ipv6_address_type_string(const uint8_t in[] /* sockaddr6_in->sin6_addr.s6_addr = char[16] */);
ipv6_address_type_t util_ipv6_address_type(const uint8_t in[] /* sockaddr6_in->sin6_addr.s6_addr = char[16] */);
std::string util_mac_addr_to_string(const uint8_t mac[6], bool invert);

std::string util_time_to_string(const time_t &stamp);
std::string util_time_to_string(std::string_view format, const time_t &stamp);

std::string util_hash_to_string(std::string_view hash); // FIXME -> encryption

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

static inline std::string util_esp_string_error(esp_err_t e, const std::string &message)
{
	return(message + ": " + std::to_string(e) + "\"" + esp_err_to_name(e) + "\"");
}

void util_init(void);
