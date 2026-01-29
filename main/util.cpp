#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <lwip/ip_addr.h>

#include "log.h"
#include "util.h"

#include <freertos/FreeRTOS.h>

#include <esp_timer.h>
#include <esp_netif.h>
#include <esp_netif_ip_addr.h>

#include <stddef.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <string>
#include <boost/format.hpp>

static bool inited;

static const char *ipv6_address_type_strings[ipv6_address_size] =
{
	[ipv6_address_loopback] = "loopback",
	[ipv6_address_link_local] = "link local",
	[ipv6_address_multicast] = "multicast",
	[ipv6_address_site_local] = "site local",
	[ipv6_address_ipv4_mapped] = "ipv4 mapped",
	[ipv6_address_unspecified] = "unspecified",
	[ipv6_address_global_slaac] = "slaac",
	[ipv6_address_global_static] = "static",
	[ipv6_address_other] = "other",
};

void util_init(void)
{
	assert(!inited);

	setenv("TZ", "CEST-1CET,M3.5.0/2:00:00,M10.5.0/2:00:00", 1);
	tzset();

	inited = true;
}

void util_sleep(unsigned int msec)
{
	unsigned int ticks;

	ticks = msec / portTICK_PERIOD_MS;

	assert(ticks > 0);

	vTaskDelay(ticks);
}

unsigned int util_partition_to_slot(const esp_partition_t *partition)
{
	unsigned int slot;

	assert(partition);
	assert(partition->type == ESP_PARTITION_TYPE_APP);

	if(partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0)
		slot = 0;
	else
		if(partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1)
			slot = 1;
		else
			util_abort("util_partition_to_slot: unknown OTA partition type");

	return(slot);
}

std::string util_ipv4_addr_to_string(const uint32_t *in /* sockaddr_in->sin_addr.in_addr = uint32_t */)
{
	std::string dst;
	const char *c_dst;

	assert(in);

	dst.resize(INET_ADDRSTRLEN);
	c_dst = inet_ntop(AF_INET, reinterpret_cast<const void *>(in), dst.data(), dst.size());
	dst.resize(strlen(c_dst));

	return(dst);
}

std::string util_ipv6_addr_to_string(const uint8_t in[] /* sockaddr6_in->sin6_addr.in6_addr = uint8_t[16] */)
{
	std::string dst;
	const char *c_dst;
	std::string::iterator it;

	assert(in);

	if((in[0] == 0) && (in[1] == 0) && (in[2] == 0) && (in[3] == 0) &&
			(in[4] == 0) && (in[5] == 0) && (in[6] == 0) && (in[7] == 0) &&
			(in[8] == 0) && (in[9] == 0) && (in[10] == 0xff) && (in[11] == 0xff))
		return(std::format("{:d}.{:d}.{:d}.{:d}", in[12], in[13], in[14], in[15]));

	dst.resize(INET6_ADDRSTRLEN);
	c_dst = inet_ntop(AF_INET6, in, dst.data(), dst.size());
	dst.resize(strlen(c_dst));

	for(it = dst.begin(); it != dst.end(); it++)
		switch(*it)
		{
			case('A'): { *it = 'a'; break; }
			case('B'): { *it = 'b'; break; }
			case('C'): { *it = 'c'; break; }
			case('D'): { *it = 'd'; break; }
			case('E'): { *it = 'e'; break; }
			case('F'): { *it = 'f'; break; }
			default: { break; }
		}

	return(dst);
}

ipv6_address_type_t util_ipv6_address_type(const uint8_t in[] /* sockaddr6_in->sin6_addr.s6_addr = char[16] */)
{
	struct in6_addr s6addr;

	memcpy(&s6addr.s6_addr, in, sizeof(s6addr.s6_addr));

	if(IN6_IS_ADDR_LOOPBACK(&s6addr))
		return(ipv6_address_loopback);

	if(IN6_IS_ADDR_LINKLOCAL(&s6addr))
		return(ipv6_address_link_local);

	if(IN6_IS_ADDR_MULTICAST(&s6addr))
		return(ipv6_address_multicast);

	if(IN6_IS_ADDR_SITELOCAL(&s6addr))
		return(ipv6_address_site_local);

	if(IN6_IS_ADDR_V4MAPPED(&s6addr))
		return(ipv6_address_ipv4_mapped);

	if(IN6_IS_ADDR_UNSPECIFIED(&s6addr))
		return(ipv6_address_unspecified);

	if((in[11] == 0xff) && (in[12] == 0xfe))
		return(ipv6_address_global_slaac);

	return(ipv6_address_global_static);
}

std::string util_ipv6_address_type_string(const uint8_t in[] /* sockaddr6_in->sin6_addr.s6_addr = char[16] */)
{
	ipv6_address_type_t type;

	type = util_ipv6_address_type(in);

	if((type < 0) || (type >= ipv6_address_size))
		return("<illegal>");

	return(ipv6_address_type_strings[type]);
}

void util_time_to_string(string_t dst, const time_t *ticks)
std::string util_mac_addr_to_string(const uint8_t mac[6], bool invert)
{
	struct tm tm;
	char timestring[64];
	std::string dst;

	localtime_r(ticks, &tm);
	strftime(timestring, sizeof(timestring), "%Y/%m/%d %H:%M:%S", &tm);
	if(invert)
		dst = std::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
				mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
	else
		dst = std::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
				mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	string_assign_cstr(dst, timestring);
	return(dst);
}

void util_hash_to_string(string_t dst, unsigned int hash_size, const uint8_t *hash)
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

void util_hexdump_cstr(string_t dst, unsigned int src_length, const uint8_t *src)
{
	unsigned int current, nibble, value, nibble_value;

	assert(inited);
	assert(src);
	assert(dst);

	string_clear(dst);

	for(current = 0; current < src_length; current++)
	{
		value = src[current];

		if((value > ' ') && (value <= '~'))
			string_append(dst, value);
		else
		{
			string_append_cstr(dst, "[0x");
			for(nibble = 0; nibble < 2; nibble++)
			{
				nibble_value = nibble ? (value & 0x0f) >> 0 : (value & 0xf0) >> 4;

				assert(nibble_value < 16);

				if(nibble_value >= 10)
					string_append(dst, (nibble_value - 10) + 'a');
				else
					string_append(dst, (nibble_value -  0) + '0');
			}
			string_append_cstr(dst, "]");
		}
	}
}

void util_hexdump(string_t dst, const const_string_t src)
{
	assert(src);

	util_hexdump_cstr(dst, string_length(src), string_data(src));
}

uint64_t stat_util_time_malloc_min = 0;
uint64_t stat_util_time_malloc_max = 0;
uint64_t stat_util_time_memcpy_min = 0;
uint64_t stat_util_time_memcpy_max = 0;

void *_util_memory_alloc_spiram(unsigned int amount, const char *file, unsigned int line)
{
	uint64_t time_start, time_spent;
	void *memory;

	time_start = esp_timer_get_time();

	if(amount == 0)		// heap_caps_malloc returns NULL when 0 bytes are requested
		amount = 1;

	memory = heap_caps_malloc(amount, MALLOC_CAP_SPIRAM);

	if(!memory)
	{
		log_format("util_memory_alloc_spiram: out of memory, called from: %s:%u", file, line);
		abort();
	}

	time_spent = esp_timer_get_time() - time_start;

	if(stat_util_time_malloc_min == 0)
		stat_util_time_malloc_min = time_spent;
	else
		if(stat_util_time_malloc_min > time_spent)
			stat_util_time_malloc_min = time_spent;

	if(stat_util_time_malloc_max == 0)
		stat_util_time_malloc_max = time_spent;
	else
		if(stat_util_time_malloc_max < time_spent)
			stat_util_time_malloc_max = time_spent;

	return(memory);
}

void *_util_memory_alloc_dma(unsigned int amount, const char *file, unsigned int line)
{
	uint64_t time_start, time_spent;
	void *memory;

	time_start = esp_timer_get_time();

	if(amount == 0)		// heap_caps_malloc returns NULL when 0 bytes are requested
		amount = 1;

	memory = heap_caps_malloc(amount, MALLOC_CAP_DMA);

	if(!memory)
	{
		log_format("util_memory_alloc_dma: out of memory, called from: %s:%u", file, line);
		abort();
	}

	time_spent = esp_timer_get_time() - time_start;

	if(stat_util_time_malloc_min == 0)
		stat_util_time_malloc_min = time_spent;
	else
		if(stat_util_time_malloc_min > time_spent)
			stat_util_time_malloc_min = time_spent;

	if(stat_util_time_malloc_max == 0)
		stat_util_time_malloc_max = time_spent;
	else
		if(stat_util_time_malloc_max < time_spent)
			stat_util_time_malloc_max = time_spent;

	return(memory);
}

void _util_memcpy(void *to, const void *from, unsigned int length, const char *file, unsigned int line)
{
	uint64_t time_start, time_spent;

	time_start = esp_timer_get_time();

	if(!to)
	{
		log_format("util_memcpy: to is NULL, called from: %s:%u", file, line);
		abort();
	}

	if(!from)
	{
		log_format("util_memcpy: from is NULL, called from: %s:%u", file, line);
		abort();
	}

	memcpy(to, from, length);

	time_spent = esp_timer_get_time() - time_start;

	if(stat_util_time_memcpy_min == 0)
		stat_util_time_memcpy_min = time_spent;
	else
		if(stat_util_time_memcpy_min > time_spent)
			stat_util_time_memcpy_min = time_spent;

	if(stat_util_time_memcpy_max == 0)
		stat_util_time_memcpy_max = time_spent;
	else
		if(stat_util_time_memcpy_max < time_spent)
			stat_util_time_memcpy_max = time_spent;
}
