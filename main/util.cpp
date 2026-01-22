#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <lwip/ip_addr.h>

#include "string.h"
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
	[ipv6_address_link_local] =		"link local",
	[ipv6_address_global_slaac] =	"slaac",
	[ipv6_address_global_static] =	"static",
	[ipv6_address_other] =			"other",
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

void util_esp_ipv4_addr_to_string(string_t dst, const esp_ip4_addr_t *src)
{
	string_format(dst, IPSTR, IP2STR(src));
}

void util_esp_ipv6_addr_to_string(string_t dst, const esp_ip6_addr_t *src)
{
	string_format(dst, "%s", ip6addr_ntoa((const ip6_addr_t *)src));
	string_tolower(dst);
}

bool util_sin6_addr_is_ipv4(const void *in)
{
	const struct sockaddr_in6 *sockaddr_in6 = (const struct sockaddr_in6 *)in;
	const struct in6_addr *addr_in6;
	const uint8_t *addr;

	if(sockaddr_in6->sin6_family == AF_INET)
		return(true);

	if(sockaddr_in6->sin6_family != AF_INET6)
		return(false);

	addr_in6 = &sockaddr_in6->sin6_addr;
	addr = &addr_in6->s6_addr[0];

	if((addr[0] == 0) && (addr[1] == 0) && (addr[2] == 0) && (addr [3] == 0) &&
			(addr[4] == 0) && (addr[5] == 0) && (addr[6] == 0) && (addr [7] == 0) &&
			(addr[8] == 0) && (addr[9] == 0) && (addr[10] == 0xff) && (addr [11] == 0xff))
		return(true);

	return(false);
}

void util_sin6_addr_to_string(string_t dst, unsigned int length, const void *data)
{
	const struct sockaddr_in6 *addr_in6;
	string_auto(address_string, 128);

	if(length != sizeof(struct sockaddr_in6))
	{
		string_assign_cstr(dst, "invalid length");
		return;
	}

	addr_in6 = (const struct sockaddr_in6 *)data;

	if(util_sin6_addr_is_ipv4(addr_in6))
		//util_esp_ipv4_addr_to_string(address_string, &addr_in6->sin6_addr.s6_addr[12]);
		//string_assign_cstr(ipaddr_ntoa(&addr_in6->sin6_addr));
		string_format(address_string, "%u.%u.%u.%u",
				addr_in6->sin6_addr.s6_addr[12],
				addr_in6->sin6_addr.s6_addr[13],
				addr_in6->sin6_addr.s6_addr[14],
				addr_in6->sin6_addr.s6_addr[15]);
	else
	{
		string_assign_cstr(address_string, ip6addr_ntoa((const ip6_addr_t *)&addr_in6->sin6_addr));
		string_tolower(address_string);
	}

	string_format(dst, "[address: %s, port %u]",
			string_cstr(address_string),
			addr_in6->sin6_port);
}

void util_mac_addr_to_string(string_t dst, const uint8_t mac[6], bool invert)
{
	if(invert)
		string_format(dst, "%02x:%02x:%02x:%02x:%02x:%02x",
				mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
	else
		string_format(dst, "%02x:%02x:%02x:%02x:%02x:%02x",
				mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

ipv6_address_t util_ipv6_address_type(const void *addr)
{
	const esp_ip6_addr_t *_addr = (const esp_ip6_addr_t *)addr;
	unsigned int b[2];

	if(ip6_addr_islinklocal(_addr))
		return(ipv6_address_link_local);

	if(!ip6_addr_isglobal(_addr))
		return(ipv6_address_other);

	b[0] = (htonl(_addr->addr[2]) & 0x000000ffUL) >> 0;
	b[1] = (htonl(_addr->addr[3]) & 0xff000000UL) >> 24;

	if((b[0] == 0xff) && (b[1]== 0xfe))
		return(ipv6_address_global_slaac);

	return(ipv6_address_global_static);
}

const char *util_ipv6_address_type_string(const void *addr)
{
	ipv6_address_t type;

	type = util_ipv6_address_type(addr);

	if((type < 0) || (type >= ipv6_address_size))
		return("<illegal>");

	return(ipv6_address_type_strings[type]);
}

void util_time_to_string(string_t dst, const time_t *ticks)
{
	struct tm tm;
	char timestring[64];

	localtime_r(ticks, &tm);
	strftime(timestring, sizeof(timestring), "%Y/%m/%d %H:%M:%S", &tm);

	string_assign_cstr(dst, timestring);
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
