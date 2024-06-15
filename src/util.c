#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "string.h"
#include "cli-command.h"
#include "log.h"
#include "util.h"

#include <freertos/FreeRTOS.h>

#include <mbedtls/base64.h>
#include <mbedtls/md5.h>

#include <esp_netif_ip_addr.h>
#include <esp_timer.h>

#include <stddef.h>
#include <sys/time.h>

static bool inited;

void util_init(void)
{
	assert(!inited);

	setenv("TZ", "CEST-1CET,M3.2.0/2:00:00,M11.1.0/2:00:00", 1);
	tzset();

	inited = true;
}

uint32_t util_md5_32(unsigned int length, const uint8_t *data)
{
	uint8_t hash[16];

	mbedtls_md5(data, length, hash);

	return((hash[0] << 24) | (hash[1] << 16) | (hash[2] << 8) | (hash[3] << 0));
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
	string_format(dst, IPV6STR, IPV62STR(*src));
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
