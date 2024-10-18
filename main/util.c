#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <lwip/ip_addr.h>

#include "string.h"
#include "log.h"
#include "util.h"

#include <freertos/FreeRTOS.h>

#include <mbedtls/base64.h>
#include <mbedtls/md5.h>
#include <mbedtls/aes.h>

#include <esp_timer.h>
#include <esp_netif.h>
#include <esp_netif_ip_addr.h>

#include <stddef.h>
#include <sys/time.h>

static bool inited;

static const char *ipv6_address_type_strings[ipv6_address_size] =
{
	[ipv6_address_link_local] =		"link local",
	[ipv6_address_global_slaac] =	"autoconfig",
	[ipv6_address_global_static] =	"static",
	[ipv6_address_other] =			"other",
};

void util_init(void)
{
	assert(!inited);

	setenv("TZ", "CEST-1CET,M3.2.0/2:00:00,M11.1.0/2:00:00", 1);
	tzset();

	inited = true;
}

void util_sleep(unsigned int msec)
{
	vTaskDelay(msec / portTICK_PERIOD_MS);
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
	string_format(dst, "%s", ip6addr_ntoa((const ip6_addr_t *)src));
	string_tolower(dst);
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
	unsigned int in, out, value;

	assert(inited);
	assert(src);
	assert(dst);

	string_clear(dst);

	for(in = 0, out = 0; (in < src_length) && (out < string_size(dst)); out++)
	{
		if(out & 0x1)
		{
			value = (src[in] & 0x0f) >> 0;
			in++;
		}
		else
		{
			if(in != 0)
				string_append_cstr(dst, " ");

			string_append_cstr(dst, "0x");
			value = (src[in] & 0xf0) >> 4;
		}

		if(value >= 0xa)
			string_append(dst, (value - 10) + 'a');
		else
			string_append(dst, (value -  0) + '0');
	}
}

void util_hexdump(string_t dst, const const_string_t src)
{
	assert(src);

	util_hexdump_cstr(dst, string_length(src), string_data(src));
}

void decrypt_aes_256(string_t dst, const string_t src)
{
	static const uint8_t aes_256_key[32] =
	{
		0x3a, 0xe0, 0xbe, 0x96, 0xeb, 0x7c, 0xfe, 0xbc, 0x97, 0xe9, 0x7e, 0x98, 0x73, 0x8e, 0x4e, 0x88,
		0xeb, 0xd7, 0x76, 0xa7, 0x55, 0x8a, 0xd3, 0x36, 0x96, 0x4b, 0xaf, 0x0b, 0x35, 0xa4, 0x84, 0xf5,
	};
	static const uint8_t aes_256_iv_init[16] = { 0x4f, 0x8f, 0xee, 0x60, 0xe9, 0x56, 0x4d, 0x0f, 0x81, 0xf0, 0x8a, 0xe5, 0x8d, 0x1c, 0x08, 0xd6 };
	uint8_t aes_256_iv[16];

	uint8_t output[16];

    mbedtls_aes_context context;

	assert(string_length(src) == sizeof(output));
	assert(string_size(dst) >= sizeof(output));

	memcpy(aes_256_iv, aes_256_iv_init, sizeof(aes_256_iv));

    mbedtls_aes_init(&context);
    mbedtls_aes_setkey_dec(&context, aes_256_key, sizeof(aes_256_key) * 8);
    mbedtls_aes_crypt_cbc(&context, MBEDTLS_AES_DECRYPT, string_length(src), aes_256_iv, string_data(src), output);

	string_assign_data(dst, sizeof(output), output);
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

// code below slightly modified from https://github.com/madler/crcany, zlib license

static const uint32_t crc32_table_byte[] =
{
	0xb1f7404b, 0xb5365dfc, 0xb8757b25, 0xbcb46692, 0xa2f33697, 0xa6322b20,
	0xab710df9, 0xafb0104e, 0x97ffadf3, 0x933eb044, 0x9e7d969d, 0x9abc8b2a,
	0x84fbdb2f, 0x803ac698, 0x8d79e041, 0x89b8fdf6, 0xfde69b3b, 0xf927868c,
	0xf464a055, 0xf0a5bde2, 0xeee2ede7, 0xea23f050, 0xe760d689, 0xe3a1cb3e,
	0xdbee7683, 0xdf2f6b34, 0xd26c4ded, 0xd6ad505a, 0xc8ea005f, 0xcc2b1de8,
	0xc1683b31, 0xc5a92686, 0x29d4f6ab, 0x2d15eb1c, 0x2056cdc5, 0x2497d072,
	0x3ad08077, 0x3e119dc0, 0x3352bb19, 0x3793a6ae, 0x0fdc1b13, 0x0b1d06a4,
	0x065e207d, 0x029f3dca, 0x1cd86dcf, 0x18197078, 0x155a56a1, 0x119b4b16,
	0x65c52ddb, 0x6104306c, 0x6c4716b5, 0x68860b02, 0x76c15b07, 0x720046b0,
	0x7f436069, 0x7b827dde, 0x43cdc063, 0x470cddd4, 0x4a4ffb0d, 0x4e8ee6ba,
	0x50c9b6bf, 0x5408ab08, 0x594b8dd1, 0x5d8a9066, 0x8571303c, 0x81b02d8b,
	0x8cf30b52, 0x883216e5, 0x967546e0, 0x92b45b57, 0x9ff77d8e, 0x9b366039,
	0xa379dd84, 0xa7b8c033, 0xaafbe6ea, 0xae3afb5d, 0xb07dab58, 0xb4bcb6ef,
	0xb9ff9036, 0xbd3e8d81, 0xc960eb4c, 0xcda1f6fb, 0xc0e2d022, 0xc423cd95,
	0xda649d90, 0xdea58027, 0xd3e6a6fe, 0xd727bb49, 0xef6806f4, 0xeba91b43,
	0xe6ea3d9a, 0xe22b202d, 0xfc6c7028, 0xf8ad6d9f, 0xf5ee4b46, 0xf12f56f1,
	0x1d5286dc, 0x19939b6b, 0x14d0bdb2, 0x1011a005, 0x0e56f000, 0x0a97edb7,
	0x07d4cb6e, 0x0315d6d9, 0x3b5a6b64, 0x3f9b76d3, 0x32d8500a, 0x36194dbd,
	0x285e1db8, 0x2c9f000f, 0x21dc26d6, 0x251d3b61, 0x51435dac, 0x5582401b,
	0x58c166c2, 0x5c007b75, 0x42472b70, 0x468636c7, 0x4bc5101e, 0x4f040da9,
	0x774bb014, 0x738aada3, 0x7ec98b7a, 0x7a0896cd, 0x644fc6c8, 0x608edb7f,
	0x6dcdfda6, 0x690ce011, 0xd8fba0a5, 0xdc3abd12, 0xd1799bcb, 0xd5b8867c,
	0xcbffd679, 0xcf3ecbce, 0xc27ded17, 0xc6bcf0a0, 0xfef34d1d, 0xfa3250aa,
	0xf7717673, 0xf3b06bc4, 0xedf73bc1, 0xe9362676, 0xe47500af, 0xe0b41d18,
	0x94ea7bd5, 0x902b6662, 0x9d6840bb, 0x99a95d0c, 0x87ee0d09, 0x832f10be,
	0x8e6c3667, 0x8aad2bd0, 0xb2e2966d, 0xb6238bda, 0xbb60ad03, 0xbfa1b0b4,
	0xa1e6e0b1, 0xa527fd06, 0xa864dbdf, 0xaca5c668, 0x40d81645, 0x44190bf2,
	0x495a2d2b, 0x4d9b309c, 0x53dc6099, 0x571d7d2e, 0x5a5e5bf7, 0x5e9f4640,
	0x66d0fbfd, 0x6211e64a, 0x6f52c093, 0x6b93dd24, 0x75d48d21, 0x71159096,
	0x7c56b64f, 0x7897abf8, 0x0cc9cd35, 0x0808d082, 0x054bf65b, 0x018aebec,
	0x1fcdbbe9, 0x1b0ca65e, 0x164f8087, 0x128e9d30, 0x2ac1208d, 0x2e003d3a,
	0x23431be3, 0x27820654, 0x39c55651, 0x3d044be6, 0x30476d3f, 0x34867088,
	0xec7dd0d2, 0xe8bccd65, 0xe5ffebbc, 0xe13ef60b, 0xff79a60e, 0xfbb8bbb9,
	0xf6fb9d60, 0xf23a80d7, 0xca753d6a, 0xceb420dd, 0xc3f70604, 0xc7361bb3,
	0xd9714bb6, 0xddb05601, 0xd0f370d8, 0xd4326d6f, 0xa06c0ba2, 0xa4ad1615,
	0xa9ee30cc, 0xad2f2d7b, 0xb3687d7e, 0xb7a960c9, 0xbaea4610, 0xbe2b5ba7,
	0x8664e61a, 0x82a5fbad, 0x8fe6dd74, 0x8b27c0c3, 0x956090c6, 0x91a18d71,
	0x9ce2aba8, 0x9823b61f, 0x745e6632, 0x709f7b85, 0x7ddc5d5c, 0x791d40eb,
	0x675a10ee, 0x639b0d59, 0x6ed82b80, 0x6a193637, 0x52568b8a, 0x5697963d,
	0x5bd4b0e4, 0x5f15ad53, 0x4152fd56, 0x4593e0e1, 0x48d0c638, 0x4c11db8f,
	0x384fbd42, 0x3c8ea0f5, 0x31cd862c, 0x350c9b9b, 0x2b4bcb9e, 0x2f8ad629,
	0x22c9f0f0, 0x2608ed47, 0x1e4750fa, 0x1a864d4d, 0x17c56b94, 0x13047623,
	0x0d432626, 0x09823b91, 0x04c11d48, 0x000000ff
};

uint32_t util_crc32cksum_byte(uint32_t crc, void const *mem, size_t len)
{
	const uint8_t *data = mem;

	if(!data)
		return(0xffffffffUL);

	for(size_t i = 0; i < len; i++)
		crc = (crc << 8) ^ crc32_table_byte[((crc >> 24) ^ data[i]) & 0xff];

	return(crc);
}
