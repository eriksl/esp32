#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "log.h"
#include "util.h"

#include <string>
#include <format>
#include <chrono>

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

std::string util_mac_addr_to_string(const uint8_t mac[6], bool invert)
{
	std::string dst;

	if(invert)
		dst = std::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
				mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
	else
		dst = std::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
				mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	return(dst);
}

std::string util_time_to_string(std::string_view format, const time_t &stamp)
{
	// FIXME: timezone
	std::chrono::zoned_time chrono_stamp{"Europe/Amsterdam", std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::from_time_t(stamp))};

	try
	{
		return(std::vformat(format, std::make_format_args(chrono_stamp)));
	}
	catch(const std::format_error& e)
	{
		return(std::string("[util_time_to_string: ") + e.what() + ", format string: " + std::string(format) + "]");
	}
}

std::string util_time_to_string(const time_t &stamp)
{
	return(util_time_to_string("{:%Y/%m/%d %H:%M:%S}", stamp));
}

std::string_view yesno(bool yesno_)
{
	static const char *no = "no";
	static const char *yes = "yes";

	return(yesno_ ? yes : no);
}

void util_init(void)
{
	assert(!inited);

	setenv("TZ", "CEST-1CET,M3.5.0/2:00:00,M10.5.0/2:00:00", 1); // FIXME
	tzset();

	inited = true;
}
