#pragma once

#include "log.h"

#include <cstdint>
#include <string>
#include <array>
#include <map>

class System final
{
	public:

		enum class IPV6AddressType
		{
			loopback,
			link_local,
			multicast,
			site_local,
			ipv4_mapped,
			unspecified,
			global_slaac,
			global_static,
			other,
			size,
		};

		explicit System() = delete;
		explicit System(Log &);
		explicit System(System &) = delete;

		static System& get();

		void identify(std::string &out, int mtu);
		void info(std::string &out);
		void memory(std::string &out);
		void partitions(std::string &dst, int partition = -1);
		void process_list(std::string &out, int core = -1);
		bool process_kill(const std::string &name);

		int get_initial_free_heap();
		int get_initial_free_spiram();
		int get_initial_free_internal();
		int get_initial_free_total();
		int get_initial_free_rtcram();

		std::string		ipv4_addr_to_string(const std::uint32_t * /* sockaddr_in->sin_addr.in_addr = uint32_t */);
		IPV6AddressType	ipv6_address_type(const void * /* sockaddr6_in->sin6_addr.s6_addr = char[16] */);
		std::string		ipv6_address_type_string(const void * /* sockaddr6_in->sin6_addr.s6_addr = char[16] */);
		std::string		ipv6_addr_to_string(const void * /* sockaddr6_in->sin6_addr.in6_addr = uint8_t[16] */);
		std::string		mac_addr_to_string(std::string_view address, bool invert);

	private:

		static constexpr unsigned int task_id_size = 48;

		struct task_info_cache_t
		{
			unsigned int task_id;
			std::int64_t previous_runtime;
		};

		typedef std::map<IPV6AddressType, std::string> ipv6_address_type_map_t;

		static System *singleton;
		Log &log;

		int initial_free_heap;
		int initial_free_spiram;
		int initial_free_internal;
		int initial_free_total;
		int initial_free_rtcram;

		std::array<task_info_cache_t, task_id_size> task_info_cache;
		static const ipv6_address_type_map_t ipv6_address_type_strings;
};
