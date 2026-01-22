#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <assert.h>
#include <thread>
#include <esp_pthread.h>

#include "string.h"
#include "cli.h"
#include "log.h"
#include "util.h"
#include "packet.h"
#include "cli-command.h"
#include "udp.h"

#include <string>
#include <boost/format.hpp>

class UDP
{
	public:

		enum
		{
			mtu = 16 * 1024, // emperically derived
		};

		int socket_fd;

		unsigned int send_bytes;
		unsigned int send_packets;
		unsigned int send_errors;
		unsigned int send_no_connection;
		unsigned int receive_bytes;
		unsigned int receive_packets;
		unsigned int receive_errors;
		unsigned int receive_incomplete_packets;
		unsigned int receive_invalid_packets;

		UDP();

		void send(const command_response_t *command_response);
		void command_info(cli_command_call_t *call);
		__attribute__((noreturn)) static void run_wrapper(void *);
		__attribute__((noreturn)) void run();
};

UDP::UDP() :
	socket_fd(-1),
	send_bytes(0),
	send_packets(0),
	send_errors(0),
	send_no_connection(0),
	receive_bytes(0),
	receive_packets(0),
	receive_errors(0),
	receive_incomplete_packets(0),
	receive_invalid_packets(0)
{
	esp_pthread_cfg_t thread_config = esp_pthread_get_default_config();

	thread_config.thread_name = "udp";
	thread_config.pin_to_core = 1;
	thread_config.stack_size = 4 * 1024; // FIXME
	thread_config.prio = 1;
	esp_pthread_set_cfg(&thread_config);

	std::thread new_thread(UDP::run_wrapper, this);

	new_thread.detach();
}

void UDP::run()
{
	std::string udp_receive_buffer;
	int rv;
	struct sockaddr_in6 si6_addr;
	socklen_t si6_addr_length;
	int length;
	struct pollfd pfd;

	this->socket_fd = socket(AF_INET6, SOCK_DGRAM, 0);
	assert(this->socket_fd >= 0);

	memset(&si6_addr, 0, sizeof(si6_addr));
	si6_addr.sin6_family = AF_INET6;
	si6_addr.sin6_port = htons(24);

	rv = ::bind(this->socket_fd, (const struct sockaddr *)&si6_addr, sizeof(si6_addr));
	assert(rv == 0);

	for(;;)
	{
		si6_addr_length = sizeof(si6_addr);

		pfd.fd = this->socket_fd;
		pfd.events = POLLIN;
		pfd.revents = 0;

		rv = poll(&pfd, 1, -1);

		if(rv < 0)
		{
			log_errno("udp: poll error");
			continue;
		}

		if(!(pfd.revents & POLLIN))
		{
			log_errno("udp: socket error");
			util_abort("udp socket error");
		}

		if(ioctl(this->socket_fd, FIONREAD, &length))
		{
			log_errno("udp: ioctl");
			util_abort("udp ioctl error");
		}

		udp_receive_buffer.resize(length);

		length = ::recvfrom(this->socket_fd, udp_receive_buffer.data(), udp_receive_buffer.size(), 0, reinterpret_cast<sockaddr *>(&si6_addr), &si6_addr_length);

		if(length < 0)
		{
			this->receive_errors++;
			util_sleep(100);
			continue;
		}

		if(length == 0)
		{
			this->receive_errors++;
			log("udp: zero packet received");
			util_sleep(100);
			continue;
		}

		udp_receive_buffer.resize(length);

		this->receive_bytes += length;

		if(!Packet::valid(udp_receive_buffer))
		{
			this->receive_invalid_packets++;
			continue;
		}

		if(!Packet::complete(udp_receive_buffer))
		{
			this->receive_incomplete_packets++;
			continue;
		}

		command_response_t *command_response = new command_response_t;

		static_assert(sizeof(command_response->ip.address.sin6_addr) >= sizeof(struct sockaddr));
		static_assert(sizeof(command_response->ip.address.sin6_addr) >= sizeof(struct sockaddr_in));
		static_assert(sizeof(command_response->ip.address.sin6_addr) >= sizeof(struct sockaddr_in6));

		util_memcpy(&command_response->ip.address.sin6_addr, &si6_addr, si6_addr_length);
		command_response->ip.address.sin6_length = si6_addr_length;

		command_response->source = cli_source_wlan_udp;
		command_response->packetised = 1;
		command_response->mtu = this->mtu;
		command_response->packet = udp_receive_buffer;

		cli_receive_queue_push(command_response);

		command_response = nullptr;

		this->receive_packets++;
	}

	close(this->socket_fd);
	this->socket_fd = -1;

	udp_receive_buffer.clear();
}

void UDP::run_wrapper(void *ptr)
{
	UDP *_this = static_cast<UDP *>(ptr);

	_this->run();
}

void UDP::send(const command_response_t *command_response)
{
	int sent;

	assert(command_response);

	if(this->socket_fd < 0)
	{
		this->send_no_connection++;
		return;
	}

	sent = ::sendto(this->socket_fd, command_response->packet.data(), command_response->packet.length(), 0,
			(const struct sockaddr *)&command_response->ip.address.sin6_addr, command_response->ip.address.sin6_length);

	if(sent <= 0)
	{
		this->send_errors++;
		return;
	}

	this->send_packets++;
	this->send_bytes += sent;
}

void UDP::command_info(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	call->result = "UDP INFO";
	call->result += "\nsending";
	call->result += (boost::format("\n- sent bytes %u") % this->send_bytes).str();
	call->result += (boost::format("\n- sent packets: %u") % this->send_packets).str();
	call->result += (boost::format("\n- send errors: %u") % this->send_errors).str();
	call->result += (boost::format("\n- disconnected socket events: %u") % this->send_no_connection).str();
	call->result += "\nreceiving";
	call->result += (boost::format("\n- received bytes: %u") % this->receive_bytes).str();
	call->result += (boost::format("\n- received packets: %u") % this->receive_packets).str();
	call->result += (boost::format("\n- received incomplete packets: %u") % this->receive_incomplete_packets).str();
	call->result += (boost::format("\n- received invalid packets: %u") % this->receive_invalid_packets).str();
	call->result += (boost::format("\n- receive errors: %u") % this->receive_errors).str();
}

static UDP *UDP_singleton = nullptr;

void net_udp_init(void)
{
	assert(!UDP_singleton);

	UDP_singleton = new UDP();

	assert(UDP_singleton);
}

void net_udp_send(const command_response_t *command_response)
{
	assert(UDP_singleton);

	return(UDP_singleton->send(command_response));
}

void net_udp_command_info(cli_command_call_t *call)
{
	assert(UDP_singleton);

	return(UDP_singleton->command_info(call));
}
