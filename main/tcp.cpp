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
#include "tcp.h"

#include <string>
#include <boost/format.hpp>

class TCP
{
	public:

		enum
		{
			mtu = 16 * 1024, // emperically determined
		};

		int socket_fd = -1;

		unsigned int send_bytes;
		unsigned int send_segments;
		unsigned int send_packets;
		unsigned int send_errors;
		unsigned int send_no_connection;
		unsigned int receive_bytes;
		unsigned int receive_packets;
		unsigned int receive_accepts;
		unsigned int receive_accept_errors;
		unsigned int receive_errors;
		unsigned int receive_invalid_packets;
		unsigned int receive_incomplete_packets;

		TCP();

		void send(const command_response_t *command_response);
		void command_info(cli_command_call_t *call);
		static __attribute__((noreturn)) void run_wrapper(void *);
		__attribute__((noreturn)) void run();
};

TCP::TCP() :
		socket_fd(-1),
		send_bytes(0),
		send_segments(0),
		send_packets(0),
		send_errors(0),
		send_no_connection(0),
		receive_bytes(0),
		receive_packets(0),
		receive_accepts(0),
		receive_accept_errors(0),
		receive_errors(0),
		receive_invalid_packets(0),
		receive_incomplete_packets(0)
{
	esp_pthread_cfg_t thread_config = esp_pthread_get_default_config();

	thread_config.thread_name = "tcp";
	thread_config.pin_to_core = 1;
	thread_config.stack_size = 2 * 1024;
	thread_config.prio = 1;
	//thread_config.stack_alloc_caps = MALLOC_CAP_SPIRAM;
	esp_pthread_set_cfg(&thread_config);

	std::thread new_thread(TCP::run_wrapper, this);

	new_thread.detach();
}

void TCP::run()
{
	std::string tcp_receive_buffer;
	int accept_fd, rv;
	struct sockaddr_in6 si6_addr;
	socklen_t si6_addr_length;
	int length;
	struct pollfd pfd;

	accept_fd = socket(AF_INET6, SOCK_STREAM, 0);
	assert(accept_fd >= 0);

	memset(&si6_addr, 0, sizeof(si6_addr));
	si6_addr.sin6_family = AF_INET6;
	si6_addr.sin6_port = htons(24);

	rv = bind(accept_fd, (const struct sockaddr *)&si6_addr, sizeof(si6_addr));
	assert(rv == 0);

	rv = listen(accept_fd, 0);
	assert(rv == 0);

	for(;;)
	{
		si6_addr_length = sizeof(si6_addr);

		if((socket_fd = accept(accept_fd, (struct sockaddr *)&si6_addr, &si6_addr_length)) < 0)
		{
			receive_accept_errors++;
			continue;
		}

		receive_accepts++;

		for(;;)
		{
			tcp_receive_buffer.clear();

			pfd.fd = this->socket_fd;
			pfd.events = POLLIN;
			pfd.revents = 0;

			rv = poll(&pfd, 1, -1);

			if(rv < 0)
			{
				log_errno("tcp: poll error");
				continue;
			}

			if(!(pfd.revents & POLLIN))
			{
				log_errno("tcp: socket error");
				util_abort("tcp socket error");
			}

			if(ioctl(this->socket_fd, FIONREAD, &length))
			{
				log_errno("tcp: ioctl");
				util_abort("tcp ioctl error");
			}

			tcp_receive_buffer.clear();
			tcp_receive_buffer.resize(length);

			length = ::recv(this->socket_fd, tcp_receive_buffer.data(), tcp_receive_buffer.size(), 0);

			if(length < 0)
			{
				log_format_errno("tcp: receive error: %d", length);
				receive_errors++;
				break;
			}

			if(length == 0)
				break;

			tcp_receive_buffer.resize(length);

			receive_bytes += length;

			if(!Packet::valid(tcp_receive_buffer))
			{
				receive_invalid_packets++;
				continue;
			}

			if(!Packet::complete(tcp_receive_buffer))
			{
				unsigned int offset, pending;

				offset = length;
				pending = Packet::length(tcp_receive_buffer) - offset;
				tcp_receive_buffer.resize(Packet::length(tcp_receive_buffer));

				while(pending > 0)
				{
					pfd.fd = this->socket_fd;
					pfd.events = POLLIN;
					pfd.revents = 0;

					rv = poll(&pfd, 1, 1000);

					if(rv < 0)
					{
						log_errno("tcp: poll error (2)");
						break;
					}

					if(rv == 0)
					{
						log("tcp: timeout");
						break;
					}

					if(!(pfd.revents & POLLIN))
					{
						log_errno("tcp: socket error (2)");
						util_abort("tcp socket error (2)");
					}

					length = ::recv(this->socket_fd, tcp_receive_buffer.data() + offset, pending, 0);

					if(length == 0)
						break;

					if(length < 0)
					{
						log_format_errno("tcp: receive error (2): %d", length);
						receive_errors++;
						break;
					}

					pending -= length;
					offset += length;
				}

				if(!Packet::complete(tcp_receive_buffer))
				{
					log("tcp: packet incomplete");
					receive_incomplete_packets++;
					continue;
				}

				length = Packet::length(tcp_receive_buffer);
				tcp_receive_buffer.resize(length);
			}

			command_response_t *command_response = new command_response_t;

			static_assert(sizeof(command_response->ip.address.sin6_addr) >= sizeof(struct sockaddr));
			static_assert(sizeof(command_response->ip.address.sin6_addr) >= sizeof(struct sockaddr_in));
			static_assert(sizeof(command_response->ip.address.sin6_addr) >= sizeof(struct sockaddr_in6));

			util_memcpy(&command_response->ip.address.sin6_addr, &si6_addr, si6_addr_length);
			command_response->ip.address.sin6_length = si6_addr_length;
			command_response->source = cli_source_wlan_tcp;
			command_response->mtu = this->mtu;
			command_response->packetised = 1;
			command_response->packet = tcp_receive_buffer;

			cli_receive_queue_push(command_response);

			command_response = nullptr;
			tcp_receive_buffer.clear();
			receive_packets++;
		}

		close(socket_fd);
		socket_fd = -1;
	}
}

void TCP::run_wrapper(void *ptr)
{
	TCP *_this = static_cast<TCP *>(ptr);

	_this->run();
}

void TCP::send(const command_response_t *command_response)
{
	int length, offset, chunk_length, sent;

	assert(command_response);

	if(socket_fd < 0)
	{
		send_no_connection++;
		goto error; // FIXME
	}

	send_packets++;

	offset = 0;
	length = command_response->packet.length();

	if(!command_response->packetised && (length > command_response->mtu))
		length = command_response->mtu;

	for(;;)
	{
		chunk_length = length;

		if(chunk_length > mtu)
			chunk_length = mtu;

		sent = ::send(socket_fd, command_response->packet.data() + offset, chunk_length, 0);

		send_segments++;

		if(sent <= 0)
		{
			send_errors++;
			break;
		}

		send_bytes += sent;

		length -= sent;
		offset += sent;

		assert(length >= 0);
		assert(offset <= command_response->packet.length());

		if(length == 0)
			break;

		assert(offset < command_response->packet.length());
	}

error:
}

void TCP::command_info(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	call->result = "TCP INFO";
	call->result += "\nsending";
	call->result += (boost::format("\n- sent bytes %u") % send_bytes).str();
	call->result += (boost::format("\n- sent segments %u") % send_segments).str();
	call->result += (boost::format("\n- sent packets: %u") % send_packets).str();
	call->result += (boost::format("\n- send errors: %u") % send_errors).str();
	call->result += (boost::format("\n- disconnected socket events: %u") % send_no_connection).str();
	call->result += "\nreceiving";
	call->result += (boost::format("\n- received bytes: %u") % receive_bytes).str();
	call->result += (boost::format("\n- received packets: %u") % receive_packets).str();
	call->result += (boost::format("\n- incomplete packets: %u") % receive_incomplete_packets).str();
	call->result += (boost::format("\n- receive errors: %u") % receive_errors).str();
	call->result += (boost::format("\n- accepted connections: %u") % receive_accepts).str();
	call->result += (boost::format("\n- accept errors: %u") % receive_accept_errors).str();
}

static TCP *TCP_singleton = nullptr;

void net_tcp_init(void)
{
	assert(!TCP_singleton);

	TCP_singleton = new TCP();

	assert(TCP_singleton);
}

void net_tcp_send(const command_response_t *command_response)
{
	assert(TCP_singleton);

	return(TCP_singleton->send(command_response));
}

void net_tcp_command_info(cli_command_call_t *call)
{
	assert(TCP_singleton);

	return(TCP_singleton->command_info(call));
}
