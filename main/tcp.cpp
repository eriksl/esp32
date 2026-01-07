#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <assert.h>
#include <thread>
#include <esp_pthread.h>

extern "C"
{
#include "string.h"
#include "cli.h"
#include "log.h"
#include "util.h"
#include "packet.h"
#include "cli-command.h"
}

#include "tcp.h"

class TCP
{
	public:

		enum
		{
			packet_size = 4096,
			packet_overhead = 128,
			tcp_mtu = 1200,
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
		unsigned int receive_incomplete_packets;

		TCP();

		void send(const cli_buffer_t *src);
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
		receive_incomplete_packets(0)
{
	esp_pthread_cfg_t thread_config = esp_pthread_get_default_config();

	thread_config.thread_name = "tcp";
	thread_config.pin_to_core = 1;
	thread_config.stack_size = 3 * 1024;
	thread_config.prio = 1;
	esp_pthread_set_cfg(&thread_config);

	std::thread new_thread(TCP::run_wrapper, this);

	new_thread.detach();
}

void TCP::run()
{
	string_t tcp_receive_buffer; // FIXME convert to std::string when possible
	int accept_fd, rv;
	struct sockaddr_in6 si6_addr;
	socklen_t si6_addr_length;
	int length;
	cli_buffer_t cli_buffer;

	static_assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr));
	static_assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr_in));
	static_assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr_in6));

	tcp_receive_buffer = string_new(packet_size + packet_overhead);

	memset(&si6_addr, 0, sizeof(si6_addr));
	si6_addr.sin6_family = AF_INET6;
	si6_addr.sin6_port = htons(24);

	accept_fd = socket(AF_INET6, SOCK_STREAM, 0);
	assert(accept_fd >= 0);

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
		string_clear(tcp_receive_buffer);

		for(;;)
		{
			util_memcpy(&cli_buffer.ip.address.sin6_addr, &si6_addr, si6_addr_length);
			cli_buffer.ip.address.sin6_length = si6_addr_length;

			length = string_recvfrom_fd(tcp_receive_buffer, socket_fd, (unsigned int *)0, (void *)0);

			if(length == 0)
				break;

			if(length <= 0)
			{
				log_format("tcp receive error: %d", length);
				receive_errors++;
				break;
			}

			receive_bytes += length;

			if(packet_valid(tcp_receive_buffer) && !packet_complete(tcp_receive_buffer))
			{
				receive_incomplete_packets++;
				continue;
			}

			cli_buffer.packetised = !!packet_valid(tcp_receive_buffer);

			receive_packets++;

			cli_buffer.source = cli_source_wlan_tcp;
			cli_buffer.mtu = tcp_mtu;
			cli_buffer.data = string_new(string_length(tcp_receive_buffer));
			string_assign_string(cli_buffer.data, tcp_receive_buffer);
			string_clear(tcp_receive_buffer);

			cli_receive_queue_push(&cli_buffer);
		}

		close(socket_fd);
		socket_fd = -1;
	}

	string_free(&tcp_receive_buffer);
}

void TCP::run_wrapper(void *ptr)
{
	TCP *_this = static_cast<TCP *>(ptr);

	_this->run();
}

void TCP::send(const cli_buffer_t *src)
{
	int length, offset, chunk_length, sent;

	if(socket_fd < 0)
	{
		send_no_connection++;
		return;
	}

	send_packets++;

	offset = 0;
	length = string_length(src->data);

	if(!src->packetised && (length > src->mtu))
		length = src->mtu;

	for(;;)
	{
		chunk_length = length;

		if(chunk_length > tcp_mtu)
			chunk_length = tcp_mtu;

		sent = ::send(socket_fd, string_data(src->data) + offset, chunk_length, 0);

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
		assert(offset <= string_length(src->data));

		if(length == 0)
			break;

		assert(offset < string_length(src->data));
	}
}

void TCP::command_info(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "TCP INFO");
	string_append_cstr(call->result, "\nsending");
	string_format_append(call->result, "\n- sent bytes %u", send_bytes);
	string_format_append(call->result, "\n- sent segments %u", send_segments);
	string_format_append(call->result, "\n- sent packets: %u", send_packets);
	string_format_append(call->result, "\n- send errors: %u", send_errors);
	string_format_append(call->result, "\n- disconnected socket events: %u", send_no_connection);
	string_append_cstr(call->result, "\nreceiving");
	string_format_append(call->result, "\n- received bytes: %u", receive_bytes);
	string_format_append(call->result, "\n- received packets: %u", receive_packets);
	string_format_append(call->result, "\n- incomplete packets: %u", receive_incomplete_packets);
	string_format_append(call->result, "\n- receive errors: %u", receive_errors);
	string_format_append(call->result, "\n- accepted connections: %u", receive_accepts);
	string_format_append(call->result, "\n- accept errors: %u", receive_accept_errors);
}

static TCP *TCP_singleton = nullptr;

void net_tcp_init(void)
{
	assert(!TCP_singleton);

	TCP_singleton = new TCP();

	assert(TCP_singleton);
}

void net_tcp_send(const cli_buffer_t *src)
{
	assert(TCP_singleton);

	return(TCP_singleton->send(src));
}

void net_tcp_command_info(cli_command_call_t *call)
{
	assert(TCP_singleton);

	return(TCP_singleton->command_info(call));
}
