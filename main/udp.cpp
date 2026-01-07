#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <assert.h>
#include <thread>
#include <esp_pthread.h>

extern "C" {
#include "string.h"
#include "cli.h"
#include "log.h"
#include "util.h"
#include "packet.h"
#include "cli-command.h"
}

#include "udp.h"

class UDP
{
	public:

		enum
		{
			mtu = 5 * 4096, // emperical derived from LWIP max UDP reassembly (~22k)
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

		void send(const cli_buffer_t *src);
		void command_info(cli_command_call_t *call);
		static __attribute__((noreturn)) void run_wrapper(void *);
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
	thread_config.stack_size = 3 * 1024;
	thread_config.prio = 1;
	esp_pthread_set_cfg(&thread_config);

	std::thread new_thread(UDP::run_wrapper, this);

	new_thread.detach();
}

void UDP::run()
{
	string_t udp_receive_buffer; // FIXME convert to std::string when possible
	int rv;
	struct sockaddr_in6 si6_addr;
	unsigned int si6_addr_length;
	int length;
	cli_buffer_t cli_buffer;

	static_assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr));
	static_assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr_in));
	static_assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr_in6));

	udp_receive_buffer = string_new(this->mtu);
	assert(udp_receive_buffer);

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

		string_clear(udp_receive_buffer);
		length = string_recvfrom_fd(udp_receive_buffer, this->socket_fd, &si6_addr_length, &si6_addr);

		util_memcpy(&cli_buffer.ip.address.sin6_addr, &si6_addr, si6_addr_length);
		cli_buffer.ip.address.sin6_length = si6_addr_length;

		if(length == 0)
		{
			this->receive_errors++;
			log("udp: zero packet received");
			util_sleep(100);
			continue;
		}

		if(length < 0)
		{
			this->receive_errors++;
			util_sleep(100);
			continue;
		}

		this->receive_bytes += length;

		if(!packet_complete(udp_receive_buffer))
		{
			this->receive_incomplete_packets++;
			continue;
		}

		if(!packet_valid(udp_receive_buffer))
		{
			this->receive_invalid_packets++;
			continue;
		}

		this->receive_packets++;

		cli_buffer.source = cli_source_wlan_udp;
		cli_buffer.packetised = 1;
		cli_buffer.mtu = this->mtu;
		cli_buffer.data = string_new(string_length(udp_receive_buffer));
		string_assign_string(cli_buffer.data, udp_receive_buffer);

		cli_receive_queue_push(&cli_buffer);
	}

	close(this->socket_fd);
	this->socket_fd = -1;

	string_free(&udp_receive_buffer);
}

void UDP::run_wrapper(void *ptr)
{
	UDP *_this = static_cast<UDP *>(ptr);

	_this->run();
}

void UDP::send(const cli_buffer_t *src)
{
	int sent;

	if(this->socket_fd < 0)
	{
		this->send_no_connection++;
		return;
	}

	sent = ::sendto(this->socket_fd, string_data(src->data), string_length(src->data), 0, (const struct sockaddr *)&src->ip.address.sin6_addr, src->ip.address.sin6_length);

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

	string_assign_cstr(call->result, "UDP INFO");
	string_append_cstr(call->result, "\nsending");
	string_format_append(call->result, "\n- sent bytes %u", this->send_bytes);
	string_format_append(call->result, "\n- sent packets: %u", this->send_packets);
	string_format_append(call->result, "\n- send errors: %u", this->send_errors);
	string_format_append(call->result, "\n- disconnected socket events: %u", this->send_no_connection);
	string_append_cstr(call->result, "\nreceiving");
	string_format_append(call->result, "\n- received bytes: %u", this->receive_bytes);
	string_format_append(call->result, "\n- received packets: %u", this->receive_packets);
	string_format_append(call->result, "\n- received incomplete packets: %u", this->receive_incomplete_packets);
	string_format_append(call->result, "\n- received invalid packets: %u", this->receive_invalid_packets);
	string_format_append(call->result, "\n- receive errors: %u", this->receive_errors);
}

static UDP *UDP_singleton = nullptr;

void net_udp_init(void)
{
	assert(!UDP_singleton);

	UDP_singleton = new UDP();

	assert(UDP_singleton);
}

void net_udp_send(const cli_buffer_t *src)
{
	assert(UDP_singleton);

	return(UDP_singleton->send(src));
}

void net_udp_command_info(cli_command_call_t *call)
{
	assert(UDP_singleton);

	return(UDP_singleton->command_info(call));
}
