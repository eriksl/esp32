#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>

extern "C" {
#include "string.h"
#include "cli.h"
#include "log.h"
#include "util.h"
#include "packet.h"
#include "cli-command.h"
}

#include "udp.h"

#include <assert.h>

class UDP;

extern "C"
{
	static void run(UDP *);
}

class UDP
{
	public:

		enum
		{
			packet_size = 4096, // FIXME
			packet_overhead = 128, // FIXME
			udp_mtu = packet_size + packet_overhead,
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

	if(xTaskCreatePinnedToCore(reinterpret_cast<TaskFunction_t>(&run), "udp", 3 * 1024, static_cast<void *>(this), 1, (TaskHandle_t *)0, 1) != pdPASS)
        util_abort("udp: xTaskCreatePinnedToCore");
}

static void run(UDP *_this)
{
	string_t udp_receive_buffer;
	int rv;
	struct sockaddr_in6 si6_addr;
	unsigned int si6_addr_length;
	int length;
	cli_buffer_t cli_buffer;

	static_assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr));
	static_assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr_in));
	static_assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr_in6));

	udp_receive_buffer = string_new(UDP::udp_mtu);
	assert(udp_receive_buffer);

	_this->socket_fd = socket(AF_INET6, SOCK_DGRAM, 0);
	assert(_this->socket_fd >= 0);

	memset(&si6_addr, 0, sizeof(si6_addr));
	si6_addr.sin6_family = AF_INET6;
	si6_addr.sin6_port = htons(24);

	rv = ::bind(_this->socket_fd, (const struct sockaddr *)&si6_addr, sizeof(si6_addr));
	assert(rv == 0);

	for(;;)
	{
		si6_addr_length = sizeof(si6_addr);

		string_clear(udp_receive_buffer);
		length = string_recvfrom_fd(udp_receive_buffer, _this->socket_fd, &si6_addr_length, &si6_addr);

		util_memcpy(&cli_buffer.ip.address.sin6_addr, &si6_addr, si6_addr_length);
		cli_buffer.ip.address.sin6_length = si6_addr_length;

		if(length == 0)
		{
			_this->receive_errors++;
			log("udp: zero packet received");
			util_sleep(100);
			continue;
		}

		if(length < 0)
		{
			_this->receive_errors++;
			util_sleep(100);
			continue;
		}

		_this->receive_bytes += length;

		if(!packet_complete(udp_receive_buffer))
		{
			_this->receive_incomplete_packets++;
			continue;
		}

		if(!packet_valid(udp_receive_buffer))
		{
			_this->receive_invalid_packets++;
			continue;
		}

		_this->receive_packets++;

		cli_buffer.source = cli_source_wlan_udp;
		cli_buffer.packetised = 1;
		cli_buffer.mtu = UDP::udp_mtu;
		cli_buffer.data = string_new(string_length(udp_receive_buffer));
		string_assign_string(cli_buffer.data, udp_receive_buffer);

		cli_receive_queue_push(&cli_buffer);
	}

	close(_this->socket_fd);
	_this->socket_fd = -1;

	string_free(&udp_receive_buffer);
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
