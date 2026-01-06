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

enum
{
	packet_size = 4096, // FIXME
	packet_overhead = 128, // FIXME
	udp_mtu = packet_size + packet_overhead,
};

static bool inited = false;

static unsigned int udp_send_bytes;
static unsigned int udp_send_packets;
static unsigned int udp_send_errors;
static unsigned int udp_send_no_connection;
static unsigned int udp_receive_bytes;
static unsigned int udp_receive_packets;
static unsigned int udp_receive_errors;
static unsigned int udp_receive_incomplete_packets;
static unsigned int udp_receive_invalid_packets;

static int udp_socket_fd = -1;

static void run(void *)
{
	string_t udp_receive_buffer;
	int rv;
	struct sockaddr_in6 si6_addr;
	unsigned int si6_addr_length;
	int length;
	cli_buffer_t cli_buffer;

	assert(inited);
	static_assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr));
	static_assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr_in));
	static_assert(sizeof(cli_buffer.ip.address.sin6_addr) >= sizeof(struct sockaddr_in6));

	udp_receive_buffer = string_new(udp_mtu);
	assert(udp_receive_buffer);

	udp_socket_fd = socket(AF_INET6, SOCK_DGRAM, 0);
	assert(udp_socket_fd >= 0);

	memset(&si6_addr, 0, sizeof(si6_addr));
	si6_addr.sin6_family = AF_INET6;
	si6_addr.sin6_port = htons(24);

	rv = bind(udp_socket_fd, (const struct sockaddr *)&si6_addr, sizeof(si6_addr));
	assert(rv == 0);

	for(;;)
	{
		si6_addr_length = sizeof(si6_addr);

		string_clear(udp_receive_buffer);
		length = string_recvfrom_fd(udp_receive_buffer, udp_socket_fd, &si6_addr_length, &si6_addr);

		util_memcpy(&cli_buffer.ip.address.sin6_addr, &si6_addr, si6_addr_length);
		cli_buffer.ip.address.sin6_length = si6_addr_length;

		if(length == 0)
		{
			udp_receive_errors++;
			log("udp: zero packet received");
			util_sleep(100);
			continue;
		}

		if(length < 0)
		{
			udp_receive_errors++;
			util_sleep(100);
			continue;
		}

		udp_receive_bytes += length;

		if(!packet_complete(udp_receive_buffer))
		{
			udp_receive_incomplete_packets++;
			continue;
		}

		if(!packet_valid(udp_receive_buffer))
		{
			udp_receive_invalid_packets++;
			continue;
		}

		udp_receive_packets++;

		cli_buffer.source = cli_source_wlan_udp;
		cli_buffer.packetised = 1;
		cli_buffer.mtu = udp_mtu;
		cli_buffer.data = string_new(string_length(udp_receive_buffer));
		string_assign_string(cli_buffer.data, udp_receive_buffer);

		cli_receive_queue_push(&cli_buffer);
	}

	close(udp_socket_fd);
	udp_socket_fd = -1;

	string_free(&udp_receive_buffer);
}

void net_udp_send(const cli_buffer_t *src)
{
	int sent;

	assert(inited);

	if(udp_socket_fd < 0)
	{
		udp_send_no_connection++;
		return;
	}

	sent = sendto(udp_socket_fd, string_data(src->data), string_length(src->data), 0, (const struct sockaddr *)&src->ip.address.sin6_addr, src->ip.address.sin6_length);

	if(sent <= 0)
	{
		udp_send_errors++;
		return;
	}

	udp_send_packets++;
	udp_send_bytes += sent;
}

void net_udp_init(void)
{
	assert(!inited);

	if(xTaskCreatePinnedToCore(run, "udp", 3 * 1024, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
        util_abort("udp: xTaskCreatePinnedToCore");

	inited = true;
}

void net_udp_command_info(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "UDP INFO");
	string_append_cstr(call->result, "\nsending");
	string_format_append(call->result, "\n- sent bytes %u", udp_send_bytes);
	string_format_append(call->result, "\n- sent packets: %u", udp_send_packets);
	string_format_append(call->result, "\n- send errors: %u", udp_send_errors);
	string_format_append(call->result, "\n- disconnected socket events: %u", udp_send_no_connection);
	string_append_cstr(call->result, "\nreceiving");
	string_format_append(call->result, "\n- received bytes: %u", udp_receive_bytes);
	string_format_append(call->result, "\n- received packets: %u", udp_receive_packets);
	string_format_append(call->result, "\n- received incomplete packets: %u", udp_receive_incomplete_packets);
	string_format_append(call->result, "\n- received invalid packets: %u", udp_receive_invalid_packets);
	string_format_append(call->result, "\n- receive errors: %u", udp_receive_errors);
}
