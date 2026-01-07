#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>

extern "C"
{
#include "string.h"
#include "cli.h"
#include "wlan.h"
#include "config.h"
#include "log.h"
#include "util.h"
#include "packet.h"
#include "cli-command.h"
#include "notify.h"
}

#include "tcp.h"

#include <esp_timer.h>

#include <assert.h>

enum
{
	packet_size = 4096,
	packet_overhead = 128,
	tcp_mtu = 1200,
};

static int tcp_socket_fd = -1;
static TimerHandle_t tcp_defragmentation_timer;

static unsigned int tcp_send_bytes;
static unsigned int tcp_send_segments;
static unsigned int tcp_send_packets;
static unsigned int tcp_send_errors;
static unsigned int tcp_send_no_connection;
static unsigned int tcp_receive_bytes;
static unsigned int tcp_receive_packets;
static unsigned int tcp_receive_accepts;
static unsigned int tcp_receive_accept_errors;
static unsigned int tcp_receive_errors;
static unsigned int tcp_receive_defragmentation_timeouts;

static string_t tcp_receive_buffer;
static bool tcp_defragmentation_incomplete;

static void tcp_defragmentation_callback(TimerHandle_t handle)
{
	tcp_receive_defragmentation_timeouts++;
	log("tcp: defragmentation timed out");

	if(!tcp_defragmentation_incomplete)
		log("tcp: defragmentation while not active");

	tcp_defragmentation_incomplete = false;
	string_clear(tcp_receive_buffer);
}

static void run_tcp(void *)
{
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

		if((tcp_socket_fd = accept(accept_fd, (struct sockaddr *)&si6_addr, &si6_addr_length)) < 0)
		{
			tcp_receive_accept_errors++;
			continue;
		}

		int option = 1;
		rv = setsockopt(tcp_socket_fd, IPPROTO_TCP, TCP_NODELAY, &option, sizeof(option));
		assert(rv == 0);

		tcp_receive_accepts++;
		tcp_defragmentation_incomplete = false;
		string_clear(tcp_receive_buffer);

		for(;;)
		{
			util_memcpy(&cli_buffer.ip.address.sin6_addr, &si6_addr, si6_addr_length);
			cli_buffer.ip.address.sin6_length = si6_addr_length;

			length = string_recvfrom_fd(tcp_receive_buffer, tcp_socket_fd, (unsigned int *)0, (void *)0);

			if(length == 0)
				break;

			if(length <= 0)
			{
				log_format("tcp receive error: %d", length);
				tcp_receive_errors++;
				break;
			}

			tcp_receive_bytes += length;

			if(packet_valid(tcp_receive_buffer))
			{
				if(packet_complete(tcp_receive_buffer))
				{
					tcp_defragmentation_incomplete = false;
					xTimerStop(tcp_defragmentation_timer, portMAX_DELAY);
					cli_buffer.packetised = 1;
				}
				else
				{
					if(!tcp_defragmentation_incomplete)
					{
						tcp_defragmentation_incomplete = true;
						xTimerStart(tcp_defragmentation_timer, portMAX_DELAY);
					}
				}
			}
			else
			{
				tcp_defragmentation_incomplete = false;
				cli_buffer.packetised = 0;
			}

			if(!tcp_defragmentation_incomplete)
			{
				tcp_receive_packets++;

				cli_buffer.source = cli_source_wlan_tcp;
				cli_buffer.mtu = tcp_mtu;
				cli_buffer.data = string_new(string_length(tcp_receive_buffer));
				string_assign_string(cli_buffer.data, tcp_receive_buffer);
				string_clear(tcp_receive_buffer);
				tcp_defragmentation_incomplete = false;

				cli_receive_queue_push(&cli_buffer);
			}
		}

		close(tcp_socket_fd);
		tcp_socket_fd = -1;
	}

	string_free(&tcp_receive_buffer);
}

void net_tcp_send(const cli_buffer_t *src)
{
	int length, offset, chunk_length, sent;

	if(tcp_socket_fd < 0)
	{
		tcp_send_no_connection++;
		return;
	}

	tcp_send_packets++;

	offset = 0;
	length = string_length(src->data);

	if(!src->packetised && (length > src->mtu))
		length = src->mtu;

	for(;;)
	{
		chunk_length = length;

		if(chunk_length > tcp_mtu)
			chunk_length = tcp_mtu;

		sent = send(tcp_socket_fd, string_data(src->data) + offset, chunk_length, 0);

		tcp_send_segments++;

		if(sent <= 0)
		{
			tcp_send_errors++;
			break;
		}

		tcp_send_bytes += sent;

		length -= sent;
		offset += sent;

		assert(length >= 0);
		assert(offset <= string_length(src->data));

		if(length == 0)
			break;

		assert(offset < string_length(src->data));
	}
}

void net_tcp_init(void)
{
	tcp_defragmentation_timer = xTimerCreate("tcp-defrag", pdMS_TO_TICKS(500), pdFALSE, (void *)0, tcp_defragmentation_callback);
	assert(tcp_defragmentation_timer);
	tcp_defragmentation_incomplete = false;

	if(xTaskCreatePinnedToCore(run_tcp, "wlan-tcp", 3 * 1024, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
        util_abort("wlan: xTaskCreatePinnedToNode run_tcp");
}

void net_tcp_command_info(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "TCP INFO");
	string_append_cstr(call->result, "\nsending");
	string_format_append(call->result, "\n- sent bytes %u", tcp_send_bytes);
	string_format_append(call->result, "\n- sent segments %u", tcp_send_segments);
	string_format_append(call->result, "\n- sent packets: %u", tcp_send_packets);
	string_format_append(call->result, "\n- send errors: %u", tcp_send_errors);
	string_format_append(call->result, "\n- disconnected socket events: %u", tcp_send_no_connection);
	string_append_cstr(call->result, "\nreceiving");
	string_format_append(call->result, "\n- received bytes: %u", tcp_receive_bytes);
	string_format_append(call->result, "\n- received packets: %u", tcp_receive_packets);
	string_format_append(call->result, "\n- received defragmentation timeouts: %u", tcp_receive_defragmentation_timeouts);
	string_format_append(call->result, "\n- receive errors: %u", tcp_receive_errors);
	string_format_append(call->result, "\n- accepted connections: %u", tcp_receive_accepts);
	string_format_append(call->result, "\n- accept errors: %u", tcp_receive_accept_errors);
}
