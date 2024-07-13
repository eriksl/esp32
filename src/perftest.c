#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>

#include "perftest.h"
#include "string.h"
#include "log.h"
#include "util.h"

static bool inited = false;

enum
{
	//malloc_type = MALLOC_CAP_INTERNAL
	malloc_type = MALLOC_CAP_SPIRAM
};

static void run_tcp_receive(void *)
{
	enum { size = 4096 };
	char *receive_buffer;
	int accept_fd;
	struct sockaddr_in6 si6_addr;
	socklen_t si6_addr_length;
	int length;
	int tcp_socket_fd;
	static const char *ack = "ACK";
	enum { attempts = 8 };
	unsigned int attempt;

	assert(inited);

	receive_buffer = heap_caps_malloc(size, malloc_type);

	memset(&si6_addr, 0, sizeof(si6_addr));
	si6_addr.sin6_family = AF_INET6;
	si6_addr.sin6_port = htons(9); // discard

	assert((accept_fd = socket(AF_INET6, SOCK_STREAM, 0)) >= 0);
	assert(bind(accept_fd, (const struct sockaddr *)&si6_addr, sizeof(si6_addr)) == 0);
	assert(listen(accept_fd, 0) == 0);

	for(;;)
	{
		si6_addr_length = sizeof(si6_addr);

		if((tcp_socket_fd = accept(accept_fd, (struct sockaddr *)&si6_addr, &si6_addr_length)) < 0)
		{
			log_format_errno("perftest: accept fails: %d", tcp_socket_fd);
			continue;
		}

		int option = 1;
		assert(!setsockopt(tcp_socket_fd, IPPROTO_TCP, TCP_NODELAY, &option, sizeof(option)));

		assert(sizeof(si6_addr) >= si6_addr_length);

		for(;;)
		{
			length = recv(tcp_socket_fd, receive_buffer, size, 0);

			if(length <= 0)
			{
				log_format("perftest tcp recv: %d", length);
				break;
			}

			for(attempt = attempts; attempt > 0; attempt--)
			{
				length = send(tcp_socket_fd, ack, sizeof(ack), 0);

				if(length == sizeof(ack))
					break;

				log_format("perftest tcp send ack: %d, try %d", length, attempt);
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}

			if(attempt == 0)
				log("perftest tcp send ack: no more tries");
		}

		close(tcp_socket_fd);
	}
}

static void run_tcp_send(void *)
{
	enum { size = 4096 };
	char *send_buffer;
	int accept_fd;
	struct sockaddr_in6 si6_addr;
	socklen_t si6_addr_length;
	int length;
	int tcp_socket_fd;
	static const char *ack = "ACK";
	enum { attempts = 8 };
	unsigned int attempt;

	assert(inited);

	send_buffer = heap_caps_malloc(size, malloc_type);

	memset(&si6_addr, 0, sizeof(si6_addr));
	si6_addr.sin6_family = AF_INET6;
	si6_addr.sin6_port = htons(19); // chargen

	assert((accept_fd = socket(AF_INET6, SOCK_STREAM, 0)) >= 0);
	assert(bind(accept_fd, (const struct sockaddr *)&si6_addr, sizeof(si6_addr)) == 0);
	assert(listen(accept_fd, 0) == 0);

	for(;;)
	{
		si6_addr_length = sizeof(si6_addr);

		if((tcp_socket_fd = accept(accept_fd, (struct sockaddr *)&si6_addr, &si6_addr_length)) < 0)
		{
			log_format_errno("perftest: accept fails: %d", tcp_socket_fd);
			continue;
		}

		int option = 1;
		assert(!setsockopt(tcp_socket_fd, IPPROTO_TCP, TCP_NODELAY, &option, sizeof(option)));

		assert(sizeof(si6_addr) >= si6_addr_length);

		for(;;)
		{
			length = recv(tcp_socket_fd, send_buffer, sizeof(ack), 0);

			if(length <= 0)
			{
				log_format("perftest tcp revc 2: %d", length);
				break;
			}

			for(attempt = attempts; attempt > 0; attempt--)
			{
				length = send(tcp_socket_fd, send_buffer, size, 0);

				if(length == size)
					break;

				if((length < 0) && ((errno == ENOTCONN) || (errno == ECONNRESET)))
					goto abort;

				log_format_errno("perftest tcp send 2: %d, try %d", length, attempt);
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}

			if(attempt == 0)
				log("perftest tcp send 2: no more tries");
		}

abort:
		close(tcp_socket_fd);
	}
}

static void run_udp_receive(void *)
{
	enum { size = 4096 };
	char *receive_buffer;
	struct sockaddr_in6 si6_addr;
	socklen_t si6_addr_length;
	int length;
	int udp_socket_fd;
	static const char *ack = "ACK";
	enum { attempts = 8 };
	unsigned int attempt;

	assert(inited);

	receive_buffer = heap_caps_malloc(size, malloc_type);

	memset(&si6_addr, 0, sizeof(si6_addr));
	si6_addr.sin6_family = AF_INET6;
	si6_addr.sin6_port = htons(9); // discard

	assert((udp_socket_fd = socket(AF_INET6, SOCK_DGRAM, 0)) >= 0);
	assert(bind(udp_socket_fd, (const struct sockaddr *)&si6_addr, sizeof(si6_addr)) == 0);

	for(;;)
	{
		si6_addr_length = sizeof(si6_addr);

		length = recvfrom(udp_socket_fd, receive_buffer, size, 0, (struct sockaddr *)&si6_addr, &si6_addr_length);

		assert(sizeof(si6_addr) >= si6_addr_length);

		if(length <= 0)
		{
			log_format("perftest udp recv: %d", length);
			continue;
		}

		for(attempt = attempts; attempt > 0; attempt--)
		{
			length = sendto(udp_socket_fd, ack, sizeof(ack), 0, (const struct sockaddr *)&si6_addr, si6_addr_length);

			if(length == sizeof(ack))
				break;

			log_format("perftest udp send ack: %d, try %d", length, attempt);
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}

		if(attempt == 0)
			log("perftest udp send ack: no more tries");
	}

	close(udp_socket_fd);
}

static void run_udp_send(void *)
{
	enum { size = 4096 };
	char *send_buffer;
	struct sockaddr_in6 si6_addr;
	socklen_t si6_addr_length;
	int length;
	int udp_socket_fd;
	static const char *ack = "ACK";
	enum { attempts = 8 };
	unsigned int attempt;

	assert(inited);

	send_buffer = heap_caps_malloc(size, malloc_type);

	memset(&si6_addr, 0, sizeof(si6_addr));
	si6_addr.sin6_family = AF_INET6;
	si6_addr.sin6_port = htons(19); // chargen

	assert((udp_socket_fd = socket(AF_INET6, SOCK_DGRAM, 0)) >= 0);
	assert(bind(udp_socket_fd, (const struct sockaddr *)&si6_addr, sizeof(si6_addr)) == 0);

	for(;;)
	{
		si6_addr_length = sizeof(si6_addr);

		length = recvfrom(udp_socket_fd, send_buffer, sizeof(ack), 0, (struct sockaddr *)&si6_addr, &si6_addr_length);

		assert(sizeof(si6_addr) >= si6_addr_length);

		if(length <= 0)
		{
			log_format("perftest udp recv 2: %d", length);
			continue;
		}

		for(attempt = attempts; attempt > 0; attempt--)
		{
			length = sendto(udp_socket_fd, send_buffer, size, 0, (const struct sockaddr *)&si6_addr, si6_addr_length);

			if(length == size)
				break;

			log_format("perftest udp send 2: %d, try %d", length, attempt);
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}

		if(attempt == 0)
			log("perftest udp send 2: no more tries");
	}

	close(udp_socket_fd);
}

void perftest_init(void)
{
	assert(!inited);

	inited = true;

	if(xTaskCreatePinnedToCore(run_tcp_receive, "perf-tcp-recv", 2 * 1024, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("perftest: xTaskCreatePinnedToNode tcp receive");

	if(xTaskCreatePinnedToCore(run_tcp_send, "perf-tcp-send", 2 * 1024, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("perftest: xTaskCreatePinnedToNode tcp send");

	if(xTaskCreatePinnedToCore(run_udp_receive, "perf-udp-recv", 2 * 1024, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("perftest: xTaskCreatePinnedToNode udp receive");

	if(xTaskCreatePinnedToCore(run_udp_send, "perf-udp-send", 2 * 1024, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("perftest: xTaskCreatePinnedToNode udp send");
}
