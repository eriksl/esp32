#include "udp.h"

#include "command.h"

#include "exception.h"
#include "packet.h"

#include <string.h> // for memcpy
#include <sys/socket.h>
#include <sys/poll.h>

#include <esp_pthread.h>

#include <thread>
#include <chrono>

UDP *UDP::singleton = nullptr;

UDP::UDP(Log &log_in) : log(log_in)
{
	if(this->singleton)
		throw(hard_exception("UDP: already active"));

	this->socket_fd = -1;
	this->running = false;
	this->command = nullptr;
	this->singleton = this;
}

UDP &UDP::get()
{
	if(!UDP::singleton)
		throw(hard_exception("UDP::get: not active"));

	return(*this->singleton);
}

void UDP::set(Command *in)
{
	if(!in)
		throw(hard_exception("UDP::set: invalid argument"));

	this->command = in;
}

void UDP::run()
{
	esp_err_t rv;
	esp_pthread_cfg_t thread_config = esp_pthread_get_default_config();

	if(this->running)
		throw(hard_exception("UDP::run: already running"));

	thread_config.thread_name = "udp";
	thread_config.pin_to_core = 1;
	thread_config.stack_size = 2 * 1024;
	thread_config.prio = 1;
	thread_config.stack_alloc_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

	if((rv = esp_pthread_set_cfg(&thread_config)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "esp_pthread_set_cfg")));

	std::thread new_thread(UDP::thread_wrapper, this);

	this->running = true;

	new_thread.detach();
}

void UDP::thread_wrapper(void *in)
{
	UDP *this_;

	this_ = reinterpret_cast<UDP *>(in);

	this_->thread_runner();
}

void UDP::thread_runner()
{
	std::string receive_buffer;
	int rv, length;
	struct sockaddr_in6 si6_addr;
	socklen_t si6_addr_length;
	struct pollfd pfd;

	try
	{
		if(!this->command)
			throw(hard_exception("UDP::thread_runner: command not set"));

		if((this->socket_fd = ::socket(AF_INET6, SOCK_DGRAM, 0)) < 0)
			throw(transient_exception(this->log.errno_string_error(errno, "UDP::thread_runner: open")));

		memset(&si6_addr, 0, sizeof(si6_addr));
		si6_addr.sin6_family = AF_INET6;
		si6_addr.sin6_port = htons(24);

		if(::bind(this->socket_fd, reinterpret_cast<const struct sockaddr *>(&si6_addr), sizeof(si6_addr)) != 0)
			throw(transient_exception(this->log.errno_string_error(errno, "UDP::thread_runner: bind")));

		for(;;)
		{
			si6_addr_length = sizeof(si6_addr);

			pfd.fd = this->socket_fd;
			pfd.events = POLLIN;
			pfd.revents = 0;

			rv = poll(&pfd, 1, -1);

			if(rv < 0)
			{
				this->log.log_errno(errno, "udp: poll error");
				this->stats["poll generic error"]++;
				continue;
			}

			if(!(pfd.revents & POLLIN))
				throw(hard_exception("udp: socket error"));

			if(ioctl(this->socket_fd, FIONREAD, &length))
				throw(hard_exception("udp: ioctl fionread"));

			receive_buffer.clear();
			receive_buffer.resize(length);

			length = ::recvfrom(this->socket_fd, receive_buffer.data(), receive_buffer.size(), 0, reinterpret_cast<sockaddr *>(&si6_addr), &si6_addr_length);

			if(length < 0)
			{
				this->stats["receive errors"]++;
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				continue;
			}

			if(length == 0)
			{
				this->stats["receive zero size packets"]++;
				this->log << "udp: zero packet received";
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				continue;
			}

			receive_buffer.resize(length);

			this->stats["receive bytes"] += length;

			if(!Packet::valid(receive_buffer))
			{
				this->stats["receive invalid packets"]++;
				continue;
			}

			if(!Packet::complete(receive_buffer))
			{
				this->stats["receive incomplete packets"]++;
				continue;
			}

			command_response_t *command_response = new command_response_t;

			static_assert(sizeof(command_response->ip.address.sin6_addr) >= sizeof(struct sockaddr));
			static_assert(sizeof(command_response->ip.address.sin6_addr) >= sizeof(struct sockaddr_in));
			static_assert(sizeof(command_response->ip.address.sin6_addr) >= sizeof(struct sockaddr_in6));

			memcpy(&command_response->ip.address.sin6_addr, &si6_addr, si6_addr_length);
			command_response->ip.address.sin6_length = si6_addr_length;

			command_response->source = cli_source_wlan_udp;
			command_response->packetised = 1;
			command_response->mtu = this->mtu;
			command_response->packet = receive_buffer;

			this->command->receive_queue_push(command_response);

			command_response = nullptr;

			this->stats["receive packets"]++;
		}

		throw(hard_exception("loop breaks"));
	}
	catch(const hard_exception &e)
	{
		this->log.abort(std::format("udp thread: hard exception: {}", e.what()).c_str());
	}
	catch(const transient_exception &e)
	{
		this->log.abort(std::format("udp thread: transient exception: {}", e.what()).c_str());
	}
	catch(...)
	{
		this->log.abort("udp thread: unknown exception");
	}

	for(;;)
		(void)0;
}

void UDP::send(const command_response_t *command_response)
{
	int sent;

	if(!command_response)
		throw(hard_exception("UDP::send: invalid argument"));

	if(this->socket_fd < 0)
	{
		this->stats["send no connection"]++;
		return;
	}

	sent = ::sendto(this->socket_fd, command_response->packet.data(), command_response->packet.length(), 0,
			reinterpret_cast<const struct sockaddr *>(&command_response->ip.address.sin6_addr), command_response->ip.address.sin6_length);

	if(sent <= 0)
	{
		this->stats["send errors"]++;
		return;
	}

	this->stats["send packets"]++;
	this->stats["send bytes"] += sent;
}

void UDP::info(std::string &out)
{
	for(const auto &it : this->stats)
		out += std::format("\n{:<32s} {:d}", it.first, it.second);
}
