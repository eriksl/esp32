#include "tcp.h"

#include "command.h"

#include "exception.h"
#include "packet.h"

#include <string.h> // for memcpy
#include <sys/socket.h>
#include <sys/poll.h>

#include <esp_pthread.h>

#include <thread>
#include <chrono>

TCP *TCP::singleton = nullptr;

TCP::TCP(Log &log_in) : log(log_in)
{
	if(this->singleton)
		throw(hard_exception("TCP: already active"));

	this->socket_fd = -1;
	this->running = false;
	this->command = nullptr;
	this->singleton = this;
}

TCP &TCP::get()
{
	if(!TCP::singleton)
		throw(hard_exception("TCP::get: not active"));

	return(*this->singleton);
}

void TCP::set(Command *in)
{
	if(!in)
		throw(hard_exception("TCP::set: invalid argument"));

	this->command = in;
}

void TCP::run()
{
	esp_err_t rv;
	esp_pthread_cfg_t thread_config = esp_pthread_get_default_config();

	if(this->running)
		throw(hard_exception("TCP::run: already running"));

	thread_config.thread_name = "tcp";
	thread_config.pin_to_core = 1;
	thread_config.stack_size = 2 * 1024;
	thread_config.prio = 1;
	thread_config.stack_alloc_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

	if((rv = esp_pthread_set_cfg(&thread_config)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "esp_pthread_set_cfg")));

	std::thread new_thread([this]() { this->thread_runner(); });

	this->running = true;

	new_thread.detach();
}

void TCP::thread_runner()
{
	std::string receive_buffer;
	int accept_fd, rv, length;
	struct sockaddr_in6 si6_addr;
	socklen_t si6_addr_length;
	struct pollfd pfd;

	try
	{
		if(!this->command)
			throw(hard_exception("TCP::thread_runner: command not set"));

		if((accept_fd = ::socket(AF_INET6, SOCK_STREAM, 0)) < 0)
			throw(transient_exception(this->log.errno_string_error(errno, "TCP::thread_runner: socket")));

		memset(&si6_addr, 0, sizeof(si6_addr));
		si6_addr.sin6_family = AF_INET6;
		si6_addr.sin6_port = htons(24);

		if(::bind(accept_fd, reinterpret_cast<const struct sockaddr *>(&si6_addr), sizeof(si6_addr)) != 0)
			throw(transient_exception(this->log.errno_string_error(errno, "TCP::thread_runner: bind")));

		if(::listen(accept_fd, 0) != 0)
			throw(transient_exception(this->log.errno_string_error(errno, "TCP::thread_runner: listen")));

		for(;;)
		{
			si6_addr_length = sizeof(si6_addr);

			if((socket_fd = accept(accept_fd, (struct sockaddr *)&si6_addr, &si6_addr_length)) < 0)
			{
				this->stats["connections failed"]++;
				continue;
			}

			this->stats["connections accepted"]++;

			for(;;)
			{
				receive_buffer.clear();

				pfd.fd = this->socket_fd;
				pfd.events = POLLIN;
				pfd.revents = 0;

				rv = poll(&pfd, 1, -1);

				if(rv < 0)
				{
					this->log.log_errno(errno, "tcp: poll error");
					this->stats["poll generic error"]++;
					break;
				}

				if(!(pfd.revents & POLLIN))
				{
					this->log.log_errno(errno, "tcp: socket error");
					this->stats["poll receive error"]++;
					break;
				}

				if(ioctl(this->socket_fd, FIONREAD, &length))
					throw(hard_exception("tcp: ioctl fionread"));

				receive_buffer.clear();
				receive_buffer.resize(length);

				length = ::recv(this->socket_fd, receive_buffer.data(), receive_buffer.size(), 0);

				if(length < 0)
				{
					this->stats["receive errors"]++;
					break;
				}

				if(length == 0)
				{
					this->stats["receive zero size packets"]++;
					this->log << "tcp: zero packet received";
					break;
				}

				receive_buffer.resize(length);

				this->stats["receive bytes"] += length;

				if(!Packet::valid(receive_buffer))
				{
					this->stats["receive invalid packet"]++;
					continue;
				}

				if(!Packet::complete(receive_buffer))
				{
					int offset, pending;

					offset = length;
					pending = Packet::length(receive_buffer) - offset;
					receive_buffer.resize(Packet::length(receive_buffer));

					while(pending > 0)
					{
						pfd.fd = this->socket_fd;
						pfd.events = POLLIN;
						pfd.revents = 0;

						rv = poll(&pfd, 1, 1000);

						if(rv < 0)
						{
							this->log.log_errno(errno, "tcp: poll error (2)");
							this->stats["receive fragment poll failures"]++;
							break;
						}

						if(rv == 0)
						{
							this->log << "tcp: timeout";
							this->stats["receive fragment poll timeouts"]++;
							break;
						}

						if(!(pfd.revents & POLLIN))
						{
							this->log.log_errno(errno, "tcp: socket error (2)");
							this->stats["receive fragment poll errors"]++;
							break;
						}

						length = ::recv(this->socket_fd, receive_buffer.data() + offset, pending, 0);

						if(length == 0)
							break;

						if(length < 0)
						{
							this->log << std::format("tcp: receive error (2): {:d}", length);
							this->stats["receive fragment receive errors"]++;
							break;
						}

						pending -= length;
						offset += length;
					}

					if(!Packet::complete(receive_buffer))
					{
						this->log << "tcp: packet incomplete";
						this->stats["receive packets incomplete"]++;
						continue;
					}

					length = Packet::length(receive_buffer);
					receive_buffer.resize(length);
				}

				command_response_t *command_response = new command_response_t;

				static_assert(sizeof(command_response->ip.address.sin6_addr) >= sizeof(struct sockaddr));
				static_assert(sizeof(command_response->ip.address.sin6_addr) >= sizeof(struct sockaddr_in));
				static_assert(sizeof(command_response->ip.address.sin6_addr) >= sizeof(struct sockaddr_in6));

				memcpy(&command_response->ip.address.sin6_addr, &si6_addr, si6_addr_length);
				command_response->ip.address.sin6_length = si6_addr_length;
				command_response->source = cli_source_wlan_tcp;
				command_response->mtu = this->mtu;
				command_response->packetised = 1;
				command_response->packet = receive_buffer;

				this->command->receive_queue_push(command_response);

				command_response = nullptr;
				receive_buffer.clear();

				this->stats["receive packets"]++;
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			close(socket_fd);
			socket_fd = -1;
		}

		throw(hard_exception("tcp threadrunner: loop breaks"));
	}
	catch(const hard_exception &e)
	{
		this->log.abort(std::format("tcp thread: hard exception: {}", e.what()).c_str());
	}
	catch(const transient_exception &e)
	{
		this->log.abort(std::format("tcp thread: transient exception: {}", e.what()).c_str());
	}
	catch(...)
	{
		this->log.abort("tcp thread: unknown exception");
	}

	for(;;)
		(void)0;
}

void TCP::send(const command_response_t *command_response)
{
	int length, offset, chunk_length, sent;

	if(!command_response)
		throw(hard_exception("TCP::send: invalid argument"));

	if(socket_fd < 0)
	{
		this->stats["send no connection"]++;
		return;
	}

	this->stats["send packets"]++;

	offset = 0;
	length = command_response->packet.length();

	if(!command_response->packetised && (length > command_response->mtu))
		length = command_response->mtu;

	for(;;)
	{
		chunk_length = length;

		if(chunk_length > this->mtu)
			chunk_length = this->mtu;

		sent = ::send(socket_fd, command_response->packet.data() + offset, chunk_length, 0);

		this->stats["send segments"]++;

		if(sent <= 0)
		{
			this->stats["send errors"]++;
			break;
		}

		this->stats["send bytes"] += sent;

		length -= sent;
		offset += sent;

		if(length < 0)
			throw(hard_exception("TCP::send: length < 0"));

		if(offset > command_response->packet.length())
			throw(hard_exception("TCP::send; offset >= command_response->packet.length()"));

		if(length == 0)
			break;
	}
}

void TCP::info(std::string &out)
{
	for(const auto &it : this->stats)
		out += std::format("\n{:<32s} {:d}", it.first, it.second);
}
