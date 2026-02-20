#include <errno.h>
#include <string.h> // for strerror()

#include "log.h"

#include "util.h"
#include "console.h"
#include "exception.h"

#include <string>
#include <format>

#include <stdlib.h>

#include <esp_log.h> // for log function
#include <esp_random.h> // for esp_random

Log *Log::singleton = nullptr;

RTC_NOINIT_ATTR char Log::rtc_slow_memory[Log::log_buffer_size];

Log::Log(Console &console_in, Util &util_in) :
		console(console_in), util(util_in)
{
	if(this->singleton)
		throw(hard_exception("Log: already active"));

	this->monitor = false;

	this->log_buffer = reinterpret_cast<log_t *>(reinterpret_cast<void *>(rtc_slow_memory));

	if(!(this->data_mutex = xSemaphoreCreateMutex()))
		throw(hard_exception("Log: cannot create semaphore"));

	if(!(display_queue = xQueueCreate(log_buffer_entries, sizeof(int))))
		throw(hard_exception("Log: cannot create queue"));

	if((this->log_buffer->magic_word != this->log_buffer_magic_word) ||
		(this->log_buffer->magic_word_salted != (this->log_buffer_magic_word ^ this->log_buffer->random_salt)))
	{
		this->clear();
		this->log("log: log buffer corrupt, reinit");
	}

	this->singleton = this;

	esp_log_set_vprintf(idf_logging_function);

	this->log("boot");
}

Log &Log::get()
{
	if(!Log::singleton)
		throw(hard_exception("Log: not active"));

	return(*Log::singleton);
}

void Log::clear(void)
{
	std::uint32_t random_value;

	random_value = esp_random(); // FIXME -> mbedtls

	data_mutex_take();

	log_buffer->magic_word = log_buffer_magic_word;
	log_buffer->random_salt = random_value;
	log_buffer->magic_word_salted = log_buffer_magic_word ^ random_value;
	log_buffer->entries = log_buffer_entries;
	log_buffer->in = 0;
	log_buffer->out = 0;

	data_mutex_give();
}

void Log::log(std::string_view in)
{
	int current, ix;
	log_entry_t *entry;

	this->data_mutex_take();

	current = this->log_buffer->in;
	entry = &this->log_buffer->entry[current];

	entry->timestamp = time(nullptr);

	ix = 0;

	for(const auto ini : in)
	{
		if((ix + 1) >= sizeof(entry->data))
			break;

		entry->data[ix++] = ini;
	}

	entry->data[ix] = '\0';

	if(++this->log_buffer->in >= this->log_buffer_entries)
		this->log_buffer->in = 0;

	this->data_mutex_give();

	if(monitor)
		this->console.write(in);

	this->signal_display(current);
}

void Log::log_esperr(esp_err_t e, std::string_view in)
{
	this->log(std::format("{}: {} [{:#x}]", in, esp_err_to_name(e), e));
}

void Log::log_errno(int e, std::string_view in)
{
	this->log(std::format("{}: {} [{:#x}]", in, strerror(e), e));
}

int Log::idf_logging_function(const char *fmt, va_list ap)
{
	char buffer[log_buffer_data_size];
	char *start;
	char *end;
	int length;

	length = vsnprintf(buffer, sizeof(buffer), fmt, ap);

	for(start = buffer; *start; start++)
		if(*start == ':')
			break;

	if((start[0] == ':') && (start[1]))
		start = start + 1;
	else
		start = buffer;

	for(end = start; *end; end++)
		if(*end < ' ')
			break;

	*end = '\0';

	if(start != end)
		Log::get().log(start);

	return(length);
}

QueueHandle_t Log::get_display_queue(void)
{
	return(display_queue);
}

void Log::get_entry(int entry_index, time_t &stamp, std::string &out)
{
	const log_entry_t *entry;

	stamp = static_cast<time_t>(0);

	data_mutex_take();

	if((entry_index >= 0) && (entry_index < this->log_buffer->entries))
	{
		entry = &log_buffer->entry[entry_index];

		stamp = entry->timestamp;
		out = entry->data;
	}

	data_mutex_give();
}

void Log::setmonitor(bool val)
{
	this->monitor = val;
}

bool Log::getmonitor()
{
	return(this->monitor);
}

void Log::signal_display(int item)
{
	assert((item >= 0) && (item < log_buffer_entries));

	if(display_queue)
		xQueueSend(this->display_queue, &item, static_cast<TickType_t>(0));
}

void Log::info(std::string &out)
{
	this->data_mutex_take();

	out += std::format(  "  buffer: {:p}", static_cast<const void *>(this->log_buffer));
	out += std::format("\n  magic word: {:#10x}", this->log_buffer->magic_word);
	out += std::format("\n  random salt: {:#10x}", this->log_buffer->random_salt);
	out += std::format("\n  magic word salted: {:#10x}", this->log_buffer->magic_word_salted);
	out += std::format("\n  entries: {:d}", this->log_buffer->entries);
	out += std::format("\n  last entry added: {:d}", this->log_buffer->in);
	out += std::format("\n  last entry viewed: {:d}", this->log_buffer->out);

	this->data_mutex_give();
}

void Log::command_log(std::string &out, int entry)
{
	unsigned int entries, amount;

	if(entry >= 0)
		this->log_buffer->out = entry;

	this->data_mutex_take();

	if(this->log_buffer->in > this->log_buffer->out)
		entries = this->log_buffer->in - this->log_buffer->out;
	else
		entries = this->log_buffer->in + (this->log_buffer_entries - this->log_buffer->out);

	if(entries == this->log_buffer_entries)
		entries = 0;

	out += std::format("LOG {:d} entries:", entries);
	amount = 0;

	for(amount = 0; (amount < 24) && (amount < entries); amount++)
	{
		out += std::format("\n{:3d} {} {}",
				this->log_buffer->out,
				this->util.time_to_string(this->log_buffer->entry[this->log_buffer->out].timestamp),
				this->log_buffer->entry[this->log_buffer->out].data);

		if(++this->log_buffer->out >= this->log_buffer_entries)
			this->log_buffer->out = 0;
	}

	this->data_mutex_give();

	if(amount != entries)
		out += std::format("\n[{:d} more]", entries - amount);
}

void Log::warn_on_esp_err(std::string_view what, unsigned int rv)
{
	if(rv == ESP_OK)
		return;

	try
	{
		this->log_esperr(rv, what);
	}
	catch(...)
	{
	}
}

void Log::abort_on_esp_err(std::string_view what, int rv)
{
	if(rv == ESP_OK)
		return;

	try
	{
		this->setmonitor(true);
	}
	catch(...)
	{
	}

	this->warn_on_esp_err(what, rv);

	::abort();
}

void Log::abort(std::string_view what)
{
	try
	{
		this->setmonitor(true);
	}
	catch(...)
	{
	}

	try
	{
		this->log(std::format("abort: %s", what));
	}
	catch(...)
	{
	}

	::abort();
}

std::string Log::esp_string_error(esp_err_t e, std::string_view message)
{
	return(std::format("{}: \"{}\" [{:d}]", message, esp_err_to_name(e), e));
}

std::string Log::errno_string_error(int e, std::string_view message)
{
	return(std::format("{}: \"{}\" [{:d}]", message, strerror(e), e));
}
