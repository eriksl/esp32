#include "display.h"
#include "display-module.h"
#include "display-spi.h"
#include "display-spi-generic.h"
#include "spi.h"

#include "exception.h"

DisplayModuleSPI::SPIDataBuffer::SPIDataBuffer(DisplayModuleSPI& parent_in, int size_in)
		:
	parent(parent_in),
	length(0),
	size(size_in)
{
	if(!(this->buffer = static_cast<unsigned char *>(heap_caps_malloc(this->size, MALLOC_CAP_DMA))))
		throw(hard_exception("DisplayModuleSPI: out of memory"));
}

void DisplayModuleSPI::SPIDataBuffer::set_leader(SPI::rodata_t in)
{
	this->leader = in;
}

void DisplayModuleSPI::SPIDataBuffer::clear_leader()
{
	this->leader.clear();
}

void DisplayModuleSPI::SPIDataBuffer::push(SPI::rodata_t in)
{
	if((this->length + in.length()) >= this->size)
		this->flush();

	if((this->length + in.length()) >= this->size)
		throw(hard_exception("DisplayModuleSPI::SPIDataBuffer::push: data length out of range"));

	for(auto c : in)
		this->buffer[this->length++] = c;
}

void DisplayModuleSPI::SPIDataBuffer::flush()
{
	SPI::rodata_t out(this->buffer, this->length);
	int address_bits = 0;
	std::uint64_t address = 0;

	for(auto c : this->leader)
	{
		if((address_bits += 8) > 64)
			throw("DisplayModuleSPI::SPIDataBuffer::flush: leader length out of range");

		address <<= 8;
		address |= (c & 0xff);
	}

	this->parent.transfer({
			.send =
			{
				.command = {},
				.address =
				{
					.bits = address_bits,
					.data = address,
				},
				.data = &out,
			},
			.receive = {},
	});

	this->length = 0;
}

DisplayModuleSPI::DisplayModuleSPI(Config& config_in, Log &log_in, Util& util_in, SPI& spi, LedPWM& ledpwm_in, int module_index, int x_size_in, int y_size_in,
	bool flip_in, bool invert_in, bool rotate_in, bool blinvert_in)
		:
	DisplayModule(config_in, log_in, util_in, ledpwm_in, x_size_in, y_size_in, flip_in, invert_in, rotate_in, blinvert_in),
	spi_module(spi[module_index]),
	spi_data_buffer(*this, this->spi_module.max_transaction_length())
{
}

std::string DisplayModuleSPI::_interface()
{
	return(std::format("SPI [{}]", this->spi_module.info()));
}

int DisplayModuleSPI::speed()
{
	return(this->spi_module.speed());
}

void DisplayModuleSPI::speed(int speed_in)
{
	this->spi_module.speed(speed_in);
}

void DisplayModuleSPI::transfer(const SPI::transfer_t& transfer)
{
	this->spi_module.transfer(transfer);
}

void DisplayModuleSPI::set_leader(SPI::rodata_t in)
{
	this->spi_data_buffer.set_leader(in);
}

void DisplayModuleSPI::clear_leader()
{
	this->spi_data_buffer.clear_leader();
}

void DisplayModuleSPI::push(SPI::rodata_t in)
{
	this->spi_data_buffer.push(in);
}

void DisplayModuleSPI::flush()
{
	this->spi_data_buffer.flush();
}
