#include "spi.h"

#include "log.h"
#include "util.h"
#include "config.h"
#include "exception.h"

#include <driver/spi_master.h>

#include <format>

#include "magic_enum/magic_enum.hpp"

class SPI::Module::PrivateData
{
	public:

		spi_device_handle_t device_handle;
		spi_host_device_t spi_host;
};

SPI* SPI::singleton = nullptr;

SPI& SPI::get()
{
	if(!SPI::singleton)
		throw(hard_exception("SPI::get not active"));

	return(*SPI::singleton);
}

SPI::SPI(Log &log_in, Config &config_in) : log(log_in), config(config_in)
{
	if(this->singleton)
		throw(hard_exception("SPI: already active"));

	// FIXME hardcoded / BSP -> symbolic / environment

	this->modules.emplace_back(log, config, 0, /* CS */ 10, /* SCK */ 12, /* MOSI */ 11, /* MISO */ 13);

#if defined(CONFIG_BSP_SPI3_DISPLAY_CS) && defined(CONFIG_BSP_SPI3_SCK) && defined(CONFIG_BSP_SPI3_MOSI) && defined(CONFIG_BSP_SPI3_MISO)
	this->modules.emplace_back(log, config, 1, /* CS */ CONFIG_BSP_SPI3_DISPLAY_CS, /* SCK */ CONFIG_BSP_SPI3_SCK, /* MOSI */ CONFIG_BSP_SPI3_MOSI, /* MISO */ CONFIG_BSP_SPI3_MISO);
#endif

	this->singleton = this;
}

SPI::Module& SPI::operator[](int index)
{
	if((index < 0) || (index >= this->modules.size()))
		throw(transient_exception("SPI::[]: index out of range"));

	return(this->modules.at(index));
}

SPI::Module::Module(Log& log_in, Config& config_in, int module_in, int default_cs, int default_sck, int default_mosi, int default_miso)
	: log(log_in), config(config_in), module(module_in)
{
	esp_err_t rv;

	this->private_data = std::make_unique<PrivateData>();

	switch(this->module)
	{
		case(0): { this->private_data->spi_host = SPI2_HOST; break; }
		case(1): { this->private_data->spi_host = SPI3_HOST; break; }
		default: { throw(hard_exception("SPI::Module:: invalid SPI module")); break; }
	}

	this->cs =				this->get_config("cs", default_cs);
	this->sck =				this->get_config("sck", default_sck);
	this->miso =			this->get_config("miso", default_miso);
	this->mosi =			this->get_config("mosi", default_mosi);
	this->_speed =			this->get_config("speed", 100'000); // FIXME

	spi_bus_config_t bus_config =
	{
		.mosi_io_num = this->mosi,
		.miso_io_num = this->miso,
		.sclk_io_num = this->sck,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.data4_io_num = -1,
		.data5_io_num = -1,
		.data6_io_num = -1,
		.data7_io_num = -1,
		.data_io_default_level = false,
		.max_transfer_sz = 0,
		.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI,
		.isr_cpu_id = ESP_INTR_CPU_AFFINITY_1,
		.intr_flags = 0,
	};

	spi_device_interface_config_t device =
	{
		.command_bits = 0,
		.address_bits = 0,
		.dummy_bits = 0,
		.mode = 0,
		.clock_source = SPI_CLK_SRC_DEFAULT,
		.duty_cycle_pos = 0,
		.cs_ena_pretrans = 0,
		.cs_ena_posttrans = 0,
		.clock_speed_hz = this->_speed,
		.input_delay_ns = 0,
		.sample_point = static_cast<spi_sampling_point_t>(0),
		.spics_io_num = static_cast<int>(this->cs),
		.flags = 0,
		.queue_size = 1,
		.pre_cb = nullptr /* FIXME pre_callback, */,
		.post_cb = nullptr,
	};

	if((rv = spi_bus_initialize(this->private_data->spi_host, &bus_config, SPI_DMA_CH_AUTO)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "spi_bus_initialize")));

	if((rv = spi_bus_add_device(this->private_data->spi_host, &device, &this->private_data->device_handle)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "spi_bus_add_device")));

	if((rv = spi_device_acquire_bus(this->private_data->device_handle, portMAX_DELAY)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "spi_device_acquire_bus")));
}

SPI::Module::~Module()
{
}

int SPI::Module::get_config(const std::string& id, int default_value)
{
	int value;

	try
	{
		value = this->config.get_int(std::format("spi.{:d}.{}", this->module, id));
	}
	catch(const transient_exception &)
	{
		return(default_value);
	}

	return(value);
}

std::string SPI::Module::info()
{
	return(std::format("module {:d} ({},{:d}), cs: {:d}, sck: {:d}, mosi: {:d}, miso: {:d}, current speed MHz: {:.3f}",
			this->module, magic_enum::enum_name(this->private_data->spi_host), magic_enum::enum_integer(this->private_data->spi_host),
			this->cs, this->sck, this->mosi, this->miso, this->_speed / 1'000'000.0));
}

int SPI::Module::speed()
{
	return(this->_speed);
}

void SPI::Module::speed(int speed_in)
{
	if(speed_in <= 0)
		throw(hard_exception("SPI::Module::speed: invalid argument"));

	this->_speed = speed_in;
}

int SPI::Module::max_transaction_length()
{
	esp_err_t rv;
	size_t length;

	if((rv = spi_bus_get_max_transaction_len(this->private_data->spi_host, &length)) != ESP_OK)
		throw(transient_exception(this->log.esp_string_error(rv, "SPI::Module::spi_bus_get_max_transaction_len")));

	return(length);
}

void SPI::Module::transfer(const transfer_t& transfer)
{
	int rv;
	spi_transaction_ext_t transaction_ext;
	std::scoped_lock<std::mutex> lock(this->mutex);
	int out_length;
	int in_length;

	memset(&transaction_ext, 0, sizeof(transaction_ext));

	transaction_ext.base.flags = 0;
	transaction_ext.base.cmd = 0;
	transaction_ext.base.addr = 0;
	transaction_ext.base.length = 0;
	transaction_ext.base.rxlength = 0;
	transaction_ext.base.tx_buffer = nullptr;
	transaction_ext.base.rx_buffer = nullptr;
	transaction_ext.base.user = nullptr; /* FIXME &callback_data_gpio_off; */
	transaction_ext.command_bits = 0;
	transaction_ext.address_bits = 0;
	transaction_ext.dummy_bits = 0;

	out_length = 0;
	in_length = 0;

	if(transfer.send.command.bits > 0)
	{
		transaction_ext.base.flags |= SPI_TRANS_VARIABLE_CMD;
		transaction_ext.command_bits = transfer.send.command.bits;
		transaction_ext.base.cmd = transfer.send.command.data;
	}

	if(transfer.send.address.bits > 0)
	{
		transaction_ext.base.flags |= SPI_TRANS_VARIABLE_ADDR;
		transaction_ext.address_bits = transfer.send.address.bits;
		transaction_ext.base.addr = transfer.send.address.data;
	}

	if(transfer.send.data && transfer.send.data->size() > 0)
	{
		out_length = transfer.send.data->size();
		transaction_ext.base.tx_buffer = transfer.send.data->data();
	}

	if(transfer.receive.data && transfer.receive.length > 0)
	{
		in_length = transfer.receive.length;
		transfer.receive.data->resize(in_length);
		transaction_ext.base.rx_buffer = transfer.receive.data->data();
		transaction_ext.base.rxlength = transfer.receive.length * 8;
	}

	transaction_ext.base.length = (out_length * 8) + (in_length * 8);
	transaction_ext.base.override_freq_hz = this->_speed;

	if((rv = spi_device_transmit(this->private_data->device_handle, reinterpret_cast<spi_transaction_t *>(&transaction_ext))) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "spi_device_transmit")));
}
