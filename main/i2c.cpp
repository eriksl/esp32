#include "i2c.h"

#include "log.h"
#include "util.h"
#include "config.h"
#include "exception.h"

#include <ulp_riscv.h>
#include <ulp_riscv_i2c.h>

#include <format>

namespace I2C
{
	I2c* singleton = nullptr;
}

using namespace I2C;

I2c& I2C::get()
{
	if(!singleton)
		throw(hard_exception("I2c::get not active"));

	return(*singleton);
}

I2c::I2c(Log &log_in, Config &config_in) : log(log_in), config(config_in)
{
	if(singleton)
		throw(hard_exception("I2c: already active"));

#if((CONFIG_BSP_I2C0_SDA >= 0) && (CONFIG_BSP_I2C0_SCL >= 0))
	this->_modules[0] = new MainModule(log, config, *this, 0, 0, CONFIG_BSP_I2C0_SDA, CONFIG_BSP_I2C0_SCL);
#endif

#if((CONFIG_BSP_I2C1_SDA >= 0) && (CONFIG_BSP_I2C1_SCL >= 0))
	this->_modules[1] = new MainModule(log, config, *this, 1, 1, CONFIG_BSP_I2C1_SDA, CONFIG_BSP_I2C1_SCL);
#endif

#if((CONFIG_BSP_I2C2_SDA >= 0) && (CONFIG_BSP_I2C2_SCL >= 0))
	this->_modules[2] = new ULPModule(log, config, *this, 2, 0, CONFIG_BSP_I2C2_SDA, CONFIG_BSP_I2C2_SCL);
#endif

	singleton = this;
}

bool I2c::probe(int module_index, int bus_index, int address)
{
	std::map<int, Module *>::const_iterator module;

	if((module = this->_modules.find(module_index)) == this->_modules.end())
		throw(transient_exception(std::format("I2c::probe: invalid module {:d}", module_index)));

	return(module->second->probe(bus_index, address));
}

Device* I2c::find(int module, int bus, int address)
{
	std::map<int, Module *>::const_iterator it;

	if((it = this->_modules.find(module)) == this->_modules.end())
		return(nullptr);

	return(it->second->find(bus, address));
}

void I2c::speed(int module_index, int speed)
{
	std::map<int, Module *>::iterator it;

	if((it = this->_modules.find(module_index)) == this->_modules.end())
		throw(transient_exception("invalid module"));

	it->second->speed(speed);
}

void I2c::info(std::string &out)
{
	std::map<int, Module *>::const_iterator it;

	for(it = this->_modules.begin(); it != this->_modules.end(); it++)
		out += it->second->info();
}

int I2c::speed(int module_index)
{
	std::map<int, Module *>::iterator it;

	if((it = this->_modules.find(module_index)) == this->_modules.end())
		throw(transient_exception("invalid module"));

	return(it->second->speed());
}

std::map<int, Module *>& I2c::modules()
{
	return(this->_modules);
}

Device* I2c::new_device(int module_index, int bus_index, int address, std::string_view name)
{
	std::map<int, Module *>::const_iterator module;
	Bus* bus;
	Bus* bus0;
	Device *device;

	if((module = this->_modules.find(module_index)) == this->_modules.end())
		throw(transient_exception(std::format("I2c::new_device: {}: invalid module {:d}", name, module_index)));

	try
	{
		bus = module->second->bus(bus_index);
	}
	catch(const transient_exception &e)
	{
		throw(transient_exception(std::format("I2c::new_device: {}: invalid bus {:d}", name, bus_index)));
	}

	try
	{
		if(bus_index != 0)
		{
			bus0 = module->second->bus(0);

			if(bus0->find(address))
				throw(transient_exception(std::format("I2c::new_device: {}: device on bus {:d} already registered on root bus", name, bus_index)));
		}
	}
	catch(const transient_exception &e)
	{
	}

	try
	{
		if(!(device = bus->new_device(address, name)))
			throw(hard_exception("bus->new_device returns nullptr"));
	}
	catch(const transient_exception &e)
	{
		throw(transient_exception(std::format("I2c::new_device: {}: invalid device/address {:#04x}@{:d}/{:d}: {}", name, address, module_index, bus_index,  e.what())));
	}

	return(device);
}

Module::Module(Log &log_in, Config &config_in, I2c &root_in, int module_index_in, int device_port_in, int sda_in, int scl_in)
	: log(log_in), config(config_in), sda(sda_in), scl(scl_in), module_index(module_index_in), device_port(device_port_in), root_ref(root_in)
{
}

Module::~Module()
{
}

I2c& Module::root()
{
	return(this->root_ref);
}

Bus* Module::bus(int bus_index)
{
	std::map<int, Bus *>::iterator it;

	if((it = this->_buses.find(bus_index)) == this->_buses.end())
		throw(transient_exception("I2c::Module::bus invalid bus"));

	return(it->second);
}

bool Module::probe_mux()
{
	data_t buffer;

	if(!this->_probe(bus_mux_address))
		return(false);

	try
	{
		this->_send_receive(bus_mux_address, data_t(1, 0xff), 1, buffer);
	}
	catch(const transient_exception &e)
	{
		return(false);
	}

	if((buffer.at(0) != 0xff))
		return(false);

	try
	{
		this->_send_receive(bus_mux_address, data_t(1, 0x00), 1, buffer);
	}
	catch(const transient_exception &e)
	{
		return(false);
	}

	if(buffer.at(0) != 0x00)
		return(false);

	return(true);
}

void Module::create_buses(int amount)
{
	int bus_index;
	std::string bus_name;

	for(bus_index = 0; bus_index < amount; bus_index++)
	{
		if(bus_index == 0)
			bus_name = "root bus";
		else
			bus_name = std::format("bus {:d}", bus_index);

		this->_buses[bus_index] = new Bus(this->log, *this, bus_index, bus_name);
	}
}

void Module::select_bus(int bus)
{
	if(this->_buses.find(bus) == this->_buses.end())
		throw(hard_exception(std::format("I2c::Module::select_bus: invalid bus {:d}", bus)));

	try
	{
		if(this->has_mux)
		{
			int bus_bits;

			if(bus == 0)
				bus_bits = 0;
			else
				bus_bits = 1 << (bus - 1);

			this->_set_mux(bus_bits);
		}
	}
	catch(const transient_exception &e)
	{
		throw(transient_exception("I2C: select_bus failed"));
	}
}

void Module::speed(int speed_in)
{
	this->speed_khz = speed_in;
	this->config.set_int(std::format("i2c.%d.speed", this->module_index), speed_in);
}

int Module::speed()
{
	return(this->speed_khz);
}

std::string Module::info()
{
	std::map<int, Bus *>::const_iterator it;
	std::string out;

	out = std::format("\n- module {:d}, name: \"{}\", device port: {}, speed: {:d} kHz", this->module_index, this->name, this->device_port, this->speed_khz);

	for(it = this->_buses.begin(); it != this->_buses.end(); it++)
		out += it->second->info();

	return(out);
}

int Module::index()
{
	return(this->module_index);
};

std::map<int, Bus *>& Module::buses()
{
	return(this->_buses);
}

bool Module::probe(int bus, int address)
{
	std::lock_guard<std::mutex> mutex_guard(this->mutex);

	this->select_bus(bus);
	return(this->_probe(address));
}

Device* Module::find(int bus, int address)
{
	std::map<int, Bus *>::const_iterator it;

	std::lock_guard<std::mutex> mutex_guard(this->mutex);

	if((it = this->_buses.find(bus)) == this->_buses.end())
		return(nullptr);

	return(it->second->find(address));
}

void Module::send(int bus, int address, const data_t &data)
{
	std::lock_guard<std::mutex> mutex_guard(this->mutex);

	this->select_bus(bus);
	return(this->_send(address, data));
}

void Module::receive(int bus, int address, int length, data_t &data)
{
	std::lock_guard<std::mutex> mutex_guard(this->mutex);

	this->select_bus(bus);
	return(this->_receive(address, length, data));
}

void Module::send_receive(int bus, int address, const data_t &send_buffer, int length, data_t &receive_buffer)
{
	std::lock_guard<std::mutex> mutex_guard(this->mutex);

	this->select_bus(bus);
	return(this->_send_receive(address, send_buffer, length, receive_buffer));
}

bool Module::restricted()
{
	return(this->_restricted());
}

MainModule::MainModule(Log &log_in, Config &config_in, I2c &root_in, int module_index_in, int device_port_in, int sda_in, int scl_in)
	: Module(log_in, config_in, root_in, module_index_in, device_port_in, sda_in, scl_in)
{
	i2c_master_bus_config_t module_config =
	{
		.i2c_port = device_port_in,
		.sda_io_num = static_cast<gpio_num_t>(this->sda),
		.scl_io_num = static_cast<gpio_num_t>(this->scl),
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.intr_priority = 0,
		.trans_queue_depth = 0,
		.flags =
		{
			.enable_internal_pullup = 0,
			.allow_pd = 0,
		},
	};

	i2c_device_config_t device_config =
	{
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = I2C_DEVICE_ADDRESS_NOT_USED,
		.scl_speed_hz = 100 * 1000,
		.scl_wait_us = 0,
		.flags =
		{
			.disable_ack_check = 0,
		}
	};

	esp_err_t rv;
	std::string buffer;
	std::string bus_name;

	try
	{
		this->speed_khz = this->config.get_int(std::format("i2c.{:d}.speed", this->module_index));
	}
	catch(const transient_exception &)
	{
		this->speed_khz = 100;
	}

	device_config.scl_speed_hz = this->speed_khz * 1000;

	if((rv = i2c_new_master_bus(&module_config, &this->module_handle)) != ESP_OK)
		throw(hard_exception(log.esp_string_error(rv, "I2c: i2c_new_master_bus")));

	if((rv = i2c_master_bus_add_device(this->module_handle, &device_config, &this->device_handle)) != ESP_OK)
		throw(hard_exception(log.esp_string_error(rv, "I2c: i2c_master_bus_add_device")));

	this->has_mux = this->probe_mux();
	this->create_buses(this->has_mux ? 9 : 1);
	this->name = std::format("main module {:d}", this->device_port);
}

void MainModule::_set_mux(int bus_bits)
{
	data_t data;

	data.push_back(bus_bits);

	this->_send(bus_mux_address, data);
}

bool MainModule::_probe(int address)
{
	try
	{
		this->_send(address, data_t());
	}
	catch(const transient_exception &)
	{
		return(false);
	}

	return(true);
}

void MainModule::_send(int address, const data_t& out)
{
	esp_err_t rv;
	int current;
	i2c_operation_job_t i2c_operations[3];
	data_t cooked_out;

	cooked_out.push_back((address << address_shift) | write_flag);
	cooked_out += out;

	current = 0;

	i2c_operations[current++].command = I2C_MASTER_CMD_START;

	i2c_operations[current].command = I2C_MASTER_CMD_WRITE;
	i2c_operations[current].write.ack_check = true;
	i2c_operations[current].write.data = cooked_out.data();
	i2c_operations[current++].write.total_bytes = cooked_out.size();

	i2c_operations[current++].command = I2C_MASTER_CMD_STOP;

	if((rv = i2c_master_execute_defined_operations(this->device_handle, i2c_operations, current, 500)) != ESP_OK)
		throw(transient_exception(this->log.esp_string_error(rv, "I2c::MainModule::send: i2c_master_execute_defined_operations")));
}

void MainModule::_receive(int address_in, int length, data_t &in)
{
	esp_err_t rv;
	unsigned char address;
	int current;
	i2c_operation_job_t i2c_operations[5];

	// FIXME For the moment no zero size reads are permitted due to a bug in the ESP-IDF driver code.
	// When that's fixed, this code and the next comment can be removed.

	if(length == 0)
		throw(hard_exception(std::format("I2c::MainModule::receive: address {:#x}: cannot handle zero byte reads", address_in)));

	in.clear();
	in.resize(length);

	address = ((address_in << address_shift) | read_flag);

	current = 0;

	i2c_operations[current++].command = I2C_MASTER_CMD_START;

	i2c_operations[current].command = I2C_MASTER_CMD_WRITE;
	i2c_operations[current].write.ack_check = true;
	i2c_operations[current].write.data = &address;
	i2c_operations[current].write.total_bytes = 1;
	current++;

	if(length > 1)
	{
		i2c_operations[current].command = I2C_MASTER_CMD_READ;
		i2c_operations[current].read.ack_value = I2C_ACK_VAL;
		i2c_operations[current].read.data = in.data();
		i2c_operations[current].read.total_bytes = in.size() - 1;
		current++;
	}

	i2c_operations[current].command = I2C_MASTER_CMD_READ;
	i2c_operations[current].read.ack_value = I2C_NACK_VAL;
	i2c_operations[current].read.data = in.data() + (length - 1);
	i2c_operations[current].read.total_bytes = 1;
	current++;

	i2c_operations[current++].command = I2C_MASTER_CMD_STOP;

	if((rv = i2c_master_execute_defined_operations(this->device_handle, i2c_operations, current, 500)) != ESP_OK)
		throw(transient_exception(this->log.esp_string_error(rv, "I2c::MainModule::receive: i2c_master_defined_operations")));
}

void MainModule::_send_receive(int address, const data_t& out, int length, data_t& in)
{
	esp_err_t rv;
	int current;
	i2c_operation_job_t i2c_operations[7];
	unsigned char read_address;
	data_t cooked_out;

	read_address = ((address << address_shift) | read_flag);
	cooked_out.push_back((address << address_shift) | write_flag);
	cooked_out += out;

	in.resize(length);

	current = 0;

	i2c_operations[current++].command = I2C_MASTER_CMD_START;

	i2c_operations[current].command = I2C_MASTER_CMD_WRITE;
	i2c_operations[current].write.ack_check = true;
	i2c_operations[current].write.data = cooked_out.data();
	i2c_operations[current++].write.total_bytes = cooked_out.size();

	i2c_operations[current++].command = I2C_MASTER_CMD_START;

	i2c_operations[current].command = I2C_MASTER_CMD_WRITE;
	i2c_operations[current].write.ack_check = true;
	i2c_operations[current].write.data = &read_address;
	i2c_operations[current++].write.total_bytes = 1;

	if(length > 0)
	{

		i2c_operations[current].command = I2C_MASTER_CMD_READ;
		i2c_operations[current].read.ack_value = I2C_ACK_VAL;
		i2c_operations[current].read.data = in.data();
		i2c_operations[current++].read.total_bytes = length - 1;
	}

	i2c_operations[current].command = I2C_MASTER_CMD_READ;
	i2c_operations[current].read.ack_value = I2C_NACK_VAL;
	i2c_operations[current].read.data = in.data() + (length - 1);
	i2c_operations[current++].read.total_bytes = 1;

	i2c_operations[current++].command = I2C_MASTER_CMD_STOP;

	if((rv = i2c_master_execute_defined_operations(this->device_handle, i2c_operations, current, 500)) != ESP_OK)
		throw(transient_exception(this->log.esp_string_error(rv, "I2c::MainModule::send_receive: i2c_master_defined_operations")));
}

bool MainModule::_restricted()
{
	return(false);
}

ULPModule::ULPModule(Log &log_in, Config &config_in, I2c &root_in, int module_index_in, int device_port_in, int sda_in, int scl_in)
	: Module(log_in, config_in, root_in, module_index_in, device_port_in, sda_in, scl_in)
{
	ulp_riscv_i2c_cfg_t module_config_slow =
	{
		.i2c_pin_cfg =
		{
			.sda_io_num = 0,
			.scl_io_num = 0,
			.sda_pullup_en = false,
			.scl_pullup_en = false,
		},
		.i2c_timing_cfg =
		{
			.scl_low_period = 5.0f,
			.scl_high_period = 5.0f,
			.sda_duty_period = 2.0f,
			.scl_start_period = 3.0f,
			.scl_stop_period = 6.0f,
			.i2c_trans_timeout = 20,
		},
	};

	ulp_riscv_i2c_cfg_t module_config_fast =
	{
		.i2c_pin_cfg =
		{
			.sda_io_num = 0,
			.scl_io_num = 0,
			.sda_pullup_en = false,
			.scl_pullup_en = false,
		},
		.i2c_timing_cfg =
		{
			.scl_low_period = 1.3f,
			.scl_high_period = 0.5f,
			.sda_duty_period = 1.0f,
			.scl_start_period = 2.0f,
			.scl_stop_period = 1.3f,
			.i2c_trans_timeout = 20,
		},
	};

	esp_err_t rv;
	ulp_riscv_i2c_cfg_t *module_config;
	int speed;
	std::string buffer;
	std::string bus_name;

	try
	{
		speed = this->config.get_int(std::format("i2c.%u.speed", this->module_index));
	}
	catch(const transient_exception &)
	{
		speed = 100;
	}

	if(speed >= 400)
	{
		this->speed_khz = 400;
		module_config = &module_config_fast;
	}
	else
	{
		this->speed_khz = 100;
		module_config = &module_config_slow;
	}

	this->module_handle = nullptr;

	module_config->i2c_pin_cfg.sda_io_num = sda;
	module_config->i2c_pin_cfg.scl_io_num = scl;

	if((rv = ulp_riscv_i2c_master_init(module_config)) != ESP_OK)
		throw(hard_exception(log.esp_string_error(rv, "I2c::ULPModule: ulp_riscv_i2c_master_init")));

	this->has_mux = this->probe_mux();
	this->create_buses(this->has_mux ? 9 : 1);
	this->name = std::format("ULP module {:d}", this->device_port);
}

void ULPModule::_set_mux(int bus_bits)
{
	data_t data;

	data.push_back(bus_bits);
	data.push_back(bus_bits);

	this->_send(bus_mux_address, data);
}

bool ULPModule::_probe(int address)
{
	data_t buffer;

	try
	{
		this->_send_receive(address, data_t(1, 0x00), 1, buffer);
	}
	catch(const transient_exception &)
	{
		return(false);
	}

	return(true);
}

void ULPModule::_send(int address, const data_t& data)
{
	esp_err_t rv;
	unsigned char dummy[1];

	if(data.size() == 0)
		throw(hard_exception(std::format("I2c::ULPModule::send: address {:#x}: cannot send 0 bytes using ULP I2c", address)));

	ulp_riscv_i2c_master_set_slave_addr(address);
	ulp_riscv_i2c_master_set_slave_reg_addr(data.at(0));

	if(data.size() > 1)
	{
		if((rv = ulp_riscv_i2c_master_write_to_device(data.data() + 1, data.size() - 1)) != ESP_OK)
			throw(transient_exception(this->log.esp_string_error(rv, "I2c::ULPModule::send: ulp_riscv_i2c_master_write_to_device")));
	}
	else
	{
		// workaround for writing only one byte, which the ULP I2c can't do
		// instead write one byte, then read one (dummy) byte, which it can do

		this->log << std::format("I2c::ULPModule::send: address: {:#x}, emulating one byte write by a write/read cycle", address);

		if((rv = ulp_riscv_i2c_master_read_from_device(dummy, sizeof(dummy))) != ESP_OK)
			throw(transient_exception(this->log.esp_string_error(rv, "I2c::ULPModule::send: ulp_riscv_i2c_master_write_to_device")));
	}
}

void ULPModule::_receive(int address, int, data_t&)
{
	throw(hard_exception(std::format("I2c::receive: address {:#x}: reading without writing unsupported on ULP module", address)));
}

void ULPModule::_send_receive(int address, const data_t &send_buffer, int length, data_t &receive_buffer)
{
	esp_err_t rv;

	if(send_buffer.size() != 1)
		throw(hard_exception(std::format("I2c::ULPModule::send_receive: address {:#x}: can only send one byte in write/read transaction", address)));

	ulp_riscv_i2c_master_set_slave_addr(address);
	ulp_riscv_i2c_master_set_slave_reg_addr(send_buffer.at(0));

	receive_buffer.resize(length);

	if((rv = ulp_riscv_i2c_master_read_from_device(receive_buffer.data(), receive_buffer.size())) != ESP_OK)
		throw(transient_exception(this->log.esp_string_error(rv, "I2c::ULPModule::send_receive: ulp_riscv_i2c_master_write_to_device")));
}

bool ULPModule::_restricted()
{
	return(true);
}

Bus::Bus(Log &log_in, Module &module_in, int bus_index_in, std::string_view bus_name) :
		log(log_in), module_ref(module_in), bus_index(bus_index_in), name(bus_name)
{
}

Module& Bus::module()
{
	return(this->module_ref);
}

int Bus::index()
{
	return(this->bus_index);
}

Device* Bus::new_device(int address, std::string_view device_name)
{
	Device *device;

	if(this->devices.find(address) != this->devices.end())
		throw(transient_exception("I2c::module::new_device: address in use"));

	device = new Device(this->log, *this, address, device_name);

	this->devices[address] = device;

	return(device);
}

Device* Bus::find(int address)
{
	std::map<int, Device *>::const_iterator it;

	if((it = this->devices.find(address)) == this->devices.end())
		return(nullptr);

	return(it->second);
}

std::string Bus::info()
{
	std::map<int, Device *>::const_iterator it;
	std::string out;

	out = std::format("\n-  bus {:d}, name: {}", this->bus_index, this->name);

	for(it = this->devices.begin(); it != this->devices.end(); it++)
		out += it->second->info();

	return(out);
}

void Bus::delete_device(Device *device)
{
	std::map<int, Device *>::iterator it;

	if(!device)
		throw(hard_exception("I2c::Bus::delete_device: invalid argument"));

	for(it = this->devices.begin(); it != this->devices.end(); it++)
		if(it->second == device)
			this->devices.erase(it->first);
}

void Bus::send(int address, const data_t &data)
{
	this->module_ref.send(this->bus_index, address, data);
}

void Bus::receive(int address, int length, data_t &data)
{
	this->module_ref.receive(this->bus_index, address, length, data);
}

void Bus::send_receive(int address, const data_t &out, int length, data_t &in)
{
	this->module_ref.send_receive(this->bus_index, address, out, length, in);
}

Device::Device(Log &log_in, Bus &bus_in, int address_in, std::string_view name_in) : 
		log(log_in), bus_ref(bus_in), address(address_in), name(name_in)
{
}
 
std::string Device::info()
{
	return(std::format("\n-   device: address {:#04x}, name: \"{}\"", this->address, this->name));
}

void Device::data(int &module_in, int &bus_in, int &address_in, std::string &name_in)
{
	address_in = this->address;
	bus_in = this->bus_ref.index();
	module_in = this->bus_ref.module().index();
	name_in = this->name;
}

std::string Device::data()
{
	return(std::format("{}@{:d}/{:d}/{:02x}", this->name, this->bus_ref.module().index(), this->bus_ref.index(), this->address));
}

void Device::send(const data_t &data)
{
	this->bus_ref.send(this->address, data);
}

void Device::send(unsigned char b1)
{
	data_t data;

	data.push_back(b1);
	this->bus_ref.send(this->address, data);
}

void Device::send(unsigned char b1, unsigned char b2)
{
	data_t data;

	data.push_back(b1);
	data.push_back(b2);
	this->bus_ref.send(this->address, data);
}

void Device::send(unsigned char b1, unsigned char b2, unsigned char b3)
{
	data_t data;

	data.push_back(b1);
	data.push_back(b2);
	data.push_back(b3);
	this->bus_ref.send(this->address, data);
}

void Device::receive(int length, data_t &data)
{
	this->bus_ref.receive(this->address, length, data);
}

void Device::send_receive(const data_t &out, int length, data_t &in)
{
	this->bus_ref.send_receive(this->address, out, length, in);
}

void Device::send_receive(unsigned char b1, int length, data_t &in)
{
	data_t data;

	data.push_back(b1);
	this->bus_ref.send_receive(this->address, data, length, in);
}

void Device::send_receive(unsigned char b1, unsigned char b2, int length, data_t &in)
{
	data_t data;

	data.push_back(b1);
	data.push_back(b2);
	this->bus_ref.send_receive(this->address, data, length, in);
}

void Device::send_receive(unsigned char b1, unsigned char b2, unsigned char b3, int length, data_t &in)
{
	data_t data;

	data.push_back(b1);
	data.push_back(b2);
	data.push_back(b3);
	this->bus_ref.send_receive(this->address, data, length, in);
}

Device::~Device()
{
	this->bus_ref.delete_device(this);
}
