#include "sensor.h"

#include "log.h"
#include "i2c.h"
#include "exception.h"
#include "cli-command.h"
#include "crypt.h"

#include <stdint.h>
#include <time.h>
#include <math.h>
#include <assert.h>

#include <string>
#include <boost/format.hpp>
#include <format>
#include <thread>
#include <chrono>

#include "magic_enum/magic_enum.hpp"

#include <esp_pthread.h>

using namespace magic_enum;
using namespace magic_enum::bitwise_operators;
using namespace SENSORS;

Sensors *Sensors::singleton = nullptr;

Sensors::Sensors(Log &log_in, I2c &i2c_in) : log(log_in), i2c(i2c_in)
{
	if(this->singleton)
		throw(hard_exception("Sensors: already active"));

	this->singleton = this;
}

template<typename T> void Sensors::detect_internal(int dummy_module)
{
	Sensor* new_sensor;

	this->_stats["internal sensors probed"]++;
	new_sensor = new T(this->log, dummy_module);
	this->_stats["internal sensors found"]++;
	this->_sensors.push_back(new_sensor);
}

template<typename T> void Sensors::detect_i2c()
{
	I2C::Device* new_device;
	Sensor* new_sensor;

	for(auto& module : this->i2c.modules())
	{
		for(auto& bus : module.second->buses())
		{
			for(auto const& address : T::addresses)
			{
				this->_stats["I2C sensors searched"]++;

				if(T::probe(this->i2c, module.first, bus.first, address, module.second->restricted()))
				{
					new_device = nullptr;
					new_sensor = nullptr;

					this->_stats["I2C sensors probed"]++;

					try
					{
						try
						{
							new_device = i2c.new_device(module.first, bus.first, address, T::basic_name());
						}
						catch(const transient_exception &)
						{
							this->_stats["I2C sensors address in use"]++;
							throw(transient_exception());
						}

						this->_stats["I2C sensors identified"]++;
						new_sensor = new T(this->log, new_device);
						this->_stats["I2C sensors found"]++;
						this->_sensors.push_back(new_sensor);
					}
					catch(const transient_exception &)
					{
						this->_stats["I2C sensors init aborted"]++;

						if(new_device)
							delete new_device;

						if(new_sensor)
							delete new_sensor;
					}
				}
			}
		}
	}
}

void Sensors::run_thread()
{
	try
	{
		this->detect_internal<SensorInternalTemperature>(this->i2c.modules().size());
		this->detect_i2c<SensorGeneric75>();
		this->detect_i2c<SensorMCP9808>();
		this->detect_i2c<SensorHTU21>();
		this->detect_i2c<SensorAsair>();
		this->detect_i2c<SensorSHT3X>();
		this->detect_i2c<SensorSHT4X>();
		this->detect_i2c<SensorHDC1080>();
		this->detect_i2c<SensorBMP280>();
		this->detect_i2c<SensorBME280>();
		this->detect_i2c<SensorBME680>();
		this->detect_i2c<SensorBH1750>();
		this->detect_i2c<SensorOPT3001>();
		this->detect_i2c<SensorMAX44009>();
		this->detect_i2c<SensorTSL2561>();
		this->detect_i2c<SensorTSL2591RO>();
		this->detect_i2c<SensorTSL2591RW>();
		this->detect_i2c<SensorVEML7700>();
		this->detect_i2c<SensorLTR390>();

		if(this->_sensors.size() != 0)
		{
			for(;;)
			{
				uint64_t begin, end;
				begin = esp_timer_get_time();

				this->_stats["poll loop runs"]++;

				for(auto& sensor : this->_sensors)
				{
					bool ok = true;

					try
					{
						sensor->poll();
					}
					catch(const transient_exception &e)
					{
						this->log << std::format("Sensors::poll:{} {}", sensor->name, e.what());
						ok = false;
					}

					if(ok)
						this->_stats["poll succeeds"]++;
					else
						this->_stats["poll errors"]++;
				}

				end = esp_timer_get_time();

				this->_stats["poll time"] = (end - begin) / 1000;
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
			}
		}
	}
	catch(const transient_exception &e)
	{
		Log::get().abort(std::format("sensor thread: uncaught transient exception: {}", e.what()));
	}
	catch(const hard_exception &e)
	{
		Log::get().abort(std::format("sensor thread: uncaught hard exception: {}", e.what()));
	}
	catch(const std::exception &e)
	{
		Log::get().abort(std::format("sensor thread: uncaught standard exception: {}", e.what()));
	}
	catch(...)
	{
		Log::get().abort("sensor thread: uncaught unknown exception");
	}
}

void Sensors::run()
{
	esp_err_t rv;
	esp_pthread_cfg_t thread_config = esp_pthread_get_default_config();

	thread_config.thread_name = "sensors";
	thread_config.pin_to_core = 1;
	thread_config.stack_size = 3 * 1024;
	thread_config.prio = 1;
	thread_config.stack_alloc_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

	if((rv = esp_pthread_set_cfg(&thread_config)) != ESP_OK)
		throw(hard_exception(Log::get().esp_string_error(rv, "esp_pthread_set_cfg")));

	std::thread new_thread([this]() { this->run_thread(); });

	new_thread.detach();
}

std::string Sensors::info(int filter)
{
	int index = 0;
	std::string out;

	for(auto& sensor : this->_sensors)
	{
		if((filter < 0) || (filter == index))
			out += std::format("- {}\n", sensor->info(index));

		index++;
	}

	return(out);
}

std::string Sensors::dump(int filter)
{
	int index = 0;
	std::string out;

	for(auto& sensor : this->_sensors)
	{
		if((filter < 0) || (filter == index))
			out += std::format("- {}\n", sensor->dump(index));

		index++;
	}

	return(out);
}

std::string Sensors::json()
{
	int index;
	std::string out;

	out = "{\n";

	index = 0;

	for(auto& sensor : this->_sensors)
	{
		if(index != 0)
			out += ",\n";

		out += sensor->json(index++);
	}

	out += "\n}";

	return(out);
}

std::string Sensors::stats()
{
	std::string out;

	for(const auto& stat : this->_stats)
		out += std::format("- {:<24} {:d}\n", std::string(stat.first) + ": ", stat.second);

	return(out);
}

Sensor::Sensor(Log& log_in, std::string_view name_in) : log(log_in), name(name_in), state(state_t::init)
{
}

void Sensor::poll()
{
	this->_poll();
}

std::string Sensor::info(int index)
{
	std::string out;
	int module, bus, address;

	this->_address(module, bus, address);

	out = std::format("{:d}:{}@{:d}/{:d}/{:x}", index, this->name, module, bus, address);

	for(auto const& value : this->values)
		out += std::format(", {0}: {1:.{3}f} {2}", value.first, value.second.value, value.second.unity, value.second.precision);

	return(out);
}

std::string Sensor::dump(int index)
{
	std::string out;
	int module, bus, address;
	int value_index;

	this->_address(module, bus, address);

	out = std::format("{:d}:{}@{:d}/{:d}/{:x}", index, this->name, module, bus, address);

	for(auto const& value : this->values)
		out += std::format(", {} {:f} {}, last update: {}", value.first, value.second.value, value.second.unity, this->time_string(value.second.stamp));

	out += std::format(", state: {} [{:d}]", enum_name(this->state), enum_integer(this->state));

	if(this->raw_values.size() > 0)
	{
		out += ", raw values: ";

		value_index = 0;

		for(auto const& raw_value : this->raw_values)
		{
			if(value_index > 0)
				out += ", ";

			out += std::format("{}: {:d}/{:#04x}", raw_value.first, raw_value.second, raw_value.second);

			value_index++;
		}
	}

	return(out);
}

std::string Sensor::json(int index)
{
	int module, bus, address, subaddress;
	std::string out;

	this->_address(module, bus, address);

	out = std::format("\"{:d}-{:d}-{:x}\":", module, bus, address);
	out +=	"\n[";
	out +=	"\n{";
	out += std::format("\n\"module\": {:d},", module);
	out += std::format("\n\"bus\": {:d},", bus);
	out += std::format("\n\"name\": \"{}\",", this->name);
	out +=	"\n\"values\":";
	out +=	"\n[";

	subaddress = 0; // sensor sub-adress / value

	for(auto const& value : this->values)
	{
		if(subaddress > 0)
			out += ",";

		out +=	"\n{";
		out += std::format("\n\"type\": \"{}\",", value.first);
		out += std::format("\n\"id\": {:d},", index);
		out += std::format("\n\"address\": {:d},", subaddress);
		out += std::format("\n\"unity\": \"{}\",", value.second.unity);
		out += std::format("\n\"value\": {:f},", value.second.value);
		out += std::format("\n\"time\": {:d}", value.second.stamp);
		out +=	"\n}";

		subaddress++;
	}

	out += "\n]";
	out += "\n}";
	out += "\n]";

	return(out);
}

unsigned char Sensor::u8(const I2C::data_t &data, int offset)
{
	return(static_cast<unsigned char>(data.at(offset) & 0xff));
}

signed char Sensor::s8(const I2C::data_t &data, int offset)
{
	int rv = static_cast<unsigned int>(data.at(offset) & 0xff);

	if(rv > (1 << 7))
		rv = 0 - ((1 << 8) - rv);

	return(rv);
}

unsigned int Sensor::u12topbe(const I2C::data_t &data, int offset)
{
	return((((data.at(offset + 0) & 0xff) >> 0) << 4) | (((data.at(offset + 1) & 0xf0) >> 4) << 0));
}

unsigned int Sensor::u12bottomle(const I2C::data_t &data, int offset)
{
	return((((data.at(offset + 0) & 0x0f) >> 0) << 0) | (((data.at(offset + 1) & 0xff) >> 0) << 4));
}

unsigned int Sensor::u16be(const I2C::data_t &data, int offset)
{
	return(((data.at(offset + 0) & 0xff) << 8) | ((data.at(offset + 1) & 0xff) << 0));
}

unsigned int Sensor::u16le(const I2C::data_t &data, int offset)
{
	return(((data.at(offset + 0) & 0xff) << 0) | ((data.at(offset + 1) & 0xff) << 8));
}

signed int Sensor::s16le(const I2C::data_t &data, int offset)
{
	int rv = ((data.at(offset + 1) & 0xff) << 8) | (data.at(offset + 0) & 0xff);

	if(rv > (1 << 15))
		rv = 0 - ((1 << 16) - rv);

	return(rv);
}

unsigned int Sensor::u20tople(const I2C::data_t &data, int offset)
{
	return((((data.at(offset + 0) & 0xff) >> 0) << 0) | (((data.at(offset + 1) & 0xff) >> 0) << 8) | (((data.at(offset + 2) & 0x0f) >> 0) << 16));
}

unsigned int Sensor::u20topbe(const I2C::data_t &data, int offset)
{
	return((((data.at(offset + 0) & 0xff) >> 0) << 12) | (((data.at(offset + 1) & 0xff) >> 0) << 4) | (((data.at(offset + 2) & 0xf0) >> 4) << 0));
}

unsigned int Sensor::u20bottombe(const I2C::data_t &data, int offset)
{
	return((((data.at(offset + 0) & 0x0f) >> 0) << 16) | (((data.at(offset + 1) & 0xff) >> 0) << 8) | (((data.at(offset + 2) & 0xff) >> 0) << 0));
}

unsigned char Sensor::u16low(int u16)
{
	return((u16 & 0x00ff) >> 0);
}

unsigned char Sensor::u16high(int u16)
{
	return((u16 & 0xff00) >> 8);
}

void Sensor::update(const std::string &id, float value_in, const std::string &unity, int precision)
{
	value_t value;

	value.stamp = time(nullptr);
	value.value = value_in;
	value.unity = unity;
	value.precision = precision;

	this->values.insert_or_assign(id, value);
}

std::string Sensor::time_string(time_t stamp)
{
	time_t now;
	std::string rv;

	now = time(nullptr);

	if(((now - stamp) > (24 * 60 * 60)) || ((now - stamp) < 0))
			rv = "never";
		else
			rv = std::format("{:d} seconds ago", now - stamp);

	return(rv);
}

Sensor::~Sensor()
{
}

SensorI2C::SensorI2C(Log& log_in, std::string_view name_in, I2C::Device *device_in) : Sensor(log_in, name_in), device(device_in)
{
}

SensorI2C::~SensorI2C()
{
}

void SensorI2C::_address(int &module, int &bus, int &address)
{
	std::string device_name;

	this->device->data(module, bus, address, device_name);
}

bool SensorGeneric75::probe(I2c &i2c, int module, int bus, int address, bool)
{
	return(i2c.probe(module, bus, address));
}

bool SensorGeneric75::probe_register(int reg)
{
	I2C::data_t data;

	try
	{
		this->device->send_receive(reg, 1, data);
	}
	catch(const transient_exception &)
	{
		return(false);
	}

	return(true);
}

SensorGeneric75::SensorGeneric75(Log &log_in, I2C::Device *device_in) : SensorI2C(log_in, this->basic_name(), device_in)
{
	I2C::data_t data;
	unsigned int config_value;
	static const constexpr unsigned int config_value_tmp75 = enum_integer(tmp75_conf_t::res_12 | tmp75_conf_t::no_shutdown);
	static const constexpr unsigned int config_value_lm75 = enum_integer(lm75_conf_t::no_shutdown);

	this->model = model_t::generic75;

	if(this->model == model_t::generic75)
	{
		try // tmp75
		{
			this->device->send_receive(enum_integer(tmp75_reg_t::conf), 1, data);

			if((data.at(0) & enum_integer(tmp75_probe_t::conf_mask)) != enum_integer(tmp75_probe_t::conf))
				throw(transient_exception("1"));

			this->device->send_receive(enum_integer(tmp75_reg_t::tlow), 2, data);

			if((data.at(0) != enum_integer(tmp75_probe_t::tl_h)) || (data.at(1) != enum_integer(tmp75_probe_t::tl_l)))
				throw(transient_exception("2"));

			this->device->send_receive(enum_integer(tmp75_reg_t::thigh), 2, data);

			if((data.at(0) != enum_integer(tmp75_probe_t::th_h)) || (data.at(1) != enum_integer(tmp75_probe_t::th_l)))
				throw(transient_exception("3"));

			if(this->probe_register(enum_integer(tmp75_probe_t::offset_04)))
				throw(transient_exception("4"));
			if(this->probe_register(enum_integer(tmp75_probe_t::offset_a1)))
				throw(transient_exception("5"));
			if(this->probe_register(enum_integer(tmp75_probe_t::offset_a2)))
				throw(transient_exception("6"));
			if(this->probe_register(enum_integer(tmp75_probe_t::offset_aa)))
				throw(transient_exception("7"));
			if(this->probe_register(enum_integer(tmp75_probe_t::offset_ac)))
				throw(transient_exception("8"));

			this->model = model_t::tmp75;
		}
		catch(const transient_exception &)
		{
		}
	}

	if(this->model == model_t::generic75)
	{
		try // lm75(b)
		{
			this->device->send_receive(enum_integer(lm75_reg_t::conf), 1, data);

			if((data.at(0) & enum_integer(lm75_probe_t::conf_mask)) != enum_integer(lm75_probe_t::conf))
				throw(transient_exception("1"));

			this->device->send_receive(enum_integer(lm75_reg_t::thyst), 2, data);

			if((data.at(0) != enum_integer(lm75_probe_t::thyst_h)) || (data.at(1) != enum_integer(lm75_probe_t::thyst_l)))
				throw(transient_exception("2"));

			this->device->send_receive(enum_integer(lm75_reg_t::tos), 2, data);

			if(((data.at(0) != enum_integer(lm75_probe_t::tos_1_h)) || (data.at(1) != enum_integer(lm75_probe_t::tos_1_l))) &&
				((data.at(0) != enum_integer(lm75_probe_t::tos_2_h)) || (data.at(1) != enum_integer(lm75_probe_t::tos_2_l))))
				throw(transient_exception("3"));

			this->model = model_t::lm75;
		}
		catch(const transient_exception &)
		{
		}
	}

	switch(this->model)
	{
		case(model_t::tmp75):
		{
			config_value = config_value_tmp75;

			break;
		}

		case(model_t::lm75):
		{
			config_value = config_value_lm75;

			break;
		}

		default:
		{
			throw(transient_exception("unknown generic lm75 type"));
		}
	}

	this->name = enum_name(this->model);

	this->device->send(enum_integer(reg_t::conf), config_value);
	this->device->send_receive(enum_integer(reg_t::conf), 1, data);

	if(data.at(0) != config_value)
		throw(transient_exception("invalid config register contents"));
}

void SensorGeneric75::_poll()
{
	I2C::data_t data;
	unsigned int temperature;

	this->device->send_receive(enum_integer(reg_t::temp), 2, data);

	this->raw_values["byte 0"] = data.at(0);
	this->raw_values["byte 1"] = data.at(1);

	temperature = this->u16be(data);

	this->raw_values["raw temperature"] = temperature;
	this->update("temperature", static_cast<float>(temperature) / 256.0f, "degrees", 1);
	this->state = state_t::measuring;
}

bool SensorMCP9808::probe(I2c &i2c, int module, int bus, int address, bool)
{
	return(i2c.probe(module, bus, address));
}

SensorMCP9808::SensorMCP9808(Log &log_in, I2C::Device *device_in) : SensorI2C(log_in, this->basic_name(), device_in)
{
	I2C::data_t data;

	this->device->send_receive(enum_integer(reg_t::manufacturer), 2, data);

	if((data.at(0) != enum_integer(id_t::manufacturer_0)) || data.at(1) != enum_integer(id_t::manufacturer_1))
		throw(transient_exception());

	this->device->send_receive(enum_integer(reg_t::device), 1, data);

	if(data.at(0) != enum_integer(id_t::device))
		throw(transient_exception());

	this->device->send(enum_integer(reg_t::config), enum_integer(config_t::int_clear));
	this->device->send(enum_integer(reg_t::resolution), enum_integer(resolution_t::res_0_0625));
}

void SensorMCP9808::_poll()
{
	I2C::data_t data;

	switch(this->state)
	{
		case(state_t::init):
		{
			this->state = state_t::measuring;
			break;
		}

		case(state_t::measuring):
		{
			unsigned int raw;
			float temperature;

			this->device->send_receive(enum_integer(reg_t::temperature), 2, data);
			raw = this->u16be(data);

			this->raw_values["byte 0"] = data.at(0);
			this->raw_values["byte 1"] = data.at(1);
			this->raw_values["raw temperature"] = raw;

			temperature = static_cast<float>(raw & 0x0fff) / 16.0f;

			if(raw & (1 << 12))
				temperature = 256 - temperature;

			this->update("temperature", temperature, "degrees", 1);
			this->state = state_t::finished;

			break;
		}

		case(state_t::finished):
		{
			this->state = state_t::measuring;
			break;
		}

		default:
		{
			throw(hard_exception("invalid state"));
		}
	}
}

unsigned int SensorHTU21::get_data()
{
	I2C::data_t data;
	unsigned char crc1, crc2;

	device->receive(4, data);

	crc1 = data.at(2);
	crc2 = Crypt::crc8_31(data.substr(0, 2), 0x00);

	if(crc1 != crc2)
	{
		this->raw_values["crc errors"]++;
		throw(transient_exception());
	}

	return(this->u16be(data) & ~(enum_integer(status_t::mask)));
}

bool SensorHTU21::probe(I2c &i2c, int module, int bus, int address, bool restricted)
{
	if(restricted)
		return(false);

	return(i2c.probe(module, bus, address));
}

SensorHTU21::SensorHTU21(Log &log_in, I2C::Device *device_in) : SensorI2C(log_in, this->basic_name(), device_in)
{
	I2C::data_t data;

	try
	{
		device->send_receive(enum_integer(cmd_t::read_user), 1, data);
	}
	catch(const transient_exception &)
	{
		throw(transient_exception());
	}
}

void SensorHTU21::_poll()
{
	unsigned int result;
	I2C::data_t data;

	switch(this->state)
	{
		case(state_t::init):
		{
			device->send(enum_integer(cmd_t::reset));

			this->state = state_t::reset;

			break;
		}

		case(state_t::reset):
		{
			unsigned char cmd;

			device->send_receive(enum_integer(cmd_t::read_user), 1, data);

			cmd = data.at(0);
			cmd &= enum_integer(reg_t::reserved | reg_t::bat_stat);
			cmd |= enum_integer(reg_t::rh11_temp11 | reg_t::otp_reload_disable);

			device->send(enum_integer(cmd_t::write_user), cmd);
			device->send_receive(enum_integer(cmd_t::read_user), 1, data);

			cmd = data.at(0);
			cmd &= ~(enum_integer(reg_t::reserved | reg_t::bat_stat));

			if(cmd != enum_integer(reg_t::rh11_temp11 | reg_t::otp_reload_disable))
				throw(transient_exception("htu21: invalid status"));

			this->state = state_t::ready;

			break;
		}

		case(state_t::ready):
		case(state_t::finished):
		{
			device->send(enum_integer(cmd_t::meas_temp_no_hold_master));
			this->state = state_t::measuring_temperature;

			break;
		}

		case(state_t::measuring_temperature):
		{
			try
			{
				result = this->get_data();
			}
			catch(const transient_exception &e)
			{
				throw(transient_exception(std::format("htu21: poll error temperature: {}", e.what())));
			}

			this->raw_values["raw temperature"] = result;
			this->state = state_t::finished_temperature;

			break;
		}

		case(state_t::finished_temperature):
		{
			device->send(enum_integer(cmd_t::meas_hum_no_hold_master));
			this->state = state_t::measuring_humidity;

			break;
		}

		case(state_t::measuring_humidity):
		{
			try
			{
				result = this->get_data();
			}
			catch(const transient_exception &e)
			{
				throw(transient_exception(std::format("htu21: poll error hummidity: {}", e.what())));
			}

			this->raw_values["raw humidity"] = result;
			this->state = state_t::finished_humidity;

			break;
		}

		case(state_t::finished_humidity):
		{
			float temperature, humidity;

			temperature = ((this->raw_values["raw temperature"] * 175.72f) / 65536.0f) - 46.85f;
			humidity = (((this->raw_values["raw humidity"] * 125.0f) / 65536.0f) - 6.0f) + ((25.0f - temperature) * -0.10f); // TempCoeff guessed

			if(humidity < 0)
				humidity = 0;

			if(humidity > 100)
				humidity = 100;

			this->update("temperature", temperature, "degrees", 1);
			this->update("humidity", humidity, "%", 0);
			this->state = state_t::finished;

			break;
		}

		default:
		{
			throw(hard_exception("htu21: invalid state"));
		}
	}
}

bool SensorAsair::probe(I2c &i2c, int module, int bus, int address, bool restricted)
{
	return(i2c.probe(module, bus, address));
}

SensorAsair::SensorAsair(Log &log_in, I2C::Device *device_in) : SensorI2C(log_in, this->basic_name(), device_in), model(model_t::asair)
{
	this->device->send(enum_integer(cmd_t::reset));
	this->state = state_t::reset;
}

void SensorAsair::_poll()
{
	I2C::data_t data;

	switch(this->state)
	{
		case(state_t::reset):
		{
			this->state = state_t::ready;

			if(this->model != model_t::asair)
			{
				this->state = state_t::ready;
				break;
			}

			try
			{
				this->device->send(enum_integer(aht10_cal_init_t::cal_1), enum_integer(aht10_cal_init_t::cal_2), enum_integer(aht10_cal_init_t::cal_3));
				this->model = model_t::aht10;
				break;
			}
			catch(const transient_exception &)
			{
			}

			try
			{
				this->device->send(enum_integer(aht20_cal_init_t::cal_1), enum_integer(aht20_cal_init_t::cal_2), enum_integer(aht20_cal_init_t::cal_3));
				this->model = model_t::aht20;
				break;
			}
			catch(const transient_exception &)
			{
			}

			throw(transient_exception("asair: unknown sensor type"));

			this->name = enum_name(this->model);

			break;
		}

		case(state_t::ready):
		{
			this->device->send_receive(enum_integer(cmd_t::get_all_data), enum_integer(data_off_t::state) + 1, data);

			if(data.at(enum_integer(data_off_t::state)) & enum_integer(status_t::busy))
			{
				this->raw_values["busy1"]++;
				break;
			}

			this->state = state_t::measuring;

			break;
		}

		case(state_t::measuring):
		{
			this->device->send(enum_integer(cmd_t::measure_0), enum_integer(cmd_t::measure_1), enum_integer(cmd_t::measure_2));

			this->state = state_t::finished;

			break;
		}

		case(state_t::finished):
		{
			unsigned temperature, humidity;
			unsigned int crc_local, crc_remote;

			this->device->send_receive(enum_integer(cmd_t::get_all_data), enum_integer(data_off_t::size), data);

			if(data.at(enum_integer(data_off_t::state)) & enum_integer(status_t::busy))
			{
				this->raw_values["busy2"]++;
				break;
			}

			this->raw_values["raw 0"] = data.at(0);
			this->raw_values["raw 1"] = data.at(1);
			this->raw_values["raw 2"] = data.at(2);
			this->raw_values["raw 3"] = data.at(3);
			this->raw_values["raw 4"] = data.at(4);
			this->raw_values["raw 5"] = data.at(5);
			this->raw_values["raw 6"] = data.at(6);

			crc_local = Crypt::crc8_31(data.substr(0, enum_integer(data_off_t::crc)), 0xff);
			crc_remote = data.at(enum_integer(data_off_t::crc));

			if(crc_local != crc_remote)
				this->raw_values["crc errors"]++;

			temperature = this->u20bottombe(data, enum_integer(data_off_t::temperature_0));
			humidity = this->u20topbe(data, enum_integer(data_off_t::humidity_0));

			this->raw_values["raw temperature"] = temperature;
			this->raw_values["raw humidity"] = humidity;

			this->update("temperature", ((static_cast<float>(temperature) / 1048576.0f) * 200) - 50.0f, "degrees", 1);
			this->update("humidity", (static_cast<float>(humidity) / 1048576.0f) * 100, "%", 0);
			this->state = state_t::ready;

			break;
		}

		default:
		{
			throw(hard_exception("asair: invalid state"));
		}
	}
}

void SensorSHT3X::send_command(cmd_t cmd)
{
	this->device->send(u16high(enum_integer(cmd)), u16low(enum_integer(cmd)));
}

unsigned int SensorSHT3X::receive_command(cmd_t cmd)
{
	unsigned char crc_local, crc_remote;
	I2C::data_t data;

	this->device->send_receive(u16high(enum_integer(cmd)), u16low(enum_integer(cmd)), 3, data);

	crc_local = Crypt::crc8_31(data.substr(0, 2), 0xff);
	crc_remote = data.at(2);

	if(crc_local != crc_remote)
	{
		this->raw_values["crc errors"]++;
		throw(transient_exception());
	}

	return(this->u16be(data));
}

bool SensorSHT3X::probe(I2c &i2c, int module, int bus, int address, bool restricted)
{
	if(restricted)
		return(false);

	return(i2c.probe(module, bus, address));
}

SensorSHT3X::SensorSHT3X(Log &log_in, I2C::Device *device_in) : SensorI2C(log_in, this->basic_name(), device_in)
{
	try
	{
		this->send_command(cmd_t::cmd_break);
	}
	catch(const transient_exception &e)
	{
		throw(transient_exception());
	}

	this->raw_values["crc errors"] = 0;
}

void SensorSHT3X::_poll()
{
	unsigned int result;

	switch(this->state)
	{
		case(state_t::init):
		{
			this->send_command(cmd_t::reset);
			this->state = state_t::reset;

			break;
		}

		case(state_t::reset):
		{
			result = this->receive_command(cmd_t::read_status);

			if((result & enum_integer(status_t::write_checksum | status_t::command_status)) != 0x00)
			{
				this->raw_values["poll errors"]++;
				throw(transient_exception());
			}

			this->send_command(cmd_t::clear_status);
			this->state = state_t::ready;

			break;
		}

		case(state_t::ready):
		{
			result = this->receive_command(cmd_t::read_status);

			if((result & enum_integer(status_t::write_checksum | status_t::command_status | status_t::reset_detected)) != 0x00)
			{
				this->raw_values["poll errors"]++;
				throw(transient_exception());
			}

			this->state = state_t::finished;

			break;
		}

		case(state_t::finished):
		{
			this->send_command(cmd_t::single_meas_noclock_high);
			this->state = state_t::measuring;

			break;
		}

		case(state_t::measuring):
		{
			I2C::data_t data;
			unsigned int raw_temperature, raw_humidity;
			float temperature, humidity;
			unsigned char crc_local, crc_remote;

			this->device->send_receive(this->u16high(enum_integer(cmd_t::fetch_data)), this->u16low(enum_integer(cmd_t::fetch_data)), 6, data);

			crc_remote = data.at(2);
			crc_local = Crypt::crc8_31(data.substr(0, 2), 0xff);

			if(crc_local != crc_remote)
			{
				this->raw_values["crc errors"]++;
				throw(transient_exception());
			}

			crc_remote = data.at(5);
			crc_local = Crypt::crc8_31(data.substr(3, 2), 0xff);

			if(crc_local != crc_remote)
			{
				this->raw_values["crc errors"]++;
				throw(transient_exception());
			}

			this->raw_values["raw temperature"] = raw_temperature = this->u16be(data, 0);
			this->raw_values["raw humidity"] = raw_humidity = this->u16be(data, 3);

			temperature = ((static_cast<float>(raw_temperature) * 175.f) / ((1 << 16) - 1.0f)) - 45.0f;
			humidity = (static_cast<float>(raw_humidity) * 100.0f) / ((1 << 16) - 1.0f);

			this->update("temperature", temperature, "degrees", 1);
			this->update("humidity", humidity, "%", 0);
			this->state = state_t::finished;

			break;
		}

		default:
		{
			throw(hard_exception("sht3x: invalid state"));
		}
	}
}

bool SensorSHT4X::probe(I2c &i2c, int module, int bus, int address, bool restricted)
{
	if(restricted)
		return(false);

	return(i2c.probe(module, bus, address));
}

SensorSHT4X::SensorSHT4X(Log &log_in, I2C::Device *device_in) : SensorI2C(log_in, this->basic_name(), device_in)
{
	I2C::data_t data;
	unsigned int serial, serial_high, serial_low, local_crc_high, local_crc_low, remote_crc_high, remote_crc_low;

	try
	{
		this->device->send(enum_integer(cmd_t::read_serial));
		this->device->receive(6, data);
		serial_high = this->u16be(data, 0);
		serial_low = this->u16be(data, 3);
		serial = (serial_high << 16) | (serial_low << 0);
		remote_crc_high = data.at(2);
		remote_crc_low = data.at(5);
		local_crc_high = Crypt::crc8_31(data.substr(0, 2), 0xff);
		local_crc_low = Crypt::crc8_31(data.substr(3, 2), 0xff);

		if(remote_crc_high != local_crc_high)
			throw(transient_exception());

		if(remote_crc_low != local_crc_low)
			throw(transient_exception());
	}
	catch(const transient_exception &e)
	{
		throw(transient_exception());
	}

	this->raw_values["serial"] = serial;
}

void SensorSHT4X::_poll()
{
	switch(this->state)
	{
		case(state_t::init):
		{
			this->device->send(enum_integer(cmd_t::soft_reset));
			this->state = state_t::ready;

			break;
		}

		case(state_t::ready):
		case(state_t::finished):
		{
			this->device->send(enum_integer(cmd_t::measure_high_precision));
			this->state = state_t::measuring;

			break;
		}

		case(state_t::measuring):
		{
			I2C::data_t data;
			unsigned int raw_temperature, raw_humidity;
			float temperature, humidity;
			unsigned char crc_local, crc_remote;

			try
			{
				this->device->receive(6, data);
			}
			catch(const transient_exception &e)
			{
				this->raw_values["not ready"]++;
				throw;
			}

			crc_remote = data.at(2);
			crc_local = Crypt::crc8_31(data.substr(0, 2), 0xff);

			if(crc_local != crc_remote)
			{
				this->raw_values["crc errors"]++;
				throw(transient_exception());
			}

			crc_remote = data.at(5);
			crc_local = Crypt::crc8_31(data.substr(3, 2), 0xff);

			if(crc_local != crc_remote)
			{
				this->raw_values["crc errors"]++;
				throw(transient_exception());
			}

			this->raw_values["raw temperature"] = raw_temperature = this->u16be(data, 0);
			this->raw_values["raw humidity"] = raw_humidity = this->u16be(data, 3);

			temperature = ((static_cast<float>(raw_temperature) * 175.f) / 65535.0f) - 45.0f;
			humidity = ((static_cast<float>(raw_humidity) * 125.0f) / 65535.0f) - 6.0f;

			this->update("temperature", temperature, "degrees", 1);
			this->update("humidity", humidity, "%", 0);
			this->state = state_t::finished;

			break;
		}

		default:
		{
			throw(hard_exception("sht4x: invalid state"));
		}
	}
}

void SensorHDC1080::write_word(reg_t reg, conf_t value)
{
	this->device->send(enum_integer(reg), this->u16high(enum_integer(value)), this->u16low(enum_integer(value)));
}

bool SensorHDC1080::probe(I2c &i2c, int module, int bus, int address, bool restricted)
{
	if(restricted)
		return(false);

	return(i2c.probe(module, bus, address));
}

SensorHDC1080::SensorHDC1080(Log &log_in, I2C::Device *device_in) : SensorI2C(log_in, this->basic_name(), device_in)
{
	I2C::data_t data;

	this->device->send_receive(enum_integer(reg_t::man_id), 2, data);

	if(this->u16be(data) != enum_integer(id_t::manufacturer))
		throw(transient_exception("manufacturer id mismatch"));

	this->device->send_receive(enum_integer(reg_t::dev_id), 2, data);

	if(this->u16be(data) != enum_integer(id_t::device))
		throw(transient_exception("device id mismatch"));

	this->write_word(reg_t::conf, conf_t::rst);
}

void SensorHDC1080::_poll()
{
	static constexpr const conf_t default_config = conf_t::tres_14 | conf_t::hres_14 | conf_t::mode_two;
	I2C::data_t data;

	switch(this->state)
	{
		case(state_t::init):
		{
			this->state = state_t::reset;
			break;
		}

		case(state_t::reset):
		{
			this->device->send_receive(enum_integer(reg_t::conf), 2, data);

			if(this->u16le(data) & enum_integer(conf_t::rst))
				throw(transient_exception("poll error 1"));

			this->write_word(reg_t::conf, default_config);
			this->state = state_t::ready;

			break;
		}

		case(state_t::ready):
		case(state_t::finished):
		{
			this->device->send(enum_integer(reg_t::data_temp));
			this->state = state_t::measuring;

			break;
		}

		case(state_t::measuring):
		{
			unsigned int temperature;
			unsigned int humidity;

			this->device->receive(4, data);

			temperature = this->u16be(data, 0);
			humidity = this->u16be(data, 2);

			this->raw_values["raw temperature"] = temperature;
			this->raw_values["raw humidity"] = humidity;

			this->update("temperature", ((temperature * 165.0f) / 65536.0f) - 40.0f, "degrees", 1);
			this->update("humidity", (humidity * 100.0f) / 65536.0f, "%", 0);
			this->state = state_t::finished;

			break;
		}

		default:
		{
			throw(hard_exception("hdc1080: invalid state"));
		}
	}
}

SensorBMX280::SensorBMX280(Log &log_in, I2C::Device *device_in) : SensorI2C(log_in, this->basic_name(), device_in)
{
}

bool SensorBMP280::probe(I2c &i2c, int module, int bus, int address, bool restricted)
{
	return(i2c.probe(module, bus, address));
}

SensorBMP280::SensorBMP280(Log &log_in, I2C::Device *device_in) : SensorBMX280(log_in, device_in)
{
	I2C::data_t data;

	this->device->send_receive(enum_integer(reg_t::id), 1, data);

	if(data.at(0) != enum_integer(id_t::id_bmp280))
		throw(transient_exception());

	this->device->send(enum_integer(reg_t::reset), enum_integer(reset_t::reset_value));
	this->device->send_receive(enum_integer(reg_t::reset), 1, data);

	if(data.at(0) != 0x00)
		throw(transient_exception("reset failed"));

	this->name = this->basic_name();
}

void SensorBMP280::_poll()
{
	I2C::data_t data;

	switch(this->state)
	{
		case(state_t::init):
		{
			this->device->send_receive(enum_integer(cal_off_t::base), enum_integer(cal_off_t::size), data);

			this->calibration_data.t1 = this->u16le(data, enum_integer(cal_off_t::off_0x88_0x89_dig_t1));
			this->calibration_data.t2 = this->s16le(data, enum_integer(cal_off_t::off_0x8a_0x8b_dig_t2));
			this->calibration_data.t3 = this->s16le(data, enum_integer(cal_off_t::off_0x8c_0x8d_dig_t3));
			this->calibration_data.p1 = this->u16le(data, enum_integer(cal_off_t::off_0x8e_0x8f_dig_p1));
			this->calibration_data.p2 = this->s16le(data, enum_integer(cal_off_t::off_0x90_0x91_dig_p2));
			this->calibration_data.p3 = this->s16le(data, enum_integer(cal_off_t::off_0x92_0x93_dig_p3));
			this->calibration_data.p4 = this->s16le(data, enum_integer(cal_off_t::off_0x94_0x95_dig_p4));
			this->calibration_data.p5 = this->s16le(data, enum_integer(cal_off_t::off_0x96_0x97_dig_p5));
			this->calibration_data.p6 = this->s16le(data, enum_integer(cal_off_t::off_0x98_0x99_dig_p6));
			this->calibration_data.p7 = this->s16le(data, enum_integer(cal_off_t::off_0x9a_0x9b_dig_p7));
			this->calibration_data.p8 = this->s16le(data, enum_integer(cal_off_t::off_0x9c_0x9d_dig_p8));
			this->calibration_data.p9 = this->s16le(data, enum_integer(cal_off_t::off_0x9e_0x9f_dig_p9));

			this->raw_values["c.t1"] = this->calibration_data.t1;
			this->raw_values["c.t2"] = this->calibration_data.t2;
			this->raw_values["c.t3"] = this->calibration_data.t3;
			this->raw_values["c.p1"] = this->calibration_data.p1;
			this->raw_values["c.p2"] = this->calibration_data.p2;
			this->raw_values["c.p3"] = this->calibration_data.p3;
			this->raw_values["c.p4"] = this->calibration_data.p4;
			this->raw_values["c.p5"] = this->calibration_data.p5;
			this->raw_values["c.p6"] = this->calibration_data.p6;
			this->raw_values["c.p7"] = this->calibration_data.p7;
			this->raw_values["c.p8"] = this->calibration_data.p8;
			this->raw_values["c.p9"] = this->calibration_data.p9;

			if((this->calibration_data.t1 > 0) && (this->calibration_data.t2 > 0))
				this->state = state_t::ready;

			break;
		}

		case(state_t::ready):
		case(state_t::finished):
		{
			this->device->send_receive(enum_integer(reg_t::ctrl_meas), 1, data);

			if((data.at(0) & enum_integer(reg_ctrl_meas_t::mode_mask)) != enum_integer(reg_ctrl_meas_t::mode_sleep))
				throw(transient_exception("poll error 1"));

			this->device->send(enum_integer(reg_t::config), enum_integer(reg_config_t::filter_2));
			this->device->send(enum_integer(reg_t::ctrl_meas), enum_integer(reg_ctrl_meas_t::osrs_t_16 | reg_ctrl_meas_t::osrs_p_16 | reg_ctrl_meas_t::mode_forced));

			this->state = state_t::measuring;

			break;
		}

		case(state_t::measuring):
		{
			unsigned int raw_airpressure, raw_temperature;
			float t_fine_1, var1, var2, airpressure;

			this->device->send_receive(enum_integer(reg_t::adc), 8, data);

			this->raw_values["raw airpressure"] = raw_airpressure = this->u20topbe(data, 0);
			this->raw_values["raw temperature"] = raw_temperature = this->u20topbe(data, 3);

			var1 = ((raw_temperature / 16384.0f) - (this->calibration_data.t1 / 1024.0f)) * this->calibration_data.t2;
			var2 = ((raw_temperature / 131072.0f) - (this->calibration_data.t1 / 8192.0f)) *
					((raw_temperature / 131072.0f) - (this->calibration_data.t1 / 8192.0f)) *
					this->calibration_data.t3;

			this->update("temperature", (var1 + var2) / 5120.0f, "degrees", 1);

			var1 = (raw_temperature / 16384.0f - this->calibration_data.t1 / 1024.0f) * this->calibration_data.t2;
			var2 = (raw_temperature / 131072.0f - this->calibration_data.t1 / 8192.0f) *
					(raw_temperature / 131072.0f - this->calibration_data.t1 / 8192.0f) * this->calibration_data.t3;
			t_fine_1 = var1 + var2;
			this->raw_values["t_fine_1 * 1000"] = t_fine_1 * 1000;

			var1 = (t_fine_1 / 2.0f) - 64000.0f;
			var2 = var1 * var1 * this->calibration_data.p6 / 32768.0f;
			var2 = var2 + var1 * this->calibration_data.p5 * 2.0f;
			var2 = (var2 / 4.0f) + (this->calibration_data.p4 * 65536.0f);
			var1 = (this->calibration_data.p3 * var1 * var1 / 524288.0f + this->calibration_data.p2 * var1) / 524288.0f;
			var1 = (1.0f + var1 / 32768.0f) * this->calibration_data.p1;

			if(static_cast<int>(var1) == 0)
				airpressure = 0;
			else
			{
				airpressure = 1048576.0f - raw_airpressure;
				airpressure = (airpressure - (var2 / 4096.0f)) * 6250.0f / var1;
				var1 = this->calibration_data.p9 * airpressure * airpressure / 2147483648.0f;
				var2 = airpressure * this->calibration_data.p8 / 32768.0f;
				airpressure = airpressure + (var1 + var2 + this->calibration_data.p7) / 16.0f;
			}

			this->update("airpressure", airpressure / 100.0f, "hPa", 0);
			this->state = state_t::finished;

			break;
		}

		default:
		{
			throw(hard_exception("bmp280: invalid state"));
		}
	}
}

bool SensorBME280::probe(I2c &i2c, int module, int bus, int address, bool restricted)
{
	return(i2c.probe(module, bus, address));
}

SensorBME280::SensorBME280(Log &log_in, I2C::Device *device_in) : SensorBMX280::SensorBMX280(log_in, device_in)
{
	I2C::data_t data;

	this->device->send_receive(enum_integer(reg_t::id), 1, data);

	if(data.at(0) != enum_integer(id_t::id_bme280))
		throw(transient_exception());

	this->device->send(enum_integer(reg_t::reset), enum_integer(reset_t::reset_value));
	this->device->send_receive(enum_integer(reg_t::reset), 1, data);

	if(data.at(0) != 0x00)
		throw(transient_exception("reset failed"));

	this->name = this->basic_name();
}

void SensorBME280::_poll()
{
	I2C::data_t data;

	switch(this->state)
	{
		case(state_t::init):
		{
			unsigned int e4, e5, e6;

			this->device->send_receive(enum_integer(cal_off_t::base), enum_integer(cal_off_t::size), data);

			this->calibration_data.t1 = this->u16le(data, enum_integer(cal_off_t::off_0x88_0x89_dig_t1));
			this->calibration_data.t2 = this->s16le(data, enum_integer(cal_off_t::off_0x8a_0x8b_dig_t2));
			this->calibration_data.t3 = this->s16le(data, enum_integer(cal_off_t::off_0x8c_0x8d_dig_t3));
			this->calibration_data.p1 = this->u16le(data, enum_integer(cal_off_t::off_0x8e_0x8f_dig_p1));
			this->calibration_data.p2 = this->s16le(data, enum_integer(cal_off_t::off_0x90_0x91_dig_p2));
			this->calibration_data.p3 = this->s16le(data, enum_integer(cal_off_t::off_0x92_0x93_dig_p3));
			this->calibration_data.p4 = this->s16le(data, enum_integer(cal_off_t::off_0x94_0x95_dig_p4));
			this->calibration_data.p5 = this->s16le(data, enum_integer(cal_off_t::off_0x96_0x97_dig_p5));
			this->calibration_data.p6 = this->s16le(data, enum_integer(cal_off_t::off_0x98_0x99_dig_p6));
			this->calibration_data.p7 = this->s16le(data, enum_integer(cal_off_t::off_0x9a_0x9b_dig_p7));
			this->calibration_data.p8 = this->s16le(data, enum_integer(cal_off_t::off_0x9c_0x9d_dig_p8));
			this->calibration_data.p9 = this->s16le(data, enum_integer(cal_off_t::off_0x9e_0x9f_dig_p9));
			this->calibration_data.h1 = data.at(enum_integer(cal_off_t::off_0xa1_dig_h1));
			this->calibration_data.h2 = this->s16le(data, enum_integer(cal_off_t::off_0xe1_0xe2_dig_h2));
			this->calibration_data.h3 = data.at(enum_integer(cal_off_t::off_0xe3_dig_h3));
			e4 = data.at(enum_integer(cal_off_t::off_0xe4_0xe5_0xe6_dig_h4_h5) + 0);
			e5 = data.at(enum_integer(cal_off_t::off_0xe4_0xe5_0xe6_dig_h4_h5) + 1);
			e6 = data.at(enum_integer(cal_off_t::off_0xe4_0xe5_0xe6_dig_h4_h5) + 2);
			this->calibration_data.h4 = ((e4 & 0xff) << 4) | ((e5 & 0x0f) >> 0);
			this->calibration_data.h5 = ((e6 & 0xff) << 4) | ((e5 & 0xf0) >> 4);
			this->calibration_data.h6 = data.at(enum_integer(cal_off_t::off_0xe7_dig_h6));

			this->raw_values["c.t1"] = this->calibration_data.t1;
			this->raw_values["c.t2"] = this->calibration_data.t2;
			this->raw_values["c.t3"] = this->calibration_data.t3;
			this->raw_values["c.p1"] = this->calibration_data.p1;
			this->raw_values["c.p2"] = this->calibration_data.p2;
			this->raw_values["c.p3"] = this->calibration_data.p3;
			this->raw_values["c.p4"] = this->calibration_data.p4;
			this->raw_values["c.p5"] = this->calibration_data.p5;
			this->raw_values["c.p6"] = this->calibration_data.p6;
			this->raw_values["c.p7"] = this->calibration_data.p7;
			this->raw_values["c.p8"] = this->calibration_data.p8;
			this->raw_values["c.p9"] = this->calibration_data.p9;
			this->raw_values["c.h1"] = this->calibration_data.h1;
			this->raw_values["c.h2"] = this->calibration_data.h2;
			this->raw_values["c.h3"] = this->calibration_data.h3;
			this->raw_values["c.h4"] = this->calibration_data.h4;
			this->raw_values["c.h5"] = this->calibration_data.h5;
			this->raw_values["c.h6"] = this->calibration_data.h6;

			if((this->calibration_data.t1 > 0) && (this->calibration_data.t2 > 0))
				this->state = state_t::ready;

			break;
		}

		case(state_t::ready):
		case(state_t::finished):
		{
			this->device->send_receive(enum_integer(reg_t::ctrl_meas), 1, data);

			if((data.at(0) & enum_integer(reg_ctrl_meas_t::mode_mask)) != enum_integer(reg_ctrl_meas_t::mode_sleep))
				throw(transient_exception("poll error 1"));

			this->device->send(enum_integer(reg_t::ctrl_hum), enum_integer(reg_ctrl_hum_t::osrs_h_16));
			this->device->send(enum_integer(reg_t::config), enum_integer(reg_config_t::filter_2));
			this->device->send(enum_integer(reg_t::ctrl_meas), enum_integer(reg_ctrl_meas_t::osrs_t_16 | reg_ctrl_meas_t::osrs_p_16 | reg_ctrl_meas_t::mode_forced));

			this->state = state_t::measuring;

			break;
		}

		case(state_t::measuring):
		{
			unsigned int raw_airpressure, raw_temperature, raw_humidity;
			float t_fine_1, t_fine_2, var1, var2, airpressure, humidity;

			this->device->send_receive(enum_integer(reg_t::adc), 8, data);

			this->raw_values["raw airpressure"] = raw_airpressure = this->u20topbe(data, 0);
			this->raw_values["raw temperature"] = raw_temperature = this->u20topbe(data, 3);
			this->raw_values["raw humidity"] = raw_humidity = this->u16be(data, 6);

			var1 = ((raw_temperature / 16384.0f) - (this->calibration_data.t1 / 1024.0f)) * this->calibration_data.t2;
			var2 = ((raw_temperature / 131072.0f) - (this->calibration_data.t1 / 8192.0f)) *
					((raw_temperature / 131072.0f) - (this->calibration_data.t1 / 8192.0f)) *
					this->calibration_data.t3;

			this->update("temperature", (var1 + var2) / 5120.0f, "degrees", 1);

			var1 = (raw_temperature / 16384.0f - this->calibration_data.t1 / 1024.0f) * this->calibration_data.t2;
			var2 = (raw_temperature / 131072.0f - this->calibration_data.t1 / 8192.0f) *
					(raw_temperature / 131072.0f - this->calibration_data.t1 / 8192.0f) * this->calibration_data.t3;
			t_fine_1 = var1 + var2;
			this->raw_values["t_fine_1 * 1000"] = t_fine_1 * 1000;

			var1 = (t_fine_1 / 2.0f) - 64000.0f;
			var2 = var1 * var1 * this->calibration_data.p6 / 32768.0f;
			var2 = var2 + var1 * this->calibration_data.p5 * 2.0f;
			var2 = (var2 / 4.0f) + (this->calibration_data.p4 * 65536.0f);
			var1 = (this->calibration_data.p3 * var1 * var1 / 524288.0f + this->calibration_data.p2 * var1) / 524288.0f;
			var1 = (1.0f + var1 / 32768.0f) * this->calibration_data.p1;

			if(static_cast<int>(var1) == 0)
				airpressure = 0;
			else
			{
				airpressure = 1048576.0f - raw_airpressure;
				airpressure = (airpressure - (var2 / 4096.0f)) * 6250.0f / var1;
				var1 = this->calibration_data.p9 * airpressure * airpressure / 2147483648.0f;
				var2 = airpressure * this->calibration_data.p8 / 32768.0f;
				airpressure = airpressure + (var1 + var2 + this->calibration_data.p7) / 16.0f;
			}

			this->update("airpressure", airpressure / 100.0f, "hPa", 0);

			var1 = (raw_temperature / 16384.0f	- (this->calibration_data.t1 / 1024.0f)) * this->calibration_data.t2;
			var2 = (raw_temperature / 131072.0f	- (this->calibration_data.t1 / 8192.0f)) *
					((raw_temperature / 131072.0f) - (this->calibration_data.t1 / 8192.0f)) * this->calibration_data.t3;
			t_fine_2 = var1 + var2 - 76800;
			this->raw_values["t_fine_2 * 1000"] = t_fine_2 * 1000;

			humidity = (raw_humidity - ((this->calibration_data.h4 * 64.0f) + (this->calibration_data.h5 / 16384.0f) * t_fine_2)) *
					(this->calibration_data.h2 / 65536.0f * (1.0f + this->calibration_data.h6 / 67108864.0f * t_fine_2 *
					(1.0f + this->calibration_data.h3 / 67108864.0f * t_fine_2)));
			humidity = humidity * (1.0f - this->calibration_data.h1 * humidity / 524288.0f);

			if(humidity > 100.0f)
				humidity = 100.0f;

			if(humidity < 0.0f)
				humidity = 0.0f;

			this->update("humidity", humidity, "%", 0);
			this->state = state_t::finished;

			break;
		}

		default:
		{
			throw(hard_exception("bme280: invalid state"));
		}
	}
}

bool SensorBME680::probe(I2c &i2c, int module, int bus, int address, bool restricted)
{
	return(i2c.probe(module, bus, address));
}

SensorBME680::SensorBME680(Log &log_in, I2C::Device *device_in) : SensorI2C(log_in, this->basic_name(), device_in)
{
	I2C::data_t data;

	this->device->send_receive(enum_integer(reg_t::id), 1, data);

	if(data.at(0) != enum_integer(reg_id_t::bme680))
		throw(transient_exception());

	this->device->send(enum_integer(reg_t::reset), enum_integer(reg_reset_t::value));
}

void SensorBME680::_poll()
{
	I2C::data_t data, cal1, cal2;

	switch(this->state)
	{
		case(state_t::init):
		{
			this->device->send(enum_integer(reg_t::config), enum_integer(reg_config_t::filter_127));
			this->device->send(enum_integer(reg_t::ctrl_gas_0), enum_integer(reg_ctrl_t::gas_0_heat_off));
			this->device->send_receive(enum_integer(reg_t::calibration_1), enum_integer(cal_off_t::size_1), cal1);
			this->device->send_receive(enum_integer(reg_t::calibration_2), enum_integer(cal_off_t::size_2), cal2);

			data = cal1 + cal2;

			this->calibration_data.t1 = this->u16le(data, enum_integer(cal_off_t::t1));
			this->calibration_data.t2 = this->s16le(data, enum_integer(cal_off_t::t2));
			this->calibration_data.t3 = this->s8(data, enum_integer(cal_off_t::t3));

			this->calibration_data.p1 = this->u16le(data, enum_integer(cal_off_t::p1));
			this->calibration_data.p2 = this->s16le(data, enum_integer(cal_off_t::p2));
			this->calibration_data.p3 = this->s8(data, enum_integer(cal_off_t::p3));
			this->calibration_data.p4 = this->s16le(data, enum_integer(cal_off_t::p4));
			this->calibration_data.p5 = this->s16le(data, enum_integer(cal_off_t::p5));
			this->calibration_data.p6 = this->s8(data, enum_integer(cal_off_t::p6));
			this->calibration_data.p7 = this->s8(data, enum_integer(cal_off_t::p7));
			this->calibration_data.p8 = this->s16le(data, enum_integer(cal_off_t::p8));
			this->calibration_data.p9 = this->s16le(data, enum_integer(cal_off_t::p9));
			this->calibration_data.p10 = this->u8(data, enum_integer(cal_off_t::p10));

			this->calibration_data.h1 = this->u12bottomle(data, enum_integer(cal_off_t::h1));
			this->calibration_data.h2 = this->u12topbe(data, enum_integer(cal_off_t::h2));
			this->calibration_data.h3 = this->s8(data, enum_integer(cal_off_t::h3));
			this->calibration_data.h4 = this->s8(data, enum_integer(cal_off_t::h4));
			this->calibration_data.h5 = this->s8(data, enum_integer(cal_off_t::h5));
			this->calibration_data.h6 = this->u8(data, enum_integer(cal_off_t::h6));
			this->calibration_data.h7 = this->s8(data, enum_integer(cal_off_t::h7));

			this->raw_values["c.t1"] = this->calibration_data.t1;
			this->raw_values["c.t2"] = this->calibration_data.t2;
			this->raw_values["c.t3"] = this->calibration_data.t3;

			this->raw_values["c.p1"] = this->calibration_data.p1;
			this->raw_values["c.p2"] = this->calibration_data.p2;
			this->raw_values["c.p3"] = this->calibration_data.p3;
			this->raw_values["c.p4"] = this->calibration_data.p4;
			this->raw_values["c.p5"] = this->calibration_data.p5;
			this->raw_values["c.p6"] = this->calibration_data.p6;
			this->raw_values["c.p7"] = this->calibration_data.p7;
			this->raw_values["c.p8"] = this->calibration_data.p8;
			this->raw_values["c.p9"] = this->calibration_data.p9;
			this->raw_values["c.p10"] = this->calibration_data.p10;

			this->raw_values["c.h1"] = this->calibration_data.h1;
			this->raw_values["c.h2"] = this->calibration_data.h2;
			this->raw_values["c.h3"] = this->calibration_data.h3;
			this->raw_values["c.h4"] = this->calibration_data.h4;
			this->raw_values["c.h5"] = this->calibration_data.h5;
			this->raw_values["c.h6"] = this->calibration_data.h6;
			this->raw_values["c.h7"] = this->calibration_data.h7;

			if((calibration_data.t1 != 0) && (calibration_data.t2 != 0))
				this->state = state_t::ready;

			break;
		}

		case(state_t::finished):
		case(state_t::ready):
		{
			this->device->send(enum_integer(reg_t::ctrl_hum), enum_integer(reg_hum_t::osrh_h_16));
			this->device->send(enum_integer(reg_t::ctrl_meas), enum_integer(reg_meas_t::osrs_t_16 | reg_meas_t::osrs_p_8 | reg_meas_t::forced));

			this->state = state_t::measuring;

			break;
		}

		case(state_t::measuring):
		{
			int raw_temperature, raw_airpressure, raw_humidity;
			float temperature, humidity, airpressure, airpressure_256;
			float var1, var2, var3, var4, t1_scaled, t_fine;

			this->device->send_receive(enum_integer(reg_t::meas_status_0), 1, data);

			if(data.at(0) & enum_integer(reg_meas_status_0_t::measuring))
				this->raw_values["not ready"]++;

			this->device->send_receive(enum_integer(reg_t::temp), 3, data);
			raw_temperature = this->u20topbe(data);

			this->device->send_receive(enum_integer(reg_t::press), 3, data);
			raw_airpressure = this->u20topbe(data);

			this->device->send_receive(enum_integer(reg_t::hum), 3, data);
			raw_humidity = this->u16be(data);

			this->raw_values["raw temperature"] = raw_temperature;
			this->raw_values["raw airpressure"] = raw_airpressure;
			this->raw_values["raw humidity"] = raw_humidity;

			t1_scaled =	(raw_temperature / 131072.0f) - (this->calibration_data.t1 / 8192.0f);
			t_fine =	((raw_temperature / 16384.0f) - (this->calibration_data.t1 / 1024.0f)) *
					this->calibration_data.t2 + (t1_scaled * t1_scaled * this->calibration_data.t3 * 16.0f);

			temperature	= t_fine / 5120.0f;

			var1 = (t_fine / 2.0f) - 64000.0f;
			var2 = var1 * var1 * this->calibration_data.p6 / 131072.0f;
			var2 = var2 + (var1 * this->calibration_data.p5 * 2.0f);
			var2 = (var2 / 4) + (this->calibration_data.p4 * 65536.f);
			var1 = (((this->calibration_data.p3 * var1 * var1) / 16384.0f) +(this->calibration_data.p2 * var1)) / 524288.0f;
			var1 = (1 + (var1 / 32768.0f)) * this->calibration_data.p1;
			airpressure = 1048576.0f - raw_airpressure;

			if(static_cast<int>(var1) != 0)
			{
				airpressure = ((airpressure - (var2 / 4096.0f)) * 6250.0f) / var1;
				airpressure_256 = airpressure / 256.0f;
				var1 = (this->calibration_data.p9 * airpressure * airpressure) / 2147483648.0f;
				var2 = airpressure * (this->calibration_data.p8 / 32768.0f);
				var3 = airpressure_256 * airpressure_256 * airpressure_256 * (this->calibration_data.p10 / 131072.0f);
				airpressure = (airpressure + (var1 + var2 + var3 + (this->calibration_data.p7 * 128.0f)) / 16.0f) / 100.0f;
			}
			else
				airpressure = 0;

			var1 = raw_humidity - ((this->calibration_data.h1 * 16.0f) + ((this->calibration_data.h3 / 2.0f) * temperature));
			var2 = var1 * ((this->calibration_data.h2 / 262144.0f) *
					(1.0f + ((this->calibration_data.h4 / 16384.0f) *
					temperature) + ((this->calibration_data.h5 / 1048576.0f) *
					temperature * temperature)));
			var3 = this->calibration_data.h6 / 16384.0f;
			var4 = this->calibration_data.h7 / 2097152.0f;

			humidity = var2 + ((var3 + (var4 * temperature)) * var2 * var2);

			if(humidity > 100.0f)
				humidity = 100.0f;

			if(humidity < 0.0f)
				humidity = 0.0f;

			this->update("temperature", temperature, "degrees", 1);
			this->update("airpressure", airpressure, "hPa", 0);
			this->update("humidity", humidity, "%", 0);
			this->state = state_t::finished;

			break;
		}

		default:
		{
			throw(hard_exception("bme680: invalid state"));
		}
	}
}

const SensorBH1750::autorangings_t SensorBH1750::autoranging_data
{
	{	.mode = opcode_t::oneshot_hmode2,	.timing = 254,	.lower = 0,		.upper = 50000,	.factor = 0.13f },
	{	.mode = opcode_t::oneshot_hmode2,	.timing = 69,	.lower = 1000,	.upper = 50000,	.factor = 0.50f },
	{	.mode = opcode_t::oneshot_hmode2,	.timing = 31,	.lower = 1000,	.upper = 50000,	.factor = 1.10f },
	{	.mode = opcode_t::oneshot_lmode,	.timing = 31,	.lower = 1000,	.upper = 65536,	.factor = 2.40f },
};

bool SensorBH1750::probe(I2c &i2c, int module, int bus, int address, bool restricted)
{
	if(restricted)
		return(false);

	return(i2c.probe(module, bus, address));
}

SensorBH1750::SensorBH1750(Log& log_in, I2C::Device* device_in) : SensorI2C(log_in, this->basic_name(), device_in)
{
	I2C::data_t data;

	device->send_receive(enum_integer(opcode_t::poweron), 8, data);

	if((data.at(2) != 0xff) || (data.at(3) != 0xff) || (data.at(4) != 0xff) ||
			(data.at(5) != 0xff) || (data.at(6) != 0xff) || (data.at(7) != 0xff))
		throw(transient_exception("bh1750 not found"));

	this->raw_values["scaling"] = 0;
	this->raw_values["last poll"] = time(nullptr);
}

void SensorBH1750::_poll()
{
	I2C::data_t data;
	int value;
	int scaling = this->raw_values["scaling"];
	const autoranging_t &autoranging = this->autoranging_data.at(scaling);
	int autoranging_max = this->autoranging_data.size() - 1;
	time_t now = time(nullptr);

	if((now - this->raw_values["last poll"]) < 1)
	{
		this->raw_values["skips"]++;
		return;
	}

	this->raw_values["last poll"] = now;

	switch(this->state)
	{
		case(state_t::init):
		{
			device->send(enum_integer(opcode_t::reset));

			this->state = state_t::reset;

			break;
		}

		case(state_t::reset):
		case(state_t::finished):
		{
			device->send(enum_integer(opcode_t::change_meas_hi) | ((autoranging.timing >> 5) & 0b0000'0111));
			device->send(enum_integer(opcode_t::change_meas_lo) | ((autoranging.timing >> 0) & 0b0001'1111));
			device->send(enum_integer(autoranging.mode));

			this->state = state_t::measuring;

			break;
		}

		case(state_t::measuring):
		{
			device->receive(2, data);

			this->raw_values["byte 0"] = data.at(0);
			this->raw_values["byte 1"] = data.at(1);

			value = this->u16be(data);

			this->raw_values["both"] = value;

			this->state = state_t::finished;

			if((value >= 65535) && (scaling >= autoranging_max))
			{
				this->raw_values["overflows"]++;
				break;
			}

			if((value < autoranging.lower) && (scaling > 0))
			{
				scaling--;
				this->raw_values["scaling"] = scaling;
				this->raw_values["scalings down"]++;
				break;
			}

			if((value >= autoranging.upper) && (scaling < autoranging_max))
			{
				scaling++;
				this->raw_values["scaling"] = scaling;
				this->raw_values["scalings up"]++;
				break;
			}

			this->update("visible light", static_cast<float>(value) * autoranging.factor, "lx", 1);

			break;
		}

		default:
		{
			throw(hard_exception("bh1750::poll: invalid state"));
		}
	}
}

void SensorOPT3001::start_measurement()
{
	I2C::data_t data;

	device->send(enum_integer(reg_t::config), this->u16high(enum_integer(conf_t::default_config)), this->u16low(enum_integer(conf_t::default_config)));
}

bool SensorOPT3001::probe(I2c& i2c, int module, int bus, int address, bool restricted)
{
	return(i2c.probe(module, bus, address));
}

SensorOPT3001::SensorOPT3001(Log& log_in, I2C::Device* device_in) : SensorI2C(log_in, this->basic_name(), device_in)
{
	I2C::data_t data;
	unsigned int read_config;

	device->send_receive(enum_integer(reg_t::id_manuf), 2, data);

	if(this->u16be(data) != enum_integer(id_t::manufacturer_ti))
		throw(transient_exception());

	device->send_receive(enum_integer(reg_t::id_dev), 2, data);

	if(this->u16be(data) != enum_integer(id_t::device_opt3001))
		throw(transient_exception());

	this->start_measurement();

	device->send_receive(enum_integer(reg_t::config), 2, data);

	read_config = this->u16be(data) & enum_integer(conf_t::mask_exp | conf_t::conv_mode | conf_t::conv_time | conf_t::range);

	if(read_config != enum_integer(conf_t::default_config))
		throw(transient_exception("invalid config"));
}

void SensorOPT3001::_poll()
{
	I2C::data_t data;
	unsigned int config;

	switch(this->state)
	{
		case(state_t::init):
		case(state_t::finished):
		{
			this->start_measurement();
			this->state = state_t::measuring;

			break;
		}

		case(state_t::measuring):
		{
			unsigned int exponent;
			unsigned int mantissa;

			device->send_receive(enum_integer(reg_t::config), 2, data);

			config = this->u16be(data);

			if(!(config & enum_integer(conf_t::flag_ready)))
				return;

			this->state = state_t::finished;

			if(config & enum_integer(conf_t::flag_ovf))
			{
				this->raw_values["overflows"]++;
				return;
			}

			device->send_receive(enum_integer(reg_t::result), 2, data);

			exponent = (data.at(0) & 0xf0) >> 4;
			mantissa = ((data.at(0) & 0x0f) << 8) | data.at(1);

			this->raw_values["exponent"] = exponent;
			this->raw_values["mantissa"] = mantissa;

			this->update("visible light", 0.01f * static_cast<float>(1 << exponent) * static_cast<float>(mantissa), "lx", 2);

			break;
		}

		default:
		{
			throw(hard_exception("opt3001::poll: invalid state"));
		}
	}
}

bool SensorMAX44009::probe(I2c &i2c, int module, int bus, int address, bool restricted)
{
	return(i2c.probe(module, bus, address));
}

SensorMAX44009::SensorMAX44009(Log& log_in, I2C::Device* device_in) : SensorI2C(log_in, this->basic_name(), device_in)
{
	I2C::data_t data;

	device->send_receive(enum_integer(reg_t::ints), 2, data);

	if((data.at(0) != enum_integer(probe_t::ints)) || (data.at(1) != enum_integer(probe_t::ints)))
		throw(transient_exception());

	device->send_receive(enum_integer(reg_t::inte), 2, data);

	if((data.at(0) != enum_integer(probe_t::inte)) || (data.at(1) != enum_integer(probe_t::inte)))
		throw(transient_exception());

	device->send_receive(enum_integer(reg_t::thresh_msb), 2, data);

	if((data.at(0) != enum_integer(probe_t::thresh_msb)) || (data.at(1) != enum_integer(probe_t::thresh_msb)))
		throw(transient_exception());

	device->send_receive(enum_integer(reg_t::thresh_lsb), 2, data);

	if((data.at(0) != enum_integer(probe_t::thresh_lsb)) || (data.at(1) != enum_integer(probe_t::thresh_lsb)))
		throw(transient_exception());

	device->send_receive(enum_integer(reg_t::thresh_timer), 2, data);

	if((data.at(0) != enum_integer(probe_t::thresh_timer)) || (data.at(1) != enum_integer(probe_t::thresh_timer)))
		throw(transient_exception());

	device->send(enum_integer(reg_t::conf), enum_integer(conf_t::cont));
	device->send_receive(enum_integer(reg_t::conf), 2, data);

	if((data.at(0) & enum_integer(conf_t::cont | conf_t::manual)) != enum_integer(conf_t::cont))
		throw(transient_exception());
}

void SensorMAX44009::_poll()
{
	I2C::data_t data;
	unsigned int exponent;
	unsigned int mantissa;

	device->send_receive(enum_integer(reg_t::data_msb), 2, data);

	exponent =	(data.at(0) & 0xf0) >> 4;
	mantissa =	(data.at(0) & 0x0f) << 4;
	mantissa |=	(data.at(1) & 0x0f) << 0;

	this->raw_values["mantissa"] = mantissa;
	this->raw_values["exponent"] = exponent;

	if(exponent != 0b1111)
		this->update("visible light", (1 << exponent) * static_cast<float>(mantissa) * 0.045f, "lx", 2);
	else
		this->raw_values["overflows"]++;

	this->state = state_t::measuring;
}

const SensorTSL2561::autorangings_t SensorTSL2561::autoranging_data
{
	{ .timing = conf_t::integ_402ms,	.gain = conf_t::high_gain,	.lower = 0,		.upper = 50000,	.overflow = 65535,	.factor = 0.35f		},
	{ .timing = conf_t::integ_402ms,	.gain = conf_t::low_gain,	.lower = 256,	.upper = 50000,	.overflow = 65535,	.factor = 7.4f		},
	{ .timing = conf_t::integ_101ms,	.gain = conf_t::low_gain,	.lower = 256,	.upper = 30000,	.overflow = 37177,	.factor = 28.0f		},
	{ .timing = conf_t::integ_13ms,		.gain = conf_t::low_gain,	.lower = 256,	.upper = 65536,	.overflow = 5047,	.factor = 200.0f	},
};

void SensorTSL2561::write_byte(SensorTSL2561::reg_t reg, unsigned char value)
{
	device->send(enum_integer(cmd_t::cmd) | enum_integer(cmd_t::clear) | (enum_integer(reg) & enum_integer(cmd_t::address)), value);
}

unsigned char SensorTSL2561::read_byte(SensorTSL2561::reg_t reg)
{
	I2C::data_t data;

	device->send_receive(enum_integer(cmd_t::cmd) | (enum_integer(reg) & enum_integer(cmd_t::address)), 1, data);

	return(data.at(0));
}

unsigned int SensorTSL2561::read_word(SensorTSL2561::reg_t reg)
{
	I2C::data_t data;

	device->send_receive(enum_integer(cmd_t::cmd) | (enum_integer(reg) & enum_integer(cmd_t::address)), 2, data);

	return(this->u16le(data));
}

bool SensorTSL2561::write_check(SensorTSL2561::reg_t reg, unsigned char value)
{
	unsigned rv;

	try
	{
		this->write_byte(reg, value);
		rv = this->read_byte(reg);
	}
	catch(const transient_exception &)
	{
		return(false);
	}

	if(value != rv)
		return(false);

	return(true);
}

bool SensorTSL2561::probe(I2c &i2c, int module, int bus, int address, bool restricted)
{
	return(i2c.probe(module, bus, address));
}

SensorTSL2561::SensorTSL2561(Log& log_in, I2C::Device* device_in) : SensorI2C(log_in, this->basic_name(), device_in)
{
	unsigned int regval;

	regval = this->read_byte(reg_t::id);

	if(regval != enum_integer(id_t::tsl2561))
		throw(transient_exception());

	regval = this->read_word(reg_t::threshlow);

	if(regval != enum_integer(id_t::probe_threshold))
		throw(transient_exception());

	regval = this->read_word(reg_t::threshhigh);

	if(regval != enum_integer(id_t::probe_threshold))
		throw(transient_exception());

	if(!this->write_check(reg_t::control, enum_integer(control_t::power_off)))
		throw(transient_exception());

	if(this->write_check(reg_t::id, 0x00)) // id register should not be writable
		throw(transient_exception());

	if(!this->write_check(reg_t::interrupt, 0x00))
		throw(transient_exception());

	this->write_byte(reg_t::control, enum_integer(control_t::power_on));

	regval = this->read_byte(reg_t::control);

	if((regval & 0x0f) != enum_integer(control_t::power_on))
		throw(transient_exception("power on failed"));

	this->raw_values["scaling"] = 0;
}

void SensorTSL2561::_poll()
{
	unsigned int channel_0, channel_1;
	float value, ratio;
	int scaling = this->raw_values["scaling"];
	const autoranging_t &autoranging = this->autoranging_data.at(scaling);
	int autoranging_max = this->autoranging_data.size() - 1;

	switch(this->state)
	{
		case(state_t::init):
		case(state_t::finished):
		{
			if(!this->write_check(reg_t::timeint, enum_integer(autoranging.timing) | enum_integer(autoranging.gain)))
				throw(transient_exception("tsl2561 poll 1"));

			state = state_t::measuring;

			break;
		}

		case(state_t::measuring):
		{
			this->state = state_t::finished;

			channel_0 = this->read_word(reg_t::data0);
			channel_1 = this->read_word(reg_t::data1);

			this->raw_values["channel 0"] = channel_0;
			this->raw_values["channel 1"] = channel_1;

			if(((channel_0 >= autoranging.overflow) || (channel_1 >= autoranging.overflow)) && (scaling >= autoranging_max))
			{
				this->raw_values["overflows"]++;
				return;
			}

			if(((channel_0 < autoranging.lower) || (channel_1 < autoranging.lower)) && (scaling > 0))
			{
				scaling--;
				this->raw_values["scaling"] = scaling;
				this->raw_values["scalings down"]++;
				return;
			}

			if(((channel_0 >= autoranging.upper) || (channel_1 >= autoranging.upper)) && (scaling < autoranging_max))
			{
				scaling++;
				this->raw_values["scaling"] = scaling;
				this->raw_values["scalings up"]++;
				break;
			}

			if(channel_0 == 0)
				ratio = 0;
			else
				ratio = channel_1 / channel_0;

			if(ratio > 1.30f)
			{
				this->raw_values["invalids"]++;
				return;
			}

			if(ratio >= 0.80f)
				value = (0.00146f * channel_0) - (0.00112f * channel_1);
			else
				if(ratio >= 0.61f)
					value = (0.0128f * channel_0) - (0.0153f * channel_1);
				else
					if(ratio >= 0.50f)
						value = (0.0224f * channel_0) - (0.031f * channel_1);
					else
						value = (0.0304f * channel_0) - (0.062f * channel_1 * (float)pow(ratio, 1.4f));

			value = (value * autoranging.factor);

			if(value < 0)
				value = 0;

			this->update("visible light", value, "lx", 5);

			break;
		}

		default:
		{
			throw(hard_exception("tsl2561: invalid state"));
		}
	}
}

SensorTSL2591Base::SensorTSL2591Base(Log& log_in, I2C::Device* device_in) : SensorI2C(log_in, this->basic_name(), device_in)
{
}

void SensorTSL2591Base::write_byte(reg_t reg, unsigned char value)
{
	this->device->send(enum_integer(cmd_t::cmd | cmd_t::transaction_normal) | enum_integer(reg), value);
}

unsigned char SensorTSL2591Base::read_byte(reg_t reg)
{
	I2C::data_t data;

	this->device->send_receive(enum_integer(cmd_t::cmd | cmd_t::transaction_normal) | enum_integer(reg), 1, data);

	return(data.at(0));
}

bool SensorTSL2591Base::write_check(reg_t reg, unsigned char value)
{
	unsigned char rv;

	try
	{
		this->write_byte(reg, value);

		rv = this->read_byte(reg);

		if(value != rv)
			return(false);
	}
	catch(const transient_exception &)
	{
		return(false);
	}

	return(true);
}

bool SensorTSL2591RO::probe(I2c &i2c, int module, int bus, int address, bool restricted)
{
	return(!restricted);
}

SensorTSL2591RO::SensorTSL2591RO(Log& log_in, I2C::Device* device_in) : SensorTSL2591Base(log_in, device_in)
{
	I2C::data_t data;

	try
	{
		this->device->receive(enum_integer(reg_t::size), data);
	}
	catch(const transient_exception &)
	{
		throw(transient_exception());
	}

	if((data.at(enum_integer(reg_t::pid)) & enum_integer(pid_t::mask)) != enum_integer(pid_t::value))
		throw(transient_exception());

	if((data.at(enum_integer(reg_t::id)) & enum_integer(id_t::mask)) != enum_integer(id_t::alt_value))
		throw(transient_exception());

	this->name = this->basic_name();
}

void SensorTSL2591RO::_poll()
{
}

const SensorTSL2591RW::autorangings_t SensorTSL2591RW::autoranging_data
{
	{	control_t::again_9500,	control_t::atime_600, 0,	50000, 56095,	0.096,	},
	{	control_t::again_400,	control_t::atime_600, 100,	50000, 56095,	5.8,	},
	{	control_t::again_25,	control_t::atime_400, 100,	50000, 65536,	49,		},
	{	control_t::again_0,		control_t::atime_400, 100,	50000, 65536,	490,	},
	{	control_t::again_0,		control_t::atime_100, 100,	50000, 65536,	1960,	},
};

const SensorTSL2591RW::corrections_t SensorTSL2591RW::correction_data
{
	{	0.0f,	0.125f,	{	1.0f,	-0.895f	}},
	{	0.125f,	0.250f,	{	1.070f,	-1.145f	}},
	{	0.250f,	0.375f,	{	1.115f,	-1.790f	}},
	{	0.375f,	0.500f,	{	1.126f,	-2.050f	}},
	{	0.500f,	0.610f,	{	0.740f,	-1.002f	}},
	{	0.610f,	0.800f,	{	0.420f,	-0.500f	}},
	{	0.800f,	1.300f,	{	0.48f,	-0.037f	}},
};

bool SensorTSL2591RW::probe(I2c &i2c, int module, int bus, int address, bool restricted)
{
	if(restricted)
		return(false);

	return(i2c.probe(module, bus, address));
}

SensorTSL2591RW::SensorTSL2591RW(Log& log_in, I2C::Device* device_in) : SensorTSL2591Base(log_in, device_in)
{
	unsigned char regval;

	regval = this->read_byte(reg_t::pid);

	if((regval & enum_integer(pid_t::mask)) != enum_integer(pid_t::value))
		throw(transient_exception());

	regval = this->read_byte(reg_t::id);

	if((regval & enum_integer(id_t::mask)) != enum_integer(id_t::value))
		throw(transient_exception());

	if(this->write_check(reg_t::id, 0x00)) // id register should not be writable
		throw(transient_exception());

	this->raw_values["scaling"] = 0;

	this->name = this->basic_name();
}

void SensorTSL2591RW::_poll()
{
	I2C::data_t data;
	int scaling;

	scaling = this->raw_values["scaling"];

	switch(this->state)
	{
		case(state_t::init):
		{
			try
			{
				this->write_byte(reg_t::control, enum_integer(control_t::sreset));
			}
			catch(const transient_exception &)
			{
			}

			this->state = state_t::reset;

			break;
		}

		case(state_t::reset):
		{
			if(this->read_byte(reg_t::control) & enum_integer(control_t::sreset))
			{
				this->raw_values["resetting waits"]++;
				return;
			}

			if(!this->write_check(reg_t::enable, enum_integer(enable_t::aen | enable_t::pon)))
			{
				this->state = state_t::init;
				throw(transient_exception("poll error 1"));
			}

			this->state = state_t::ready;

			break;
		}

		case(state_t::ready):
		case(state_t::finished):
		{
			control_t opcode;

			opcode = this->autoranging_data[scaling].gain | this->autoranging_data[scaling].timing;

			if(!this->write_check(reg_t::control, enum_integer(opcode)))
				throw(transient_exception("poll error 2"));

			this->state = state_t::measuring;

			break;
		}

		case(state_t::measuring):
		{
			unsigned int channel0, channel1, overflow, scale_down_threshold, scale_up_threshold, max_scaling;
			float ratio, factor[2], value;

			max_scaling = this->autoranging_data.size() - 1;

			scale_down_threshold =	this->autoranging_data[scaling].lower;
			scale_up_threshold =	this->autoranging_data[scaling].upper;
			overflow =				this->autoranging_data[scaling].overflow;

			this->device->send_receive(enum_integer(cmd_t::cmd) | enum_integer(reg_t::c0datal), 4, data);

			channel0 = this->u16le(data, 0);
			channel1 = this->u16le(data, 2);

			this->state = state_t::finished;

			if(((channel0 < scale_down_threshold) || (channel1 < scale_down_threshold)) && (scaling > 0))
			{
				scaling--;
				this->raw_values["scaling"] = scaling;
				this->raw_values["scalings down"]++;
				break;
			}

			if(((channel0 >= scale_up_threshold) || (channel1 >= scale_up_threshold)) && (scaling < max_scaling))
			{
				scaling++;
				this->raw_values["scaling"] = scaling;
				this->raw_values["scalings up"]++;
				break;
			}

			if((channel0 <= 0) || (channel1 <= 0) || (channel0 >= overflow) || (channel1 >= overflow))
			{
				this->raw_values["overflows"]++;
				break;
			}

			ratio = static_cast<float>(channel1) / static_cast<float>(channel0);

			this->raw_values["channel 0"] = channel0;
			this->raw_values["channel 1"] = channel1;
			this->raw_values["ratio * 1000"] = ratio * 1000;

			factor[0] = factor[1] = 0;

			for(const auto& entry : this->correction_data)
			{
				if((ratio >= entry.lower_bound) && (ratio < entry.upper_bound))
				{
					factor[0] = entry.ch_factor[0];
					factor[1] = entry.ch_factor[1];

					break;
				}
			}

			if((factor[0] <= 0) || (factor[1] >= 0))
			{
				this->raw_values["overflows"]++;
				break;
			}

			this->raw_values["factor 0 * 1000"] = factor[0] * 1000;
			this->raw_values["factor 1 * 1000"] = factor[1] * 1000;

			value = (((channel0 * factor[0]) + (channel1 * factor[1])) / 1000.0f) * this->autoranging_data[scaling].factor;

			this->update("visible light", value, "lx", 5);

			break;
		}

		default:
		{
			throw(hard_exception("tsl2591: invalid state"));
		}
	}
}

const SensorVEML7700::autorangings_t SensorVEML7700::autoranging_data
{
	{	.timing = conf_t::als_it_800,	.gain = conf_t::als_gain_2,		.lower = 0,		.upper = 32768,	.factor = 0.0036f	},
	{	.timing = conf_t::als_it_800,	.gain = conf_t::als_gain_1_8,	.lower = 100,	.upper = 32768,	.factor = 0.0576f	},
	{	.timing = conf_t::als_it_200,	.gain = conf_t::als_gain_2,		.lower = 100,	.upper = 32768,	.factor = 0.0144f	},
	{	.timing = conf_t::als_it_200,	.gain = conf_t::als_gain_1_8,	.lower = 100,	.upper = 32768,	.factor = 0.2304f	},
	{	.timing = conf_t::als_it_25,	.gain = conf_t::als_gain_2,		.lower = 100,	.upper = 32768,	.factor = 0.1152f	},
	{	.timing = conf_t::als_it_25,	.gain = conf_t::als_gain_1_8,	.lower = 100,	.upper = 65536,	.factor = 1.8432f	},
};

bool SensorVEML7700::probe(I2c &i2c, int module, int bus, int address, bool restricted)
{
	return(i2c.probe(module, bus, address));
}

SensorVEML7700::SensorVEML7700(Log& log_in, I2C::Device* device_in) : SensorI2C(log_in, this->basic_name(), device_in)
{
	I2C::data_t data;

	device->send_receive(enum_integer(reg_t::id), 2, data);

	if((data.at(0) != enum_integer(id_t::id_1)) || (data.at(1) != enum_integer(id_t::id_2)))
		throw(transient_exception());
}

void SensorVEML7700::_poll()
{
	I2C::data_t data;
	int scaling = this->raw_values["scaling"];
	autoranging_t autoranging = this->autoranging_data.at(scaling);
	int autoranging_max = this->autoranging_data.size() - 1;

	switch(this->state)
	{
		case(state_t::init):
		case(state_t::finished):
		{
			int opcode = enum_integer(autoranging.timing) | enum_integer(autoranging.gain);

			device->send(enum_integer(reg_t::conf), this->u16low(opcode), this->u16high(opcode));

			this->state = state_t::measuring;

			break;
		}

		case(state_t::measuring):
		{
			int raw_als;
			float lux, raw_lux;

			this->state = state_t::finished;

			device->send_receive(enum_integer(reg_t::white), 3, data);
			this->raw_values["white"] = this->u16le(data);

			device->send_receive(enum_integer(reg_t::als), 3, data);
			raw_als = this->u16le(data);
			this->raw_values["als"] = raw_als;

			if((raw_als < autoranging.lower) && (scaling > 0))
			{
				scaling--;
				this->raw_values["scaling"] = scaling;
				this->raw_values["scalings down"]++;
				break;
			}

			if((raw_als >= autoranging.upper) && (scaling < autoranging_max))
			{
				scaling++;
				this->raw_values["scaling"] = scaling;
				this->raw_values["scalings up"]++;
				break;
			}

			raw_lux = raw_als * autoranging.factor;

			this->raw_values["raw lux"] = raw_lux * 1000;

			lux = (raw_lux * raw_lux * raw_lux * raw_lux * 6.0135e-13f)
					- (raw_lux * raw_lux * raw_lux * 9.3924e-09f)
					+ (raw_lux * raw_lux * 8.1488e-05f)
					+ (raw_lux * 1.0023e+00f);

			this->update("visible light", lux, "lx", 3);

			break;
		}

		default:
		{
			throw(hard_exception("veml7700: invalid state"));
		}
	}
}

const SensorLTR390::autorangings_t SensorLTR390::autoranging_data
{
	{
		.resolution = meas_rate_t::resolution_20,
		.rate = meas_rate_t::rate_500,
		.gain = gain_t::gain_18,
		.raw_lower = 0, .raw_upper = 1200,
		.gain_factor = 18.0f, .integration_time_factor = 4.0f,
		.lux_lower = 0.0083f, .lux_upper = 10.0f,
		.precision = 3,
	},
	{
		.resolution = meas_rate_t::resolution_20,
		.rate = meas_rate_t::rate_500,
		.gain = gain_t::gain_1,
		.raw_lower = 67, .raw_upper = 6667,
		.gain_factor = 1.0f, .integration_time_factor = 4.0f,
		.lux_lower = 10.0f, .lux_upper = 1000.0f,
		.precision = 1,
	},
	{
		.resolution = meas_rate_t::resolution_18,
		.rate = meas_rate_t::rate_500,
		.gain = gain_t::gain_1,
		.raw_lower = 1667, .raw_upper = 1048575,
		.gain_factor = 1.0f, .integration_time_factor = 1.0f,
		.lux_lower = 100.0f, .lux_upper = 629145.0f,
		.precision = 0,
	},
};

bool SensorLTR390::probe(I2c &i2c, int module, int bus, int address, bool restricted)
{
	return(i2c.probe(module, bus, address));
}

SensorLTR390::SensorLTR390(Log& log_in, I2C::Device* device_in) : SensorI2C(log_in, this->basic_name(), device_in)
{
	I2C::data_t data;

	this->device->send_receive(enum_integer(reg_t::part_id), 1, data);

	if(data.at(0) != enum_integer(part_id_t::part_number | part_id_t::revision))
		throw(transient_exception());

	this->device->send(enum_integer(reg_t::main_ctrl), enum_integer(main_ctrl_t::sw_reset));

	this->raw_values["scaling"] = 0;
}

void SensorLTR390::_poll()
{
	I2C::data_t data;

	switch(this->state)
	{
		case(state_t::init):
		{
			this->state = state_t::reset;
			break;
		}

		case(state_t::reset):
		{
			this->device->send_receive(enum_integer(reg_t::main_ctrl), 1, data);

			if(data.at(0) & enum_integer(main_ctrl_t::sw_reset))
				break;

			this->state = state_t::ready;

			break;
		}

		case(state_t::ready):
		case(state_t::finished_uv_light):
		{
			unsigned int scaling;
			unsigned char meas_rate, gain;

			scaling = this->raw_values["scaling"];

			meas_rate  =	enum_integer(this->autoranging_data[scaling].resolution);
			meas_rate |=	enum_integer(this->autoranging_data[scaling].rate);
			gain =			enum_integer(this->autoranging_data[scaling].gain);

			this->device->send(enum_integer(reg_t::meas_rate), meas_rate);
			this->device->send(enum_integer(reg_t::gain), gain);
			this->device->send(enum_integer(reg_t::main_ctrl), enum_integer(main_ctrl_t::als_uvs_enable));
			this->state = state_t::measuring_visible_light;

			break;
		}

		case(state_t::measuring_visible_light):
		{
			unsigned int raw, scaling, max;
			float gain_factor, integration_time_factor, lux;

			this->device->send_receive(enum_integer(reg_t::main_status), 1, data);

			if(!(data.at(0) & enum_integer(main_status_t::data_status)))
			{
				this->raw_values["visible light timeouts"]++;
				break;
			}

			this->device->send_receive(enum_integer(reg_t::als_data_0), 3, data);

			this->raw_values["raw visible light"] = raw = this->u20tople(data);

			scaling = this->raw_values["scaling"];
			max = this->autoranging_data.size() - 1;

			if((scaling < max) && (raw > this->autoranging_data[scaling].raw_upper))
			{
				scaling++;
				this->raw_values["scalings up"]++;
				this->raw_values["scaling"] = scaling;
				this->state = state_t::ready;
				break;
			}

			if((scaling > 0) && (raw < this->autoranging_data[scaling].raw_lower))
			{
				scaling--;
				this->raw_values["scalings down"]++;
				this->raw_values["scaling"] = scaling;
				this->state = state_t::ready;
				break;
			}

			gain_factor = this->autoranging_data[scaling].gain_factor;
			integration_time_factor = this->autoranging_data[scaling].integration_time_factor;

			lux = (static_cast<float>(raw) * this->normalisation_factor) / (gain_factor * integration_time_factor);
			this->update("visible light", lux, "lx", this->autoranging_data[scaling].precision);
			this->state = state_t::finished_visible_light;

			break;
		}

		case(state_t::finished_visible_light):
		{
			this->device->send(enum_integer(reg_t::main_ctrl), enum_integer(main_ctrl_t::als_uvs_enable | main_ctrl_t::uvs_mode));

			this->state = state_t::measuring_uv_light;

			break;
		}

		case(state_t::measuring_uv_light):
		{
			unsigned int raw;
			float uvi;

			this->device->send_receive(enum_integer(reg_t::main_status), 1, data);

			if(!(data.at(0) & enum_integer(main_status_t::data_status)))
			{
				this->raw_values["uv light timeouts"]++;
				break;
			}

			this->device->send_receive(enum_integer(reg_t::uvs_data_0), 3, data);

			this->raw_values["raw uv light"] = raw = this->u20tople(data);

			uvi = static_cast<float>(raw) / this->uv_sensitivity;

			this->update("uv light index", uvi, "UVI", 1);
			this->state = state_t::finished_uv_light;

			break;
		}

		default:
		{
			throw(hard_exception("invalid state"));
		}
	}
}

SensorInternal::SensorInternal(Log& log_in, std::string_view name_in, int dummy_module_in) : Sensor(log_in, name_in), dummy_module(dummy_module_in)
{
}

SensorInternal::~SensorInternal()
{
}

void SensorInternal::_address(int &module, int &bus, int &address)
{
	module = this->dummy_module;
	bus = 0;
	address = 0;
}

SensorInternalTemperature::SensorInternalTemperature(Log& log_in, int dummy_module_in) : SensorInternal(log_in, this->basic_name(), dummy_module_in)
{
	esp_err_t rv;
	static const temperature_sensor_config_t config
	{
		.range_min = -10,
		.range_max = 60,
		.clk_src = TEMPERATURE_SENSOR_CLK_SRC_DEFAULT,
		.flags =
		{
			.allow_pd = false
		},
	};

	if((rv = temperature_sensor_install(&config, &this->handle)) != ESP_OK)
	{
		this->log.log_esperr(rv, "temperature_sensor_install");
		throw(transient_exception());
	}

	if((rv = temperature_sensor_enable(this->handle)) != ESP_OK)
	{
		this->log.log_esperr(rv, "temperature_sensor_enable");
		throw(transient_exception());
	}
}

void SensorInternalTemperature::_poll()
{
	esp_err_t rv;
	float temperature;

	if((rv = temperature_sensor_get_celsius(this->handle, &temperature)) != ESP_OK)
		throw(transient_exception());

	this->update("temperature", temperature, "degrees", 0);
	this->state = state_t::measuring;
}
