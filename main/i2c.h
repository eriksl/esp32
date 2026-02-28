#pragma once

#include "log.h"
#include "config.h"

#include <string>
#include <map>
#include <mutex>

#include <driver/i2c_master.h>

class I2C final
{
	public:

		class Module;
		class Bus;
		class Device;

		typedef std::basic_string<unsigned char> data_t;

		static constexpr int i2c_bus_mux_address = 0x70;
		static constexpr int i2c_address_shift = 1;
		static constexpr int i2c_write_flag = 0x00;
		static constexpr int i2c_read_flag = 0x01;

		explicit I2C() = delete;
		explicit I2C(const I2C &) = delete;
		explicit I2C(Log &, Config &);

		static I2C &get();

		bool probe(int module, int bus, int address);
		Device* find(int module, int bus, int address);
		Device* new_device(int module_index, int bus_index, int address, std::string_view name);
		int speed(int module_index);
		void speed(int module_index, int speed);
		void info(std::string &);
		std::map<int, Module *>& modules();

		class Module
		{
			public:

				explicit Module() = delete;
				explicit Module(const Module &) = delete;
				explicit Module(Log &, Config &, I2C &, int index, int subindex, int sda, int scl);
				virtual ~Module();
				
				bool probe(int bus, int address);
				Device* find(int bus, int address);
				void send(int bus, int address, const data_t &);
				void receive(int bus, int address_in, int length, data_t &);
				void send_receive(int bus, int address, const data_t &, int length, data_t &);
				bool restricted();

				I2C &root();
				Bus *bus(int index);
				void speed(int);
				int speed();
				std::string info();
				int index();
				std::map<int, Bus *>& buses();

			protected:

				Log &log;
				Config &config;
				std::string name;
				int module_index;
				int device_port;
				int sda;
				int scl;
				bool has_mux;
				std::map<int, Bus *> _buses;
				int speed_khz;
				i2c_master_bus_handle_t module_handle;
				i2c_master_dev_handle_t device_handle;

				bool probe_mux();
				void create_buses(int amount);
				void select_bus(int bus);

				virtual void _set_mux(int bus_bits) = 0;
				virtual bool _probe(int address) = 0;
				virtual void _send(int address, const data_t &) = 0;
				virtual void _receive(int address, int length, data_t &) = 0;
				virtual void _send_receive(int address, const data_t &, int length, data_t &) = 0;
				virtual bool _restricted() = 0;

			private:

				std::mutex mutex;
				I2C &root_ref;
		};

		class MainModule final : public Module
		{
			public:

				explicit MainModule() = delete;
				explicit MainModule(const MainModule &) = delete;
				explicit MainModule(Log &, Config &, I2C &, int, int, int, int);

				void _set_mux(int bus_bits) override;
				bool _probe(int address) override;

			private:

				void _send(int address, const data_t &) override;
				void _receive(int address, int length, data_t &) override;
				void _send_receive(int address, const data_t &, int length, data_t &) override;
				bool _restricted() override;
		};

		class ULPModule final : public Module
		{
			public:

				explicit ULPModule() = delete;
				explicit ULPModule(const ULPModule &) = delete;
				explicit ULPModule(Log &, Config &, I2C &, int, int, int, int);

				void _set_mux(int bus_bits) override;
				bool _probe(int address) override;

			private:

				void _send(int address, const data_t &) override;
				void _receive(int address_in, int length, data_t &) override;
				void _send_receive(int address, const data_t &, int length, data_t &) override;
				bool _restricted() override;
		};

		class Bus final
		{
			public:

				explicit Bus() = delete;
				explicit Bus(const Bus &) = delete;
				explicit Bus(Log &, Module &, int index, std::string_view name);

				Device* new_device(int address, std::string_view name);
				Device* find(int address);
				std::string info();
				int index();
				void delete_device(Device *);

				void send(int address, const data_t &);
				void receive(int address, int length, data_t &);
				void send_receive(int address, const data_t&, int length, data_t &);

				Module &module();

			private:

				std::map<int, Device *> devices;
				Log &log;
				Module &module_ref;
				int bus_index;
				std::string name;
		};

		class Device final
		{
			public:

				explicit Device() = delete;
				explicit Device(const Device &) = delete;
				explicit Device(Log &, Bus &, int address, std::string_view name);
				~Device();

				std::string info();
				void data(int &module, int &bus, int &address, std::string &name);
				std::string data();
				void send(const data_t &);
				void send(unsigned char b1);
				void send(unsigned char b1, unsigned char b2);
				void send(unsigned char b1, unsigned char b2, unsigned char b3);
				void receive(int length, data_t &);
				void send_receive(const data_t &, int length, data_t &);
				void send_receive(unsigned char b1, int length, data_t &);
				void send_receive(unsigned char b1, unsigned char b2, int length, data_t &in);
				void send_receive(unsigned char b1, unsigned char b2, unsigned char b3, int length, data_t &in);

			private:

				Log &log;
				Bus &bus_ref;
				int address;
				std::string name;
		};

	private:

		static I2C *singleton;
		Log &log;
		Config &config;
		std::map<int, Module *> _modules;
};
