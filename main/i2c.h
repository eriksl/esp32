#pragma once

#include "log.h"
#include "config.h"

#include <string>
#include <map>
#include <mutex>

#include <driver/i2c_master.h>

namespace I2C
{
	using data_t = std::basic_string<unsigned char>;

	class I2c;
	class Module;
	class MainModule;
	class ULPModule;
	class Bus;
	class Device;

	I2c& get();

	class I2c final
	{
		public:

			explicit I2c() = delete;
			explicit I2c(const I2c &) = delete;
			explicit I2c(const I2c &&) = delete;
			explicit I2c(Log &, Config &);
			I2c& operator =(const I2c *) = delete;

			bool probe(int module, int bus, int address);
			Device* find(int module, int bus, int address);
			Device* new_device(int module_index, int bus_index, int address, std::string_view name);
			int speed(int module_index);
			void speed(int module_index, int speed);
			void info(std::string &);
			std::map<int, Module *>& modules();

		private:

			Log &log;
			Config &config;
			std::map<int, I2C::Module *> _modules;
	};

	class Module
	{
		friend class I2c;
		friend class MainModule;
		friend class ULPModule;
		friend class Bus;
		friend class Device;

		public:

			std::map<int, Bus *>& buses();
			bool restricted();

		private:

			static constexpr int bus_mux_address = 0x70;
			static constexpr int address_shift = 1;
			static constexpr int write_flag = 0x00;
			static constexpr int read_flag = 0x01;

			explicit Module() = delete;
			explicit Module(const Module &) = delete;
			explicit Module(const Module &&) = delete;
			explicit Module(Log &, Config &, I2c &, int index, int subindex, int sda, int scl);
			Module& operator =(const Module &) = delete;
			virtual ~Module();

			Log &log;
			Config &config;
			int sda;
			int scl;
			int speed_khz;
			int module_index;
			int device_port;
			i2c_master_bus_handle_t module_handle;
			i2c_master_dev_handle_t device_handle;
			bool has_mux;
			std::string name;
			std::map<int, Bus *> _buses;
			std::mutex mutex;
			I2c &root_ref;

			bool probe(int bus, int address);
			Device* find(int bus, int address);
			void send(int bus, int address, const data_t &);
			void receive(int bus, int address_in, int length, data_t &);
			void send_receive(int bus, int address, const data_t &, int length, data_t &);
			bool probe_mux();
			void create_buses(int amount);
			I2c &root();
			Bus *bus(int index);
			void speed(int);
			int speed();
			std::string info();
			int index();
			void select_bus(int bus);

			virtual void _set_mux(int bus_bits) = 0;
			virtual bool _probe(int address) = 0;
			virtual void _send(int address, const data_t &) = 0;
			virtual void _receive(int address, int length, data_t &) = 0;
			virtual void _send_receive(int address, const data_t &, int length, data_t &) = 0;
			virtual bool _restricted() = 0;
	};

	class MainModule final : private Module
	{
		friend class I2c;

		explicit MainModule() = delete;
		explicit MainModule(const MainModule &) = delete;
		explicit MainModule(Log &, Config &, I2c &, int, int, int, int);

		void _set_mux(int bus_bits) override;
		bool _probe(int address) override;
		void _send(int address, const data_t &) override;
		void _receive(int address, int length, data_t &) override;
		void _send_receive(int address, const data_t &, int length, data_t &) override;
		bool _restricted() override;
	};

	class ULPModule final : private Module
	{
		friend class I2c;

		explicit ULPModule() = delete;
		explicit ULPModule(const ULPModule &) = delete;
		explicit ULPModule(Log &, Config &, I2c &, int, int, int, int);

		void _set_mux(int bus_bits) override;
		bool _probe(int address) override;
		void _send(int address, const data_t &) override;
		void _receive(int address_in, int length, data_t &) override;
		void _send_receive(int address, const data_t &, int length, data_t &) override;
		bool _restricted() override;
	};

	class Bus final
	{
		friend class I2c;
		friend class Module;
		friend class Device;

		explicit Bus() = delete;
		explicit Bus(const Bus &) = delete;
		explicit Bus(Log &, Module &, int index, std::string_view name);

		Device* new_device(int address, std::string_view name);
		Device* find(int address);
		std::string info();
		int index();
		void delete_device(Device *);
		Module &module();
		void send(int address, const data_t &);
		void receive(int address, int length, data_t &);
		void send_receive(int address, const data_t&, int length, data_t &);

		std::map<int, Device *> devices;
		Log &log;
		Module &module_ref;
		int bus_index;
		std::string name;
	};

	class Device final
	{
		friend class Bus;

		public:

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
			~Device();

		private:

			explicit Device() = delete;
			explicit Device(const Device &) = delete;
			explicit Device(Log &, Bus &, int address, std::string_view name);

			Log &log;
			Bus &bus_ref;
			int address;
			std::string name;
	};
}

using I2c = I2C::I2c;
