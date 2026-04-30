#pragma once

#include "log.h"
#include "config.h"

#include <cstdint>

#include <string>
#include <deque>
#include <mutex>
#include <memory>

class SPI final
{
	public:

		using data_t = std::basic_string<unsigned char>;
		using rodata_t = std::basic_string_view<unsigned char>;

		struct transfer_t
		{
			struct
			{
				struct
				{
					int bits = 0;
					std::uint16_t data = 0;
				} command;

				struct
				{
					int bits = 0;
					std::uint64_t data = 0;
				} address;

				const rodata_t* data = nullptr;
			} send;

			struct
			{
				int length = 0;
				data_t* data = nullptr;
			} receive;
		};

		class Module final
		{
			public:

				explicit Module() = delete;
				explicit Module(const Module &) = delete;
				explicit Module(const Module &&) = delete;
				explicit Module(Log&, Config&, int module, int default_cs, int default_sck, int default_mosi, int default_miso);
				Module& operator =(const Module &) = delete;
				~Module();

			private:

				class PrivateData;

				Log& log;
				Config& config;
				std::mutex mutex;
				int module, cs, sck, mosi, miso, _speed;
				std::unique_ptr<PrivateData> private_data;

				int get_config(const std::string& name, int default_value);

			public:

				std::string info();
				int max_transaction_length();
				int speed();
				void speed(int);
				void transfer(const transfer_t&);
		};

		explicit SPI() = delete;
		explicit SPI(const SPI &) = delete;
		explicit SPI(const SPI &&) = delete;
		SPI& operator =(const SPI *) = delete;
		explicit SPI(Log &, Config &);

		SPI& get();
		Module& operator[](int);

	private:

		static SPI* singleton;
		Log &log;
		Config &config;
		std::deque<Module> modules;
};
