#pragma once

#include "log.h"
#include "i2c.h"

#include <time.h>

#include <deque>
#include <array>
#include <utility>

#include <driver/temperature_sensor.h>

namespace SENSORS
{
	class Sensor;

	class Sensors final
	{
		public:

			explicit Sensors() = delete;
			explicit Sensors(const Sensors &) = delete;
			explicit Sensors(const Sensors &&) = delete;
			explicit Sensors(Log &, I2c &);
			Sensors& operator =(const Sensors &) = delete;

			static Sensors *singleton;

			void run();
			std::string info(int index = -1);
			std::string dump(int index = -1);
			std::string stats();
			std::string json();

		private:

			Log &log;
			I2c &i2c;

			std::deque<Sensor *> _sensors;
			std::map<std::string, int> _stats;

			void run_thread();
			template<typename T> void detect_internal(int dummy_module);
			template<typename T> void detect_i2c();
	};

	class Sensor
	{
		friend class Sensors;

		protected:

			explicit Sensor() = delete;
			explicit Sensor(const Sensor &) = delete;
			explicit Sensor(const Sensor &&) = delete;
			explicit Sensor(Log &, std::string_view name);
			Sensor & operator =(const Sensor &) = delete;
			virtual ~Sensor();

			void poll();
			std::string info(int index);
			std::string dump(int index);
			std::string json(int index);

			virtual void _address(int &module, int &bus, int &address) = 0;
			virtual void _poll() = 0;

			enum class state_t : unsigned char
			{
				init,
				reset,
				ready,
				measuring,
				measuring_temperature,
				measuring_humidity,
				measuring_visible_light,
				measuring_uv_light,
				finished,
				finished_temperature,
				finished_humidity,
				finished_visible_light,
				finished_uv_light,
			};

			using raw_values_t = std::map<std::string, int>;

			Log &log;
			std::string name;
			state_t state;
			raw_values_t raw_values;

			static unsigned char u8(const I2C::data_t &data, int offset = 0);
			static signed char s8(const I2C::data_t &data, int offset = 0);
			static unsigned int u12topbe(const I2C::data_t &data, int offset = 0);
			static unsigned int u12bottomle(const I2C::data_t &data, int offset = 0);
			static unsigned int u16be(const I2C::data_t &data, int offset = 0);
			static unsigned int u16le(const I2C::data_t &data, int offset = 0);
			static signed int s16le(const I2C::data_t &data, int offset = 0);
			static unsigned int u20tople(const I2C::data_t &data, int offset = 0);
			static unsigned int u20topbe(const I2C::data_t &data, int offset = 0);
			static unsigned int u20bottombe(const I2C::data_t &data, int offset = 0);
			static unsigned char u16low(int u16);
			static unsigned char u16high(int u16);

			std::string time_string(time_t stamp);
			void update(const std::string &id, float value, const std::string &unity, int precision);

		private:

			struct value_t
			{
				time_t stamp;
				float value;
				std::string unity;
				int precision;
			};

			using values_t = std::map<std::string, value_t>;

			values_t values;
	};

	class SensorI2C : protected Sensor
	{
		protected:

			explicit SensorI2C() = delete;
			explicit SensorI2C(const SensorI2C &) = delete;
			explicit SensorI2C(const SensorI2C &&) = delete;
			explicit SensorI2C(Log &, std::string_view name, I2C::Device *);
			SensorI2C & operator =(const SensorI2C &) = delete;
			virtual ~SensorI2C();

			virtual void _address(int &module, int &bus, int &address) override;
			virtual void _poll() = 0;

			I2C::Device *device;
	};

	class SensorGeneric75 final : private SensorI2C
	{
		friend class Sensors;

		private:

			explicit SensorGeneric75() = delete;
			explicit SensorGeneric75(const SensorGeneric75 &) = delete;
			explicit SensorGeneric75(const SensorGeneric75 &&) = delete;
			explicit SensorGeneric75(Log &, I2C::Device *);
			SensorGeneric75& operator =(const SensorGeneric75 &) = delete;

			enum class model_t : unsigned char
			{
				generic75,
				tmp75,
				lm75,
			};

			enum class reg_t : unsigned char
			{
				temp = 0x00,
				conf = 0x01,
			};

			enum class lm75_reg_t : unsigned char
			{
				temp =	0x00,
				conf =	0x01,
				thyst =	0x02,
				tos =	0x03,
			};

			enum class tmp75_reg_t : unsigned char
			{
				temp =	0x00,
				conf =	0x01,
				tlow =	0x02,
				thigh = 0x03,
			};

			enum class lm75_conf_t : unsigned char
			{
				reserved =		0b1110'0000,
				f_queue =		0b0001'1000,
				pol =			0b0000'0100,
				comp_int =		0b0000'0010,
				shutdown =		0b0000'0001,
				no_shutdown = 	0b0000'0000,
			};

			enum class tmp75_conf_t : unsigned char
			{
				os =			0b100'00000,
				res_9 =			0b000'00000,
				res_10 =		0b001'00000,
				res_11 =		0b010'00000,
				res_12 =		0b011'00000,
				f_queue =		0b000'11000,
				pol =			0b000'00100,
				tm =			0b000'00010,
				shutdown =		0b000'00001,
				no_shutdown =	0b000'00000,
			};

			enum class lm75_probe_t : unsigned char
			{
				thyst_h =	0x4b,
				thyst_l =	0x00,
				tos_1_h =	0x50,
				tos_1_l =	0x00,
				tos_2_h =	0x00,
				tos_2_l =	0x00,
				conf =		0b0000'0000,
				conf_mask =	0b1001'1111,
			};

			enum class tmp75_probe_t : unsigned char
			{
				offset_04 =		0x04,
				offset_a1 =		0xa1,
				offset_a2 =		0xa2,
				offset_aa =		0xaa,
				offset_ac =		0xac,

				tl_h =			0x4b,
				tl_l =			0x00,
				th_h =			0x50,
				th_l =			0x00,
				conf =			0b0000'0000,
				conf_mask = 	0b1000'0000,
			};

			static constexpr const std::array<int,1> addresses {0x48};
			static std::string basic_name() { return("compat75"); };
			static bool probe(I2c &, int module, int bus, int address, bool);

			void _poll() override;

			bool probe_register(int);

			model_t model;
	};

	class SensorMCP9808 final : private SensorI2C
	{
		friend class Sensors;

		private:

			explicit SensorMCP9808() = delete;
			explicit SensorMCP9808(const SensorMCP9808 &) = delete;
			explicit SensorMCP9808(const SensorMCP9808 &&) = delete;
			explicit SensorMCP9808(Log &, I2C::Device *);
			SensorMCP9808& operator =(const SensorMCP9808 &) = delete;

			enum class reg_t : unsigned char
			{
				rfu =			0b00000000,
				config =		0b00000001,
				alert_u =		0b00000010,
				alert_l =		0b00000011,
				critical =		0b00000100,
				temperature =	0b00000101,
				manufacturer =	0b00000110,
				device =		0b00000111,
				resolution =	0b00001000,
			};

			enum class config_t : unsigned int
			{
				hyst_0_0 =		0b0000000000000000,
				hyst_1_5 =		0b0000001000000000,
				hyst_3_0 =		0b0000010000000000,
				hyst_6_0 =		0b0000011000000000,
				shutdown =		0b0000000100000000,
				lock_crit =		0b0000000010000000,
				lock_wind =		0b0000000001000000,
				int_clear =		0b0000000000100000,
				alert_status =	0b0000000000010000,
				alert_control =	0b0000000000001000,
				alert_select =	0b0000000000000100,
				alert_pol =		0b0000000000000010,
				alert_mode =	0b0000000000000001,
			};

			enum class resolution_t : unsigned char
			{
				res_0_5 =		0b00000000,
				res_0_25 =		0b00000001,
				res_0_125 =		0b00000010,
				res_0_0625 =	0b00000011,
			};

			enum class id_t : unsigned char
			{
				manufacturer_0 =	0x00,
				manufacturer_1 =	0x54,
				device =			0x04,
			};

			static std::string basic_name() { return("mcp9808"); };
			static constexpr const std::array<int,1> addresses {0x18};
			static bool probe(I2c &, int module, int bus, int address, bool);

			void _poll() override;

			bool probe_register(int);
	};

	class SensorHTU21 final : public SensorI2C
	{
		friend class Sensors;

		private:

			explicit SensorHTU21() = delete;
			explicit SensorHTU21(const SensorHTU21 &) = delete;
			explicit SensorHTU21(const SensorHTU21 &&) = delete;
			explicit SensorHTU21(Log &, I2C::Device *);
			SensorHTU21& operator =(const SensorHTU21 &) = delete;

			static std::string basic_name() { return("htu21"); };
			static constexpr const std::array<int,1> addresses {0x40};
			static bool probe(I2c &, int module, int bus, int address, bool);

			void _poll() override;

			enum class cmd_t : unsigned char
			{
				meas_temp_hold_master =		0xe3,
				meas_hum_hold_master =		0xe5,
				write_user =				0xe6,
				read_user =					0xe7,
				meas_temp_no_hold_master =	0xf3,
				meas_hum_no_hold_master =	0xf5,
				reset =						0xfe,
			};

			enum class reg_t : unsigned char
			{
				rh12_temp14 =			0b0000'0000,
				rh8_temp12 =			0b0000'0001,
				rh10_temp13 =			0b1000'0000,
				rh11_temp11 =			0b1000'0001,
				bat_stat =				0b0100'0000,
				reserved =				0b0011'1000,
				heater_enable =			0b0000'0100,
				otp_reload_disable =	0b0000'0010,
			};

			enum class status_t : unsigned int
			{
				mask =						0b0000'0011,
				measure_temperature =		0b0000'0000,
				measure_humidity =			0b0000'0010,
			};

			unsigned int get_data();
	};

	class SensorAsair final : private SensorI2C
	{
		friend class Sensors;

		private:

			explicit SensorAsair() = delete;
			explicit SensorAsair(const SensorAsair &) = delete;
			explicit SensorAsair(const SensorAsair &&) = delete;
			explicit SensorAsair(Log &, I2C::Device *);
			SensorAsair& operator =(const SensorAsair &) = delete;

			enum class model_t : unsigned char
			{
				asair,
				aht10,
				aht20,
			};

			enum class cmd_t : unsigned char
			{
				measure_0 =				0xac,
				measure_1 =				0x33,
				measure_2 =				0x00,
				get_all_data =			0x71,
				reset =					0xba,
			};

			enum class data_off_t : unsigned char
			{
				state =					0x00,
				humidity_0 =			0x01,
				humidity_1 =			0x02,
				humidity_2 =			0x03,
				temperature_0 =			0x03,
				temperature_1 =			0x04,
				temperature_2 =			0x05,
				crc =					0x06,
				size =					0x07,
			};

			enum class status_t : unsigned char
			{
				busy =					1 << 7,
				calibration_ready =		1 << 3,
			};

			enum class aht10_cal_init_t : unsigned char
			{
				cal_1 =	0xe1,
				cal_2 =	0x08,
				cal_3 =	0x00,
			};

			enum class aht20_cal_init_t : unsigned char
			{
				cal_1 =	0xbe,
				cal_2 =	0x08,
				cal_3 =	0x00,
			};

			static std::string basic_name() { return("asair"); };
			static constexpr const std::array<int,1> addresses {0x38};
			static bool probe(I2c &, int module, int bus, int address, bool);

			void _poll() override;

			model_t model;
	};

	class SensorSHT3X : private SensorI2C
	{
		friend class Sensors;

		private:

			explicit SensorSHT3X() = delete;
			explicit SensorSHT3X(const SensorSHT3X &) = delete;
			explicit SensorSHT3X(const SensorSHT3X &&) = delete;
			explicit SensorSHT3X(Log &, I2C::Device *);
			SensorSHT3X& operator =(const SensorSHT3X &) = delete;

			static std::string basic_name() { return("sht3x"); };
			static constexpr const std::array<int,1> addresses {0x44};
			static bool probe(I2c &, int module, int bus, int address, bool);

			void _poll() override;

			enum class cmd_t : unsigned int
			{
				single_meas_clock_high =		0x2c06,
				single_meas_clock_medium =		0x2c0d,
				single_meas_clock_low =			0x2c10,

				single_meas_noclock_high =		0x2400,
				single_meas_noclock_medium =	0x240b,
				single_meas_noclock_low =		0x2416,

				auto_meas_high_05 =				0x2032,
				auto_meas_medium_05 =			0x2024,
				auto_meas_low_05 =				0x202f,

				auto_meas_high_1 =				0x2130,
				auto_meas_medium_1 =			0x2126,
				auto_meas_low_1 =				0x212d,

				auto_meas_high_2 =				0x2236,
				auto_meas_medium_2 =			0x2220,
				auto_meas_low_2 =				0x222b,

				auto_meas_high_4 =				0x2334,
				auto_meas_medium_4 =			0x2322,
				auto_meas_low_4 =				0x2329,

				auto_meas_high_10 =				0x2737,
				auto_meas_medium_10 =			0x2721,
				auto_meas_low_10 =				0x272a,

				fetch_data =					0xe000,
				art =							0x2b32,
				cmd_break =						0x3093,
				reset =							0x30a2,
				heater_en =						0x306d,
				heater_dis =					0x3066,
				read_status =					0xf32d,
				clear_status =					0x3041,
			};

			enum class status_t : unsigned int
			{
				none =				0x00,
				write_checksum =	(1 << 0),
				command_status =	(1 << 1),
				reset_detected =	(1 << 4),
				temp_track_alert =	(1 << 10),
				hum_track_alert =	(1 << 11),
				heater =			(1 << 13),
				alert =				(1 << 15),
			};

			void send_command(cmd_t cmd);
			unsigned int receive_command(cmd_t cmd);
	};

	class SensorSHT4X : private SensorI2C
	{
		friend class Sensors;

		private:

			explicit SensorSHT4X() = delete;
			explicit SensorSHT4X(const SensorSHT4X &) = delete;
			explicit SensorSHT4X(const SensorSHT4X &&) = delete;
			explicit SensorSHT4X(Log &, I2C::Device *);
			SensorSHT4X& operator =(const SensorSHT4X &) = delete;

			enum class cmd_t : unsigned char
			{
				measure_high_precision =	0xfd,
				measure_medium_precision =	0xf6,
				measure_low_precision =		0xe0,
				read_serial =				0x89,
				soft_reset =				0x94,
				heater_200mw_1s =			0x39,
				heater_200mw_0_1s =			0x32,
				heater_110mw_1s =			0x2f,
				heater_110mw_0_1s =			0x24,
				heater_20mw_1s =			0x1e,
				heater_20mw_0_1s =			0x15,
			};

			static std::string basic_name() { return("sht4x"); };
			static constexpr const std::array<int,1> addresses {0x44};
			static bool probe(I2c &, int module, int bus, int address, bool);

			void _poll() override;

			unsigned int receive_command(cmd_t cmd);
	};

	class SensorHDC1080 : private SensorI2C
	{
		friend class Sensors;

		private:

			explicit SensorHDC1080() = delete;
			explicit SensorHDC1080(const SensorHDC1080 &) = delete;
			explicit SensorHDC1080(const SensorHDC1080 &&) = delete;
			explicit SensorHDC1080(Log &, I2C::Device *);
			SensorHDC1080& operator =(const SensorHDC1080 &) = delete;

			static std::string basic_name() { return("hdc1080"); };
			static constexpr const std::array<int,1> addresses {0x40};
			static bool probe(I2c &, int module, int bus, int address, bool);

			void _poll() override;

			enum class reg_t : unsigned char
			{
				data_temp =	0x00,
				data_hum =	0x01,
				conf =		0x02,
				serial1 =	0xfb,
				serial2 =	0xfc,
				serial3 =	0xfd,
				man_id =	0xfe,
				dev_id =	0xff,
			};

			enum class id_t : unsigned int
			{
				manufacturer =		0x5449,
				device =			0x1050,
			};

			enum class conf_t : unsigned int
			{
				rst =		0b1000'0000'0000'0000,
				reservd0 =	0b0100'0000'0000'0000,
				heat =		0b0010'0000'0000'0000,
				mode_two =	0b0001'0000'0000'0000,
				mode_one =	0b0000'0000'0000'0000,
				btst =		0b0000'1000'0000'0000,
				tres_11 =	0b0000'0100'0000'0000,
				tres_14 =	0b0000'0000'0000'0000,
				hres_8 =	0b0000'0010'0000'0000,
				hres_11 =	0b0000'0001'0000'0000,
				hres_14 =	0b0000'0000'0000'0000,
				reservd1 =	0b0000'0000'1111'1111,
			};

			void write_word(reg_t reg, conf_t value);
	};

	class SensorBMX280 : protected SensorI2C
	{
		protected:

			explicit SensorBMX280() = delete;
			explicit SensorBMX280(const SensorBMX280 &) = delete;
			explicit SensorBMX280(const SensorBMX280 &&) = delete;
			explicit SensorBMX280(Log &, I2C::Device *);
			SensorBMX280& operator =(const SensorBMX280 &) = delete;

			enum class reg_t : unsigned char
			{
				id =					0xd0,
				reset =					0xe0,
				ctrl_hum =				0xf2,
				status =				0xf3,
				ctrl_meas =				0xf4,
				config =				0xf5,
				adc =					0xf7,
				adc_pressure_msb =		0xf7,
				adc_pressure_lsb =		0xf8,
				adc_pressure_xlsb =		0xf9,
				adc_temperature_msb =	0xfa,
				adc_temperature_lsb =	0xfb,
				adc_temperature_xlsb =	0xfc,
				adc_humidity_msb =		0xfd,
				adc_humidity_lsb =		0xfe,
			};

			enum class id_t : unsigned char
			{
				id_bmp280 =		0x58,
				id_bme280 =		0x60,
			};

			enum class reset_t : unsigned char
			{
				reset_value =	0xb6,
			};

			enum class reg_status_t : unsigned char
			{
				measuring =		0b00001'000,
				im_update =		0b0000'0001,
			};

			enum class reg_ctrl_hum_t : unsigned char
			{
				osrs_h_skip =	0b0000'0000,
				osrs_h_1 =		0b0000'0001,
				osrs_h_2 =		0b0000'0010,
				osrs_h_4 =		0b0000'0011,
				osrs_h_8 =		0b0000'0100,
				osrs_h_16 =		0b0000'0101,
			};

			enum class reg_ctrl_meas_t : unsigned char
			{
				osrs_t_skip =	0b0000'0000,
				osrs_t_1 =		0b0010'0000,
				osrs_t_2 =		0b0100'0000,
				osrs_t_4 =		0b0110'0000,
				osrs_t_8 =		0b1000'0000,
				osrs_t_16 =		0b1010'0000,
				osrs_p_skip =	0b0000'0000,
				osrs_p_1 =		0b0000'0100,
				osrs_p_2 =		0b0000'1000,
				osrs_p_4 =		0b0000'1100,
				osrs_p_8 =		0b0001'0000,
				osrs_p_16 =		0b0001'0100,
				mode_mask =		0b0000'0011,
				mode_sleep =	0b0000'0000,
				mode_forced =	0b0000'0010,
				mode_normal =	0b0000'0011,
			};

			enum class reg_config_t : unsigned char
			{
				t_sb_05 =		0b0000'0000,
				t_sb_62 =		0b0010'0000,
				t_sb_125 =		0b0100'0000,
				t_sb_250 =		0b0110'0000,
				t_sb_500 =		0b1000'0000,
				t_sb_1000 =		0b1010'0000,
				t_sb_10000 =	0b1100'0000,
				t_sb_20000 =	0b1110'0000,
				filter_off =	0b0000'0000,
				filter_2 =		0b0000'0100,
				filter_4 =		0b0000'1000,
				filter_8 =		0b0000'1100,
				filter_16 =		0b0001'0000,
				spi3w_en =		0b0000'0001,
			};

			enum class cal_off_t : unsigned int
			{
				base =							0x88,
				off_0x88_0x89_dig_t1 =			0x88 - base,
				off_0x8a_0x8b_dig_t2 =			0x8a - base,
				off_0x8c_0x8d_dig_t3 =			0x8c - base,
				off_0x8e_0x8f_dig_p1 =			0x8e - base,
				off_0x90_0x91_dig_p2 =			0x90 - base,
				off_0x92_0x93_dig_p3 =			0x92 - base,
				off_0x94_0x95_dig_p4 =			0x94 - base,
				off_0x96_0x97_dig_p5 =			0x96 - base,
				off_0x98_0x99_dig_p6 =			0x98 - base,
				off_0x9a_0x9b_dig_p7 =			0x9a - base,
				off_0x9c_0x9d_dig_p8 =			0x9c - base,
				off_0x9e_0x9f_dig_p9 =			0x9e - base,
				off_0xa1_dig_h1 =				0xa1 - base,
				off_0xe1_0xe2_dig_h2 =			0xe1 - base,
				off_0xe3_dig_h3 =				0xe3 - base,
				off_0xe4_0xe5_0xe6_dig_h4_h5 =	0xe4 - base,
				off_0xe7_dig_h6 =				0xe7 - base,
				size =							0xe8 - base,
			};

			static std::string basic_name() { return("bmx280"); };

		private:

			void _poll() = 0;
	};

	class SensorBMP280 final : private SensorBMX280
	{
		friend class Sensors;

		private:

			explicit SensorBMP280() = delete;
			explicit SensorBMP280(const SensorBMP280 &) = delete;
			explicit SensorBMP280(const SensorBMP280 &&) = delete;
			explicit SensorBMP280(Log &, I2C::Device *);
			SensorBMP280& operator =(const SensorBMP280 &) = delete;

			struct calibration_data_t
			{
				uint16_t t1;
				int16_t t2;
				int16_t t3;
				uint16_t p1;
				int16_t p2;
				int16_t p3;
				int16_t p4;
				int16_t p5;
				int16_t p6;
				int16_t p7;
				int16_t p8;
				int16_t p9;
			};

			static std::string basic_name() { return("bmp280"); };
			static constexpr const std::array<int,1> addresses {0x76};
			static bool probe(I2c &, int module, int bus, int address, bool);

			calibration_data_t calibration_data;

			void _poll() override;
	};

	class SensorBME280 final : private SensorBMX280
	{
		friend class Sensors;

		private:

			explicit SensorBME280() = delete;
			explicit SensorBME280(const SensorBME280 &) = delete;
			explicit SensorBME280(const SensorBME280 &&) = delete;
			explicit SensorBME280(Log &, I2C::Device *);
			SensorBME280& operator =(const SensorBME280 &) = delete;

			struct calibration_data_t
			{
				uint16_t t1;
				int16_t t2;
				int16_t t3;
				uint16_t p1;
				int16_t p2;
				int16_t p3;
				int16_t p4;
				int16_t p5;
				int16_t p6;
				int16_t p7;
				int16_t p8;
				int16_t p9;
				uint8_t h1;
				int16_t h2;
				uint8_t h3;
				int16_t h4;
				int16_t h5;
				uint8_t h6;
			};

			static std::string basic_name() { return("bme280"); };
			static constexpr const std::array<int,1> addresses {0x76};
			static bool probe(I2c &, int module, int bus, int address, bool);

			calibration_data_t calibration_data;

			void _poll() override;
	};

	class SensorBME680 final : private SensorI2C
	{
		friend class Sensors;

		private:

			explicit SensorBME680() = delete;
			explicit SensorBME680(const SensorBME680 &) = delete;
			explicit SensorBME680(const SensorBME680 &&) = delete;
			explicit SensorBME680(Log &, I2C::Device *);
			SensorBME680& operator =(const SensorBME680 &) = delete;

			enum class reg_t : unsigned char
			{
				meas_status_0 =	0x1d,
				press =			0x1f,
				temp =			0x22,
				hum =			0x25,
				ctrl_gas_0 =	0x70,
				ctrl_hum =		0x72,
				status =		0x73,
				ctrl_meas =		0x74,
				config =		0x75,
				calibration_1 =	0x89,
				id =			0xd0,
				reset =			0xe0,
				calibration_2 =	0xe1,
			};

			enum class reg_ctrl_t : unsigned char
			{
				gas_0_heat_on =		0b0000'0000,
				gas_0_heat_off =	0b0000'1000,

				gas_1_run_gas =		0b0001'0000,
			};

			enum class reg_hum_t : unsigned char
			{
				osrh_h_skip =	0b0000'0000,
				osrh_h_1 =		0b0000'0001,
				osrh_h_2 =		0b0000'0010,
				osrh_h_4 =		0b0000'0011,
				osrh_h_8 =		0b0000'0100,
				osrh_h_16 =		0b0000'0101,
			};

			enum class reg_meas_t : unsigned char
			{
				osrs_t_skip =	0b0000'0000,
				osrs_t_1 =		0b0010'0000,
				osrs_t_2 =		0b0100'0000,
				osrs_t_4 =		0b0110'0000,
				osrs_t_8 =		0b1000'0000,
				osrs_t_16 =		0b1010'0000,

				osrs_mask =		0b0001'1100,
				osrs_p_skip =	0b0000'0000,
				osrs_p_1 =		0b0000'0100,
				osrs_p_2 =		0b0000'1000,
				osrs_p_4 =		0b0000'1100,
				osrs_p_8 =		0b0001'0000,

				sleep =			0b0000'0000,
				forced =		0b0000'0001,
			};

			enum class reg_config_t : unsigned char
			{
				filter_mask =		0b0001'1100,
				filter_0 =			0b0000'0000,
				filter_1 =			0b0000'0100,
				filter_3 =			0b0000'1000,
				filter_7 =			0b0000'1100,
				filter_15 =			0b0001'0000,
				filter_31 =			0b0001'0100,
				filter_63 =			0b0001'1000,
				filter_127 =		0b0001'1100,
			};

			enum class reg_meas_status_0_t : unsigned char
			{
				new_data =	0b1000'0000,
				measuring =	0b0010'0000,
			};

			enum class reg_id_t : unsigned char
			{
				bme680 =			0x61,
			};

			enum class reg_reset_t : unsigned char
			{
				value =	0xb6,
			};

			enum class cal_off_t : unsigned int
			{
				size_1 =	25,
				size_2 =	16,

				t1 =		33,
				t2 =		1,
				t3 =		3,

				p1 =		5,
				p2 =		7,
				p3 =		9,
				p4 =		11,
				p5 =		13,
				p6 =		16,
				p7 =		15,
				p8 =		19,
				p9 =		21,
				p10 =		23,

				h1 =		26,
				h2 =		25,
				h3 =		28,
				h4 =		29,
				h5 =		30,
				h6 =		31,
				h7 =		32,
			};

			struct calibration_data_t
			{
				uint16_t		t1;
				int16_t			t2;
				int8_t			t3;
				uint16_t		p1;
				int16_t			p2;
				int8_t			p3;
				int16_t			p4;
				int16_t			p5;
				int8_t			p6;
				int8_t			p7;
				int16_t			p8;
				int16_t			p9;
				uint8_t			p10;
				uint16_t		h1;
				uint16_t		h2;
				int8_t			h3;
				int8_t			h4;
				int8_t			h5;
				uint8_t			h6;
				int8_t			h7;
			} bme680_private_data_t;

			calibration_data_t calibration_data;

			static std::string basic_name() { return("bme680"); };
			static constexpr const std::array<int,1> addresses {0x76};
			static bool probe(I2c &, int module, int bus, int address, bool);

			void _poll() override;
	};

	class SensorBH1750 final : private SensorI2C
	{
		friend class Sensors;

		private:

			explicit SensorBH1750() = delete;
			explicit SensorBH1750(const SensorBH1750 &) = delete;
			explicit SensorBH1750(const SensorBH1750 &&) = delete;
			explicit SensorBH1750(Log &, I2C::Device *);
			SensorBH1750& operator =(const SensorBH1750 &) = delete;

			static std::string basic_name() { return("bh1750"); };
			static constexpr const std::array<int,1> addresses {0x23};
			static bool probe(I2c &, int module, int bus, int address, bool restricted);

			void _poll() override;

			enum class opcode_t : unsigned char
			{
				powerdown =			0b0000'0000, // 0x00
				poweron =			0b0000'0001, // 0x01
				reset =				0b0000'0111, // 0x07
				cont_hmode =		0b0001'0000, // 0x10
				cont_hmode2 =		0b0001'0001, // 0x11
				cont_lmode =		0b0001'0011, // 0x13
				oneshot_hmode =		0b0010'0000, // 0x20
				oneshot_hmode2 =	0b0010'0001, // 0x21
				oneshot_lmode =		0b0010'0011, // 0x23
				change_meas_hi =	0b0100'0000, // 0x40
				change_meas_lo =	0b0110'0000, // 0x60
			};

			struct autoranging_t
			{
				opcode_t mode;
				int timing;
				int lower;
				int upper;
				float factor;
			};

			using autorangings_t = std::deque<autoranging_t>;
			static const autorangings_t autoranging_data;
	};

	class SensorOPT3001 final : private SensorI2C
	{
		friend class Sensors;

		private:

			explicit SensorOPT3001() = delete;
			explicit SensorOPT3001(const SensorOPT3001 &) = delete;
			explicit SensorOPT3001(const SensorOPT3001 &&) = delete;
			explicit SensorOPT3001(Log &, I2C::Device *);
			SensorOPT3001& operator =(const SensorOPT3001 &) = delete;

			static std::string basic_name() { return("opt3001"); };
			static constexpr const std::array<int,1> addresses {0x45};
			static bool probe(I2c &, int module, int bus, int address, bool restricted);

			void _poll() override;

			enum class reg_t : unsigned char
			{
				result =		0x00,
				config =		0x01,
				limit_low =		0x02,
				limit_high =	0x03,
				id_manuf =		0x7e,
				id_dev =		0x7f,
			};

			enum class id_t : unsigned int
			{
				manufacturer_ti =	0x5449,
				device_opt3001 =	0x3001,
			};

			enum class conf_t : unsigned int
			{
				fault_count =		0b0000'0000'0000'0011,
				mask_exp =			0b0000'0000'0000'0100,
				pol =				0b0000'0000'0000'1000,
				latch =				0b0000'0000'0001'0000,
				flag_low =			0b0000'0000'0010'0000,
				flag_high =			0b0000'0000'0100'0000,
				flag_ready =		0b0000'0000'1000'0000,
				flag_ovf =			0b0000'0001'0000'0000,
				conv_mode =			0b0000'0110'0000'0000,
				conv_time =			0b0000'1000'0000'0000,
				range =				0b1111'0000'0000'0000,

				range_auto =		0b1100'0000'0000'0000,
				conv_time_100 =		0b0000'0000'0000'0000,
				conv_time_800 =		0b0000'1000'0000'0000,
				conv_mode_shut =	0b0000'0000'0000'0000,
				conv_mode_single =	0b0000'0010'0000'0000,
				conv_mode_cont =	0b0000'0110'0000'0000,

				default_config = range_auto | conv_time_800 | conv_mode_single,
			};

			void start_measurement();
	};

	class SensorMAX44009 final : private SensorI2C
	{
		friend class Sensors;

		private:

			explicit SensorMAX44009() = delete;
			explicit SensorMAX44009(const SensorMAX44009 &) = delete;
			explicit SensorMAX44009(const SensorMAX44009 &&) = delete;
			explicit SensorMAX44009(Log &, I2C::Device *);
			SensorMAX44009& operator =(const SensorMAX44009 &) = delete;

			static std::string basic_name() { return("max44009"); };
			static constexpr const std::array<int,1> addresses {0x4a};
			static bool probe(I2c &, int module, int bus, int address, bool restricted);

			void _poll() override;

			enum class reg_t : unsigned char
			{
				ints =			0x00,
				inte =			0x01,
				conf =			0x02,
				data_msb =		0x03,
				data_lsb =		0x04,
				thresh_msb =	0x05,
				thresh_lsb =	0x06,
				thresh_timer =	0x07,
			};

			enum class conf_t : unsigned char
			{
				tim_800 =	(0 << 2) | (0 << 1) | (0 << 0),
				tim_400 =	(0 << 2) | (0 << 1) | (1 << 0),
				tim_200 =	(0 << 2) | (1 << 1) | (0 << 0),
				tim_100 =	(0 << 2) | (1 << 1) | (1 << 0),
				tim_50 =	(1 << 2) | (0 << 1) | (0 << 0),
				tim_25 =	(1 << 2) | (0 << 1) | (1 << 0),
				tim_12 =	(1 << 2) | (1 << 1) | (0 << 0),
				tim_6 =		(1 << 2) | (1 << 1) | (1 << 0),
				cdr =		(1 << 3),
				reserved4 =	(1 << 4),
				reserved5 =	(1 << 5),
				manual =	(1 << 6),
				cont =		(1 << 7),
			};

			enum class probe_t : unsigned char
			{
				ints =			0x00,
				inte =			0x00,
				thresh_msb =	0xef,
				thresh_lsb =	0x00,
				thresh_timer =	0xff,
			};
	};

	class SensorTSL2561 final : private SensorI2C
	{
		friend class Sensors;

		private:

			explicit SensorTSL2561() = delete;
			explicit SensorTSL2561(const SensorTSL2561 &) = delete;
			explicit SensorTSL2561(const SensorTSL2561 &&) = delete;
			explicit SensorTSL2561(Log &, I2C::Device *);
			SensorTSL2561& operator =(const SensorTSL2561 &) = delete;

			static std::string basic_name() { return("tsl2561"); };
			static constexpr const std::array<int,1> addresses {0x39};
			static bool probe(I2c &, int module, int bus, int address, bool restricted);

			void _poll() override;

			enum class reg_t : unsigned char
			{
				control =		0x00,
				timeint =		0x01,
				threshlow =		0x02,
				threshhigh =	0x04,
				interrupt =		0x06,
				crc =			0x08,
				id =			0x0a,
				data0 =			0x0c,
				data1 =			0x0e,
			};

			enum class cmd_t : unsigned char
			{
				address =	(1 << 0) | (1 << 1) | (1 << 2) | (1 << 3),
				block =		1 << 4,
				word =		1 << 5,
				clear =		1 << 6,
				cmd =		1 << 7,
			};

			enum class conf_t : unsigned char
			{
				integ_13ms	=	(0 << 1) | (0 << 0),
				integ_101ms	=	(0 << 1) | (1 << 0),
				integ_402ms	=	(1 << 1) | (0 << 0),
				manual		=	1 << 3,
				low_gain	=	0 << 4,
				high_gain	=	1 << 4,
			};

			enum class control_t : unsigned char
			{
				power_off =	0x00,
				power_on =	0x03,
			};

			enum class id_t : unsigned char
			{
				tsl2561 =			0x50,
				probe_threshold =	0x00,
			};

			struct autoranging_t
			{
				conf_t timing;
				conf_t gain;
				int lower;
				int upper;
				int overflow;
				float factor;
			};

			using autorangings_t = std::deque<autoranging_t>;
			static const autorangings_t autoranging_data;

			void write_byte(reg_t, unsigned char);
			unsigned char read_byte(reg_t);
			unsigned int read_word(reg_t);
			bool write_check(reg_t, unsigned char);
	};

	class SensorTSL2591Base : protected SensorI2C
	{
		protected:

			explicit SensorTSL2591Base() = delete;
			explicit SensorTSL2591Base(const SensorTSL2591Base &) = delete;
			explicit SensorTSL2591Base(const SensorTSL2591Base &&) = delete;
			explicit SensorTSL2591Base(Log &, I2C::Device *);
			SensorTSL2591Base& operator =(const SensorTSL2591Base &) = delete;

			enum class reg_t : unsigned char
			{
				enable =		0x00,
				control =		0x01,
				ailtl =			0x04,
				ailth =			0x05,
				aihtl =			0x06,
				aihth =			0x07,
				npailtl =		0x08,
				npailth =		0x09,
				npaihtl =		0x0a,
				npaihth =		0x0b,
				persist =		0x0c,
				pid =			0x11,
				id =			0x12,
				status =		0x13,
				c0datal =		0x14,
				c0datah =		0x15,
				c1datal =		0x16,
				c1datah =		0x17,
				size =			0x18,
			};

			enum class cmd_t : unsigned char
			{
				cmd =							0b1 << 7,
				transaction_normal =			0b01 << 5,
				transaction_special =			0b11 << 5,
				sf_interrupt_set =				0b00100 << 0,
				sf_interrupt_clear_als =		0b00110 << 0,
				sf_interrupt_clear_als_nals =	0b00111 << 0,
				sf_interrupt_clear_nals =		0b01010 << 0,
			};

			enum class enable_t : unsigned char
			{
				npien =	1 << 7,
				sai =	1 << 6,
				aien =	1 << 4,
				aen =	1 << 1,
				pon =	1 << 0,
			};

			enum class control_t : unsigned char
			{
				sreset =		0b1 << 7,
				again_0 =		0b00 << 4,
				again_25 =		0b01 << 4,
				again_400 =		0b10 << 4,
				again_9500 =	0b11 << 4,
				atime_100 =		0b000 << 0,
				atime_200 =		0b001 << 0,
				atime_300 =		0b010 << 0,
				atime_400 =		0b011 << 0,
				atime_500 =		0b100 << 0,
				atime_600 =		0b101 << 0,
			};

			enum class pid_t : unsigned char
			{
				mask =	0b11 << 4,
				value = 0b00 << 4,
			};

			enum class id_t : unsigned char
			{
				mask =	0xff,
				value =	0x50,
				alt_value = 0x7f,
			};

			enum class intr_t : unsigned char
			{
				npintr =	0b1 << 5,
				aint =		0b1 << 4,
				avalid =	0b1 << 0,
			};

			void write_byte(reg_t, unsigned char value);
			unsigned char read_byte(reg_t);
			bool write_check(reg_t, unsigned char value);

			virtual void _poll() override = 0;

			static std::string basic_name() { return("tsl2591-base"); };
	};

	class SensorTSL2591RO final : private SensorTSL2591Base
	{
		friend class Sensors;

		private:

			explicit SensorTSL2591RO() = delete;
			explicit SensorTSL2591RO(const SensorTSL2591RO &) = delete;
			explicit SensorTSL2591RO(const SensorTSL2591RO &&) = delete;
			explicit SensorTSL2591RO(Log &, I2C::Device *);
			SensorTSL2591RO& operator =(const SensorTSL2591RO &) = delete;

			static std::string basic_name() { return("tsl2591-dummy"); };
			static constexpr const std::array<int,1> addresses {0x28};
			static bool probe(I2c &, int module, int bus, int address, bool restricted);

			void _poll() override;
	};

	class SensorTSL2591RW final : public SensorTSL2591Base
	{
		friend class Sensors;

		private:

			explicit SensorTSL2591RW() = delete;
			explicit SensorTSL2591RW(const SensorTSL2591RW &) = delete;
			explicit SensorTSL2591RW(const SensorTSL2591RW &&) = delete;
			explicit SensorTSL2591RW(Log &, I2C::Device *);
			SensorTSL2591RW& operator =(const SensorTSL2591RW &) = delete;

			static std::string basic_name() { return("tsl2591"); };
			static constexpr const std::array<int,1> addresses {0x29};
			static bool probe(I2c &, int module, int bus, int address, bool restricted);

			void _poll() override;

			struct autoranging_t
			{
				control_t gain;
				control_t timing;
				int lower;
				int upper;
				int overflow;
				float factor;
			};

			struct correction_t
			{
				float					lower_bound;
				float					upper_bound;
				std::array<float, 2>	ch_factor;
			};

			using autorangings_t = std::deque<autoranging_t>;
			static const autorangings_t autoranging_data;
			using corrections_t = std::deque<correction_t>;
			static const corrections_t correction_data;
	};

	class SensorVEML7700 final : private SensorI2C
	{
		friend class Sensors;

		private:

			explicit SensorVEML7700() = delete;
			explicit SensorVEML7700(const SensorVEML7700 &) = delete;
			explicit SensorVEML7700(const SensorVEML7700 &&) = delete;
			explicit SensorVEML7700(Log &, I2C::Device *);
			SensorVEML7700& operator =(const SensorVEML7700 &) = delete;

			static std::string basic_name() { return("veml7700"); };
			static constexpr const std::array<int,1> addresses {0x10};
			static bool probe(I2c &, int module, int bus, int address, bool restricted);

			void _poll() override;

			enum class reg_t : unsigned char
			{
				conf =		0x00,
				als_wh =	0x01,
				als_wl =	0x02,
				powsave =	0x03,
				als =		0x04,
				white =		0x05,
				als_int =	0x06,
				id =		0x07,
			};

			enum class id_t : unsigned char
			{
				id_1 =	0x81,
				id_2 =	0xc4,
			};

			enum class conf_t : unsigned int
			{
				reserved1 =		0b000 << 13,
				als_gain_1 =	0b00 << 11,
				als_gain_2 =	0b01 << 11,
				als_gain_1_8 =	0b10 << 11,
				als_gain_1_4 =	0b11 << 11,
				reserved2 =		0b1 << 10,
				als_it_25 =		0b1100 << 6,
				als_it_50 =		0b1000 << 6,
				als_it_100 =	0b0000 << 6,
				als_it_200 =	0b0001 << 6,
				als_it_400 =	0b0010 << 6,
				als_it_800 =	0b0011 << 6,
				als_pers_1 =	0b00 << 4,
				als_pers_2 =	0b01 << 4,
				als_pers_4 =	0b10 << 4,
				als_pers_8 =	0b11 << 4,
				reserved3 =		0b00 << 2,
				als_int_en =	0b1 << 1,
				als_sd =		0b1 << 0,
			};

			struct autoranging_t
			{
				conf_t timing;
				conf_t gain;
				int lower;
				int upper;
				float factor;
			};

			using autorangings_t = std::deque<autoranging_t>;
			static const autorangings_t autoranging_data;
	};

	class SensorLTR390 final : private SensorI2C
	{
		friend class Sensors;

		private:

			explicit SensorLTR390() = delete;
			explicit SensorLTR390(const SensorLTR390 &) = delete;
			explicit SensorLTR390(const SensorLTR390 &&) = delete;
			explicit SensorLTR390(Log &, I2C::Device *);
			SensorLTR390& operator =(const SensorLTR390 &) = delete;

			enum class reg_t : unsigned char
			{
				main_ctrl =				0x00,
				meas_rate =				0x04,
				gain =					0x05,
				part_id =				0x06,
				main_status =			0x07,
				als_data_0 =			0x0d,
				als_data_1 =			0x0e,
				als_data_2 =			0x0f,
				uvs_data_0 =			0x10,
				uvs_data_1 =			0x11,
				uvs_data_2 =			0x12,
				int_cfg =				0x19,
				int_pst =				0x1a,
				als_uvs_thres_up_0 =	0x21,
				als_uvs_thres_up_1 =	0x22,
				als_uvs_thres_up_2 =	0x23,
				als_uvs_thres_low_0 =	0x24,
				als_uvs_thres_low_1 =	0x25,
				als_uvs_thres_low_2 =	0x26,
			};

			enum class main_ctrl_t : unsigned char
			{
				sw_reset =				1 << 4,
				uvs_mode =				1 << 3,
				als_uvs_enable =		1 << 1,
			};

			enum class meas_rate_t : unsigned char
			{
				resolution_20 =			0b000 << 4,
				resolution_19 =			0b001 << 4,
				resolution_18 =			0b010 << 4,
				resolution_17 =			0b011 << 4,
				resolution_16 =			0b100 << 4,
				resolution_13 =			0b101 << 4,
				rate_25 =				0b000 << 0,
				rate_50 =				0b001 << 0,
				rate_100 =				0b010 << 0,
				rate_200 =				0b011 << 0,
				rate_500 =				0b100 << 0,
				rate_1000 =				0b101 << 0,
				rate_2000 =				0b110 << 0,
			};

			enum class gain_t : unsigned char
			{
				gain_1 =				0b000 << 0,
				gain_3 =				0b001 << 0,
				gain_6 =				0b010 << 0,
				gain_9 =				0b011 << 0,
				gain_18 =				0b100 << 0,
			};

			enum class part_id_t : unsigned char
			{
				part_number =			0b1011 << 4,
				revision =				0b0010 << 0,
			};

			enum class main_status_t : unsigned char
			{
				power_on_status =		1 << 5,
				int_status =			1 << 4,
				data_status =			1 << 3,
			};

			enum class int_cfg_t : unsigned char
			{
				int_select_als =		0b01 << 4,
				int_select_uv =			0b11 << 4,
				int_enabled	=			0b1 << 2,
			};

			enum class int_pst_t : unsigned char
			{
				persist_all =			0b0000 << 4,
				persist_16 =			0b1111 << 4,
			};

			struct autoranging_t
			{
				meas_rate_t resolution;
				meas_rate_t rate;
				gain_t gain;
				unsigned int raw_lower;
				unsigned int raw_upper;
				float gain_factor;
				float integration_time_factor;
				float lux_lower;
				float lux_upper;
				int precision;
			};

			using autorangings_t = std::deque<autoranging_t>;
			static const autorangings_t autoranging_data;

			static const constexpr float normalisation_factor = 0.6f;
			static const constexpr float uv_sensitivity = 2300.0f;

			static std::string basic_name() { return("ltr390"); };
			static constexpr const std::array<int,1> addresses {0x53};
			static bool probe(I2c &, int module, int bus, int address, bool restricted);

			void _poll() override;
	};

	class SensorInternal : protected Sensor
	{
		protected:

			explicit SensorInternal() = delete;
			explicit SensorInternal(const SensorInternal &) = delete;
			explicit SensorInternal(const SensorInternal &&) = delete;
			explicit SensorInternal(Log &, std::string_view name, int dummy_module);
			SensorInternal & operator =(const SensorInternal &) = delete;
			virtual ~SensorInternal();

			virtual void _address(int &module, int &bus, int &address) override;
			virtual void _poll() = 0;

			int dummy_module;
	};

	class SensorInternalTemperature final : private SensorInternal
	{
		friend class Sensors;

		private:

			explicit SensorInternalTemperature() = delete;
			explicit SensorInternalTemperature(const SensorInternalTemperature &) = delete;
			explicit SensorInternalTemperature(const SensorInternalTemperature &&) = delete;
			explicit SensorInternalTemperature(Log &, int dummy_module);
			SensorInternalTemperature& operator =(const SensorInternalTemperature &) = delete;

			static std::string basic_name() { return("sensor"); };

			void _poll() override;

			temperature_sensor_handle_t handle;
	};
}

using Sensors = SENSORS::Sensors;
