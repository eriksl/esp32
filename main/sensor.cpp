#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <assert.h>

#include "info.h"
#include "log.h"
#include "util.h"
#include "i2c.h"
#include "cli-command.h"
#include "sensor.h"

#include <string>
#include <boost/format.hpp>

typedef struct
{
	float value;
	time_t stamp;
} sensor_value_t;

typedef enum : unsigned int
{
	sensor_found,
	sensor_not_found,
	sensor_disabled,
} sensor_detect_t;

typedef struct
{
	unsigned int force_detect:1; // if the device does all of it's probing itself (e.g. if it needs additional steps before the device can be probed)
	unsigned int no_constrained:1; // if the device cannot work with constrained I2C functionality (i.e. the RTC/ULP I2C module feature set)
} sensor_flags_t;

typedef struct data_T
{
	sensor_detect_t state;
	i2c_slave_t slave;
	sensor_value_t values[sensor_type_size];
	const struct info_T *info;
	void *private_data;
	struct data_T *next;
} data_t;

typedef struct info_T
{
	const char *name;
	sensor_t id;
	unsigned int address;
	sensor_type_t type; // bitmask!
	sensor_flags_t flags;
	unsigned int precision;
	size_t private_data_size;
	sensor_detect_t (*const detect_fn)(i2c_slave_t);
	bool (*const init_fn)(data_t *);
	bool (*const poll_fn)(data_t *);
	void (*const dump_fn)(const data_t *, std::string &);
} info_t;

typedef struct
{
	const char *type;
	const char *unity;
} sensor_type_info_t;

typedef struct
{
	i2c_module_t module;
} run_parameters_t;

static const sensor_type_info_t sensor_type_info[sensor_type_size] =
{
	[sensor_type_visible_light] =
	{
		.type = "visible light",
		.unity = "lx",
	},
	[sensor_type_temperature] =
	{
		.type = "temperature",
		.unity = "°C",
	},
	[sensor_type_humidity] =
	{
		.type = "humidity",
		.unity = "%",
	},
	[sensor_type_airpressure] =
	{
		.type = "air pressure",
		.unity = "hPa",
	},
};

static bool inited = false;
static SemaphoreHandle_t data_mutex;
static data_t *data_root = nullptr;

static unsigned int stat_sensors_not_considered[i2c_module_size] = { 0 };
static unsigned int stat_sensors_skipped[i2c_module_size] = { 0 };
static unsigned int stat_sensors_not_skipped[i2c_module_size] = { 0 };
static unsigned int stat_sensors_probed[i2c_module_size] = { 0 };
static unsigned int stat_sensors_found[i2c_module_size] = { 0 };
static unsigned int stat_sensors_disabled[i2c_module_size] = { 0 };
static unsigned int stat_sensors_confirmed[i2c_module_size] = { 0 };
static unsigned int stat_poll_run[i2c_module_size] = { 0 };
static unsigned int stat_poll_ok[i2c_module_size] = { 0 };
static unsigned int stat_poll_error[i2c_module_size] = { 0 };
static unsigned int stat_poll_skipped[i2c_module_size] = { 0 };

static inline void data_mutex_take(void)
{
	assert(xSemaphoreTake(data_mutex, portMAX_DELAY));
}

static inline void data_mutex_give(void)
{
	assert(xSemaphoreGive(data_mutex));
}

typedef struct
{
	unsigned int data[2];
	struct
	{
		unsigned int down;
		unsigned int up;
	} threshold;
	unsigned int overflow;
	float factor;
} device_autoranging_data_t;

[[nodiscard]] static unsigned int unsigned_20_top_be(const uint8_t ptr[])
{
	return((((ptr[0] & 0xff) >> 0) << 12) | (((ptr[1] & 0xff) >> 0) << 4) | (((ptr[2] & 0xf0) >> 4) << 0));
}

[[nodiscard]] static unsigned int unsigned_20_bottom_be(const uint8_t ptr[])
{
	return((((ptr[0] & 0x0f) >> 0) << 16) | (((ptr[1] & 0xff) >> 0) << 8) | (((ptr[2] & 0xff) >> 0) << 0));
}

[[nodiscard]] static unsigned int unsigned_16_be(const uint8_t ptr[])
{
	return(((ptr[0] & 0xff) << 8) | (ptr[1] & 0xff));
}

[[nodiscard]] static unsigned int unsigned_16_le(const uint8_t ptr[])
{
	return(((ptr[1] & 0xff) << 8) | (ptr[0] & 0xff));
}

[[nodiscard]] static int signed_16_le(const uint8_t ptr[])
{
	int rv = ((ptr[1] & 0xff) << 8) | (ptr[0] & 0xff);

	if(rv > (1 << 15))
		rv = 0 - ((1 << 16) - rv);

	return(rv);
}

[[nodiscard]] static unsigned int unsigned_12_top_be(const uint8_t ptr[])
{
	return((((ptr[0] & 0xff) >> 0) << 4) | (((ptr[1] & 0xf0) >> 4) << 0));
}

[[nodiscard]] static unsigned int unsigned_12_bottom_le(const uint8_t ptr[])
{
	return((((ptr[0] & 0x0f) >> 0) << 0) | (((ptr[1] & 0xff) >> 0) << 4));
}

[[nodiscard]] static unsigned int unsigned_8(const uint8_t ptr[])
{
	return((unsigned int)(*ptr & 0xff));
}

[[nodiscard]] static int signed_8(const uint8_t ptr[])
{
	int rv = (unsigned int)(*ptr & 0xff);

	if(rv > (1 << 7))
		rv = 0 - ((1 << 8) - rv);

	return(rv);
}

enum : unsigned int
{
	bh1750_opcode_powerdown =		0b0000'0000, // 0x00
	bh1750_opcode_poweron =			0b0000'0001, // 0x01
	bh1750_opcode_reset =			0b0000'0111, // 0x07
	bh1750_opcode_cont_hmode =		0b0001'0000, // 0x10
	bh1750_opcode_cont_hmode2 =		0b0001'0001, // 0x11
	bh1750_opcode_cont_lmode =		0b0001'0011, // 0x13
	bh1750_opcode_one_hmode =		0b0010'0000, // 0x20
	bh1750_opcode_one_hmode2 =		0b0010'0001, // 0x21
	bh1750_opcode_one_lmode =		0b0010'0011, // 0x23
	bh1750_opcode_change_meas_hi =	0b0100'0000, // 0x40
	bh1750_opcode_change_meas_lo =	0b0110'0000, // 0x60
};

typedef enum : unsigned int
{
	bh1750_state_init,
	bh1750_state_reset,
	bh1750_state_measuring,
	bh1750_state_finished,
}  bh1750_state_t;

static constexpr unsigned int bh1750_autoranging_data_size = 4;

typedef struct
{
	bh1750_state_t state;
	unsigned int raw_value;
	unsigned int scaling;
	unsigned int scaling_up;
	unsigned int scaling_down;
	unsigned int overflows;
} bh1750_private_data_t;

static const device_autoranging_data_t bh1750_autoranging_data[bh1750_autoranging_data_size] =
{
	{{	bh1750_opcode_one_hmode2,	254	},	{ 0,	50000 }, 65535, 0.13 },
	{{	bh1750_opcode_one_hmode2,	69	},	{ 1000, 50000 }, 65535, 0.50 },
	{{	bh1750_opcode_one_hmode2,	31	},	{ 1000, 50000 }, 65535, 1.10 },
	{{	bh1750_opcode_one_lmode,	31	},	{ 1000, 65536 }, 65535, 2.40 },
};

static sensor_detect_t bh1750_detect(i2c_slave_t slave)
{
	uint8_t buffer[8];

	if(!i2c_send_1_receive(slave, bh1750_opcode_powerdown, sizeof(buffer), buffer))
		return(sensor_not_found);

	if((buffer[2] != 0xff) || (buffer[3] != 0xff) || (buffer[4] != 0xff) ||
			(buffer[5] != 0xff) || (buffer[6] != 0xff) || (buffer[7] != 0xff))
		return(sensor_not_found);

	return(sensor_found);
}

static bool bh1750_init(data_t *data)
{
	bh1750_private_data_t *pdata = static_cast<bh1750_private_data_t *>(data->private_data);

	assert(pdata);

	if(!i2c_send_1(data->slave, bh1750_opcode_poweron))
	{
		Log::get() << "bh1750: init error";
		return(false);
	}

	pdata->raw_value = 0;
	pdata->scaling = 0;
	pdata->scaling_up = 0;
	pdata->scaling_down = 0;
	pdata->overflows = 0;
	pdata->state = bh1750_state_init;

	return(true);
}

static bool bh1750_poll(data_t *data)
{
	bh1750_private_data_t *pdata = static_cast<bh1750_private_data_t *>(data->private_data);
	uint8_t buffer[2];

	assert(pdata);

	switch(pdata->state)
	{
		case(bh1750_state_init):
		{
			if(!i2c_send_1(data->slave, bh1750_opcode_reset))
			{
				Log::get() << "bh1750: poll: error 1";
				return(false);
			}

			pdata->state = bh1750_state_reset;

			break;
		}

		case(bh1750_state_reset):
		case(bh1750_state_finished):
		{
			if(!i2c_send_1(data->slave, bh1750_opcode_change_meas_hi | ((bh1750_autoranging_data[pdata->scaling].data[1] >> 5) & 0b0000'0111)))
			{
				Log::get() << "bh1750: poll error 2";
				return(false);
			}

			if(!i2c_send_1(data->slave, bh1750_opcode_change_meas_lo | ((bh1750_autoranging_data[pdata->scaling].data[1] >> 0) & 0b0001'1111)))
			{
				Log::get() << "bh1750: poll error 2";
				return(false);
			}

			if(!i2c_send_1(data->slave, bh1750_autoranging_data[pdata->scaling].data[0]))
			{
				Log::get() << "bh1750: poll error 3";
				return(false);
			}

			pdata->state = bh1750_state_measuring;

			break;
		}

		case(bh1750_state_measuring):
		{
			pdata->state = bh1750_state_finished;

			if(!i2c_receive(data->slave, sizeof(buffer), buffer))
			{
				Log::get() << "bh1750: poll: warning: error in receive data";
				return(false);
			}

			pdata->raw_value = unsigned_16_be(buffer);

			if((pdata->raw_value >= bh1750_autoranging_data[pdata->scaling].overflow) && (pdata->scaling >= (bh1750_autoranging_data_size - 1)))
			{
				pdata->overflows++;
				break;
			}

			if((pdata->raw_value < bh1750_autoranging_data[pdata->scaling].threshold.down) && (pdata->scaling > 0))
			{
				pdata->scaling--;
				pdata->scaling_down++;
				break;
			}

			if((pdata->raw_value >= bh1750_autoranging_data[pdata->scaling].threshold.up) && (pdata->scaling < (bh1750_autoranging_data_size - 1)))
			{
				pdata->scaling++;
				pdata->scaling_up++;
				break;
			}

			data->values[sensor_type_visible_light].value = ((float)pdata->raw_value * bh1750_autoranging_data[pdata->scaling].factor);
			data->values[sensor_type_visible_light].stamp = time(nullptr);

			break;
		}
	}

	return(true);
}

static void bh1750_dump(const data_t *data, std::string &output)
{
	bh1750_private_data_t *pdata = static_cast<bh1750_private_data_t *>(data->private_data);

	assert(pdata);

	output += (boost::format("state: %u, ") % pdata->state).str();
	output += (boost::format("scaling: %u, ") % pdata->scaling).str();
	output += (boost::format("scaling_up: %u, ") % pdata->scaling_up).str();
	output += (boost::format("scaling_down: %u, ") % pdata->scaling_down).str();
	output += (boost::format("overflows: %u, ") % pdata->overflows).str();
	output += (boost::format("raw: %u") % pdata->raw_value).str();
};

enum : unsigned int
{
	tmp75_reg_temp = 0x00,
	tmp75_reg_conf = 0x01,

	tmp75_reg_conf_res_12 = 0b0110'0000,
};

typedef struct
{
	unsigned int raw_value[2];
} tmp75_private_data_t;

static sensor_detect_t tmp75_detect(i2c_slave_t slave)
{
	uint8_t buffer[2];

	if(!i2c_send_1_receive(slave, tmp75_reg_conf, sizeof(buffer), buffer))
		return(sensor_not_found);

	return(sensor_found);
}

static bool tmp75_init(data_t *data)
{
	tmp75_private_data_t *pdata = static_cast<tmp75_private_data_t *>(data->private_data);
	uint8_t buffer[2];

	assert(pdata);

	pdata->raw_value[0] = 0;
	pdata->raw_value[1] = 0;

	if(!i2c_send_2(data->slave, tmp75_reg_conf, tmp75_reg_conf_res_12))
		return(false);

	if(!i2c_send_1_receive(data->slave, tmp75_reg_conf, sizeof(buffer), buffer))
		return(false);

	if(buffer[0] != tmp75_reg_conf_res_12)
	{
		Log::get() << std::format("tmp75: init: config: {:#x}", buffer[0]);
		return(false);
	}

	return(true);
}

static bool tmp75_poll(data_t *data)
{
	tmp75_private_data_t *pdata = static_cast<tmp75_private_data_t *>(data->private_data);
	uint8_t buffer[2];
	unsigned int raw_temperature;

	assert(pdata);

	if(!i2c_send_1_receive(data->slave, tmp75_reg_temp, sizeof(buffer), buffer))
	{
		Log::get() << "sensor: error in poll tmp75";
		return(false);
	}

	pdata->raw_value[0] = buffer[0];
	pdata->raw_value[1] = buffer[1];
	raw_temperature = unsigned_16_be(buffer);
	data->values[sensor_type_temperature].value = raw_temperature / 256.0f;
	data->values[sensor_type_temperature].stamp = time(nullptr);

	return(true);
}

static void tmp75_dump(const data_t *data, std::string &output)
{
	tmp75_private_data_t *pdata = static_cast<tmp75_private_data_t *>(data->private_data);

	assert(pdata);

	output += (boost::format("raw value 0: %u, ") % pdata->raw_value[0]).str();
	output += (boost::format("raw value 1: %u") % pdata->raw_value[1]).str();
};

enum : unsigned int
{
	opt3001_reg_result =		0x00,
	opt3001_reg_conf =			0x01,
	opt3001_reg_limit_low =		0x02,
	opt3001_reg_limit_high =	0x03,
	opt3001_reg_id_manuf =		0x7e,
	opt3001_reg_id_dev =		0x7f,
};

enum : unsigned int
{
	opt3001_id_manuf_ti =		0x5449,
	opt3001_id_dev_opt3001 =	0x3001,
};

enum : unsigned int
{
	opt3001_conf_fault_count =		0b0000'0000'0000'0011,
	opt3001_conf_mask_exp =			0b0000'0000'0000'0100,
	opt3001_conf_pol =				0b0000'0000'0000'1000,
	opt3001_conf_latch =			0b0000'0000'0001'0000,
	opt3001_conf_flag_low =			0b0000'0000'0010'0000,
	opt3001_conf_flag_high =		0b0000'0000'0100'0000,
	opt3001_conf_flag_ready =		0b0000'0000'1000'0000,
	opt3001_conf_flag_ovf =			0b0000'0001'0000'0000,
	opt3001_conf_conv_mode =		0b0000'0110'0000'0000,
	opt3001_conf_conv_time =		0b0000'1000'0000'0000,
	opt3001_conf_range =			0b1111'0000'0000'0000,

	opt3001_conf_range_auto =		0b1100'0000'0000'0000,
	opt3001_conf_conv_time_100 =	0b0000'0000'0000'0000,
	opt3001_conf_conv_time_800 =	0b0000'1000'0000'0000,
	opt3001_conf_conv_mode_shut =	0b0000'0000'0000'0000,
	opt3001_conf_conv_mode_single =	0b0000'0010'0000'0000,
	opt3001_conf_conv_mode_cont =	0b0000'0110'0000'0000,
};

static constexpr unsigned int opt3001_config = opt3001_conf_range_auto | opt3001_conf_conv_time_800 | opt3001_conf_conv_mode_single;

typedef enum : unsigned int
{
	opt3001_state_init,
	opt3001_state_measuring,
	opt3001_state_finished,
} opt3001_state_t;

typedef struct
{
	opt3001_state_t state;
	unsigned int overflows;
	unsigned int mantissa;
	unsigned exponent;
} opt3001_private_data_t;

static bool opt3001_start_measurement(data_t *data)
{
	uint8_t buffer[3];

	buffer[0] = opt3001_reg_conf;
	buffer[1] = (opt3001_config & 0xff00) >> 8;
	buffer[2] = (opt3001_config & 0x00ff) >> 0;

	if(!i2c_send(data->slave, sizeof(buffer), buffer))
		return(false);

	if(!i2c_send_1_receive(data->slave, opt3001_reg_conf, 2, buffer))
		return(false);

	return(true);
}

static sensor_detect_t opt3001_detect(i2c_slave_t slave)
{
	uint8_t buffer[2];

	if(!i2c_send_1_receive(slave, opt3001_reg_id_manuf, sizeof(buffer), buffer))
		return(sensor_not_found);

	if(unsigned_16_be(buffer) != opt3001_id_manuf_ti)
		return(sensor_not_found);

	if(!i2c_send_1_receive(slave, opt3001_reg_id_dev, sizeof(buffer), buffer))
		return(sensor_not_found);

	if(unsigned_16_be(buffer) != opt3001_id_dev_opt3001)
		return(sensor_not_found);

	return(sensor_found);
}

static bool opt3001_init(data_t *data)
{
	opt3001_private_data_t *pdata = static_cast<opt3001_private_data_t *>(data->private_data);
	uint8_t buffer[2];
	unsigned int read_config;

	assert(pdata);

	pdata->state = opt3001_state_init;
	pdata->mantissa = 0;
	pdata->exponent = 0;
	pdata->overflows = 0;

	if(!opt3001_start_measurement(data))
	{
		Log::get() << "opt3001: init error 1";
		return(false);
	}

	if(!i2c_send_1_receive(data->slave, opt3001_reg_conf, sizeof(buffer), buffer))
		return(false);

	read_config = unsigned_16_be(buffer) & (opt3001_conf_mask_exp | opt3001_conf_conv_mode | opt3001_conf_conv_time | opt3001_conf_range);

	if(read_config != opt3001_config)
	{
		Log::get() << "opt3001: init error 2";
		return(false);
	}

	return(true);
}

static bool opt3001_poll(data_t *data)
{
	opt3001_private_data_t *pdata = static_cast<opt3001_private_data_t *>(data->private_data);
	uint8_t buffer[2];
	unsigned int config;

	assert(pdata);

	switch(pdata->state)
	{
		case(opt3001_state_init):
		case(opt3001_state_finished):
		{
			if(!opt3001_start_measurement(data))
			{
				Log::get() << "opt3001: poll error 3";
				return(false);
			}

			pdata->state = opt3001_state_measuring;

			break;
		}

		case(opt3001_state_measuring):
		{
			if(!i2c_send_1_receive(data->slave, opt3001_reg_conf, sizeof(buffer), buffer))
			{
				Log::get() << "opt3001 poll: error 1";
				return(false);
			}

			config = unsigned_16_be(buffer);

			if(!(config & opt3001_conf_flag_ready))
				return(true);

			pdata->state = opt3001_state_finished;

			if(config & opt3001_conf_flag_ovf)
			{
				pdata->overflows++;
				return(true);
			}

			if(!i2c_send_1_receive(data->slave, opt3001_reg_result, sizeof(buffer), buffer))
			{
				Log::get() << "opt3001 poll: error 2";
				return(false);
			}

			pdata->exponent = (buffer[0] & 0xf0) >> 4;
			pdata->mantissa = ((buffer[0] & 0x0f) << 8) | buffer[1];

			data->values[sensor_type_visible_light].value = 0.01f * (float)(1 << pdata->exponent) * (float)pdata->mantissa;
			data->values[sensor_type_visible_light].stamp = time(nullptr);

			break;
		}
	}

	return(true);
}

static void opt3001_dump(const data_t *data, std::string &output)
{
	opt3001_private_data_t *pdata = static_cast<opt3001_private_data_t *>(data->private_data);

	assert(pdata);

	output += (boost::format("overflows: %u, ") % pdata->overflows).str();
	output += (boost::format("mantissa: %u, ") % pdata->mantissa).str();
	output += (boost::format("exponent: %u") % pdata->exponent).str();
}

enum : unsigned int
{
	max44009_reg_ints =			0x00,
	max44009_reg_inte =			0x01,
	max44009_reg_conf =			0x02,
	max44009_reg_data_msb =		0x03,
	max44009_reg_data_lsb =		0x04,
	max44009_reg_thresh_msb =	0x05,
	max44009_reg_thresh_lsb =	0x06,
	max44009_reg_thresh_timer =	0x07,

	max44009_conf_tim_800 =		(0 << 2) | (0 << 1) | (0 << 0),
	max44009_conf_tim_400 =		(0 << 2) | (0 << 1) | (1 << 0),
	max44009_conf_tim_200 =		(0 << 2) | (1 << 1) | (0 << 0),
	max44009_conf_tim_100 =		(0 << 2) | (1 << 1) | (1 << 0),
	max44009_conf_tim_50 =		(1 << 2) | (0 << 1) | (0 << 0),
	max44009_conf_tim_25 =		(1 << 2) | (0 << 1) | (1 << 0),
	max44009_conf_tim_12 =		(1 << 2) | (1 << 1) | (0 << 0),
	max44009_conf_tim_6 =		(1 << 2) | (1 << 1) | (1 << 0),
	max44009_conf_cdr =			(1 << 3),
	max44009_conf_reserved4 =	(1 << 4),
	max44009_conf_reserved5 =	(1 << 5),
	max44009_conf_manual =		(1 << 6),
	max44009_conf_cont =		(1 << 7),

	max44009_probe_ints =			0x00,
	max44009_probe_inte =			0x00,
	max44009_probe_thresh_msb =		0xef,
	max44009_probe_thresh_lsb =		0x00,
	max44009_probe_thresh_timer =	0xff,
};

typedef struct
{
	unsigned int overflows;
	unsigned int mantissa;
	unsigned int exponent;
} max44009_private_data_t;

static sensor_detect_t max44009_detect(i2c_slave_t slave)
{
	uint8_t buffer[2];

	if(!i2c_send_1_receive(slave, max44009_reg_ints, sizeof(buffer), buffer))
		return(sensor_not_found);

	if((buffer[0] != max44009_probe_ints) || (buffer[1] != max44009_probe_ints))
		return(sensor_not_found);

	if(!i2c_send_1_receive(slave, max44009_reg_inte, sizeof(buffer), buffer))
		return(sensor_not_found);

	if((buffer[0] != max44009_probe_inte) || (buffer[1] != max44009_probe_inte))
		return(sensor_not_found);

	if(!i2c_send_1_receive(slave, max44009_reg_thresh_msb, sizeof(buffer), buffer))
		return(sensor_not_found);

	if((buffer[0] != max44009_probe_thresh_msb) || (buffer[1] != max44009_probe_thresh_msb))
		return(sensor_not_found);

	if(!i2c_send_1_receive(slave, max44009_reg_thresh_lsb, sizeof(buffer), buffer))
		return(sensor_not_found);

	if((buffer[0] != max44009_probe_thresh_lsb) || (buffer[1] != max44009_probe_thresh_lsb))
		return(sensor_not_found);

	if(!i2c_send_1_receive(slave, max44009_reg_thresh_timer, sizeof(buffer), buffer))
		return(sensor_not_found);

	if((buffer[0] != max44009_probe_thresh_timer) || (buffer[1] != max44009_probe_thresh_timer))
		return(sensor_not_found);

	return(sensor_found);
}

static bool max44009_init(data_t *data)
{
	max44009_private_data_t *pdata = static_cast<max44009_private_data_t *>(data->private_data);
	uint8_t buffer[2];

	assert(pdata);

	pdata->overflows = 0;
	pdata->mantissa = 0;
	pdata->exponent = 0;

	if(!i2c_send_2(data->slave, max44009_reg_conf, max44009_conf_cont))
	{
		Log::get() << "sensors: max44009: init error 1";
		return(false);
	}

	if(!i2c_send_1_receive(data->slave, max44009_reg_conf, sizeof(buffer), buffer))
	{
		Log::get() << "sensors: max44009: init error 2";
		return(false);
	}

	if((buffer[0] & (max44009_conf_cont | max44009_conf_manual)) != max44009_conf_cont)
	{
		Log::get() << "sensors: max44009: init error 3";
		return(false);
	}

	return(true);
}

static bool max44009_poll(data_t *data)
{
	max44009_private_data_t *pdata = static_cast<max44009_private_data_t *>(data->private_data);
	uint8_t buffer[2];

	assert(pdata);

	if(!i2c_send_1_receive(data->slave, max44009_reg_data_msb, sizeof(buffer), buffer))
	{
		Log::get() << "sensors: max44009: poll error 1";
		return(false);
	}

	pdata->exponent =	(buffer[0] & 0xf0) >> 4;
	pdata->mantissa =	(buffer[0] & 0x0f) << 4;
	pdata->mantissa |=	(buffer[1] & 0x0f) << 0;

	if(pdata->exponent != 0b1111)
	{
		data->values[sensor_type_visible_light].value = (1 << pdata->exponent) * (float)pdata->mantissa * 0.045f;
		data->values[sensor_type_visible_light].stamp = time(nullptr);
	}
	else
		pdata->overflows++;

	return(true);
}

static void max44009_dump(const data_t *data, std::string &output)
{
	max44009_private_data_t *pdata = static_cast<max44009_private_data_t *>(data->private_data);

	assert(pdata);

	output += (boost::format("overflows: %u, ") % pdata->overflows).str();
	output += (boost::format("mantissa: %u, ") % pdata->mantissa).str();
	output += (boost::format("exponent: %u") % pdata->exponent).str();
};

enum : unsigned int
{
	asair_cmd_aht10_init_1 =	0xe1,
	asair_cmd_aht10_init_2 =	0x08,
	asair_cmd_aht10_init_3 =	0x00,
	asair_cmd_aht20_init_1 =	0xbe,
	asair_cmd_aht20_init_2 =	0x08,
	asair_cmd_aht20_init_3 =	0x00,
	asair_cmd_measure_0 =		0xac,
	asair_cmd_measure_1 =		0x33,
	asair_cmd_measure_2 =		0x00,
	asair_cmd_get_status =		0x71,
	asair_cmd_reset =			0xba,
};

enum : unsigned int
{
	asair_status_busy =		1 << 7,
	asair_status_ready =	1 << 3,
};

typedef enum : unsigned int
{
	asair_state_init,
	asair_state_ready,
	asair_state_start_measure,
	asair_state_measuring,
	asair_state_measure_complete,
} asair_state_t;

typedef struct
{
	asair_state_t state;
	unsigned int type;
	bool valid;
	unsigned int raw_temperature;
	unsigned int raw_humidity;
} asair_private_data_t;

static bool asair_ready(data_t *data)
{
	uint8_t buffer[1];

	if(!i2c_send_1_receive(data->slave, asair_cmd_get_status, sizeof(buffer), buffer))
		return(false);

	if(!(buffer[0] & asair_status_ready))
		return(false);

	return(true);
}

static bool asair_init_chip(data_t *data)
{
	asair_private_data_t *pdata = static_cast<asair_private_data_t *>(data->private_data);

	if(i2c_send_3(data->slave, asair_cmd_aht10_init_1, asair_cmd_aht10_init_2, asair_cmd_aht10_init_3))
	{
		pdata->type = 10;
		return(true);
	}

	if(i2c_send_3(data->slave, asair_cmd_aht20_init_1, asair_cmd_aht20_init_2, asair_cmd_aht20_init_3))
	{
		pdata->type = 20;
		return(true);
	}

	pdata->type = 0;
	Log::get() << "asair_init: unknown device type";

	return(false);
}

static sensor_detect_t asair_detect(i2c_slave_t slave)
{
	uint8_t buffer[1];

	if(!i2c_send_1_receive(slave, asair_cmd_get_status, sizeof(buffer), buffer))
		return(sensor_not_found);

	if(!i2c_send_1(slave, asair_cmd_reset))
		return(sensor_not_found);

	return(sensor_found);
}

static bool asair_init(data_t *data)
{
	asair_private_data_t *pdata = static_cast<asair_private_data_t *>(data->private_data);

	assert(pdata);

	pdata->state = asair_state_init;
	pdata->type = 0;
	pdata->valid = false;
	pdata->raw_temperature = 0;
	pdata->raw_humidity = 0;

	if(asair_ready(data))
	{
		if(!asair_init_chip(data))
		{
			Log::get() << "asair_init: unknown device type";
			return(false);
		}

		pdata->state = asair_state_ready;
	}

	return(true);
}

static bool asair_poll(data_t *data)
{
	asair_private_data_t *pdata = static_cast<asair_private_data_t *>(data->private_data);
	uint8_t	buffer[8];

	assert(pdata);

	switch(pdata->state)
	{
		case(asair_state_init):
		{
			if(asair_ready(data))
			{
				if(!asair_init_chip(data))
				{
					Log::get() << "asair_init: unknown device type";
					return(false);
				}

				pdata->state = asair_state_ready;
				break;
			}

			break;
		}

		case(asair_state_ready):
		{
			if(!i2c_send_1_receive(data->slave, asair_cmd_get_status, 1, buffer))
			{
				Log::get() << "sensors: asair: poll error 1";
				return(false);
			}

			if((buffer[0] & asair_status_busy) || !(buffer[0] & asair_status_ready))
			{
				Log::get() << "sensors: asair: poll error 2";
				return(false);
			}

			pdata->valid = false;
			pdata->state = asair_state_start_measure;

			break;
		}

		case(asair_state_start_measure):
		{
			if(!i2c_send_1_receive(data->slave, asair_cmd_get_status, 1, buffer))
			{
				Log::get() << "sensors: asair: poll error 3";
				return(false);
			}

			if(buffer[0] & asair_status_busy)
			{
				Log::get() << "sensors: asair: poll error 4";
				return(false);
			}

			if(!i2c_send_3(data->slave, asair_cmd_measure_0, asair_cmd_measure_1, asair_cmd_measure_2))
			{
				Log::get() << "sensors: asair: poll error 5";
				pdata->valid = false;
				return(false);
			}

			pdata->state = asair_state_measuring;

			break;
		}

		case(asair_state_measuring):
		{
			pdata->state = asair_state_measure_complete;

			break;
		}

		case(asair_state_measure_complete):
		{
			if(!i2c_send_1_receive(data->slave, asair_cmd_get_status, sizeof(buffer), buffer))
			{
				Log::get() << "sensors: asair: poll error 6";
				pdata->valid = false;
				return(false);
			}

			if(buffer[0] & asair_status_busy)
			{
				Log::get() << "sensors: asair: poll error 7";
				pdata->valid = false;
				return(false);
			}

			pdata->raw_temperature = unsigned_20_bottom_be(&buffer[3]);
			pdata->raw_humidity = unsigned_20_top_be(&buffer[1]);

			data->values[sensor_type_temperature].value = ((200.f * pdata->raw_temperature) / 1048576.f) - 50.0f;
			data->values[sensor_type_temperature].stamp = time(nullptr);

			data->values[sensor_type_humidity].value = pdata->raw_humidity * 100.f / 1048576.f;
			data->values[sensor_type_humidity].stamp = time(nullptr);

			pdata->valid = true;
			pdata->state = asair_state_start_measure;

			break;
		}
	}

	return(true);
}

static void asair_dump(const data_t *data, std::string &output)
{
	asair_private_data_t *pdata = static_cast<asair_private_data_t *>(data->private_data);

	assert(pdata);

	output += (boost::format("state: %u, ") % pdata->state).str();
	output += (boost::format("type: %u, ") % pdata->type).str();
	output += (boost::format("valid: %u, ") % (unsigned int)pdata->valid).str();
	output += (boost::format("raw temperature: %u, ") % pdata->raw_temperature).str();
	output += (boost::format("raw humidity: %u") % pdata->raw_temperature).str();
};

typedef enum : unsigned int
{
	tsl2561_reg_control =		0x00,
	tsl2561_reg_timeint =		0x01,
	tsl2561_reg_threshlow =		0x02,
	tsl2561_reg_threshhigh =	0x04,
	tsl2561_reg_interrupt =		0x06,
	tsl2561_reg_crc =			0x08,
	tsl2561_reg_id =			0x0a,
	tsl2561_reg_data0 =			0x0c,
	tsl2561_reg_data1 =			0x0e,
} tsl2561_reg_t;

enum : unsigned int
{
	tsl2561_cmd_address =	(1 << 0) | (1 << 1) | (1 << 2) | (1 << 3),
	tsl2561_cmd_block =		1 << 4,
	tsl2561_cmd_word =		1 << 5,
	tsl2561_cmd_clear =		1 << 6,
	tsl2561_cmd_cmd =		1 << 7,
};

enum : unsigned int
{
	tsl2561_tim_integ_13ms	=	(0 << 1) | (0 << 0),
	tsl2561_tim_integ_101ms	=	(0 << 1) | (1 << 0),
	tsl2561_tim_integ_402ms	=	(1 << 1) | (0 << 0),
	tsl2561_tim_manual		=	1 << 3,
	tsl2561_tim_low_gain	=	0 << 4,
	tsl2561_tim_high_gain	=	1 << 4,
};

enum : unsigned int
{
	tsl2561_ctrl_power_off =	0x00,
	tsl2561_ctrl_power_on =		0x03,

	tsl2561_id_tsl2561 =		0x50,
	tsl2561_probe_threshold =	0x00,
};

static constexpr unsigned int tsl2561_autoranging_data_size = 4;

typedef enum : unsigned int
{
	tsl2561_state_init,
	tsl2561_state_measuring,
	tsl2561_state_finished,
} tsl2561_state_t;

typedef struct
{
	tsl2561_state_t state;
	unsigned int overflows;
	unsigned int scaling_up;
	unsigned int scaling_down;
	unsigned int channel[2];
	unsigned int scaling;
} tsl2561_private_data_t;

static const device_autoranging_data_t tsl2561_autoranging_data[tsl2561_autoranging_data_size] =
{
	{{	tsl2561_tim_integ_402ms,	tsl2561_tim_high_gain	},	{	0,		50000	},	65535,	0.48	},
	{{	tsl2561_tim_integ_402ms,	tsl2561_tim_low_gain	},	{	256,	50000	},	65535,	7.4		},
	{{	tsl2561_tim_integ_101ms,	tsl2561_tim_low_gain	},	{	256,	30000	},	37177,	28		},
	{{	tsl2561_tim_integ_13ms,		tsl2561_tim_low_gain	},	{	256,	65536	},	5047,	200		},
};

static bool tsl2561_write_byte(i2c_slave_t slave, tsl2561_reg_t reg, unsigned int value)
{
	return(i2c_send_2(slave, tsl2561_cmd_cmd | tsl2561_cmd_clear | (static_cast<unsigned int>(reg) & tsl2561_cmd_address), value));
}

static bool tsl2561_read_byte(i2c_slave_t slave, tsl2561_reg_t reg, unsigned int *value)
{
	uint8_t buffer[1];

	if(!i2c_send_1_receive(slave, tsl2561_cmd_cmd | (static_cast<unsigned int>(reg) & tsl2561_cmd_address), sizeof(buffer), buffer))
		return(false);

	*value = buffer[0];

	return(true);
}

static bool tsl2561_read_word(i2c_slave_t slave, tsl2561_reg_t reg, unsigned int *value)
{
	uint8_t buffer[2];

	if(!i2c_send_1_receive(slave, tsl2561_cmd_cmd | (static_cast<unsigned int>(reg) & tsl2561_cmd_address), sizeof(buffer), buffer))
		return(false);

	*value = unsigned_16_le(buffer);

	return(true);
}

static bool tsl2561_write_check(i2c_slave_t slave, tsl2561_reg_t reg, unsigned int value)
{
	unsigned rv;

	if(!tsl2561_write_byte(slave, reg, value))
		return(false);

	if(!tsl2561_read_byte(slave, reg, &rv))
		return(false);

	if(value != rv)
		return(false);

	return(true);
}

static sensor_detect_t tsl2561_detect(i2c_slave_t slave)
{
	unsigned int regval;

	if(!tsl2561_read_byte(slave, tsl2561_reg_id, &regval))
		return(sensor_not_found);

	if(regval != tsl2561_id_tsl2561)
		return(sensor_not_found);

	if(!tsl2561_read_word(slave, tsl2561_reg_threshlow, &regval))
		return(sensor_not_found);

	if(regval != tsl2561_probe_threshold)
		return(sensor_not_found);

	if(!tsl2561_read_word(slave, tsl2561_reg_threshhigh, &regval))
		return(sensor_not_found);

	if(regval != tsl2561_probe_threshold)
		return(sensor_not_found);

	if(!tsl2561_write_check(slave, tsl2561_reg_control, tsl2561_ctrl_power_off))
		return(sensor_not_found);

	if(tsl2561_write_check(slave, tsl2561_reg_id, 0x00)) // id register should not be writable
		return(sensor_not_found);

	return(sensor_found);
}

static bool tsl2561_init(data_t *data)
{
	tsl2561_private_data_t *pdata = static_cast<tsl2561_private_data_t *>(data->private_data);
	unsigned int regval;

	assert(pdata);

	pdata->state = tsl2561_state_init;
	pdata->overflows = 0;
	pdata->scaling_up = 0;
	pdata->scaling_down = 0;
	pdata->scaling = tsl2561_autoranging_data_size - 1;
	pdata->channel[0] = 0;
	pdata->channel[1] = 0;

	if(!tsl2561_write_check(data->slave, tsl2561_reg_interrupt, 0x00))
	{
		Log::get() << "tsl2561: init: error 1";
		return(false);
	}

	if(!tsl2561_write_byte(data->slave, tsl2561_reg_control, tsl2561_ctrl_power_on))
	{
		Log::get() << "tsl2561: init: error 2";
		return(false);
	}

	if(!tsl2561_read_byte(data->slave, tsl2561_reg_control, &regval))
	{
		Log::get() << "tsl2561: init: error 3";
		return(false);
	}

	if((regval & 0x0f) != tsl2561_ctrl_power_on)
	{
		Log::get() << "tsl2561: init: error 4";
		return(false);
	}

	return(true);
}

static bool tsl2561_poll(data_t *data)
{
	tsl2561_private_data_t *pdata = static_cast<tsl2561_private_data_t *>(data->private_data);
	unsigned int overflow, scale_down_threshold, scale_up_threshold;
	float value, ratio;

	assert(pdata);

	switch(pdata->state)
	{
		case(tsl2561_state_init):
		case(tsl2561_state_finished):
		{
			if(!tsl2561_write_check(data->slave, tsl2561_reg_timeint, tsl2561_autoranging_data[pdata->scaling].data[0] | tsl2561_autoranging_data[pdata->scaling].data[1]))
			{
				Log::get() << "tsl2561: poll: error 1";
				return(false);
			}

			pdata->state = tsl2561_state_measuring;

			break;
		}

		case(tsl2561_state_measuring):
		{
			scale_down_threshold =	tsl2561_autoranging_data[pdata->scaling].threshold.down;
			scale_up_threshold =	tsl2561_autoranging_data[pdata->scaling].threshold.up;
			overflow =				tsl2561_autoranging_data[pdata->scaling].overflow;

			pdata->state = tsl2561_state_finished;

			if(!tsl2561_read_word(data->slave, tsl2561_reg_data0, &pdata->channel[0]))
			{
				Log::get() << "tsl2561: poll: error 2";
				return(false);
			}

			if(!tsl2561_read_word(data->slave, tsl2561_reg_data1, &pdata->channel[1]))
			{
				Log::get() << "tsl2561: poll: error 3";
				return(false);
			}

			if(((pdata->channel[0] >= overflow) || (pdata->channel[1] >= overflow)) && (pdata->scaling >= (tsl2561_autoranging_data_size - 1)))
			{
				pdata->overflows++;
				break;
			}

			if(((pdata->channel[0] < scale_down_threshold) || (pdata->channel[1] < scale_down_threshold)) && (pdata->scaling > 0))
			{
				pdata->scaling--;
				pdata->scaling_down++;
				break;
			}

			if(((pdata->channel[0] >= scale_up_threshold) || (pdata->channel[1] >= scale_up_threshold)) && (pdata->scaling < (tsl2561_autoranging_data_size - 1)))
			{
				pdata->scaling++;
				pdata->scaling_up++;

				break;
			}

			if(pdata->channel[0] == 0)
				ratio = 0;
			else
				ratio = pdata->channel[1] / pdata->channel[0];

			if(ratio > 1.30f)
				value = -1;
			else
			{
				if(ratio >= 0.80f)
					value = (0.00146f * pdata->channel[0]) - (0.00112f * pdata->channel[1]);
				else
					if(ratio >= 0.61f)
						value = (0.0128f * pdata->channel[0]) - (0.0153f * pdata->channel[1]);
					else
						if(ratio >= 0.50f)
							value = (0.0224f * pdata->channel[0]) - (0.031f * pdata->channel[1]);
						else
							value = (0.0304f * pdata->channel[0]) - (0.062f * pdata->channel[1] * (float)pow(ratio, 1.4f));

				value = (value * tsl2561_autoranging_data[pdata->scaling].factor);

				if(value < 0)
					value = 0;
			}

			data->values[sensor_type_visible_light].value = value;
			data->values[sensor_type_visible_light].stamp = time(nullptr);

			break;
		}
	}

	return(true);
}

static void tsl2561_dump(const data_t *data, std::string &output)
{
	tsl2561_private_data_t *pdata = static_cast<tsl2561_private_data_t *>(data->private_data);

	assert(pdata);

	output += (boost::format("state 0: %u, ") % pdata->state).str();
	output += (boost::format("scaling: %u, ") % pdata->scaling).str();
	output += (boost::format("scaling up: %u, ") % pdata->scaling_up).str();
	output += (boost::format("scaling down: %u, ") % pdata->scaling_down).str();
	output += (boost::format("overflows: %u, ") % pdata->overflows).str();
	output += (boost::format("channel 0: %u, ") % pdata->channel[0]).str();
	output += (boost::format("channel 1: %u") % pdata->channel[1]).str();
}

enum : unsigned int
{
	hdc1080_reg_data_temp =	0x00,
	hdc1080_reg_data_hum =	0x01,
	hdc1080_reg_conf =		0x02,
	hdc1080_reg_serial1 =	0xfb,
	hdc1080_reg_serial2 =	0xfc,
	hdc1080_reg_serial3 =	0xfd,
	hdc1080_reg_man_id =	0xfe,
	hdc1080_reg_dev_id =	0xff,

	hdc1080_man_id =		0x5449,
	hdc1080_dev_id =		0x1050,

	hdc1080_conf_rst =		0b1000'0000'0000'0000,
	hdc1080_conf_reservd0 =	0b0100'0000'0000'0000,
	hdc1080_conf_heat =		0b0010'0000'0000'0000,
	hdc1080_conf_mode_two =	0b0001'0000'0000'0000,
	hdc1080_conf_mode_one =	0b0000'0000'0000'0000,
	hdc1080_conf_btst =		0b0000'1000'0000'0000,
	hdc1080_conf_tres_11 =	0b0000'0100'0000'0000,
	hdc1080_conf_tres_14 =	0b0000'0000'0000'0000,
	hdc1080_conf_hres_8 =	0b0000'0010'0000'0000,
	hdc1080_conf_hres_11 =	0b0000'0001'0000'0000,
	hdc1080_conf_hres_14 =	0b0000'0000'0000'0000,
	hdc1080_conf_reservd1 =	0b0000'0000'1111'1111,
};

typedef enum : unsigned int
{
	hdc1080_state_init,
	hdc1080_state_reset,
	hdc1080_state_ready,
	hdc1080_state_measuring,
	hdc1080_state_finished,
} hdc1080_state_t;

typedef struct
{
	hdc1080_state_t state;
	bool valid;
	unsigned int raw_temperature;
	unsigned int raw_humidity;
} hdc1080_private_data_t;

static bool hdc1080_write_word(i2c_slave_t slave, unsigned int reg, unsigned int word)
{
	uint8_t buffer[3];

	buffer[0] = reg;
	buffer[1] = (word & 0xff00) >> 8;
	buffer[2] = (word & 0x00ff) >> 0;

	return(i2c_send(slave, sizeof(buffer), buffer));
}

static sensor_detect_t hdc1080_detect(i2c_slave_t slave)
{
	uint8_t buffer[2];

	if(!i2c_send_1_receive(slave, hdc1080_reg_man_id, sizeof(buffer), buffer))
		return(sensor_not_found);

	if(unsigned_16_be(buffer) != hdc1080_man_id)
		return(sensor_not_found);

	if(!i2c_send_1_receive(slave, hdc1080_reg_dev_id, sizeof(buffer), buffer))
		return(sensor_not_found);

	if(unsigned_16_be(buffer) != hdc1080_dev_id)
		return(sensor_not_found);

	return(sensor_found);
}

static bool hdc1080_init(data_t *data)
{
	hdc1080_private_data_t *pdata = static_cast<hdc1080_private_data_t *>(data->private_data);

	assert(pdata);

	pdata->state = hdc1080_state_init;

	if(!hdc1080_write_word(data->slave, hdc1080_reg_conf, hdc1080_conf_rst))
	{
		Log::get() << "hdc1080: init failed";
		return(false);
	}

	pdata->raw_temperature = 0;
	pdata->raw_humidity = 0;
	pdata->valid = false;

	pdata->state = hdc1080_state_reset;

	return(true);
}

static bool hdc1080_poll(data_t *data)
{
	hdc1080_private_data_t *pdata = static_cast<hdc1080_private_data_t *>(data->private_data);
	static constexpr unsigned int conf = hdc1080_conf_tres_14 | hdc1080_conf_hres_14 | hdc1080_conf_mode_two;
	uint8_t buffer[4];

	assert(pdata);

	pdata->valid = false;

	switch(pdata->state)
	{
		case(hdc1080_state_init):
		{
			Log::get() << "hdc1080: invalid state";
			pdata->state = hdc1080_state_reset;
			break;
		}

		case(hdc1080_state_reset):
		{
			if(!i2c_send_1_receive(data->slave, hdc1080_reg_conf, 2, buffer))
			{
				Log::get() << "hdc1080: poll error 1";
				return(false);
			}

			if(unsigned_16_le(buffer) & hdc1080_conf_rst)
			{
				Log::get() << "hdc1080: poll error 2";
				return(false);
			}

			if(!hdc1080_write_word(data->slave, hdc1080_reg_conf, conf))
			{
				Log::get() << "hdc1080: poll error 3";
				return(false);
			}

			pdata->state = hdc1080_state_ready;

			break;
		}

		case(hdc1080_state_ready):
		case(hdc1080_state_finished):
		{
			pdata->valid = false;

			if(!i2c_send_1(data->slave, hdc1080_reg_data_temp))
			{
				Log::get() << "hdc1080: poll error 4";
				return(false);
			}

			pdata->state = hdc1080_state_measuring;

			break;
		}

		case(hdc1080_state_measuring):
		{
			pdata->state = hdc1080_state_finished;

			if(!i2c_receive(data->slave, sizeof(buffer), buffer))
			{
				Log::get() << "hdc1080 poll error 5";
				return(false);
			}

			pdata->raw_temperature = unsigned_16_be(&buffer[0]);
			pdata->raw_humidity = unsigned_16_be(&buffer[2]);
			pdata->valid = true;

			data->values[sensor_type_temperature].value = ((pdata->raw_temperature * 165.0f) / (float)(1 << 16)) - 40.f;
			data->values[sensor_type_temperature].stamp = time(nullptr);
			data->values[sensor_type_humidity].value = (pdata->raw_humidity * 100.0f) / 65536.0f;
			data->values[sensor_type_humidity].stamp = time(nullptr);

			break;
		}
	}

	return(true);
}

static void hdc1080_dump(const data_t *data, std::string &output)
{
	hdc1080_private_data_t *pdata = static_cast<hdc1080_private_data_t *>(data->private_data);

	assert(pdata);

	output += (boost::format("state: %u, ") % pdata->state).str();
	output += (boost::format("valid: %u, ") % (unsigned int)pdata->valid).str();
	output += (boost::format("raw temperature: %u, ") % pdata->raw_temperature).str();
	output += (boost::format("raw humidity: %u") % pdata->raw_humidity).str();
}

typedef enum : unsigned int
{
	sht3x_cmd_single_meas_clock_high =		0x2c06,
	sht3x_cmd_single_meas_clock_medium =	0x2c0d,
	sht3x_cmd_single_meas_clock_low =		0x2c10,

	sht3x_cmd_single_meas_noclock_high =	0x2400,
	sht3x_cmd_single_meas_noclock_medium =	0x240b,
	sht3x_cmd_single_meas_noclock_low =		0x2416,

	sht3x_cmd_auto_meas_high_05 =			0x2032,
	sht3x_cmd_auto_meas_medium_05 =			0x2024,
	sht3x_cmd_auto_meas_low_05 =			0x202f,

	sht3x_cmd_auto_meas_high_1 =			0x2130,
	sht3x_cmd_auto_meas_medium_1 =			0x2126,
	sht3x_cmd_auto_meas_low_1 =				0x212d,

	sht3x_cmd_auto_meas_high_2 =			0x2236,
	sht3x_cmd_auto_meas_medium_2 =			0x2220,
	sht3x_cmd_auto_meas_low_2 =				0x222b,

	sht3x_cmd_auto_meas_high_4 =			0x2334,
	sht3x_cmd_auto_meas_medium_4 =			0x2322,
	sht3x_cmd_auto_meas_low_4 =				0x2329,

	sht3x_cmd_auto_meas_high_10 =			0x2737,
	sht3x_cmd_auto_meas_medium_10 =			0x2721,
	sht3x_cmd_auto_meas_low_10 =			0x272a,

	sht3x_cmd_fetch_data =					0xe000,
	sht3x_cmd_art =							0x2b32,
	sht3x_cmd_break =						0x3093,
	sht3x_cmd_reset =						0x30a2,
	sht3x_cmd_heater_en =					0x306d,
	sht3x_cmd_heater_dis =					0x3066,
	sht3x_cmd_read_status =					0xf32d,
	sht3x_cmd_clear_status =				0x3041,
} sht3x_cmd_t;

enum : unsigned int
{
	sht3x_status_none =				0x00,
	sht3x_status_write_checksum =	(1 << 0),
	sht3x_status_command_status =	(1 << 1),
	sht3x_status_reset_detected =	(1 << 4),
	sht3x_status_temp_track_alert =	(1 << 10),
	sht3x_status_hum_track_alert =	(1 << 11),
	sht3x_status_heater =			(1 << 13),
	sht3x_status_alert =			(1 << 15),
};

typedef enum : unsigned int
{
	sht3x_state_init,
	sht3x_state_reset,
	sht3x_state_ready,
	sht3x_state_measuring,
	sht3x_state_finished,
} sht3x_state_t;

typedef struct
{
	sht3x_state_t state;
	bool valid;
	unsigned int raw_temperature;
	unsigned int raw_humidity;
} sht3x_private_data_t;

static uint8_t sht3x_crc8(int length, const uint8_t *data)
{
	uint8_t outer, inner, testbit, crc;

	crc = 0xff;

	for(outer = 0; (int)outer < length; outer++)
	{
		crc ^= data[outer];

		for(inner = 0; inner < 8; inner++)
		{
			testbit = !!(crc & 0x80);
			crc <<= 1;
			if(testbit)
				crc ^= 0x31;
		}
	}

	return(crc);
}

static bool sht3x_send_command(i2c_slave_t slave, unsigned int cmd)
{
	uint8_t cmd_bytes[2];

	cmd_bytes[0] = (cmd & 0xff00) >> 8;
	cmd_bytes[1] = (cmd & 0x00ff) >> 0;

	if(!i2c_send(slave, sizeof(cmd_bytes), cmd_bytes))
	{
		Log::get() << "sht3x: sht3x_send_command: error";
		return(false);
	}

	return(true);
}

static bool sht3x_receive_command(i2c_slave_t slave, sht3x_cmd_t cmd, unsigned int *result)
{
	uint8_t buffer[3];
	uint8_t crc_local, crc_remote;
	uint8_t cmd_bytes[2];

	cmd_bytes[0] = (cmd & 0xff00) >> 8;
	cmd_bytes[1] = (cmd & 0x00ff) >> 0;

	if(!i2c_send_receive(slave, sizeof(cmd_bytes), cmd_bytes, sizeof(buffer), buffer))
	{
		Log::get() << "sht3x: sht3x_receive_command: error";
		return(false);
	}

	crc_local = buffer[2];
	crc_remote = sht3x_crc8(2, &buffer[0]);

	if(crc_local != crc_remote)
	{
		Log::get() << "sht3x: sht3x_receive_command: invalid crc";
		return(false);
	}

	*result = unsigned_16_be(buffer);

	return(true);
}

static bool sht3x_fetch_data(i2c_slave_t slave, unsigned int *result1, unsigned int *result2)
{
	uint8_t buffer[6];
	uint8_t crc_local, crc_remote;
	uint8_t cmd_bytes[2];

	cmd_bytes[0] = (sht3x_cmd_fetch_data & 0xff00) >> 8;
	cmd_bytes[1] = (sht3x_cmd_fetch_data & 0x00ff) >> 0;

	if(!i2c_send_receive(slave, sizeof(cmd_bytes), cmd_bytes, sizeof(buffer), buffer))
	{
		Log::get() << "sht3x: sht3x_fetch_data: error";
		return(false);
	}

	crc_local = buffer[2];
	crc_remote = sht3x_crc8(2, &buffer[0]);

	if(crc_local != crc_remote)
	{
		Log::get() << "sht3x: sht3x_fetch_data: invalid crc [0]";
		return(false);
	}

	crc_local = buffer[5];
	crc_remote = sht3x_crc8(2, &buffer[3]);

	if(crc_local != crc_remote)
	{
		Log::get() << "sht3x: sht3x_fetch_data: invalid crc [1]";
		return(false);
	}

	*result1 = unsigned_16_be(&buffer[0]);
	*result2 = unsigned_16_be(&buffer[3]);

	return(true);
}

static sensor_detect_t sht3x_detect(i2c_slave_t slave)
{
	if(!sht3x_send_command(slave, sht3x_cmd_break))
	{
		Log::get() << "sht3x: detect error";
		return(sensor_not_found);
	}

	return(sensor_found);
}

static bool sht3x_init(data_t *data)
{
	sht3x_private_data_t *pdata = static_cast<sht3x_private_data_t *>(data->private_data);

	assert(pdata);

	pdata->state = sht3x_state_init;
	pdata->valid = false;
	pdata->raw_temperature = 0;
	pdata->raw_humidity = 0;

	return(true);
}

static bool sht3x_poll(data_t *data)
{
	sht3x_private_data_t *pdata = static_cast<sht3x_private_data_t *>(data->private_data);
	unsigned int result, results[2];

	assert(pdata);

	switch(pdata->state)
	{
		case(sht3x_state_init):
		{
			if(!sht3x_send_command(data->slave, sht3x_cmd_reset))
			{
				Log::get() << "sht3x: poll error 1";
				return(false);
			}

			pdata->state = sht3x_state_reset;

			break;
		}

		case(sht3x_state_reset):
		{
			if(!sht3x_receive_command(data->slave, sht3x_cmd_read_status, &result))
			{
				Log::get() << "sht3x: poll error 2";
				return(false);
			}

			if((result & (sht3x_status_write_checksum | sht3x_status_command_status)) != 0x00)
			{
				Log::get() << "sht3x: poll error 3";
				return(false);
			}

			if(!sht3x_send_command(data->slave, sht3x_cmd_clear_status))
			{
				Log::get() << "sht3x: poll error 4";
				return(false);
			}

			pdata->state = sht3x_state_ready;

			break;
		}

		case(sht3x_state_ready):
		{
			if(!sht3x_receive_command(data->slave, sht3x_cmd_read_status, &result))
			{
				Log::get() << "sht3x: poll error 5";
				return(false);
			}

			if((result & (sht3x_status_write_checksum | sht3x_status_command_status | sht3x_status_reset_detected)) != 0x00)
			{
				Log::get() << "sht3x: poll error 6";
				return(false);
			}

			pdata->state = sht3x_state_finished;

			break;
		}

		case(sht3x_state_finished):
		{
			pdata->valid = false;

			if(!sht3x_send_command(data->slave, sht3x_cmd_single_meas_noclock_high))
			{
				Log::get() << "sht3x: poll error 7";
				return(false);
			}

			pdata->state = sht3x_state_measuring;

			break;
		}

		case(sht3x_state_measuring):
		{
			pdata->state = sht3x_state_finished;

			if(!sht3x_fetch_data(data->slave, &results[0], &results[1]))
			{
				Log::get() << "sht3x: poll error 8";
				return(false);
			}

			pdata->raw_temperature = results[0];
			pdata->raw_humidity = results[1];

			data->values[sensor_type_temperature].value = (((float)pdata->raw_temperature * 175.f) / ((1 << 16) - 1.0f)) - 45.0f;
			data->values[sensor_type_temperature].stamp = time(nullptr);
			data->values[sensor_type_humidity].value = ((float)pdata->raw_humidity * 100.0f) / ((1 << 16) - 1.0f);
			data->values[sensor_type_humidity].stamp = time(nullptr);

			pdata->valid = true;

			break;
		}
	}

	return(true);
}

static void sht3x_dump(const data_t *data, std::string &output)
{
	sht3x_private_data_t *pdata = static_cast<sht3x_private_data_t *>(data->private_data);

	assert(pdata);

	output += (boost::format("state: %u, ") % pdata->state).str();
	output += (boost::format("valid: %u, ") % (unsigned int)pdata->valid).str();
	output += (boost::format("raw temperature: %u, ") % pdata->raw_temperature).str();
	output += (boost::format("raw humidity: %u") % pdata->raw_humidity).str();
}

enum : unsigned int
{
	bmx280_reg_id =						0xd0,
	bmx280_reg_reset =					0xe0,
	bmx280_reg_ctrl_hum =				0xf2,
	bmx280_reg_status =					0xf3,
	bmx280_reg_ctrl_meas =				0xf4,
	bmx280_reg_config =					0xf5,
	bmx280_reg_adc =					0xf7,
	bmx280_reg_adc_pressure_msb =		0xf7,
	bmx280_reg_adc_pressure_lsb =		0xf8,
	bmx280_reg_adc_pressure_xlsb =		0xf9,
	bmx280_reg_adc_temperature_msb =	0xfa,
	bmx280_reg_adc_temperature_lsb =	0xfb,
	bmx280_reg_adc_temperature_xlsb =	0xfc,
	bmx280_reg_adc_humidity_msb =		0xfd,
	bmx280_reg_adc_humidity_lsb =		0xfe,

	bmx280_reg_id_bmp280 =				0x58,
	bmx280_reg_id_bme280 =				0x60,

	bmx280_reg_reset_value =			0xb6,

	bmx280_reg_ctrl_hum_osrs_h_skip =	0b0000'0000,
	bmx280_reg_ctrl_hum_osrs_h_1 =		0b0000'0001,
	bmx280_reg_ctrl_hum_osrs_h_2 =		0b0000'0010,
	bmx280_reg_ctrl_hum_osrs_h_4 =		0b0000'0011,
	bmx280_reg_ctrl_hum_osrs_h_8 =		0b0000'0100,
	bmx280_reg_ctrl_hum_osrs_h_16 =		0b0000'0101,

	bmx280_reg_status_measuring =		0b00001'000,
	bmx280_reg_status_im_update =		0b0000'0001,

	bmx280_reg_ctrl_meas_osrs_t_skip =	0b0000'0000,
	bmx280_reg_ctrl_meas_osrs_t_1 =		0b0010'0000,
	bmx280_reg_ctrl_meas_osrs_t_2 =		0b0100'0000,
	bmx280_reg_ctrl_meas_osrs_t_4 =		0b0110'0000,
	bmx280_reg_ctrl_meas_osrs_t_8 =		0b1000'0000,
	bmx280_reg_ctrl_meas_osrs_t_16 =	0b1010'0000,
	bmx280_reg_ctrl_meas_osrs_p_skip =	0b0000'0000,
	bmx280_reg_ctrl_meas_osrs_p_1 =		0b0000'0100,
	bmx280_reg_ctrl_meas_osrs_p_2 =		0b0000'1000,
	bmx280_reg_ctrl_meas_osrs_p_4 =		0b0000'1100,
	bmx280_reg_ctrl_meas_osrs_p_8 =		0b0001'0000,
	bmx280_reg_ctrl_meas_osrs_p_16 =	0b0001'0100,
	bmx280_reg_ctrl_meas_mode_mask =	0b0000'0011,
	bmx280_reg_ctrl_meas_mode_sleep =	0b0000'0000,
	bmx280_reg_ctrl_meas_mode_forced =	0b0000'0010,
	bmx280_reg_ctrl_meas_mode_normal =	0b0000'0011,

	bmx280_reg_config_t_sb_05 =			0b0000'0000,
	bmx280_reg_config_t_sb_62 =			0b0010'0000,
	bmx280_reg_config_t_sb_125 =		0b0100'0000,
	bmx280_reg_config_t_sb_250 =		0b0110'0000,
	bmx280_reg_config_t_sb_500 =		0b1000'0000,
	bmx280_reg_config_t_sb_1000 =		0b1010'0000,
	bmx280_reg_config_t_sb_10000 =		0b1100'0000,
	bmx280_reg_config_t_sb_20000 =		0b1110'0000,
	bmx280_reg_config_filter_off =		0b0000'0000,
	bmx280_reg_config_filter_2 =		0b0000'0100,
	bmx280_reg_config_filter_4 =		0b0000'1000,
	bmx280_reg_config_filter_8 =		0b0000'1100,
	bmx280_reg_config_filter_16 =		0b0001'0000,
	bmx280_reg_config_spi3w_en =		0b0000'0001,
};

enum : unsigned int
{
	bmx280_cal_base =						0x88,
	bmx280_cal_0x88_0x89_dig_t1 =			0x88 - bmx280_cal_base,
	bmx280_cal_0x8a_0x8b_dig_t2 =			0x8a - bmx280_cal_base,
	bmx280_cal_0x8c_0x8d_dig_t3 =			0x8c - bmx280_cal_base,
	bmx280_cal_0x8e_0x8f_dig_p1 =			0x8e - bmx280_cal_base,
	bmx280_cal_0x90_0x91_dig_p2 =			0x90 - bmx280_cal_base,
	bmx280_cal_0x92_0x93_dig_p3 =			0x92 - bmx280_cal_base,
	bmx280_cal_0x94_0x95_dig_p4 =			0x94 - bmx280_cal_base,
	bmx280_cal_0x96_0x97_dig_p5 =			0x96 - bmx280_cal_base,
	bmx280_cal_0x98_0x99_dig_p6 =			0x98 - bmx280_cal_base,
	bmx280_cal_0x9a_0x9b_dig_p7 =			0x9a - bmx280_cal_base,
	bmx280_cal_0x9c_0x9d_dig_p8 =			0x9c - bmx280_cal_base,
	bmx280_cal_0x9e_0x9f_dig_p9 =			0x9e - bmx280_cal_base,
	bmx280_cal_0xa1_dig_h1 =				0xa1 - bmx280_cal_base,
	bmx280_cal_0xe1_0xe2_dig_h2 =			0xe1 - bmx280_cal_base,
	bmx280_cal_0xe3_dig_h3 =				0xe3 - bmx280_cal_base,
	bmx280_cal_0xe4_0xe5_0xe6_dig_h4_h5 =	0xe4 - bmx280_cal_base,
	bmx280_cal_0xe7_dig_h6 =				0xe7 - bmx280_cal_base,
	bmx280_cal_size =						0xe8 - bmx280_cal_base,
};

typedef enum : unsigned int
{
	bmx280_state_init,
	bmx280_state_reset,
	bmx280_state_ready,
	bmx280_state_measuring,
	bmx280_state_finished,
} bmx280_state_t;

typedef struct
{
	unsigned int type;
	bmx280_state_t state;
	unsigned int adc_temperature;
	unsigned int adc_airpressure;
	unsigned int adc_humidity;
	float t_fine;
	float t_fine_2;
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
} bmx280_private_data_t;

static bool bmx280_read_otp(data_t *data)
{
	bmx280_private_data_t *pdata = static_cast<bmx280_private_data_t *>(data->private_data);
	uint8_t cal_data[bmx280_cal_size];
	uint8_t buffer[1];
	unsigned int e4, e5, e6;

	assert(pdata);

	if(!i2c_send_1_receive(data->slave, bmx280_reg_id, sizeof(buffer), buffer))
	{
		Log::get() << "bmx280: error read otp data 1";
		return(false);
	}

	pdata->type = buffer[0];

	if(!i2c_send_1_receive(data->slave, bmx280_cal_base, sizeof(cal_data), cal_data))
	{
		Log::get() << "bmx280: error read otp data 2";
		return(false);
	}

	pdata->t1 = unsigned_16_le(&cal_data[bmx280_cal_0x88_0x89_dig_t1]);
	pdata->t2 = signed_16_le(&cal_data[bmx280_cal_0x8a_0x8b_dig_t2]);
	pdata->t3 = signed_16_le(&cal_data[bmx280_cal_0x8c_0x8d_dig_t3]);
	pdata->p1 = unsigned_16_le(&cal_data[bmx280_cal_0x8e_0x8f_dig_p1]);
	pdata->p2 = signed_16_le(&cal_data[bmx280_cal_0x90_0x91_dig_p2]);
	pdata->p3 = signed_16_le(&cal_data[bmx280_cal_0x92_0x93_dig_p3]);
	pdata->p4 = signed_16_le(&cal_data[bmx280_cal_0x94_0x95_dig_p4]);
	pdata->p5 = signed_16_le(&cal_data[bmx280_cal_0x96_0x97_dig_p5]);
	pdata->p6 = signed_16_le(&cal_data[bmx280_cal_0x98_0x99_dig_p6]);
	pdata->p7 = signed_16_le(&cal_data[bmx280_cal_0x9a_0x9b_dig_p7]);
	pdata->p8 = signed_16_le(&cal_data[bmx280_cal_0x9c_0x9d_dig_p8]);
	pdata->p9 = signed_16_le(&cal_data[bmx280_cal_0x9e_0x9f_dig_p9]);

	if(pdata->type == bmx280_reg_id_bme280)
	{
		pdata->h1 = cal_data[bmx280_cal_0xa1_dig_h1];
		pdata->h2 = signed_16_le(&cal_data[bmx280_cal_0xe1_0xe2_dig_h2]);
		pdata->h3 = cal_data[bmx280_cal_0xe3_dig_h3];
		e4 = cal_data[bmx280_cal_0xe4_0xe5_0xe6_dig_h4_h5 + 0];
		e5 = cal_data[bmx280_cal_0xe4_0xe5_0xe6_dig_h4_h5 + 1];
		e6 = cal_data[bmx280_cal_0xe4_0xe5_0xe6_dig_h4_h5 + 2];
		pdata->h4 = ((e4 & 0xff) << 4) | ((e5 & 0x0f) >> 0);
		pdata->h5 = ((e6 & 0xff) << 4) | ((e5 & 0xf0) >> 4);
		pdata->h6 = cal_data[bmx280_cal_0xe7_dig_h6];
	}

	return(true);
}

static sensor_detect_t bmx280_detect(i2c_slave_t slave)
{
	uint8_t	buffer[1];

	if(!i2c_send_1_receive(slave, bmx280_reg_id, sizeof(buffer), buffer))
		return(sensor_not_found);

	if((buffer[0] != bmx280_reg_id_bmp280) && (buffer[0] != bmx280_reg_id_bme280))
		return(sensor_not_found);

	return(sensor_found);
}

static bool bmx280_init(data_t *data)
{
	bmx280_private_data_t *pdata = static_cast<bmx280_private_data_t *>(data->private_data);
	uint8_t	buffer[1];

	assert(pdata);

	pdata->type = 0;
	pdata->adc_temperature = 0;
	pdata->adc_airpressure = 0;
	pdata->adc_humidity = 0;
	pdata->t_fine = 0;
	pdata->t_fine_2 = 0;
	pdata->state = bmx280_state_init;

	if(!i2c_send_2(data->slave, bmx280_reg_reset, bmx280_reg_reset_value))
	{
		Log::get() << "bmx280: init error 1";
		return(false);
	}

	if(!i2c_send_1_receive(data->slave, bmx280_reg_reset, sizeof(buffer), buffer))
	{
		Log::get() << "bmx280: init error 2";
		return(false);
	}

	if(buffer[0] != 0x00)
	{
		Log::get() << "bmx280: init error 3";
		return(false);
	}

	pdata->state = bmx280_state_reset;

	return(true);
}

static bool bmx280_poll(data_t *data)
{
	bmx280_private_data_t *pdata = static_cast<bmx280_private_data_t *>(data->private_data);
	uint8_t buffer[8];
	float var1, var2, airpressure, humidity;

	assert(pdata);

	switch(pdata->state)
	{
		case(bmx280_state_init):
		{
			Log::get() << "bmx280: poll: invalid state";
			pdata->state = bmx280_state_reset;

			return(false);
		}

		case(bmx280_state_reset):
		{
			if(!bmx280_read_otp(data))
			{
				Log::get() << "bmx280_init: cannot read OTP data";
				return(false);
			}

			if((pdata->t1 > 0) && (pdata->t2 > 0))
				pdata->state = bmx280_state_ready;

			break;
		}

		case(bmx280_state_ready):
		case(bmx280_state_finished):
		{
			if(!i2c_send_1_receive(data->slave, bmx280_reg_ctrl_meas, 1, buffer))
			{
				Log::get() << "bmx280: poll error 1";
				return(false);
			}

			if((buffer[0] & bmx280_reg_ctrl_meas_mode_mask) != bmx280_reg_ctrl_meas_mode_sleep)
			{
				Log::get() << "bmx280: poll error 2";
				return(false);
			}

			if(!i2c_send_2(data->slave, bmx280_reg_ctrl_hum, bmx280_reg_ctrl_hum_osrs_h_16))
			{
				Log::get() << "bmx280: poll error 3";
				return(false);
			}

			if(!i2c_send_2(data->slave, bmx280_reg_config, bmx280_reg_config_filter_2))
			{
				Log::get() << "bmx280: poll error 4";
				return(false);
			}

			if(!i2c_send_2(data->slave, bmx280_reg_ctrl_meas, bmx280_reg_ctrl_meas_osrs_t_16 | bmx280_reg_ctrl_meas_osrs_p_16 | bmx280_reg_ctrl_meas_mode_forced))
			{
				Log::get() << "bmx280: poll error 5";
				return(false);
			}

			pdata->state = bmx280_state_measuring;

			break;
		}

		case(bmx280_state_measuring):
		{
			if(!i2c_send_1_receive(data->slave, bmx280_reg_adc, sizeof(buffer), buffer))
			{
				Log::get() << "bmx280: poll error 6";
				return(false);
			}

			pdata->adc_airpressure = unsigned_20_top_be(&buffer[0]);
			pdata->adc_temperature = unsigned_20_top_be(&buffer[3]);
			pdata->adc_humidity = unsigned_16_be(&buffer[6]);

			var1 = ((pdata->adc_temperature / 16384.0f) - (pdata->t1 / 1024.0f)) * pdata->t2;
			var2 = ((pdata->adc_temperature / 131072.0f) - (pdata->t1 / 8192.0f)) * ((pdata->adc_temperature / 131072.0f) - (pdata->t1 / 8192.0f)) * pdata->t3;

			data->values[sensor_type_temperature].value = (var1 + var2) / 5120.0f;
			data->values[sensor_type_temperature].stamp = time(nullptr);

			var1 = (pdata->adc_temperature / 16384.0f - pdata->t1 / 1024.0f) * pdata->t2;
			var2 = (pdata->adc_temperature / 131072.0f - pdata->t1 / 8192.0f) * (pdata->adc_temperature / 131072.0f - pdata->t1 / 8192.0f) * pdata->t3;
			pdata->t_fine = var1 + var2;

			var1 = (pdata->t_fine / 2.0f) - 64000.0f;
			var2 = var1 * var1 * pdata->p6 / 32768.0f;
			var2 = var2 + var1 * pdata->p5 * 2.0f;
			var2 = (var2 / 4.0f) + (pdata->p4 * 65536.0f);
			var1 = (pdata->p3 * var1 * var1 / 524288.0f + pdata->p2 * var1) / 524288.0f;
			var1 = (1.0f + var1 / 32768.0f) * pdata->p1;

			if((int)var1 == 0)
				airpressure = 0;
			else
			{
				airpressure = 1048576.0f - pdata->adc_airpressure;
				airpressure = (airpressure - (var2 / 4096.0f)) * 6250.0f / var1;
				var1 = pdata->p9 * airpressure * airpressure / 2147483648.0f;
				var2 = airpressure * pdata->p8 / 32768.0f;
				airpressure = airpressure + (var1 + var2 + pdata->p7) / 16.0f;
			}

			data->values[sensor_type_airpressure].value = airpressure / 100.0f;
			data->values[sensor_type_airpressure].stamp = time(nullptr);

			if(pdata->type == bmx280_reg_id_bme280)
			{
				var1 = (pdata->adc_temperature / 16384.0f	- (pdata->t1 / 1024.0f)) * pdata->t2;
				var2 = (pdata->adc_temperature / 131072.0f	- (pdata->t1 / 8192.0f)) * ((pdata->adc_temperature / 131072.0f) - (pdata->t1 / 8192.0f)) * pdata->t3;
				pdata->t_fine_2 = var1 + var2 - 76800;

				humidity = (pdata->adc_humidity - ((pdata->h4 * 64.0f) + (pdata->h5 / 16384.0f) * pdata->t_fine_2)) * (pdata->h2 / 65536.0f * (1.0f + pdata->h6 / 67108864.0f * pdata->t_fine_2 * (1.0f + pdata->h3 / 67108864.0f * pdata->t_fine_2)));
				humidity = humidity * (1.0f - pdata->h1 * humidity / 524288.0f);

				if(humidity > 100.0f)
					humidity = 100.0f;

				if(humidity < 0.0f)
					humidity = 0.0f;

				data->values[sensor_type_humidity].value = humidity;
				data->values[sensor_type_humidity].stamp = time(nullptr);
			}
			else
			{
				data->values[sensor_type_humidity].value = 0;
				data->values[sensor_type_humidity].stamp = 0;
			}

			pdata->state = bmx280_state_finished;

			break;
		}
	}

	return(true);
}

static void bmx280_dump(const data_t *data, std::string &output)
{
	bmx280_private_data_t *pdata = static_cast<bmx280_private_data_t *>(data->private_data);

	assert(pdata);

	output += (boost::format("type: %02x, ") % pdata->type).str();
	output += (boost::format("state: %u, ") % pdata->state).str();
	output += (boost::format("adc temp: %u, ") % pdata->adc_temperature).str();
	output += (boost::format("adc pressure: %u, ") % pdata->adc_airpressure).str();
	output += (boost::format("adc humidity: %u, ") % pdata->adc_humidity).str();
	output += (boost::format("t_fine: %f, ") % (double)pdata->t_fine).str();
	output += (boost::format("t_fine_2: %f, ") % (double)pdata->t_fine_2).str();
	output += (boost::format("t1: %u, ") % pdata->t1).str();
	output += (boost::format("t2: %d, ") % pdata->t2).str();
	output += (boost::format("t3: %d, ") % pdata->t3).str();
	output += (boost::format("p1: %u, ") % pdata->p1).str();
	output += (boost::format("p2: %d, ") % pdata->p2).str();
	output += (boost::format("p3: %d, ") % pdata->p3).str();
	output += (boost::format("p4: %d, ") % pdata->p4).str();
	output += (boost::format("p5: %d, ") % pdata->p5).str();
	output += (boost::format("p6: %d, ") % pdata->p6).str();
	output += (boost::format("p7: %d, ") % pdata->p7).str();
	output += (boost::format("p8: %d, ") % pdata->p8).str();
	output += (boost::format("p9: %d, ") % pdata->p9).str();
	output += (boost::format("h1: %u, ") % pdata->h1).str();
	output += (boost::format("h2: %d, ") % pdata->h2).str();
	output += (boost::format("h3: %u, ") % pdata->h3).str();
	output += (boost::format("h4: %d, ") % pdata->h4).str();
	output += (boost::format("h5: %d, ") % pdata->h5).str();
	output += (boost::format("h6: %u") % pdata->h6).str();
};

enum : unsigned int
{
	htu21_cmd_meas_temp_hold_master =		0xe3,
	htu21_cmd_meas_hum_hold_master =		0xe5,
	htu21_cmd_write_user =					0xe6,
	htu21_cmd_read_user =					0xe7,
	htu21_cmd_meas_temp_no_hold_master =	0xf3,
	htu21_cmd_meas_hum_no_hold_master =		0xf5,
	htu21_cmd_reset =						0xfe,

	htu21_user_reg_rh12_temp14 =			0b0000'0000,
	htu21_user_reg_rh8_temp12 =				0b0000'0001,
	htu21_user_reg_rh10_temp13 =			0b1000'0000,
	htu21_user_reg_rh11_temp11 =			0b1000'0001,
	htu21_user_reg_bat_stat =				0b0100'0000,
	htu21_user_reg_reserved =				0b0011'1000,
	htu21_user_reg_heater_enable =			0b0000'0100,
	htu21_user_reg_otp_reload_disable =		0b0000'0010,

	htu21_status_mask =						0b0000'0011,
	htu21_status_measure_temperature =		0b0000'0000,
	htu21_status_measure_humidity =			0b0000'0010,

	htu21_delay_reset =						2,
};

typedef enum : unsigned int
{
	htu21_state_init,
	htu21_state_reset,
	htu21_state_ready,
	htu21_state_measuring_temperature,
	htu21_state_finished_temperature,
	htu21_state_measuring_humidity,
	htu21_state_finished_humidity,
	htu21_state_finished,
} htu21_state_t;

typedef struct
{
	htu21_state_t state;
	unsigned int raw_temperature;
	unsigned int raw_humidity;
} htu21_private_data_t;

static uint8_t htu21_crc8(int length, const uint8_t *data)
{
	uint8_t outer, inner, testbit, crc;

	crc = 0;

	for(outer = 0; (int)outer < length; outer++)
	{
		crc ^= data[outer];

		for(inner = 0; inner < 8; inner++)
		{
			testbit = !!(crc & 0x80);
			crc <<= 1;
			if(testbit)
				crc ^= 0x31;
		}
	}

	return(crc);
}

static bool htu21_get_data(data_t *data, unsigned int *result)
{
	uint8_t	buffer[4];
	uint8_t crc1, crc2;

	if(!i2c_receive(data->slave, sizeof(buffer), buffer))
	{
		Log::get() << "htu21_get_data: error\n";
		return(false);
	}

	crc1 = buffer[2];
	crc2 = htu21_crc8(2, &buffer[0]);

	if(crc1 != crc2)
	{
		Log::get() << "htu21_get_data: crc invalid\n";
		return(false);
	}

	*result = unsigned_16_be(buffer) & ~htu21_status_mask;

	return(true);
}

static sensor_detect_t htu21_detect(i2c_slave_t slave)
{
	uint8_t buffer[1];

	if(!i2c_send_1_receive(slave, htu21_cmd_read_user, sizeof(buffer), buffer))
		return(sensor_not_found);

	return(sensor_found);
}

static bool htu21_init(data_t *data)
{
	htu21_private_data_t *pdata = static_cast<htu21_private_data_t *>(data->private_data);

	assert(pdata);

	pdata->state = htu21_state_init;

	pdata->raw_temperature = 0;
	pdata->raw_humidity = 0;

	return(true);
}

static bool htu21_poll(data_t *data)
{
	htu21_private_data_t *pdata = static_cast<htu21_private_data_t *>(data->private_data);
	unsigned int result;
	float temperature, humidity;
	uint8_t cmd[2];
	uint8_t buffer[1];

	assert(pdata);

	switch(pdata->state)
	{
		case(htu21_state_init):
		{
			i2c_send_1(data->slave, htu21_cmd_reset);

			pdata->state = htu21_state_reset;

			break;
		}

		case(htu21_state_reset):
		{
			if((!i2c_send_1_receive(data->slave, htu21_cmd_read_user, 1, &cmd[1])))
			{
				Log::get() << "htu21: poll: error 1";
				break;
			}

			cmd[0] = htu21_cmd_write_user;
			cmd[1] &= (htu21_user_reg_reserved | htu21_user_reg_bat_stat);
			cmd[1] |= htu21_user_reg_rh11_temp11 | htu21_user_reg_otp_reload_disable;

			if(!i2c_send(data->slave, sizeof(cmd), cmd))
			{
				Log::get() << "htu21: poll: error 2";
				break;
			}

			if(!i2c_send_1_receive(data->slave, htu21_cmd_read_user, sizeof(buffer), buffer))
			{
				Log::get() << "htu21: poll: error 3";
				break;
			}

			buffer[0] &= ~(htu21_user_reg_reserved | htu21_user_reg_bat_stat);

			if(buffer[0] != (htu21_user_reg_rh11_temp11 | htu21_user_reg_otp_reload_disable))
			{
				Log::get() << "htu21: poll: error 4";
				data->state = sensor_disabled;
				break;
			}

			pdata->state = htu21_state_ready;

			break;
		}

		case(htu21_state_ready):
		case(htu21_state_finished):
		{
			if(!i2c_send_1(data->slave, htu21_cmd_meas_temp_no_hold_master))
			{
				Log::get() << "htu21 poll: error 5";
				break;
			}

			pdata->state = htu21_state_measuring_temperature;

			break;
		}

		case(htu21_state_measuring_temperature):
		{
			if(!htu21_get_data(data, &result))
				break;

			pdata->raw_temperature = result;
			pdata->state = htu21_state_finished_temperature;

			break;
		}

		case(htu21_state_finished_temperature):
		{
			if(!i2c_send_1(data->slave, htu21_cmd_meas_hum_no_hold_master))
			{
				Log::get() << "htu21: poll: error 6";
				break;
			}

			pdata->state = htu21_state_measuring_humidity;

			break;
		}

		case(htu21_state_measuring_humidity):
		{
			if(!htu21_get_data(data, &result))
				break;

			pdata->raw_humidity = result;
			pdata->state = htu21_state_finished_humidity;

			break;
		}

		case(htu21_state_finished_humidity):
		{
			temperature = ((pdata->raw_temperature * 175.72f) / 65536.0f) - 46.85f;
			humidity = (((pdata->raw_humidity * 125.0f) / 65536.0f) - 6.0f) + ((25.0f - temperature) * -0.10f); // TempCoeff guessed

			if(humidity < 0)
				humidity = 0;

			if(humidity > 100)
				humidity = 100;

			data->values[sensor_type_temperature].value = temperature;
			data->values[sensor_type_temperature].stamp = time(nullptr);
			data->values[sensor_type_humidity].value = humidity;
			data->values[sensor_type_humidity].stamp = data->values[sensor_type_temperature].stamp;

			pdata->state = htu21_state_finished;

			break;
		}
	}

	return(true);
}

static void htu21_dump(const data_t *data, std::string &output)
{
	htu21_private_data_t *pdata = static_cast<htu21_private_data_t *>(data->private_data);

	assert(pdata);

	output += (boost::format("state: %u, ") % pdata->state).str();
	output += (boost::format("raw temperature: %u, ") % pdata->raw_temperature).str();
	output += (boost::format("raw humidity: %u") % pdata->raw_humidity).str();
}

enum : unsigned int
{
	veml7700_reg_conf =		0x00,
	veml7700_reg_als_wh =	0x01,
	veml7700_reg_als_wl =	0x02,
	veml7700_reg_powsave =	0x03,
	veml7700_reg_als =		0x04,
	veml7700_reg_white =	0x05,
	veml7700_reg_als_int =	0x06,
	veml7700_reg_id =		0x07,
};

enum : unsigned int
{
	veml7700_reg_id_id_1 =	0x81,
	veml7700_reg_id_id_2 =	0xc4,
};

enum : unsigned int
{
	veml7700_conf_reserved1 =		0b000 << 13,
	veml7700_conf_als_gain_1 =		0b00 << 11,
	veml7700_conf_als_gain_2 =		0b01 << 11,
	veml7700_conf_als_gain_1_8 =	0b10 << 11,
	veml7700_conf_als_gain_1_4 =	0b11 << 11,
	veml7700_conf_reserved2 =		0b1 << 10,
	veml7700_conf_als_it_25 =		0b1100 << 6,
	veml7700_conf_als_it_50 =		0b1000 << 6,
	veml7700_conf_als_it_100 =		0b0000 << 6,
	veml7700_conf_als_it_200 =		0b0001 << 6,
	veml7700_conf_als_it_400 =		0b0010 << 6,
	veml7700_conf_als_it_800 =		0b0011 << 6,
	veml7700_conf_als_pers_1 =		0b00 << 4,
	veml7700_conf_als_pers_2 =		0b01 << 4,
	veml7700_conf_als_pers_4 =		0b10 << 4,
	veml7700_conf_als_pers_8 =		0b11 << 4,
	veml7700_conf_reserved3 =		0b00 << 2,
	veml7700_conf_als_int_en =		0b1 << 1,
	veml7700_conf_als_sd =			0b1 << 0,
};

static constexpr unsigned int  veml7700_autoranging_data_size = 6;

static const device_autoranging_data_t veml7700_autoranging_data[veml7700_autoranging_data_size] =
{
	{{	veml7700_conf_als_it_800,	veml7700_conf_als_gain_2 },		{ 0,	32768	}, 0, 0.0036	},
	{{	veml7700_conf_als_it_800,	veml7700_conf_als_gain_1_8 },	{ 100,	32768	}, 0, 0.0576	},
	{{	veml7700_conf_als_it_200,	veml7700_conf_als_gain_2 },		{ 100,	32768	}, 0, 0.0144	},
	{{	veml7700_conf_als_it_200,	veml7700_conf_als_gain_1_8 },	{ 100,	32768	}, 0, 0.2304	},
	{{	veml7700_conf_als_it_25,	veml7700_conf_als_gain_2 },		{ 100,	32768	}, 0, 0.1152	},
	{{	veml7700_conf_als_it_25,	veml7700_conf_als_gain_1_8 },	{ 100,	65536	}, 0, 1.8432	},
};

typedef enum : unsigned int
{
	veml7700_state_init,
	veml7700_state_measuring,
	veml7700_state_finished,
} veml7700_state_t;

typedef struct
{
	veml7700_state_t state;
	unsigned int scaling;
	unsigned int scaling_up;
	unsigned int scaling_down;
	unsigned int raw_als;
	unsigned int raw_white;
} veml7700_private_data_t;

static sensor_detect_t veml7700_detect(i2c_slave_t slave)
{
	uint8_t buffer[2];

	if(!i2c_send_1_receive(slave, veml7700_reg_id, sizeof(buffer), buffer))
		return(sensor_not_found);

	if((buffer[0] != veml7700_reg_id_id_1) || (buffer[1] != veml7700_reg_id_id_2))
		return(sensor_not_found);

	return(sensor_found);
}

static bool veml7700_init(data_t *data)
{
	veml7700_private_data_t *pdata = static_cast<veml7700_private_data_t *>(data->private_data);

	assert(pdata);

	pdata->state = veml7700_state_init;
	pdata->scaling = veml7700_autoranging_data_size - 1;
	pdata->scaling_up = 0;
	pdata->scaling_down = 0;
	pdata->raw_als = 0;
	pdata->raw_white = 0;

	return(true);
}

static bool veml7700_poll(data_t *data)
{
	veml7700_private_data_t *pdata = static_cast<veml7700_private_data_t *>(data->private_data);
	uint8_t buffer[3];
	unsigned int scale_down_threshold, scale_up_threshold;
	float raw_lux;
	unsigned int opcode;

	assert(pdata);

	switch(pdata->state)
	{
		case(veml7700_state_init):
		case(veml7700_state_finished):
		{
			opcode =	veml7700_autoranging_data[pdata->scaling].data[0];
			opcode |=	veml7700_autoranging_data[pdata->scaling].data[1];

			buffer[0] = veml7700_reg_conf;
			buffer[1] = (opcode & 0x00ff) >> 0;
			buffer[2] = (opcode & 0xff00) >> 8;

			if(!i2c_send(data->slave, sizeof(buffer), buffer))
			{
				Log::get() << "veml7700: poll: error 1";
				break;
			}

			pdata->state = veml7700_state_measuring;

			break;
		}

		case(veml7700_state_measuring):
		{
			scale_down_threshold = veml7700_autoranging_data[pdata->scaling].threshold.down;
			scale_up_threshold = veml7700_autoranging_data[pdata->scaling].threshold.up;

			pdata->state = veml7700_state_finished;

			if(!i2c_send_1_receive(data->slave, veml7700_reg_white, sizeof(buffer), buffer))
			{
				Log::get() << "veml7700: poll: error 2";
				break;
			}

			pdata->raw_white = unsigned_16_le(buffer);

			if(!i2c_send_1_receive(data->slave, veml7700_reg_als, sizeof(buffer), buffer))
			{
				Log::get() << "veml7700: poll: error 3";
				break;
			}

			pdata->raw_als = unsigned_16_le(buffer);

			if((pdata->raw_als < scale_down_threshold) && (pdata->scaling > 0))
			{
				pdata->scaling--;
				pdata->scaling_down++;
				break;
			}

			if((pdata->raw_als >= scale_up_threshold) && (pdata->scaling < (veml7700_autoranging_data_size - 1)))
			{
				pdata->scaling++;
				pdata->scaling_up++;
				break;
			}

			raw_lux = pdata->raw_als * veml7700_autoranging_data[pdata->scaling].factor;

			data->values[sensor_type_visible_light].value =
					(raw_lux * raw_lux * raw_lux * raw_lux * 6.0135e-13f)
					- (raw_lux * raw_lux * raw_lux * 9.3924e-09f)
					+ (raw_lux * raw_lux * 8.1488e-05f)
					+ (raw_lux * 1.0023e+00f);
			data->values[sensor_type_visible_light].stamp = time(nullptr);

			break;
		}
	}

	return(true);
}

static void veml7700_dump(const data_t *data, std::string &output)
{
	veml7700_private_data_t *pdata = static_cast<veml7700_private_data_t *>(data->private_data);

	assert(pdata);

	output += (boost::format("state: %u, ") % pdata->state).str();
	output += (boost::format("scaling: %u, ") % pdata->scaling).str();
	output += (boost::format("scaling up: %u, ") % pdata->scaling_up).str();
	output += (boost::format("scaling down: %u, ") % pdata->scaling_down).str();
	output += (boost::format("raw als: %u, ") % pdata->raw_als).str();
	output += (boost::format("raw als: %u, ") % pdata->raw_white).str();
}

enum : unsigned int
{
	bme680_reg_meas_status_0 =	0x1d,

	bme680_reg_press =			0x1f,
	bme680_reg_temp =			0x22,
	bme680_reg_hum =			0x25,

	bme680_reg_ctrl_gas_0 =		0x70,
	bme680_reg_ctrl_hum =		0x72,
	bme680_reg_status =			0x73,
	bme680_reg_ctrl_meas =		0x74,
	bme680_reg_config =			0x75,
	bme680_reg_calibration_1 =	0x89,
	bme680_reg_id =				0xd0,
	bme680_reg_reset =			0xe0,
	bme680_reg_calibration_2 =	0xe1,

	bme680_reg_meas_status_0_new_data =		0b1000'0000,
	bme680_reg_meas_status_0_measuring =	0b0010'0000,

	bme680_reg_ctrl_gas_0_heat_on =		0b0000'0000,
	bme680_reg_ctrl_gas_0_heat_off =	0b0000'1000,

	bme680_reg_ctrl_gas_1_run_gas =		0b0001'0000,

	bme680_reg_ctrl_hum_osrh_h_skip =	0b0000'0000,
	bme680_reg_ctrl_hum_osrh_h_1 =		0b0000'0001,
	bme680_reg_ctrl_hum_osrh_h_2 =		0b0000'0010,
	bme680_reg_ctrl_hum_osrh_h_4 =		0b0000'0011,
	bme680_reg_ctrl_hum_osrh_h_8 =		0b0000'0100,
	bme680_reg_ctrl_hum_osrh_h_16 =		0b0000'0101,

	bme680_reg_ctrl_meas_osrs_t_skip =	0b0000'0000,
	bme680_reg_ctrl_meas_osrs_t_1 =		0b0010'0000,
	bme680_reg_ctrl_meas_osrs_t_2 =		0b0100'0000,
	bme680_reg_ctrl_meas_osrs_t_4 =		0b0110'0000,
	bme680_reg_ctrl_meas_osrs_t_8 =		0b1000'0000,
	bme680_reg_ctrl_meas_osrs_t_16 =	0b1010'0000,

	bme680_reg_ctrl_meas_osrs_mask =	0b0001'1100,
	bme680_reg_ctrl_meas_osrs_p_skip =	0b0000'0000,
	bme680_reg_ctrl_meas_osrs_p_1 =		0b0000'0100,
	bme680_reg_ctrl_meas_osrs_p_2 =		0b0000'1000,
	bme680_reg_ctrl_meas_osrs_p_4 =		0b0000'1100,
	bme680_reg_ctrl_meas_osrs_p_8 =		0b0001'0000,

	bme680_reg_ctrl_meas_sleep =		0b0000'0000,
	bme680_reg_ctrl_meas_forced =		0b0000'0001,

	bme680_reg_config_filter_mask =		0b0001'1100,
	bme680_reg_config_filter_0 =		0b0000'0000,
	bme680_reg_config_filter_1 =		0b0000'0100,
	bme680_reg_config_filter_3 =		0b0000'1000,
	bme680_reg_config_filter_7 =		0b0000'1100,
	bme680_reg_config_filter_15 =		0b0001'0000,
	bme680_reg_config_filter_31 =		0b0001'0100,
	bme680_reg_config_filter_63 =		0b0001'1000,
	bme680_reg_config_filter_127 =		0b0001'1100,

	bme680_reg_id_bme680 =				0x61,

	bme680_reg_reset_value =			0xb6,

	bme680_calibration_1_size =			25,
	bme680_calibration_2_size =			16,

	bme680_calibration_offset_t2 =		1,
	bme680_calibration_offset_t3 =		3,

	bme680_calibration_offset_p1 =		5,
	bme680_calibration_offset_p2 =		7,
	bme680_calibration_offset_p3 =		9,
	bme680_calibration_offset_p4 =		11,
	bme680_calibration_offset_p5 =		13,
	bme680_calibration_offset_p7 =		15,
	bme680_calibration_offset_p6 =		16,
	bme680_calibration_offset_p8 =		19,
	bme680_calibration_offset_p9 =		21,
	bme680_calibration_offset_p10 =		23,

	bme680_calibration_offset_h1 =		26,
	bme680_calibration_offset_h2 =		25,
	bme680_calibration_offset_h3 =		28,
	bme680_calibration_offset_h4 =		29,
	bme680_calibration_offset_h5 =		30,
	bme680_calibration_offset_h6 =		31,
	bme680_calibration_offset_h7 =		32,

	bme680_calibration_offset_t1 =		33,
};

typedef enum : unsigned int
{
	bme680_state_init,
	bme680_state_otp_ready,
	bme680_state_measuring,
	bme680_state_finished,
} bme680_state_t;

typedef struct
{
	bme680_state_t	state;
	float			t_fine;
	unsigned int	adc_temperature;
	unsigned int	adc_airpressure;
	unsigned int	adc_humidity;
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

static bool bme680_read_otp(data_t *data)
{
	bme680_private_data_t *pdata = static_cast<bme680_private_data_t *>(data->private_data);
	uint8_t calibration[bme680_calibration_1_size + bme680_calibration_2_size];

	assert(pdata);

	if(!i2c_send_1_receive(data->slave, bme680_reg_calibration_1, bme680_calibration_1_size, &calibration[0]))
		return(false);

	if(!i2c_send_1_receive(data->slave, bme680_reg_calibration_2, bme680_calibration_2_size, &calibration[bme680_calibration_1_size]))
		return(false);

	pdata->t1 =		unsigned_16_le(&calibration[bme680_calibration_offset_t1]);
	pdata->t2 =		signed_16_le(&calibration[bme680_calibration_offset_t2]);
	pdata->t3 =		signed_8(&calibration[bme680_calibration_offset_t3]);

	pdata->p1 =		unsigned_16_le(&calibration[bme680_calibration_offset_p1]);
	pdata->p2 =		signed_16_le(&calibration[bme680_calibration_offset_p2]);
	pdata->p3 =		signed_8(&calibration[bme680_calibration_offset_p3]);
	pdata->p4 =		signed_16_le(&calibration[bme680_calibration_offset_p4]);
	pdata->p5 =		signed_16_le(&calibration[bme680_calibration_offset_p5]);
	pdata->p6 =		signed_8(&calibration[bme680_calibration_offset_p6]);
	pdata->p7 =		signed_8(&calibration[bme680_calibration_offset_p7]);
	pdata->p8 =		signed_16_le(&calibration[bme680_calibration_offset_p8]);
	pdata->p9 =		signed_16_le(&calibration[bme680_calibration_offset_p9]);
	pdata->p10 =	unsigned_8(&calibration[bme680_calibration_offset_p10]);

	pdata->h1 =		unsigned_12_bottom_le(&calibration[bme680_calibration_offset_h1]);
	pdata->h2 =		unsigned_12_top_be(&calibration[bme680_calibration_offset_h2]);
	pdata->h3 =		signed_8(&calibration[bme680_calibration_offset_h3]);
	pdata->h4 =		signed_8(&calibration[bme680_calibration_offset_h4]);
	pdata->h5 =		signed_8(&calibration[bme680_calibration_offset_h5]);
	pdata->h6 =		unsigned_8(&calibration[bme680_calibration_offset_h6]);
	pdata->h7 =		signed_8(&calibration[bme680_calibration_offset_h7]);

	return(true);
}

static sensor_detect_t bme680_detect(i2c_slave_t slave)
{
	uint8_t buffer[1];

	if(!i2c_send_1_receive(slave, bme680_reg_id, sizeof(buffer), buffer))
		return(sensor_not_found);

	if(buffer[0] != bme680_reg_id_bme680)
		return(sensor_not_found);

	return(sensor_found);
}

static bool bme680_init(data_t *data)
{
	bme680_private_data_t *pdata = static_cast<bme680_private_data_t *>(data->private_data);

	assert(pdata);

	if(!i2c_send_2(data->slave, bme680_reg_reset, bme680_reg_reset_value))
		return(false);

	pdata->state = bme680_state_init;

	return(true);
}

static bool bme680_poll(data_t *data)
{
	uint8_t buffer[3];
	float temperature, humidity, airpressure, airpressure_256;
	float var1, var2, var3, var4;
	float t1_scaled;
	bme680_private_data_t *pdata = static_cast<bme680_private_data_t *>(data->private_data);

	assert(pdata);

	switch(pdata->state)
	{
		case(bme680_state_init):
		{
			if(!i2c_send_2(data->slave, bme680_reg_config, bme680_reg_config_filter_127))
				return(false);

			if(!i2c_send_2(data->slave, bme680_reg_ctrl_gas_0, bme680_reg_ctrl_gas_0_heat_off))
				return(false);

			if(!bme680_read_otp(data))
				return(false);

			if((pdata->t1 == 0) && (pdata->t2 == 0))
				return(false);

			pdata->state = bme680_state_otp_ready;

			break;
		}

		case(bme680_state_measuring):
		{
			if(!i2c_send_1_receive(data->slave, bme680_reg_meas_status_0, 1, buffer))
			{
				Log::get() << "sensors: bme680: poll error 1";
				return(false);
			}

			if(buffer[0] & bme680_reg_meas_status_0_measuring)
			{
				Log::get() << "sensors: bme680: sensor not ready";
				return(true);
			}

			if(!i2c_send_1_receive(data->slave, bme680_reg_temp, sizeof(buffer), buffer))
			{
				Log::get() << "sensors: bme680: poll error 2";
				return(false);
			}

			pdata->adc_temperature = unsigned_20_top_be(buffer);

			if(!i2c_send_1_receive(data->slave, bme680_reg_press, sizeof(buffer), buffer))
			{
				Log::get() << "sensors: bme680: poll error 3";
				return(false);
			}

			pdata->adc_airpressure = unsigned_20_top_be(buffer);

			if(!i2c_send_1_receive(data->slave, bme680_reg_hum, sizeof(buffer), buffer))
			{
				Log::get() << "sensors: bme680: poll error 4";
				return(false);
			}

			pdata->adc_humidity = unsigned_16_be(buffer);

			t1_scaled =		(pdata->adc_temperature / 131072.0f) - (pdata->t1 / 8192.0f);
			pdata->t_fine =	((pdata->adc_temperature / 16384.0f) - (pdata->t1 / 1024.0f)) * pdata->t2 + (t1_scaled * t1_scaled * pdata->t3 * 16.0f);

			temperature	= pdata->t_fine / 5120.0f;

			var1 = (pdata->t_fine / 2.0f) - 64000.0f;
			var2 = var1 * var1 * pdata->p6 / 131072.0f;
			var2 = var2 + (var1 * pdata->p5 * 2.0f);
			var2 = (var2 / 4) + (pdata->p4 * 65536.f);
			var1 = (((pdata->p3 * var1 * var1) / 16384.0f) + (pdata->p2 * var1)) / 524288.0f;
			var1 = (1 + (var1 / 32768.0f)) * pdata->p1;
			airpressure = 1048576.0f - pdata->adc_airpressure;

			if((int)var1 != 0)
			{
				airpressure = ((airpressure - (var2 / 4096.0f)) * 6250.0f) / var1;
				airpressure_256 = airpressure / 256.0f;
				var1 = (pdata->p9 * airpressure * airpressure) / 2147483648.0f;
				var2 = airpressure * (pdata->p8 / 32768.0f);
				var3 = airpressure_256 * airpressure_256 * airpressure_256 * (pdata->p10 / 131072.0f);
				airpressure = (airpressure + (var1 + var2 + var3 + (pdata->p7 * 128.0f)) / 16.0f) / 100.0f;
			}
			else
				airpressure = 0;

			var1 = pdata->adc_humidity - ((pdata->h1 * 16.0f) + ((pdata->h3 / 2.0f) * temperature));
			var2 = var1 * ((pdata->h2 / 262144.0f) * (1.0f + ((pdata->h4 / 16384.0f) * temperature) + ((pdata->h5 / 1048576.0f) * temperature * temperature)));
			var3 = pdata->h6 / 16384.0f;
			var4 = pdata->h7 / 2097152.0f;

			humidity = var2 + ((var3 + (var4 * temperature)) * var2 * var2);

			if(humidity > 100.0f)
				humidity = 100.0f;

			if(humidity < 0.0f)
				humidity = 0.0f;

			data->values[sensor_type_temperature].value = temperature;
			data->values[sensor_type_temperature].stamp = time(nullptr);

			if(airpressure > 0)
			{
				data->values[sensor_type_airpressure].value = airpressure;
				data->values[sensor_type_airpressure].stamp = time(nullptr);
			}

			data->values[sensor_type_humidity].value = humidity;
			data->values[sensor_type_humidity].stamp = time(nullptr);

			pdata->state = bme680_state_finished;

			break;
		}

		case(bme680_state_finished):
		case(bme680_state_otp_ready):
		{
			if(!i2c_send_2(data->slave, bme680_reg_ctrl_hum, bme680_reg_ctrl_hum_osrh_h_16))
			{
				Log::get() << "sensors: bme680: poll error 5";
				return(false);
			}

			if(!i2c_send_2(data->slave, bme680_reg_ctrl_meas, bme680_reg_ctrl_meas_osrs_t_16 | bme680_reg_ctrl_meas_osrs_p_8 | bme680_reg_ctrl_meas_forced))
			{
				Log::get() << "sensors: bme680: poll error 6";
				return(false);
			}

			pdata->state = bme680_state_measuring;

			break;
		}
	}

	return(true);
}

static void bme680_dump(const data_t *data, std::string &output)
{
	bme680_private_data_t *pdata = static_cast<bme680_private_data_t *>(data->private_data);

	assert(pdata);

	output += (boost::format("state: %u, ") % pdata->state).str();
	output += (boost::format("t_fine: %f, ") % (double)pdata->t_fine).str();
	output += (boost::format("adc temp: %u, ") % pdata->adc_temperature).str();
	output += (boost::format("adc pressure: %u, ") % pdata->adc_airpressure).str();
	output += (boost::format("adc humidity: %u, ") % pdata->adc_humidity).str();
	output += (boost::format("t1: %u, ") % pdata->t1).str();
	output += (boost::format("t2: %d, ") % pdata->t2).str();
	output += (boost::format("t3: %d, ") % pdata->t3).str();
	output += (boost::format("p1: %u, ") % pdata->p1).str();
	output += (boost::format("p2: %d, ") % pdata->p2).str();
	output += (boost::format("p3: %d, ") % pdata->p3).str();
	output += (boost::format("p4: %d, ") % pdata->p4).str();
	output += (boost::format("p5: %d, ") % pdata->p5).str();
	output += (boost::format("p6: %d, ") % pdata->p6).str();
	output += (boost::format("p7: %d, ") % pdata->p7).str();
	output += (boost::format("p8: %d, ") % pdata->p8).str();
	output += (boost::format("p9: %d, ") % pdata->p9).str();
	output += (boost::format("p10: %u, ") % pdata->p10).str();
	output += (boost::format("h1: %u, ") % pdata->h1).str();
	output += (boost::format("h2: %u, ") % pdata->h2).str();
	output += (boost::format("h3: %d, ") % pdata->h3).str();
	output += (boost::format("h4: %d, ") % pdata->h4).str();
	output += (boost::format("h5: %d, ") % pdata->h5).str();
	output += (boost::format("h6: %u, ") % pdata->h6).str();
	output += (boost::format("h7: %d") % pdata->h7).str();
};

enum : unsigned int
{
	apds9930_command_select =				0b1 << 7,
	apds9930_command_type_repeated_byte =	0b00 << 5,
	apds9930_command_type_autoincrement =	0b01 << 5,
	apds9930_command_type_reserved =		0b10 << 5,
	apds9930_command_type_special =			0b11 << 5,
	apds9930_command_address_mask =			0b11111 << 0,

	apds9930_reg_enable =		0x00,
	apds9930_reg_atime =		0x01,
	apds9930_reg_config =		0x0d,
	apds9930_reg_control =		0x0f,
	apds9930_reg_id =			0x12,
	apds9930_reg_status =		0x13,
	apds9930_reg_c0data =		0x14,
	apds9930_reg_c0datah =		0x15,
	apds9930_reg_c1data =		0x16,
	apds9930_reg_c1datah =		0x17,

	apds9930_enable_sai =		1 << 6,
	apds9930_enable_pien =		1 << 5,
	apds9930_enable_aien =		1 << 4,
	apds9930_enable_wen =		1 << 3,
	apds9930_enable_pen =		1 << 2,
	apds9930_enable_aen =		1 << 1,
	apds9930_enable_pon =		1 << 0,
	apds9930_enable_poff =		0 << 0,

	apds9930_atime_2_73 =		0xff,
	apds9930_atime_27_3 =		0xf6,
	apds9930_atime_101 =		0xdb,
	apds9930_atime_175 =		0xc0,
	apds9930_atime_699 =		0x00,

	apds9930_config_agl =		0b1 << 2,
	apds9930_config_wlong =		0b1 << 1,
	apds9930_config_pdl =		0b1 << 0,

	apds9930_ctrl_pdrive_100 =	0b00 << 6,
	apds9930_ctrl_pdrive_50 =	0b01 << 6,
	apds9930_ctrl_pdrive_25 =	0b10 << 6,
	apds9930_ctrl_pdrive_12_5 =	0b11 << 6,
	apds9930_ctrl_pdiode_ch1 =	0b10 << 4,
	apds9930_ctrl_pgain_1 =		0b00 << 2,
	apds9930_ctrl_pgain_2 =		0b01 << 2,
	apds9930_ctrl_pgain_4 =		0b10 << 2,
	apds9930_ctrl_pgain_8 =		0b11 << 2,
	apds9930_ctrl_again_1 =		0b00 << 0,
	apds9930_ctrl_again_8 =		0b01 << 0,
	apds9930_ctrl_again_16 =	0b10 << 0,
	apds9930_ctrl_again_120 =	0b11 << 0,

	apds9930_id_tmd27711 =		0x20,
	apds9930_id_tmd27713 =		0x29,
	apds9930_id_apds9930 =		0x30,

	apds9930_status_avalid =	0b1 << 0,
};

enum : unsigned int
{
	apds9930_tmd2771_autoranging_data_size = 4,
	apds9930_apds9930_autoranging_data_size = 5,
	apds9930_autoranging_disable_agl = 0 << 30,
	apds9930_autoranging_enable_agl = 1 << 30,
};

static const device_autoranging_data_t apds9930_tmd2771_autoranging_data[apds9930_tmd2771_autoranging_data_size] =
{
	{{	apds9930_atime_699,	apds9930_ctrl_again_120	},										{ 0,	65000	}, 0, 	699 *	120		},
	{{	apds9930_atime_699,	apds9930_ctrl_again_16	},										{ 100,	65000	}, 0, 	699 *	16		},
	{{	apds9930_atime_175,	apds9930_ctrl_again_16	},										{ 100,	65000	}, 0, 	175 *	16		},
	{{	apds9930_atime_175,	apds9930_ctrl_again_1	},										{ 100,	65000	}, 0, 	175 *	1		},
};

static const device_autoranging_data_t apds9930_apds9930_autoranging_data[apds9930_apds9930_autoranging_data_size] =
{
	{{	apds9930_atime_699,	static_cast<unsigned int>(apds9930_ctrl_again_120)	| apds9930_autoranging_disable_agl	},	{ 0,	65000	}, 0, 	699 *	120		},
	{{	apds9930_atime_699,	static_cast<unsigned int>(apds9930_ctrl_again_16)	| apds9930_autoranging_disable_agl	},	{ 100,	65000	}, 0, 	699 *	16		},
	{{	apds9930_atime_175,	static_cast<unsigned int>(apds9930_ctrl_again_16)	| apds9930_autoranging_disable_agl	},	{ 100,	65000	}, 0, 	175 *	16		},
	{{	apds9930_atime_175,	static_cast<unsigned int>(apds9930_ctrl_again_1)	| apds9930_autoranging_disable_agl	},	{ 100,	65000	}, 0, 	175 *	1		},
	{{	apds9930_atime_175,	static_cast<unsigned int>(apds9930_ctrl_again_1)	| apds9930_autoranging_enable_agl	},	{ 100,	65536	}, 0,	175 *	0.1667	},
};

typedef enum
{
	apds9930_state_init,
	apds9930_state_measuring,
	apds9930_state_finished,
} apds9930_state_t;

typedef struct
{
	unsigned int type;
	apds9930_state_t state;
	unsigned int scaling;
	unsigned int not_readys;
	unsigned int overflows;
	unsigned int scales_up;
	unsigned int scales_down;
	float raw_channel_correction[2];
	unsigned int raw_channel[2];
	float lpc;
	float iac1;
	float iac2;
	float iac;
	unsigned int autoranging_data_size;
	const device_autoranging_data_t *autoranging_data;
} apds9930_private_data_t;

static bool apds9930_write_register(i2c_slave_t slave, unsigned int reg, unsigned int value)
{
	uint8_t buffer[2];

	buffer[0] = apds9930_command_select | apds9930_command_type_autoincrement | (reg & apds9930_command_address_mask);
	buffer[1] = value;

	return(i2c_send(slave, sizeof(buffer), buffer));
}

static bool apds9930_read_register(i2c_slave_t slave, unsigned int reg, unsigned int *value)
{
	uint8_t buffer_in[1];
	uint8_t buffer_out[1];

	buffer_in[0] = apds9930_command_select | apds9930_command_type_autoincrement | (reg & apds9930_command_address_mask);

	if(!i2c_send_receive(slave, sizeof(buffer_in), buffer_in, sizeof(buffer_out), buffer_out))
		return(false);

	*value = buffer_out[0];

	return(true);
}

static bool apds9930_read_register_2x2(i2c_slave_t slave, unsigned int reg, unsigned int value[])
{
	uint8_t buffer_in[1];
	uint8_t buffer_out[4];

	buffer_in[0] = apds9930_command_select | apds9930_command_type_autoincrement | (reg & apds9930_command_address_mask);

	if(!i2c_send_receive(slave, sizeof(buffer_in), buffer_in, sizeof(buffer_out), buffer_out))
		return(false);

	value[0] = unsigned_16_le(&buffer_out[0]);
	value[1] = unsigned_16_le(&buffer_out[2]);

	return(true);
}

static sensor_detect_t apds9930_detect(i2c_slave_t slave)
{
	unsigned int id;

	if(!apds9930_read_register(slave, apds9930_reg_id, &id))
		return(sensor_not_found);

	if((id != apds9930_id_apds9930) && (id != apds9930_id_tmd27711) && (id != apds9930_id_tmd27713))
		return(sensor_not_found);

	return(sensor_found);
}

static bool apds9930_init(data_t *data)
{
	apds9930_private_data_t *pdata = static_cast<apds9930_private_data_t *>(data->private_data);

	assert(pdata);

	if(!apds9930_read_register(data->slave, apds9930_reg_id, &pdata->type))
		return(false);

	switch(pdata->type)
	{
		case(apds9930_id_tmd27711):
		case(apds9930_id_tmd27713):
		{
			pdata->autoranging_data_size = apds9930_tmd2771_autoranging_data_size;
			pdata->autoranging_data = apds9930_tmd2771_autoranging_data;
			pdata->raw_channel_correction[0] = 0.977f;
			pdata->raw_channel_correction[1] = 1.02f;
			break;
		}

		case(apds9930_id_apds9930):
		{
			pdata->autoranging_data_size = apds9930_apds9930_autoranging_data_size;
			pdata->autoranging_data = apds9930_apds9930_autoranging_data;
			pdata->raw_channel_correction[0] = 1.0f;
			pdata->raw_channel_correction[1] = 1.0f;
			break;
		}

		default:
		{
			Log::get() << "apds9930: invalid id";
			return(false);
		}
	}

	pdata->state = apds9930_state_init;
	pdata->scaling = pdata->autoranging_data_size - 1;
	pdata->raw_channel[0] = 0;
	pdata->raw_channel[1] = 0;
	pdata->scales_up = 0;
	pdata->scales_down = 0;
	pdata->overflows = 0;
	pdata->not_readys = 0;
	pdata->lpc = 0;
	pdata->iac1 = 0;
	pdata->iac2 = 0;
	pdata->iac = 0;

	return(true);
}

static bool apds9930_poll(data_t *data)
{
	static constexpr float apds9930_factor_df = 52.0f;
	static constexpr float apds9930_factor_ga = 0.49f;
	static constexpr float apds9930_factor_a = 1.0f;
	static constexpr float apds9930_factor_b = 1.862f;
	static constexpr float apds9930_factor_c = 0.746f;
	static constexpr float apds9930_factor_d = 1.291f;

	apds9930_private_data_t *pdata = static_cast<apds9930_private_data_t *>(data->private_data);
	unsigned int value;
	float ch0, ch1;
	unsigned int atime, again, reg_config;

	assert(pdata);

	switch(pdata->state)
	{
		case(apds9930_state_init):
		case(apds9930_state_finished):
		{
			atime = pdata->autoranging_data[pdata->scaling].data[0];
			again = pdata->autoranging_data[pdata->scaling].data[1];

			reg_config = 0x00;

			if(again & apds9930_autoranging_enable_agl)
			{
				reg_config |= apds9930_config_agl;
				again &= ~apds9930_autoranging_enable_agl;
			}

			if(!apds9930_write_register(data->slave, apds9930_reg_enable, apds9930_enable_poff))
			{
				Log::get() << "apds9930: poll: error 1";
				return(false);
			}

			if(!apds9930_write_register(data->slave, apds9930_reg_atime, atime))
			{
				Log::get() << "apds9930: poll: error 2";
				return(false);
			}

			if(!apds9930_write_register(data->slave, apds9930_reg_config, reg_config))
			{
				Log::get() << "apds9930: poll: error 3";
				return(false);
			}

			if(!apds9930_write_register(data->slave, apds9930_reg_control, apds9930_ctrl_pdrive_100 | apds9930_ctrl_pdiode_ch1 | again))
			{
				Log::get() << "apds9930: poll: error 4";
				return(false);
			}

			if(!apds9930_write_register(data->slave, apds9930_reg_enable, apds9930_enable_aen | apds9930_enable_pon))
			{
				Log::get() << "apds9930: poll: error 5";
				return(false);
			}

			pdata->state = apds9930_state_measuring;

			break;
		}

		case(apds9930_state_measuring):
		{
			pdata->state = apds9930_state_finished;

			if(!apds9930_read_register(data->slave, apds9930_reg_status, &value))
			{
				Log::get() << "apds9930: poll: error 6";
				return(false);
			}

			if(!(value & apds9930_status_avalid))
			{
				pdata->not_readys++;
				break;
			}

			if(!apds9930_read_register_2x2(data->slave, apds9930_reg_c0data, pdata->raw_channel))
			{
				Log::get() << "apds9930: poll: error 2";
				return(false);
			}

			if(((pdata->raw_channel[0] < pdata->autoranging_data[pdata->scaling].threshold.down) || (pdata->raw_channel[1] < pdata->autoranging_data[pdata->scaling].threshold.down)) &&
						(pdata->scaling > 0))
			{
				pdata->scaling--;
				pdata->scales_down++;
				break;
			}

			if(((pdata->raw_channel[0] >= pdata->autoranging_data[pdata->scaling].threshold.up) || (pdata->raw_channel[1] >= pdata->autoranging_data[pdata->scaling].threshold.up)) &&
				(pdata->scaling < (pdata->autoranging_data_size - 1)))
			{
				pdata->scaling++;
				pdata->scales_up++;
				break;
			}

			if((pdata->raw_channel[0] == 0) || (pdata->raw_channel[0] >= 65535) || (pdata->raw_channel[1] == 0) || (pdata->raw_channel[1] >= 65535))
			{
				pdata->overflows++;
				break;
			}

			ch0 = (float)pdata->raw_channel[0] * pdata->raw_channel_correction[0];
			ch1 = (float)pdata->raw_channel[1] * pdata->raw_channel_correction[1];

			pdata->lpc = (apds9930_factor_ga * apds9930_factor_df) / pdata->autoranging_data[pdata->scaling].factor;
			pdata->iac1 = (apds9930_factor_a * ch0) - (apds9930_factor_b * ch1);
			pdata->iac2 = (apds9930_factor_c * ch0) - (apds9930_factor_d * ch1);
			pdata->iac = fmaxf(fmaxf(pdata->iac1, pdata->iac2), 0);

			data->values[sensor_type_visible_light].value = pdata->lpc * pdata->iac;
			data->values[sensor_type_visible_light].stamp = time(nullptr);

			break;
		}
	}

	return(true);
}

static void apds9930_dump(const data_t *data, std::string &output)
{
	apds9930_private_data_t *pdata = static_cast<apds9930_private_data_t *>(data->private_data);

	output += (boost::format("type: 0x%02x, ") % pdata->type).str();
	output += (boost::format("state: %d, ") % pdata->state).str();
	output += (boost::format("scaling: %u, ") % pdata->scaling).str();
	output += (boost::format("not readys: %u, ") % pdata->not_readys).str();
	output += (boost::format("overflows: %u, ") % pdata->overflows).str();
	output += (boost::format("scales up: %u, ") % pdata->scales_up).str();
	output += (boost::format("scales down: %u, ") % pdata->scales_down).str();
	output += (boost::format("raw correction: %.3f/%.3f, ") % (double)pdata->raw_channel_correction[0] % (double)pdata->raw_channel_correction[1]).str();
	output += (boost::format("raw values: %u/%u, ") % pdata->raw_channel[0] % pdata->raw_channel[1]).str();
	output += (boost::format("lpc: %f, ") % (double)pdata->lpc).str();
	output += (boost::format("iac: %f/%f/%f, ") % (double)pdata->iac1 % (double)pdata->iac2 % (double)pdata->iac).str();
	output += (boost::format("autoranging data size: %u") % pdata->autoranging_data_size).str();
}

enum : unsigned int
{
	apds9960_reg_enable =		0x80,
	apds9960_reg_atime =		0x81,
	apds9960_reg_wtime =		0x83,
	apds9960_reg_ailt =			0x84,
	apds9960_reg_aiht =			0x86,
	apds9960_reg_pilt =			0x89,
	apds9960_reg_piht =			0x8b,
	apds9960_reg_pers =			0x8c,
	apds9960_reg_config1 =		0x8d,
	apds9960_reg_ppulse =		0x8e,
	apds9960_reg_control =		0x8f,
	apds9960_reg_config2 =		0x90,
	apds9960_reg_id =			0x92,
	apds9960_reg_status =		0x93,
	apds9960_reg_cdata =		0x94,
	apds9960_reg_rdata =		0x96,
	apds9960_reg_gdata =		0x98,
	apds9960_reg_bdata =		0x9a,
	apds9960_reg_pdata =		0x9c,
	apds9960_reg_poffset_ur =	0x9d,
	apds9960_reg_poffset_dl =	0x9e,
	apds9960_reg_config3 =		0x9f,
	apds9960_reg_gpenth =		0xa0,
	apds9960_reg_gexth =		0xa1,
	apds9960_reg_gconf1 =		0xa2,
	apds9960_reg_gconf2 =		0xa3,
	apds9960_reg_goffset_u =	0xa4,
	apds9960_reg_goffset_d =	0xa5,
	apds9960_reg_goffset_l =	0xa7,
	apds9960_reg_goffset_r =	0xa9,
	apds9960_reg_gpulse =		0xa6,
	apds9960_reg_gconf3 =		0xaa,
	apds9960_reg_gconf4 =		0xab,
	apds9960_reg_gflvl =		0xae,
	apds9960_reg_gstatus =		0xaf,
	apds9960_reg_iforce =		0xe4,
	apds9960_reg_piclear =		0xe5,
	apds9960_reg_ciclear =		0xe6,
	apds9960_reg_aiclear =		0xe7,
	apds9960_reg_gfifo_u =		0xfc,
	apds9960_reg_gfifo_d =		0xfd,
	apds9960_reg_gfifo_l =		0xfe,
	apds9960_reg_gfifo_r =		0xff,

	apds9960_enable_gen =		1 << 6,
	apds9960_enable_pien =		1 << 5,
	apds9960_enable_aien =		1 << 4,
	apds9960_enable_wen =		1 << 3,
	apds9960_enable_pen =		1 << 2,
	apds9960_enable_aen =		1 << 1,
	apds9960_enable_pon =		1 << 0,
	apds9960_enable_poff =		0 << 0,

	apds9960_atime_2_78 =		0xff,
	apds9960_atime_27_8 =		0xf6,
	apds9960_atime_103 =		0xdb,
	apds9960_atime_175 =		0xc0,
	apds9960_atime_200 =		0xb6,
	apds9960_atime_712 =		0x00,

	apds9960_config1_no_wlong =	0b0110'0000,
	apds9960_config1_wlong =	0b0110'0010,

	apds9960_config2_psien =	(1 << 7) | (1 << 0),
	apds9960_config2_cpsien =	(1 << 6) | (1 << 0),
	apds9960_config2_none =		(1 << 0),

	apds9960_ctrl_again_1 =		0b00 << 0,
	apds9960_ctrl_again_4 =		0b01 << 0,
	apds9960_ctrl_again_16 =	0b10 << 0,
	apds9960_ctrl_again_64 =	0b11 << 0,

	apds9960_id_apds9960_a8 =	0xa8,
	apds9960_id_apds9960_ab =	0xab,
	apds9960_id_apds9960_9c =	0x9c,

	apds9960_status_cpsat =		1 << 7,
	apds9960_status_pgsat =		1 << 6,
	apds9960_status_pint =		1 << 5,
	apds9960_status_aint =		1 << 4,
	apds9960_status_gint =		1 << 2,
	apds9960_status_pvalid =	1 << 1,
	apds9960_status_avalid =	1 << 0,
};

typedef enum : unsigned int
{
	apds9960_state_init,
	apds9960_state_measuring,
	apds9960_state_finished,
} apds9960_state_t;

typedef struct
{
	unsigned int type;
	apds9960_state_t state;
	unsigned int scaling;
	unsigned int not_readys;
	unsigned int overflows;
	unsigned int scales_up;
	unsigned int scales_down;
	struct
	{
		unsigned int clear;
		unsigned int r;
		unsigned int g;
		unsigned int b;
	} data;
} apds9960_private_data_t;

enum : unsigned int
{
	apds9960_autoranging_data_size = 5,
};

static const device_autoranging_data_t apds9960_autoranging_data[apds9960_autoranging_data_size] =
{
	{{	apds9960_atime_712,		apds9960_ctrl_again_64	},	{ 0,	32768	}, 0, 	712 * 6		},
	{{	apds9960_atime_712,		apds9960_ctrl_again_16	},	{ 100,	32768	}, 0, 	712 * 16	},
	{{	apds9960_atime_175,		apds9960_ctrl_again_4	},	{ 100,	32768	}, 0, 	175 * 4		},
	{{	apds9960_atime_175,		apds9960_ctrl_again_1	},	{ 100,	32768	}, 0, 	175 * 1		},
	{{	apds9960_atime_2_78,	apds9960_ctrl_again_1	},	{ 100,	65536	}, 0,	2.78 * 1	},
};

static bool apds9960_read_register(i2c_slave_t slave, unsigned int reg, unsigned int *value)
{
	uint8_t buffer_in[1];
	uint8_t buffer_out[1];

	buffer_in[0] = reg;

	if(!i2c_send_receive(slave, sizeof(buffer_in), buffer_in, sizeof(buffer_out), buffer_out))
		return(false);

	*value = buffer_out[0];

	return(true);
}

static bool apds9960_write_register(i2c_slave_t slave, unsigned int reg, unsigned int value)
{
	uint8_t buffer_in[2];

	buffer_in[0] = reg;
	buffer_in[1] = value;

	if(!i2c_send(slave, sizeof(buffer_in), buffer_in))
		return(false);

	return(true);
}

static sensor_detect_t apds9960_detect(i2c_slave_t slave)
{
	unsigned int id;

	if(!apds9960_read_register(slave, apds9960_reg_id, &id))
		return(sensor_not_found);

	if((id != apds9960_id_apds9960_a8) && (id != apds9960_id_apds9960_ab) && (id != apds9960_id_apds9960_9c))
		return(sensor_not_found);

	return(sensor_found);
}

static bool apds9960_init(data_t *data)
{
	apds9960_private_data_t *pdata = static_cast<apds9960_private_data_t *>(data->private_data);
	unsigned int id;

	assert(pdata);

	if(!apds9960_read_register(data->slave, apds9960_reg_id, &id))
	{
		Log::get() << "apds9960: init: error 1";
		return(false);
	}

	if(!apds9960_write_register(data->slave, apds9960_reg_config1, apds9960_config1_no_wlong))
	{
		Log::get() << "apds9960: init: error 2";
		return(false);
	}

	if(!apds9960_write_register(data->slave, apds9960_reg_config2, apds9960_config2_none))
	{
		Log::get() << "apds9960: init: error 3";
		return(false);
	}

	pdata->type = id;
	pdata->state = apds9960_state_init;
	pdata->scaling = apds9960_autoranging_data_size - 1;
	pdata->not_readys = 0;
	pdata->overflows = 0;
	pdata->scales_up = 0;
	pdata->scales_down = 0;
	pdata->data.clear = 0;
	pdata->data.r = 0;
	pdata->data.g = 0;
	pdata->data.b = 0;

	return(true);
}

static bool apds9960_poll(data_t *data)
{
	uint8_t buffer[8];
	unsigned int value, r, g, b, again, atime;
	apds9960_private_data_t *pdata = static_cast<apds9960_private_data_t *>(data->private_data);
	unsigned int scale_down_threshold, scale_up_threshold;

	assert(pdata);

	switch(pdata->state)
	{
		case(apds9960_state_init):
		case(apds9960_state_finished):
		{
			atime = apds9960_autoranging_data[pdata->scaling].data[0];
			again = apds9960_autoranging_data[pdata->scaling].data[1];

			if(!apds9960_write_register(data->slave, apds9960_reg_enable, apds9960_enable_poff))
			{
				Log::get() << "apds9960: poll: error 1";
				return(false);
			}

			if(!apds9960_write_register(data->slave, apds9960_reg_atime, atime))
			{
				Log::get() << "apds9960: poll: error 2";
				return(false);
			}

			if(!apds9960_write_register(data->slave, apds9960_reg_control, again))
			{
				Log::get() << "apds9960: poll: error 3";
				return(false);
			}

			if(!apds9960_write_register(data->slave, apds9960_reg_enable, apds9960_enable_aen | apds9960_enable_pon))
			{
				Log::get() << "apds9960: poll: error 4";
				return(false);
			}

			pdata->state = apds9960_state_measuring;
			break;
		}

		case(apds9960_state_measuring):
		{
			scale_down_threshold = apds9960_autoranging_data[pdata->scaling].threshold.down;
			scale_up_threshold =   apds9960_autoranging_data[pdata->scaling].threshold.up;

			pdata->state = apds9960_state_finished;

			if(!apds9960_read_register(data->slave, apds9960_reg_status, &value))
			{
				Log::get() << "apds9960: poll: error 1";
				return(false);
			}

			if(!(value & apds9960_status_avalid))
			{
				Log::get() << "apds9960: poll: error 2";
				pdata->not_readys++;
				return(false);
			}

			if(value & apds9960_status_cpsat)
			{
				pdata->overflows++;

				if(pdata->scaling < (apds9960_autoranging_data_size - 1))
				{
					pdata->scaling++;
					pdata->scales_up++;
				}

				break;
			}

			if(!i2c_send_1_receive(data->slave, apds9960_reg_cdata, sizeof(buffer), buffer))
			{
				Log::get() << "apds9960: poll: error 3";
				return(false);
			}

			pdata->data.clear = unsigned_16_le(&buffer[0]);
			r = unsigned_16_le(&buffer[2]);
			g = unsigned_16_le(&buffer[4]);
			b = unsigned_16_le(&buffer[6]);

			if((pdata->data.clear < scale_down_threshold) && (pdata->scaling > 0))
			{
				pdata->scaling--;
				pdata->scales_down++;
				break;
			}

			if((pdata->data.clear >= scale_up_threshold) && (pdata->scaling < (apds9960_autoranging_data_size - 1)))
			{
				pdata->scaling++;
				pdata->scales_up++;
				break;
			}

			pdata->data.r = (r * 100) / (r + g + b);
			pdata->data.g = (g * 100) / (r + g + b);
			pdata->data.b = (b * 100) / (r + g + b);

			data->values[sensor_type_visible_light].value = (pdata->data.clear * 100.0f) / apds9960_autoranging_data[pdata->scaling].factor;
			data->values[sensor_type_visible_light].stamp = time(nullptr);

			break;
		}
	}

	return(true);
}

static void apds9960_dump(const data_t *data, std::string &output)
{
	apds9960_private_data_t *pdata = static_cast<apds9960_private_data_t *>(data->private_data);

	output += (boost::format("type: 0x%02x, ") % pdata->type).str();
	output += (boost::format("state: %u, ") % pdata->state).str();
	output += (boost::format("scaling: %u, ") % pdata->scaling).str();
	output += (boost::format("not readys: %u, ") % pdata->not_readys).str();
	output += (boost::format("overflows: %u, ") % pdata->overflows).str();
	output += (boost::format("scales up: %u, ") % pdata->scales_up).str();
	output += (boost::format("scales down: %u, ") % pdata->scales_down).str();
	output += (boost::format("data clear: %u, ") % pdata->data.clear).str();
	output += (boost::format("data r: %u%%, ") % pdata->data.r).str();
	output += (boost::format("data g: %u%%, ") % pdata->data.g).str();
	output += (boost::format("data b: %u%%") % pdata->data.b).str();
}

typedef enum : unsigned int
{
	tsl2591_reg_enable =		0x00,
	tsl2591_reg_control =		0x01,
	tsl2591_reg_ailtl =			0x04,
	tsl2591_reg_ailth =			0x05,
	tsl2591_reg_aihtl =			0x06,
	tsl2591_reg_aihth =			0x07,
	tsl2591_reg_npailtl =		0x08,
	tsl2591_reg_npailth =		0x09,
	tsl2591_reg_npaihtl =		0x0a,
	tsl2591_reg_npaihth =		0x0b,
	tsl2591_reg_persist =		0x0c,
	tsl2591_reg_pid =			0x11,
	tsl2591_reg_id =			0x12,
	tsl2591_reg_status =		0x13,
	tsl2591_reg_c0datal =		0x14,
	tsl2591_reg_c0datah =		0x15,
	tsl2591_reg_c1datal =		0x16,
	tsl2591_reg_c1datah =		0x17,
} tsl2591_reg_t;

enum : unsigned int
{
	tsl2591_cmd_cmd =							0b1 << 7,
	tsl2591_cmd_transaction_normal =			0b01 << 5,
	tsl2591_cmd_transaction_special =			0b11 << 5,
	tsl2591_cmd_sf_interrupt_set =				0b00100 << 0,
	tsl2591_cmd_sf_interrupt_clear_als =		0b00110 << 0,
	tsl2591_cmd_sf_interrupt_clear_als_nals =	0b00111 << 0,
	tsl2591_cmd_sf_interrupt_clear_nals =		0b01010 << 0,
};

enum : unsigned int
{
	tsl2591_enable_npien =	0b1 << 7,
	tsl2591_enable_sai =	0b1 << 6,
	tsl2591_enable_aien =	0b1 << 4,
	tsl2591_enable_aen =	0b1 << 1,
	tsl2591_enable_pon =	0b1 << 0,
};

enum : unsigned int
{
	tsl2591_control_sreset =		0b1 << 7,
	tsl2591_control_again_0 =		0b00 << 4,
	tsl2591_control_again_25 =		0b01 << 4,
	tsl2591_control_again_400 =		0b10 << 4,
	tsl2591_control_again_9500 =	0b11 << 4,
	tsl2591_control_atime_100 =		0b000 << 0,
	tsl2591_control_atime_200 =		0b001 << 0,
	tsl2591_control_atime_300 =		0b010 << 0,
	tsl2591_control_atime_400 =		0b011 << 0,
	tsl2591_control_atime_500 =		0b100 << 0,
	tsl2591_control_atime_600 =		0b101 << 0,
};

enum : unsigned int
{
	tsl2591_pid_mask =	0b11 << 4,
	tsl2591_pid_value = 0b00 << 4,
};

enum : unsigned int
{
	tsl2591_id_mask =	0xff,
	tsl2591_id_value =	0x50,
};

enum : unsigned int
{
	tsl2591_status_npintr = 0b1 << 5,
	tsl2591_status_aint =	0b1 << 4,
	tsl2591_status_avalid =	0b1 << 0,
};

enum : unsigned int { tsl2591_autoranging_data_size = 5 };

static const device_autoranging_data_t tsl2591_autoranging_data[tsl2591_autoranging_data_size] =
{
	{{	tsl2591_control_again_9500,	tsl2591_control_atime_600 }, {	0,		50000	}, 56095,	0.096,	},
	{{	tsl2591_control_again_400,	tsl2591_control_atime_600 }, {	100,	50000	}, 56095,	5.8,	},
	{{	tsl2591_control_again_25,	tsl2591_control_atime_400 }, {	100,	50000	}, 65536,	49,		},
	{{	tsl2591_control_again_0,	tsl2591_control_atime_400 }, {	100,	50000	}, 65536,	490,	},
	{{	tsl2591_control_again_0,	tsl2591_control_atime_100 }, {	100,	50000	}, 65536,	1960,	},
};

enum : unsigned int { tsl2591_factor_size = 7 };

static const struct
{
	float	lower_bound;
	float	upper_bound;
	float	ch_factor[2];
} tsl2591_factors[tsl2591_factor_size] =
{
	{	0,		0.125,	{	1,		-0.895	}},
	{	0.125,	0.250,	{	1.070,	-1.145	}},
	{	0.250,	0.375,	{	1.115,	-1.790	}},
	{	0.375,	0.500,	{	1.126,	-2.050	}},
	{	0.500,	0.610,	{	0.740,	-1.002	}},
	{	0.610,	0.800,	{	0.420,	-0.500	}},
	{	0.800,	1.300,	{	0.48,	-0.037	}},
};

typedef enum : unsigned int
{
	tsl2591_state_init,
	tsl2591_state_reset,
	tsl2591_state_ready,
	tsl2591_state_measuring,
	tsl2591_state_finished,
} tsl2591_state_t;

typedef struct
{
	tsl2591_state_t	state;
	unsigned int	scaling;
	unsigned int	scales_up;
	unsigned int	scales_down;
	unsigned int	overflows;
	unsigned int	channel[2];
	float			ratio;
	unsigned int	ratio_index;
	float			factor[2];
} tsl2591_private_data_t;

static bool tsl2591_write(i2c_slave_t slave, tsl2591_reg_t reg, unsigned int value)
{
	return(i2c_send_2(slave, tsl2591_cmd_cmd | static_cast<unsigned int>(reg), value));
}

static bool tsl2591_read_byte(i2c_slave_t slave, tsl2591_reg_t reg, unsigned int *value)
{
	uint8_t buffer_out[1];

	if(!i2c_send_1_receive(slave, tsl2591_cmd_cmd | static_cast<unsigned int>(reg), sizeof(buffer_out), buffer_out))
		return(false);

	*value = buffer_out[0];

	return(true);
}

static bool tsl2591_write_check(i2c_slave_t slave, tsl2591_reg_t reg, unsigned int value)
{
	unsigned int rv;

	if(!tsl2591_write(slave, reg, value))
		return(false);

	if(!tsl2591_read_byte(slave, reg, &rv))
		return(false);

	if(value != rv)
		return(false);

	return(true);
}

static sensor_detect_t tsl2591_detect(i2c_slave_t slave)
{
	unsigned int regval;

	if(!tsl2591_read_byte(slave, tsl2591_reg_pid, &regval))
		return(sensor_not_found);

	if((regval & tsl2591_pid_mask) != tsl2591_pid_value)
		return(sensor_not_found);

	if(!tsl2591_read_byte(slave, tsl2591_reg_id, &regval))
		return(sensor_not_found);

	if((regval & tsl2591_id_mask) != tsl2591_id_value)
		return(sensor_not_found);

	if(tsl2591_write_check(slave, tsl2591_reg_id, 0x00)) // id register should not be writable
		return(sensor_not_found);

	return(sensor_found);
}

static bool tsl2591_init(data_t *data)
{
	tsl2591_private_data_t *pdata = static_cast<tsl2591_private_data_t *>(data->private_data);

	assert(pdata);

	pdata->state = tsl2591_state_init;
	pdata->scaling = tsl2591_autoranging_data_size - 1;
	pdata->scales_up = 0;
	pdata->scales_down = 0;
	pdata->overflows = 0;
	pdata->channel[0] = 0;
	pdata->channel[1] = 0;
	pdata->ratio = 0;
	pdata->ratio_index = 0;
	pdata->factor[0] = 0;
	pdata->factor[1] = 0;

	return(true);
}

static bool tsl2591_poll(data_t *data)
{
	tsl2591_private_data_t *pdata = static_cast<tsl2591_private_data_t *>(data->private_data);
	uint8_t buffer[4];
	unsigned int overflow, scale_down_threshold, scale_up_threshold;
	unsigned int control_opcode;
	bool found;

	assert(pdata);

	switch(pdata->state)
	{
		case(tsl2591_state_init):
		{
			tsl2591_write(data->slave, tsl2591_reg_control, tsl2591_control_sreset);
			pdata->state = tsl2591_state_reset;

			break;
		}

		case(tsl2591_state_reset):
		{
			if(!tsl2591_write_check(data->slave, tsl2591_reg_enable, tsl2591_enable_aen | tsl2591_enable_pon))
			{
				Log::get() << "tsl2591: poll: error 1";
				break;
			}

			pdata->state = tsl2591_state_ready;

			break;
		}

		case(tsl2591_state_ready):
		case(tsl2591_state_finished):
		{
			control_opcode = tsl2591_autoranging_data[pdata->scaling].data[0] | tsl2591_autoranging_data[pdata->scaling].data[1] ;

			if(!tsl2591_write_check(data->slave, tsl2591_reg_control, control_opcode))
			{
				Log::get() << "tsl2591: poll: error 2";
				break;
			}

			pdata->state = tsl2591_state_measuring;
			break;
		}

		case(tsl2591_state_measuring):
		{
			pdata->state = tsl2591_state_finished;

			scale_down_threshold =	tsl2591_autoranging_data[pdata->scaling].threshold.down;
			scale_up_threshold =	tsl2591_autoranging_data[pdata->scaling].threshold.up;
			overflow =				tsl2591_autoranging_data[pdata->scaling].overflow;

			if(!i2c_send_1_receive(data->slave, tsl2591_cmd_cmd | static_cast<unsigned int>(tsl2591_reg_c0datal), sizeof(buffer), buffer))
			{
				Log::get() << "tsl2591: poll: error 3";
				break;
			}

			pdata->channel[0] = unsigned_16_le(&buffer[0]);
			pdata->channel[1] = unsigned_16_le(&buffer[2]);

			if(((pdata->channel[0] < scale_down_threshold) || (pdata->channel[1] < scale_down_threshold)) && (pdata->scaling > 0))
			{
				pdata->scaling--;
				pdata->scales_down++;
				break;
			}

			if(((pdata->channel[0] >= scale_up_threshold) || (pdata->channel[1] >= scale_up_threshold)) && (pdata->scaling < (tsl2591_autoranging_data_size - 1)))
			{
				pdata->scaling++;
				pdata->scales_up++;
				break;
			}

			if((pdata->channel[0] <= 0) || (pdata->channel[1] <= 0) || (pdata->channel[0] >= overflow) || (pdata->channel[1] >= overflow))
			{
				pdata->overflows++;
				break;
			}

			pdata->ratio = (float)pdata->channel[1] / (float)pdata->channel[0];
			found = false;

			for(pdata->ratio_index = 0; pdata->ratio_index < tsl2591_factor_size; pdata->ratio_index++)
			{
				if((pdata->ratio >= tsl2591_factors[pdata->ratio_index].lower_bound) && (pdata->ratio < tsl2591_factors[pdata->ratio_index].upper_bound))
				{
					pdata->factor[0] = tsl2591_factors[pdata->ratio_index].ch_factor[0];
					pdata->factor[1] = tsl2591_factors[pdata->ratio_index].ch_factor[1];
					found = true;

					break;
				}
			}

			if(!found)
			{
				pdata->overflows++;
				break;
			}

			data->values[sensor_type_visible_light].value = (((pdata->channel[0] * pdata->factor[0]) + (pdata->channel[1] * pdata->factor[1])) / 1000.0f) *
					tsl2591_autoranging_data[pdata->scaling].factor;
			data->values[sensor_type_visible_light].stamp = time(nullptr);

			break;
		}
	}

	return(true);
}

static void tsl2591_dump(const data_t *data, std::string &output)
{
	tsl2591_private_data_t *pdata = static_cast<tsl2591_private_data_t *>(data->private_data);

	output += (boost::format("state: %u, ") % pdata->state).str();
	output += (boost::format("scaling: %u, ") % pdata->scaling).str();
	output += (boost::format("scales up: %u, ") % pdata->scales_up).str();
	output += (boost::format("scales down: %u, ") % pdata->scales_down).str();
	output += (boost::format("overflows: %u, ") % pdata->overflows).str();
	output += (boost::format("data channel 0: %u, ") % pdata->channel[0]).str();
	output += (boost::format("data channel 1: %u, ") % pdata->channel[1]).str();
	output += (boost::format("ratio: %f, ") % (double)pdata->ratio).str();
	output += (boost::format("ratio index: %u, ") % pdata->ratio_index).str();
	output += (boost::format("factor 0: %f, ") % (double)pdata->factor[0]).str();
	output += (boost::format("factor 1: %f") % (double)pdata->factor[1]).str();
}

static sensor_detect_t tsl2591_28_detect(i2c_slave_t slave)
{
	const data_t *data;
	i2c_module_t this_module, module;
	i2c_bus_t this_bus, bus;
	unsigned int this_address, address;
	const char *this_name, *name;

	if(i2c_get_slave_info(slave, &this_module, &this_bus, &this_address, &this_name))
		for(data = data_root; data; data = data->next)
			if(i2c_get_slave_info(data->slave, &module, &bus, &address, &name) &&
						(data->state == sensor_found) && (module == this_module) && (bus == this_bus) && (data->info->id == sensor_tsl2591))
				return(sensor_disabled);

	return(sensor_not_found);
}

enum
{
	am2320_command_read_register = 0x03,
};

enum
{
	am2320_register_values = 0x00,
	am2320_register_values_length = 0x04,
	am2320_register_id = 0x08,
	am2320_register_id_length = 0x02,
};

typedef enum
{
	am2320_state_init,
	am2320_state_waking,
	am2320_state_measuring,
} am2320_state_t;

typedef struct
{
	am2320_state_t state;
	uint32_t raw_humidity_data;
	uint32_t raw_temperature_data;
} am2320_private_data_t;

static unsigned int am2320_crc16(int length, const uint8_t *data)
{
	uint8_t outer, inner, testbit;
	uint16_t crc;

	crc = 0xffff;

	for(outer = 0; outer < length; outer++)
	{
		crc ^= data[outer];

		for(inner = 0; inner < 8; inner++)
		{
			testbit = !!(crc & 0x01);
			crc >>= 1;
			if(testbit)
				crc ^= 0xa001;
		}
	}

	return(crc);
}

static sensor_detect_t am2320_detect(i2c_slave_t slave)
{
	unsigned int crc1, crc2;
	uint8_t buffer[am2320_register_id_length + 4];
	i2c_module_t module;
	i2c_bus_t bus;
	unsigned int address;
	const char *name;

	if(!i2c_get_slave_info(slave, &module, &bus, &address, &name))
	{
		Log::get() << "am2320: detect: get_slave_info failed";
		return(sensor_not_found);
	}

	i2c_probe_slave(module, bus, address);
	util_sleep(50);

	if(!i2c_probe_slave(module, bus, address))
		return(sensor_not_found);

	// request ID (but do not check it, it's unreliable)
	if(!i2c_send_3(slave, am2320_command_read_register, am2320_register_id, am2320_register_id_length))
		return(sensor_not_found);

	if(!i2c_receive(slave, sizeof(buffer), buffer))
		return(sensor_not_found);

	if((buffer[0] != am2320_command_read_register) || (buffer[1] != am2320_register_id_length))
		return(sensor_not_found);

	crc1 = unsigned_16_le(&buffer[am2320_register_id_length + 2]);
	crc2 = am2320_crc16(am2320_register_id_length + 2, buffer);

	if(crc1 != crc2)
		return(sensor_not_found);

	return(sensor_found);
}

static bool am2320_init(data_t *data)
{
	am2320_private_data_t *pdata = static_cast<am2320_private_data_t *>(data->private_data);

	pdata->state = am2320_state_init;
	pdata->raw_temperature_data = 0;
	pdata->raw_humidity_data = 0;

	return(true);
}

static bool am2320_poll(data_t *data)
{
	am2320_private_data_t *pdata = static_cast<am2320_private_data_t *>(data->private_data);
	uint8_t buffer[am2320_register_values_length + 4];
	unsigned int crc1, crc2;
	int humidity, temperature;
	i2c_module_t module;
	i2c_bus_t bus;
	unsigned int address;
	const char *name;

	if(!i2c_get_slave_info(data->slave, &module, &bus, &address, &name))
	{
		Log::get() << "am2320: poll: get_slave_info failed";
		return(sensor_not_found);
	}

	assert(pdata);

	switch(pdata->state)
	{
		case(am2320_state_init):
		case(am2320_state_waking):
		{
			i2c_probe_slave(module, bus, address); // FIXME

			pdata->state = am2320_state_measuring;
			break;
		}

		case(am2320_state_measuring):
		{
			pdata->state = am2320_state_waking;

			if(!i2c_send_3(data->slave, am2320_command_read_register, am2320_register_values, am2320_register_values_length))
				return(false);

			if(!i2c_receive(data->slave, sizeof(buffer), buffer))
			{
				Log::get() << "am2320: poll error 1";
				return(false);
			}

			if((buffer[0] != am2320_command_read_register) || (buffer[1] != am2320_register_values_length))
			{
				Log::get() << "am2320: poll error 2";
				return(false);
			}

			crc1 = unsigned_16_le(&buffer[am2320_register_values_length + 2]);
			crc2 = am2320_crc16(am2320_register_values_length + 2, buffer);

			if(crc1 != crc2)
			{
				Log::get() << "am2320: poll error 3";
				return(false);
			}

			pdata->raw_temperature_data = unsigned_16_be(&buffer[4]);
			pdata->raw_humidity_data = unsigned_16_be(&buffer[2]);

			temperature = (unsigned int)pdata->raw_temperature_data;

			if(temperature & 0x8000)
			{
				temperature &= 0x7fff;
				temperature = 0 - temperature;
			}

			humidity = pdata->raw_humidity_data;

			if(humidity > 1000)
				humidity  = 1000;

			data->values[sensor_type_temperature].value = temperature / 10.0;
			data->values[sensor_type_temperature].stamp = time(nullptr);

			data->values[sensor_type_humidity].value = humidity / 10.0;
			data->values[sensor_type_humidity].stamp = time(nullptr);
		}

		break;
	}

	return(true);
}

static void am2320_dump(const data_t *data, std::string &output)
{
	am2320_private_data_t *pdata = static_cast<am2320_private_data_t *>(data->private_data);

	assert(pdata);

	output += (boost::format("state: %d, ") % pdata->state).str();
	output += (boost::format("raw temperature data: %lu, ") % pdata->raw_temperature_data).str();
	output += (boost::format("raw humidity data: %lu") % pdata->raw_humidity_data).str();
}

static const info_t info[sensor_size] =
{
	[sensor_bh1750] =
	{
		.name = "bh1750",
		.id = sensor_bh1750,
		.address = 0x23,
		.type = static_cast<sensor_type_t>(1 << sensor_type_visible_light),
		.flags
		{
			.force_detect = 0,
			.no_constrained = 1,
		},
		.precision = 0,
		.private_data_size = sizeof(bh1750_private_data_t),
		.detect_fn = bh1750_detect,
		.init_fn = bh1750_init,
		.poll_fn = bh1750_poll,
		.dump_fn = bh1750_dump,
	},
	[sensor_tmp75] =
	{
		.name = "tmp75",
		.id = sensor_tmp75,
		.address = 0x48,
		.type = static_cast<sensor_type_t>(1 << sensor_type_temperature),
		.flags = {},
		.precision = 1,
		.private_data_size = sizeof(tmp75_private_data_t),
		.detect_fn = tmp75_detect,
		.init_fn = tmp75_init,
		.poll_fn = tmp75_poll,
		.dump_fn = tmp75_dump,
	},
	[sensor_opt3001] =
	{
		.name = "opt3001",
		.id = sensor_opt3001,
		.address = 0x45,
		.type = static_cast<sensor_type_t>(1 << sensor_type_visible_light),
		.flags = {},
		.precision = 2,
		.private_data_size = sizeof(opt3001_private_data_t),
		.detect_fn = opt3001_detect,
		.init_fn = opt3001_init,
		.poll_fn = opt3001_poll,
		.dump_fn = opt3001_dump,
	},
	[sensor_max44009] =
	{
		.name = "max44009",
		.id = sensor_max44009,
		.address = 0x4a,
		.type = static_cast<sensor_type_t>(1 << sensor_type_visible_light),
		.flags = {},
		.precision = 2,
		.private_data_size = sizeof(max44009_private_data_t),
		.detect_fn = max44009_detect,
		.init_fn = max44009_init,
		.poll_fn = max44009_poll,
		.dump_fn = max44009_dump,
	},
	[sensor_asair] =
	{
		.name = "asair",
		.id = sensor_asair,
		.address = 0x38,
		.type = static_cast<sensor_type_t>((1 << sensor_type_temperature) | (1 << sensor_type_humidity)),
		.flags = {},
		.precision = 1,
		.private_data_size = sizeof(asair_private_data_t),
		.detect_fn = asair_detect,
		.init_fn = asair_init,
		.poll_fn = asair_poll,
		.dump_fn = asair_dump,
	},
	[sensor_apds9930] =
	{
		.name = "apds9930",
		.id = sensor_apds9930,
		.address = 0x39,
		.type = static_cast<sensor_type_t>(1 << sensor_type_visible_light),
		.flags = {},
		.precision = 2,
		.private_data_size = sizeof(apds9930_private_data_t),
		.detect_fn = apds9930_detect,
		.init_fn = apds9930_init,
		.poll_fn = apds9930_poll,
		.dump_fn = apds9930_dump,
	},
	[sensor_tsl2561] =
	{
		.name = "tsl2561",
		.id = sensor_tsl2561,
		.address = 0x39,
		.type = static_cast<sensor_type_t>(1 << sensor_type_visible_light),
		.flags = {},
		.precision = 2,
		.private_data_size = sizeof(tsl2561_private_data_t),
		.detect_fn = tsl2561_detect,
		.init_fn = tsl2561_init,
		.poll_fn = tsl2561_poll,
		.dump_fn = tsl2561_dump,
	},
	[sensor_hdc1080] =
	{
		.name = "hdc1080",
		.id = sensor_hdc1080,
		.address = 0x40,
		.type = static_cast<sensor_type_t>((1 << sensor_type_temperature) | (1 << sensor_type_humidity)),
		.flags
		{
			.force_detect = 0,
			.no_constrained = 1,
		},
		.precision = 1,
		.private_data_size = sizeof(hdc1080_private_data_t),
		.detect_fn = hdc1080_detect,
		.init_fn = hdc1080_init,
		.poll_fn = hdc1080_poll,
		.dump_fn = hdc1080_dump,
	},
	[sensor_sht3x] =
	{
		.name = "sht3x",
		.id = sensor_sht3x,
		.address = 0x44,
		.type = static_cast<sensor_type_t>((1 << sensor_type_temperature) | (1 << sensor_type_humidity)),
		.flags
		{
			.force_detect = 0,
			.no_constrained = 1,
		},
		.precision = 1,
		.private_data_size = sizeof(sht3x_private_data_t),
		.detect_fn = sht3x_detect,
		.init_fn = sht3x_init,
		.poll_fn = sht3x_poll,
		.dump_fn = sht3x_dump,
	},
	[sensor_bmx280] =
	{
		.name = "bmx280",
		.id = sensor_bmx280,
		.address = 0x76,
		.type = static_cast<sensor_type_t>((1 << sensor_type_temperature) | (1 << sensor_type_humidity) | (1 << sensor_type_airpressure)),
		.flags = {},
		.precision = 1,
		.private_data_size = sizeof(bmx280_private_data_t),
		.detect_fn = bmx280_detect,
		.init_fn = bmx280_init,
		.poll_fn = bmx280_poll,
		.dump_fn = bmx280_dump,
	},
	[sensor_htu21] =
	{
		.name = "htu21",
		.id = sensor_htu21,
		.address = 0x40,
		.type = static_cast<sensor_type_t>((1 << sensor_type_temperature) | (1 << sensor_type_humidity)),
		.flags
		{
			.force_detect = 0,
			.no_constrained = 1,
		},
		.precision = 1,
		.private_data_size = sizeof(htu21_private_data_t),
		.detect_fn = htu21_detect,
		.init_fn = htu21_init,
		.poll_fn = htu21_poll,
		.dump_fn = htu21_dump,
	},
	[sensor_veml7700] =
	{
		.name = "veml7700",
		.id = sensor_veml7700,
		.address = 0x10,
		.type = static_cast<sensor_type_t>(1 << sensor_type_visible_light),
		.flags = {},
		.precision = 2,
		.private_data_size = sizeof(veml7700_private_data_t),
		.detect_fn = veml7700_detect,
		.init_fn = veml7700_init,
		.poll_fn = veml7700_poll,
		.dump_fn = veml7700_dump,
	},
	[sensor_bme680] =
	{
		.name = "bme680",
		.id = sensor_bme680,
		.address = 0x76,
		.type = static_cast<sensor_type_t>((1 << sensor_type_temperature) | (1 << sensor_type_humidity) | (1 << sensor_type_airpressure)),
		.flags = {},
		.precision = 1,
		.private_data_size = sizeof(bme680_private_data_t),
		.detect_fn = bme680_detect,
		.init_fn = bme680_init,
		.poll_fn = bme680_poll,
		.dump_fn = bme680_dump,
	},
	[sensor_apds9960] =
	{
		.name = "apds9960",
		.id = sensor_apds9960,
		.address = 0x39,
		.type = static_cast<sensor_type_t>(1 << sensor_type_visible_light),
		.flags = {},
		.precision = 2,
		.private_data_size = sizeof(apds9960_private_data_t),
		.detect_fn = apds9960_detect,
		.init_fn = apds9960_init,
		.poll_fn = apds9960_poll,
		.dump_fn = apds9960_dump,
	},
	[sensor_tsl2591] =
	{
		.name = "tsl2591",
		.id = sensor_tsl2591,
		.address = 0x29,
		.type = static_cast<sensor_type_t>(1 << sensor_type_visible_light),
		.flags = {},
		.precision = 2,
		.private_data_size = sizeof(tsl2591_private_data_t),
		.detect_fn = tsl2591_detect,
		.init_fn = tsl2591_init,
		.poll_fn = tsl2591_poll,
		.dump_fn = tsl2591_dump,
	},
	[sensor_tsl2591_28] =
	{
		.name = "tsl2591-dummy",
		.id = sensor_tsl2591_28,
		.address = 0x28,
		.type = static_cast<sensor_type_t>(1 << sensor_type_visible_light),
		.flags = {},
		.precision = 2,
		.private_data_size = 0,
		.detect_fn = tsl2591_28_detect,
		.init_fn = nullptr,
		.poll_fn = nullptr,
		.dump_fn = nullptr,
	},
	[sensor_am2320] =
	{
		.name = "am2320",
		.id = sensor_am2320,
		.address = 0x5c,
		.type = static_cast<sensor_type_t>((1 << sensor_type_temperature) | (1 << sensor_type_humidity)),
		.flags
		{
			.force_detect = 1,
			.no_constrained = 1,
		},
		.precision = 1,
		.private_data_size = sizeof(am2320_private_data_t),
		.detect_fn = am2320_detect,
		.init_fn = am2320_init,
		.poll_fn = am2320_poll,
		.dump_fn = am2320_dump,
	},
};

static void run_sensors(void *parameters)
{
	const info_t *infoptr;
	sensor_t sensor;
	data_t *dataptr, *new_data;
	i2c_module_t module;
	i2c_bus_t bus;
	i2c_slave_t slave;
	unsigned int buses;
	sensor_type_t type;
	sensor_detect_t detected;
	auto run = (const run_parameters_t *)parameters;

	module = run->module;
	buses = i2c_buses(module);

	for(bus = i2c_bus_first; bus < buses; bus = static_cast<i2c_bus_t>(bus + 1))
	{
		for(sensor = sensor_first; sensor < sensor_size; sensor = static_cast<sensor_t>(sensor + 1))
		{
			infoptr = &info[sensor];

			if(i2c_ulp(module) && infoptr->flags.no_constrained)
			{
				stat_sensors_not_considered[module]++;
				continue;
			}

			if(i2c_find_slave(module, bus, infoptr->address))
			{
				stat_sensors_skipped[module]++;
				continue;
			}

			stat_sensors_not_skipped[module]++;

			if(!infoptr->flags.force_detect && !i2c_probe_slave(module, bus, infoptr->address))
				continue;

			stat_sensors_probed[module]++;

			if(!(slave = i2c_register_slave(infoptr->name, module, bus, infoptr->address)))
			{
				Log::get() << std::format("sensor: warning: cannot register sensor {}", infoptr->name);
				continue;
			}

			assert(infoptr->detect_fn);

			detected = infoptr->detect_fn(slave);

			if(detected == sensor_not_found)
			{
				i2c_unregister_slave(&slave);
				continue;
			}

			stat_sensors_found[module]++;

			new_data = new data_t;

			for(type = sensor_type_first; type < sensor_type_size; type = static_cast<sensor_type_t>(type + 1))
			{
				new_data->values[type].value = 0;
				new_data->values[type].stamp = (time_t)0;
			}

			new_data->state = detected;
			new_data->slave = slave;
			new_data->info = infoptr;

			new_data->private_data = nullptr;
			new_data->next = nullptr;

			if(infoptr->private_data_size > 0)
				new_data->private_data = malloc(infoptr->private_data_size);

			if(detected == sensor_disabled)
				stat_sensors_disabled[module]++;
			else
			{
				assert(infoptr->init_fn);

				if(!infoptr->init_fn(new_data))
				{
					Log::get() << std::format("sensor: warning: failed to init sensor {} on bus {:d}", infoptr->name, static_cast<unsigned int>(bus));
					i2c_unregister_slave(&slave);
					if(new_data->private_data)
						free(new_data->private_data);
					delete new_data;
					continue;
				}

				stat_sensors_confirmed[module]++;
			}

			data_mutex_take();

			if(!(dataptr = data_root))
				data_root = new_data;
			else
			{
				for(; dataptr && dataptr->next; dataptr = dataptr->next)
					(void)0;

				assert(dataptr);

				dataptr->next = new_data;
			}

			data_mutex_give();
		}
	}

	if(stat_sensors_confirmed[module] == 0)
		vTaskDelete(nullptr);

	for(;;)
	{
		stat_poll_run[module]++;

		for(dataptr = data_root; dataptr; dataptr = dataptr->next)
			if(dataptr->state == sensor_disabled)
				stat_poll_skipped[module]++;
			else
				if(dataptr->info->poll_fn)
					if(dataptr->info->poll_fn(dataptr))
						stat_poll_ok[module]++;
					else
						stat_poll_error[module]++;
				else
					Log::get() << std::format("sensor: error: no poll function for sensor {}", dataptr->info->name);

		util_sleep(1000);
	}

	util_abort("sensor: poll task returned");
}

void sensor_init(void)
{
	static run_parameters_t run[i2c_module_size] =
	{
		[i2c_module_0] =
		{
			.module = i2c_module_0,
		},
		[i2c_module_1] =
		{
			.module = i2c_module_1,
		},
		[i2c_module_2_ulp] =
		{
			.module = i2c_module_2_ulp,
		},
	};
	i2c_module_t thread;
	char name[16];

	assert(!inited);

	data_mutex = xSemaphoreCreateMutex();
	assert(data_mutex);

	inited = true;

	for(thread = i2c_module_first; thread < i2c_module_size; thread = static_cast<i2c_module_t>(thread + 1))
	{
		if(i2c_module_available(thread))
		{
			util_sleep(100);

			snprintf(name, sizeof(name), "sensors %d", thread);

			if(xTaskCreatePinnedToCore(run_sensors, name, 3 * 1024, &run[thread], 1, nullptr, 1) != pdPASS)
				util_abort("sensor: xTaskCreatePinnedToNode sensors thread");
		}
	}
}

void command_sensor_info(cli_command_call_t *call)
{
	data_t *dataptr;
	i2c_slave_t slave;
	const char *name;
	i2c_module_t module;
	i2c_bus_t bus;
	unsigned int address;
	sensor_type_t type;
	bool include_disabled;

	include_disabled = false;

	if((call->parameter_count > 0) && (call->parameters[0].unsigned_int > 0))
		include_disabled = true;

	call->result = "SENSOR info";

	if(!inited)
	{
		call->result += "\n--";
		return;
	}

	data_mutex_take();

	for(dataptr = data_root; dataptr; dataptr = dataptr->next)
	{
		slave = dataptr->slave;

		if((dataptr->state == sensor_disabled) && !include_disabled)
			continue;

		if(!slave)
		{
			call->result += "\nslave = NULL";
			continue;
		}

		if(!i2c_get_slave_info(slave, &module, &bus, &address, &name))
			call->result += "\n- unknown slave";
		else
		{
			call->result += (boost::format("\n- %s@%d/%d/%x:") % name % module % bus % address).str();

			if(dataptr->state == sensor_disabled)
				call->result += " [disabled]";
			else
				for(type = sensor_type_first; type < sensor_type_size; type = static_cast<sensor_type_t>(type + 1))
					if(dataptr->info->type & (1 << type))
					{
						std::string format_string = (boost::format(" %%s: %%.%uf %%s") % dataptr->info->precision).str();

						call->result += (boost::format(format_string) %
								sensor_type_info[type].type %
								dataptr->values[type].value %
								sensor_type_info[type].unity).str();
					}
		}
	}

	data_mutex_give();
}

void command_sensor_json(cli_command_call_t *call)
{
	data_t *dataptr;
	i2c_slave_t slave;
	const char *name;
	i2c_module_t module;
	i2c_bus_t bus;
	unsigned int address;
	sensor_type_t type;
	bool first_sensor;
	bool first_value;

	assert(call->parameter_count == 0);

	if(!inited)
		return;

	call->result = "{";

	data_mutex_take();

	for(dataptr = data_root, first_sensor = true; dataptr; dataptr = dataptr->next)
	{
		if(dataptr->state == sensor_disabled)
			continue;

		slave = dataptr->slave;

		if(slave && i2c_get_slave_info(slave, &module, &bus, &address, &name))
		{
			call->result += first_sensor ? "" : ",";
			call->result += (boost::format("\n\"%u-%u-%x\":") % module % bus % address).str();
			call->result +=	"\n[";
			call->result +=	"\n{";
			call->result += (boost::format("\n\"module\": %u,") % module).str();
			call->result += (boost::format("\n\"bus\": %u,") % bus).str();
			call->result += (boost::format("\n\"name\": \"%s\",") % name).str();
			call->result +=	"\n\"values\":";
			call->result +=	"\n[";

			for(type = sensor_type_first, first_value = true; type < sensor_type_size; type = static_cast<sensor_type_t>(type + 1))
			{
				if(dataptr->info->type & (1 << type))
				{
					call->result += first_value ? "" : ",";
					call->result +=	"\n{";
					call->result += (boost::format("\n\"type\": \"%s\",") % sensor_type_info[type].type).str();
					call->result += (boost::format("\n\"id\": %d,") % dataptr->info->id).str();
					call->result += (boost::format("\n\"address\": %u,") % address).str();
					call->result += (boost::format("\n\"unity\": \"%s\",") % sensor_type_info[type].unity).str();
					call->result += (boost::format("\n\"value\": %f,") % (double)dataptr->values[type].value).str();
					call->result += (boost::format("\n\"time\": %lld") % dataptr->values[type].stamp).str();
					call->result +=	"\n}";

					first_value = false;
				}
			}

			call->result += "\n]";
			call->result += "\n}";
			call->result += "\n]";

			first_sensor = false;
		}
	}

	call->result += "\n}";

	data_mutex_give();
}

void command_sensor_dump(cli_command_call_t *call)
{
	data_t *dataptr;
	i2c_slave_t slave;
	const char *name;
	i2c_module_t module;
	i2c_bus_t bus;
	unsigned int address;
	sensor_type_t type;
	unsigned int index;

	assert(call->parameter_count < 2);

	call->result = "SENSOR dump";

	if(!inited)
	{
		call->result += "\n--";
		return;
	}

	data_mutex_take();

	for(dataptr = data_root, index = 0; dataptr; dataptr = dataptr->next, index++)
	{
		if((call->parameter_count > 0) && (call->parameters[0].unsigned_int != index))
			continue;

		slave = dataptr->slave;

		if(!slave)
		{
			call->result += "\nslave = NULL";
			continue;
		}

		if(!i2c_get_slave_info(slave, &module, &bus, &address, &name))
			call->result += "\n- unknown slave";
		else
		{
			call->result += (boost::format("\n- sensor %s at module %u, bus %u, address 0x%x") % name % module % bus % address).str();

			if(dataptr->state == sensor_disabled)
				call->result += ": [disabled]";
			else
			{
				call->result += "\n  values:";

				for(type = sensor_type_first; type < sensor_type_size; type = static_cast<sensor_type_t>(type + 1))
				{
					if(dataptr->info->type & (1 << type))
					{
						std::string format_string = (boost::format(" %%s=%%.%uf [%%s]") % dataptr->info->precision).str();
						call->result += (boost::format(format_string) %
								sensor_type_info[type].type % dataptr->values[type].value %
								util_time_to_string(dataptr->values[type].stamp)).str();
					}
				}

				if(dataptr->info->dump_fn)
				{
					call->result += "\n  private data: ";
					dataptr->info->dump_fn(dataptr, call->result);
				}
			}
		}
	}

	data_mutex_give();
}

void command_sensor_stats(cli_command_call_t *call)
{
	i2c_module_t module;

	assert(call->parameter_count == 0);

	call->result = "SENSOR statistics";

	for(module = i2c_module_first; module < i2c_module_size; module = static_cast<i2c_module_t>(module + 1))
	{
		call->result += (boost::format("\n- module %u") % module).str();
		call->result += (boost::format("\n-  sensors not considered: %u") % stat_sensors_not_considered[module]).str();
		call->result += (boost::format("\n-  sensors skipped: %u") % stat_sensors_skipped[module]).str();
		call->result += (boost::format("\n-  sensors not skipped: %u") % stat_sensors_not_skipped[module]).str();
		call->result += (boost::format("\n-  sensors probed: %u") % stat_sensors_probed[module]).str();
		call->result += (boost::format("\n-  sensors found: %u") % stat_sensors_found[module]).str();
		call->result += (boost::format("\n-  sensors confirmed: %u") % stat_sensors_confirmed[module]).str();
		call->result += (boost::format("\n-  sensors disabled: %u") % stat_sensors_disabled[module]).str();
		call->result += (boost::format("\n-  complete poll runs: %u") % stat_poll_run[module]).str();
		call->result += (boost::format("\n-  sensor poll succeeded: %u") % stat_poll_ok[module]).str();
		call->result += (boost::format("\n-  sensor poll skipped: %u") % stat_poll_skipped[module]).str();
		call->result += (boost::format("\n-  sensor poll failed: %u") % stat_poll_error[module]).str();
	}
}
