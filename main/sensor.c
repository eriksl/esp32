#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>

#include "string.h"
#include "info.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "sensor.h"
#include "i2c.h"

#include <assert.h>

typedef struct
{
	float value;
	time_t stamp;
} sensor_value_t;

typedef struct data_T
{
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
	unsigned int precision;
	size_t private_data_size;
	bool (*const detect_fn)(i2c_slave_t);
	bool (*const init_fn)(data_t *);
	bool (*const poll_fn)(data_t *);
	void (*const dump_fn)(const data_t *, string_t);
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
static data_t *data_root = (data_t *)0;

static unsigned int stat_sensors_skipped[i2c_module_size] = { 0 };
static unsigned int stat_sensors_probed[i2c_module_size] = { 0 };
static unsigned int stat_sensors_found[i2c_module_size] = { 0 };
static unsigned int stat_sensors_confirmed[i2c_module_size] = { 0 };
static unsigned int stat_poll_run[i2c_module_size] = { 0 };
static unsigned int stat_poll_ok[i2c_module_size] = { 0 };
static unsigned int stat_poll_error[i2c_module_size] = { 0 };

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
	struct
	{
		float factor;
		float offset;
	} correction;
} device_autoranging_data_t;

static unsigned int unsigned_20_top_be(const uint8_t ptr[3])
{
	return((((ptr[0] & 0xff) >> 0) << 12) | (((ptr[1] & 0xff) >> 0) << 4) | (((ptr[2] & 0xf0) >> 4) << 0));
}

static unsigned int unsigned_20_bottom_be(const uint8_t ptr[3])
{
	return((((ptr[0] & 0x0f) >> 0) << 16) | (((ptr[1] & 0xff) >> 0) << 8) | (((ptr[2] & 0xff) >> 0) << 0));
}

static unsigned int unsigned_16_be(const uint8_t ptr[2])
{
	return(((ptr[0] & 0xff) << 8) | (ptr[1] & 0xff));
}

static unsigned int unsigned_16_le(const uint8_t ptr[2])
{
	return(((ptr[1] & 0xff) << 8) | (ptr[0] & 0xff));
}

static int signed_16_le(const uint8_t ptr[2])
{
	int rv = ((ptr[1] & 0xff) << 8) | (ptr[0] & 0xff);

	if(rv > (1 << 15))
		rv = 0 - ((1 << 16) - rv);

	return(rv);
}

static unsigned int unsigned_12_top_be(const uint8_t ptr[2])
{
	return((((ptr[0] & 0xff) >> 0) << 4) | (((ptr[1] & 0xf0) >> 4) << 0));
}

static unsigned int unsigned_12_bottom_le(const uint8_t ptr[2])
{
	return((((ptr[0] & 0x0f) >> 0) << 0) | (((ptr[1] & 0xff) >> 0) << 4));
}

static unsigned int unsigned_8(const uint8_t ptr[1])
{
	return((unsigned int)(*ptr & 0xff));
}

static int signed_8(const uint8_t ptr[1])
{
	int rv = (unsigned int)(*ptr & 0xff);

	if(rv > (1 << 7))
		rv = 0 - ((1 << 8) - rv);

	return(rv);
}

enum
{
	bh1750_opcode_powerdown =		0b00000000, // 0x00
	bh1750_opcode_poweron =			0b00000001, // 0x01
	bh1750_opcode_reset =			0b00000111, // 0x07
	bh1750_opcode_cont_hmode =		0b00010000, // 0x10
	bh1750_opcode_cont_hmode2 =		0b00010001, // 0x11
	bh1750_opcode_cont_lmode =		0b00010011, // 0x13
	bh1750_opcode_one_hmode =		0b00100000, // 0x20
	bh1750_opcode_one_hmode2 =		0b00100001, // 0x21
	bh1750_opcode_one_lmode =		0b00100011, // 0x23
	bh1750_opcode_change_meas_hi =	0b01000000, // 0x40
	bh1750_opcode_change_meas_lo =	0b01100000, // 0x60
};

typedef enum
{
	bh1750_state_init,
	bh1750_state_measuring,
	bh1750_state_finished,
}  bh1750_state_t;

enum
{
	bh1750_autoranging_data_size = 4,
};

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
	{{	bh1750_opcode_one_hmode2,	254	},	{ 0,	50000 }, 65535, { 0.13,	0,	}},
	{{	bh1750_opcode_one_hmode2,	69	},	{ 1000, 50000 }, 65535, { 0.50,	0,	}},
	{{	bh1750_opcode_one_hmode2,	31	},	{ 1000, 50000 }, 65535, { 1.10,	0,	}},
	{{	bh1750_opcode_one_lmode,	31	},	{ 1000, 65536 }, 65535, { 2.40,	0,	}},
};

static bool bh1750_detect(i2c_slave_t slave)
{
	uint8_t buffer[8];

	if(!i2c_receive(slave, sizeof(buffer), buffer))
		return(false);

	if((buffer[2] != 0xff) || (buffer[3] != 0xff) || (buffer[4] != 0xff) ||
			(buffer[5] != 0xff) || (buffer[6] != 0xff) || (buffer[7] != 0xff))
		return(false);

	return(true);
}

static bool bh1750_init(data_t *data)
{
	bh1750_private_data_t *pdata = data->private_data;

	assert(pdata);

	if(!i2c_send_1(data->slave, bh1750_opcode_poweron))
		return(false);

	if(!i2c_send_1(data->slave, bh1750_opcode_reset))
		return(false);

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
	bh1750_private_data_t *pdata = data->private_data;
	float factor, offset;
	unsigned int overflow, scale_down_threshold, scale_up_threshold;
	uint8_t buffer[2];

	assert(pdata);

	switch(pdata->state)
	{
		case(bh1750_state_finished):
		case(bh1750_state_init):
		{
			if(!i2c_send_1(data->slave, bh1750_opcode_change_meas_hi | ((bh1750_autoranging_data[pdata->scaling].data[1] >> 5) & 0b00000111)))
			{
				log("bh1750: warning: error sending change meas hi");
				return(false);
			}

			if(!i2c_send_1(data->slave, bh1750_opcode_change_meas_lo | ((bh1750_autoranging_data[pdata->scaling].data[1] >> 0) & 0b00011111)))
			{
				log("bh1750: warning: error sending change meas lo");
				return(false);
			}

			if(!i2c_send_1(data->slave, bh1750_autoranging_data[pdata->scaling].data[0]))
			{
				log("bh1750: warning: error sending opcode");
				return(false);
			}

			pdata->state = bh1750_state_measuring;

			break;
		}

		case(bh1750_state_measuring):
		{
			scale_down_threshold =	bh1750_autoranging_data[pdata->scaling].threshold.down;
			scale_up_threshold =	bh1750_autoranging_data[pdata->scaling].threshold.up;
			overflow =				bh1750_autoranging_data[pdata->scaling].overflow;
			factor =				bh1750_autoranging_data[pdata->scaling].correction.factor;
			offset =				bh1750_autoranging_data[pdata->scaling].correction.offset;

			pdata->state = bh1750_state_finished;

			if(!i2c_receive(data->slave, 2, buffer))
			{
				log("bh1750: poll: warning: error in receive data");
				return(false);
			}

			pdata->raw_value = unsigned_16_be(buffer);

			if((pdata->raw_value >= overflow) && (pdata->scaling >= (bh1750_autoranging_data_size - 1)))
			{
				pdata->overflows++;
				break;
			}

			if((pdata->raw_value < scale_down_threshold) && (pdata->scaling > 0))
			{
				pdata->scaling--;
				pdata->scaling_down++;
				break;
			}

			if((pdata->raw_value >= scale_up_threshold) && (pdata->scaling < (bh1750_autoranging_data_size - 1)))
			{
				pdata->scaling++;
				pdata->scaling_up++;
				break;
			}

			data->values[sensor_type_visible_light].value = ((float)pdata->raw_value * factor) + offset;
			data->values[sensor_type_visible_light].stamp = time((time_t *)0);

			break;
		}
	}

	return(true);
}

static void bh1750_dump(const data_t *data, string_t output)
{
	bh1750_private_data_t *pdata = data->private_data;

	assert(pdata);

	string_format_append(output, "state: %u, ", pdata->state);
	string_format_append(output, "scaling: %u, ", pdata->scaling);
	string_format_append(output, "scaling_up: %u, ", pdata->scaling_up);
	string_format_append(output, "scaling_down: %u, ", pdata->scaling_down);
	string_format_append(output, "overflows: %u, ", pdata->overflows);
	string_format_append(output, "raw: %u", pdata->raw_value);
};

enum
{
	tmp75_reg_temp =	0x00,
	tmp75_reg_conf =	0x01,
	tmp75_reg_tlow =	0x02,
	tmp75_reg_thigh =	0x03,

	tmp75_reg_conf_os =			0b10000000,
	tmp75_reg_conf_res_9 =		0b00000000,
	tmp75_reg_conf_res_10 =		0b00100000,
	tmp75_reg_conf_res_11 =		0b01000000,
	tmp75_reg_conf_res_12 =		0b01100000,
	tmp75_reg_conf_f_queue =	0b00011000,
	tmp75_reg_conf_pol =		0b00000100,
	tmp75_reg_conf_tm =			0b00000010,
	tmp75_reg_conf_shutdown =	0b00000001,
	tmp75_reg_conf_no_shut =	0b00000000,

	tmp75_probe_04 =		0x04,
	tmp75_probe_a1 =		0xa1,
	tmp75_probe_a2 =		0xa2,
	tmp75_probe_aa =		0xaa,
	tmp75_probe_ac =		0xac,

	tmp75_probe_tl_h =		0x4b,
	tmp75_probe_tl_l =		0x00,
	tmp75_probe_th_h =		0x50,
	tmp75_probe_th_l =		0x00,
	tmp75_probe_conf =		0b00000000,
	tmp75_probe_conf_mask =	0b10000000,
};

typedef struct
{
	unsigned int raw_value[2];
} tmp75_private_data_t;

static bool tmp75_detect(i2c_slave_t slave)
{
	uint8_t buffer[2];
	bool result = false;

	if(!i2c_send_1_receive(slave, tmp75_reg_conf, 2, buffer))
		return(false);

	if((buffer[0] & tmp75_probe_conf_mask) != tmp75_probe_conf)
		return(false);

	if(!i2c_send_1_receive(slave, tmp75_reg_tlow, 2, buffer))
		return(false);

	if((buffer[0] != tmp75_probe_tl_h) || (buffer[1] != tmp75_probe_tl_l))
		return(false);

	if(!i2c_send_1_receive(slave, tmp75_reg_thigh, 2, buffer))
		return(false);

	if((buffer[0] != tmp75_probe_th_h) || (buffer[1] != tmp75_probe_th_l))
		return(false);

	log("*** tmp75: ignore 5 lines of i2c bus errors following this");

	if(i2c_send_1(slave, tmp75_probe_04))
		goto error;

	if(i2c_send_1(slave, tmp75_probe_a1))
		goto error;

	if(i2c_send_1(slave, tmp75_probe_a2))
		goto error;

	if(i2c_send_1(slave, tmp75_probe_aa))
		goto error;

	if(i2c_send_1(slave, tmp75_probe_ac))
		goto error;

	result = true;

error:
	log("*** tmp75: end of spurious i2c bus errors");

	return(result);
}

static bool tmp75_init(data_t *data)
{
	tmp75_private_data_t *pdata = data->private_data;
	uint8_t buffer[2];

	assert(pdata);

	pdata->raw_value[0] = 0;
	pdata->raw_value[1] = 0;

	if(!i2c_send_2(data->slave, tmp75_reg_conf, tmp75_reg_conf_res_12 | tmp75_reg_conf_no_shut))
		return(false);

	if(!i2c_send_1_receive(data->slave, tmp75_reg_conf, sizeof(buffer), buffer))
		return(false);

	if(buffer[0] != (tmp75_reg_conf_res_12 | tmp75_reg_conf_no_shut))
		return(false);

	return(true);
}

static bool tmp75_poll(data_t *data)
{
	tmp75_private_data_t *pdata = data->private_data;
	uint8_t buffer[2];
	unsigned int raw_temperature;

	assert(pdata);

	if(!i2c_send_1_receive(data->slave, tmp75_reg_temp, sizeof(buffer), buffer))
	{
		log("sensor: error in poll tmp75");
		return(false);
	}

	pdata->raw_value[0] = buffer[0];
	pdata->raw_value[1] = buffer[1];
	raw_temperature = unsigned_16_be(buffer);
	data->values[sensor_type_temperature].value = raw_temperature / 256.0f;
	data->values[sensor_type_temperature].stamp = time((time_t *)0);

	return(true);
}

static void tmp75_dump(const data_t *data, string_t output)
{
	tmp75_private_data_t *pdata = data->private_data;

	assert(pdata);

	string_format_append(output, "raw value 0: %u, ", pdata->raw_value[0]);
	string_format_append(output, "raw value 1: %u", pdata->raw_value[1]);
};

enum
{
	lm75_reg_temp =				0x00,
	lm75_reg_conf =				0x01,
	lm75_reg_thyst =			0x02,
	lm75_reg_tos =				0x03,

	lm75_reg_conf_reserved =	0b11100000,
	lm75_reg_conf_f_queue =		0b00011000,
	lm75_reg_conf_pol =			0b00000100,
	lm75_reg_conf_comp_int =	0b00000010,
	lm75_reg_conf_shutdown =	0b00000001,
	lm75_reg_conf_no_shutdown =	0b00000000,

	lm75_probe_thyst_h =		0x4b,
	lm75_probe_thyst_l =		0x00,
	lm75_probe_tos_1_h =		0x50,
	lm75_probe_tos_1_l =		0x00,
	lm75_probe_tos_2_h =		0x00,
	lm75_probe_tos_2_l =		0x00,
	lm75_probe_conf =			0b00000000,
	lm75_probe_conf_mask =		0b10011111,
};

typedef struct
{
	unsigned int raw_value[2];
} lm75_private_data_t;

static bool lm75_detect(i2c_slave_t slave)
{
	uint8_t buffer[2];

	if(!i2c_send_1_receive(slave, lm75_reg_conf, sizeof(buffer), buffer))
		return(false);

	if(((buffer[0] & lm75_probe_conf_mask) != lm75_probe_conf))
		return(false);

	if(!i2c_send_1_receive(slave, lm75_reg_thyst, sizeof(buffer), buffer))
		return(false);

	if((buffer[0] != lm75_probe_thyst_h) || (buffer[1] != lm75_probe_thyst_l))
		return(false);

	if(!i2c_send_1_receive(slave, lm75_reg_tos, sizeof(buffer), buffer))
		return(false);

	if(((buffer[0] != lm75_probe_tos_1_h) || (buffer[1] != lm75_probe_tos_1_l)) && ((buffer[0] != lm75_probe_tos_2_h) || (buffer[1] != lm75_probe_tos_2_l)))
		return(false);

	return(true);
}

static bool lm75_init(data_t *data)
{
	lm75_private_data_t *pdata = data->private_data;
	uint8_t buffer[2];

	assert(pdata);

	pdata->raw_value[0] = 0;
	pdata->raw_value[1] = 0;

	if(!i2c_send_2(data->slave, lm75_reg_conf, lm75_reg_conf_no_shutdown))
		return(false);

	if(!i2c_send_1_receive(data->slave, lm75_reg_conf, sizeof(buffer), buffer))
		return(false);

	if((buffer[0] & ~lm75_reg_conf_reserved) != lm75_reg_conf_no_shutdown)
		return(false);

	return(true);
}

static bool lm75_poll(data_t *data)
{
	lm75_private_data_t *pdata = data->private_data;
	uint8_t buffer[2];
	unsigned int raw_temperature;

	assert(pdata);

	if(!i2c_send_1_receive(data->slave, lm75_reg_temp, 2, buffer))
	{
		log("lm75: poll error");
		return(false);
	}

	pdata->raw_value[0] = buffer[0];
	pdata->raw_value[1] = buffer[1];
	raw_temperature = unsigned_16_be(buffer);
	data->values[sensor_type_temperature].value = raw_temperature / 256.0f;
	data->values[sensor_type_temperature].stamp = time((time_t *)0);

	return(true);
}

static void lm75_dump(const data_t *data, string_t output)
{
	lm75_private_data_t *pdata = data->private_data;

	assert(pdata);

	string_format_append(output, "raw value 0: %u, ", pdata->raw_value[0]);
	string_format_append(output, "raw value 1: %u", pdata->raw_value[1]);
};

enum
{
	opt3001_reg_result =		0x00,
	opt3001_reg_conf =			0x01,
	opt3001_reg_limit_low =		0x02,
	opt3001_reg_limit_high =	0x03,
	opt3001_reg_id_manuf =		0x7e,
	opt3001_reg_id_dev =		0x7f,
} opt3001_register;

enum
{
	opt3001_id_manuf_ti =		0x5449,
	opt3001_id_dev_opt3001 =	0x3001,
} opt3001_id;

enum
{
	opt3001_conf_fault_count =		0b0000000000000011,
	opt3001_conf_mask_exp =			0b0000000000000100,
	opt3001_conf_pol =				0b0000000000001000,
	opt3001_conf_latch =			0b0000000000010000,
	opt3001_conf_flag_low =			0b0000000000100000,
	opt3001_conf_flag_high =		0b0000000001000000,
	opt3001_conf_flag_ready =		0b0000000010000000,
	opt3001_conf_flag_ovf =			0b0000000100000000,
	opt3001_conf_conv_mode =		0b0000011000000000,
	opt3001_conf_conv_time =		0b0000100000000000,
	opt3001_conf_range =			0b1111000000000000,

	opt3001_conf_range_auto =		0b1100000000000000,
	opt3001_conf_conv_time_100 =	0b0000000000000000,
	opt3001_conf_conv_time_800 =	0b0000100000000000,
	opt3001_conf_conv_mode_shut =	0b0000000000000000,
	opt3001_conf_conv_mode_single =	0b0000001000000000,
	opt3001_conf_conv_mode_cont =	0b0000011000000000,
};

enum
{
	opt3001_config = opt3001_conf_range_auto | opt3001_conf_conv_time_800 | opt3001_conf_conv_mode_single,
};

typedef enum
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

static bool opt3001_detect(i2c_slave_t slave)
{
	uint8_t buffer[2];

	if(!i2c_send_1_receive(slave, opt3001_reg_id_manuf, sizeof(buffer), buffer))
		return(false);

	if(unsigned_16_be(buffer) != opt3001_id_manuf_ti)
		return(false);

	if(!i2c_send_1_receive(slave, opt3001_reg_id_dev, sizeof(buffer), buffer))
		return(false);

	if(unsigned_16_be(buffer) != opt3001_id_dev_opt3001)
		return(false);

	return(true);
}

static bool opt3001_init(data_t *data)
{
	opt3001_private_data_t *pdata = data->private_data;
	uint8_t buffer[2];
	unsigned int read_config;

	assert(pdata);

	pdata->state = opt3001_state_init;
	pdata->mantissa = 0;
	pdata->exponent = 0;
	pdata->overflows = 0;

	if(!opt3001_start_measurement(data))
	{
		log("opt3001: init error 1");
		return(false);
	}

	if(!i2c_send_1_receive(data->slave, opt3001_reg_conf, sizeof(buffer), buffer))
		return(false);

	read_config = unsigned_16_be(buffer) & (opt3001_conf_mask_exp | opt3001_conf_conv_mode | opt3001_conf_conv_time | opt3001_conf_range);

	if(read_config != opt3001_config)
	{
		log("opt3001: init error 2");
		return(false);
	}

	return(true);
}

static bool opt3001_poll(data_t *data)
{
	opt3001_private_data_t *pdata = data->private_data;
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
				log("opt3001: poll error 3");
				return(false);
			}

			pdata->state = opt3001_state_measuring;

			break;
		}

		case(opt3001_state_measuring):
		{
			if(!i2c_send_1_receive(data->slave, opt3001_reg_conf, sizeof(buffer), buffer))
			{
				log("opt3001 poll: error 1");
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
				log("opt3001 poll: error 2");
				return(false);
			}

			pdata->exponent = (buffer[0] & 0xf0) >> 4;
			pdata->mantissa = ((buffer[0] & 0x0f) << 8) | buffer[1];

			data->values[sensor_type_visible_light].value = 0.01f * (float)(1 << pdata->exponent) * (float)pdata->mantissa;
			data->values[sensor_type_visible_light].stamp = time((time_t *)0);

			break;
		}
	}

	return(true);
}

static void opt3001_dump(const data_t *data, string_t output)
{
	opt3001_private_data_t *pdata = data->private_data;

	assert(pdata);

	string_format_append(output, "overflows: %u, ", pdata->overflows);
	string_format_append(output, "mantissa: %u, ", pdata->mantissa);
	string_format_append(output, "exponent: %u", pdata->exponent);
}

enum
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

static bool max44009_detect(i2c_slave_t slave)
{
	uint8_t buffer[2];

	if(!i2c_send_1_receive(slave, max44009_reg_ints, sizeof(buffer), buffer))
		return(false);

	if((buffer[0] != max44009_probe_ints) || (buffer[1] != max44009_probe_ints))
		return(false);

	if(!i2c_send_1_receive(slave, max44009_reg_inte, sizeof(buffer), buffer))
		return(false);

	if((buffer[0] != max44009_probe_inte) || (buffer[1] != max44009_probe_inte))
		return(false);

	if(!i2c_send_1_receive(slave, max44009_reg_thresh_msb, sizeof(buffer), buffer))
		return(false);

	if((buffer[0] != max44009_probe_thresh_msb) || (buffer[1] != max44009_probe_thresh_msb))
		return(false);

	if(!i2c_send_1_receive(slave, max44009_reg_thresh_lsb, sizeof(buffer), buffer))
		return(false);

	if((buffer[0] != max44009_probe_thresh_lsb) || (buffer[1] != max44009_probe_thresh_lsb))
		return(false);

	if(!i2c_send_1_receive(slave, max44009_reg_thresh_timer, sizeof(buffer), buffer))
		return(false);

	if((buffer[0] != max44009_probe_thresh_timer) || (buffer[1] != max44009_probe_thresh_timer))
		return(false);

	return(true);
}

static bool max44009_init(data_t *data)
{
	max44009_private_data_t *pdata = data->private_data;
	uint8_t buffer[2];

	assert(pdata);

	pdata->overflows = 0;
	pdata->mantissa = 0;
	pdata->exponent = 0;

	if(!i2c_send_2(data->slave, max44009_reg_conf, max44009_conf_cont))
	{
		log("sensors: max44009: init error 1");
		return(false);
	}

	if(!i2c_send_1_receive(data->slave, max44009_reg_conf, sizeof(buffer), buffer))
	{
		log("sensors: max44009: init error 2");
		return(false);
	}

	if((buffer[0] & (max44009_conf_cont | max44009_conf_manual)) != max44009_conf_cont)
	{
		log("sensors: max44009: init error 3");
		return(false);
	}

	return(true);
}

static bool max44009_poll(data_t *data)
{
	max44009_private_data_t *pdata = data->private_data;
	uint8_t buffer[2];

	assert(pdata);

	if(!i2c_send_1_receive(data->slave, max44009_reg_data_msb, sizeof(buffer), buffer))
	{
		log("sensors: max44009: poll error 1");
		return(false);
	}

	pdata->exponent =	(buffer[0] & 0xf0) >> 4;
	pdata->mantissa =	(buffer[0] & 0x0f) << 4;
	pdata->mantissa |=	(buffer[1] & 0x0f) << 0;

	if(pdata->exponent != 0b1111)
	{
		data->values[sensor_type_visible_light].value = (1 << pdata->exponent) * (float)pdata->mantissa * 0.045f;
		data->values[sensor_type_visible_light].stamp = time((time_t *)0);
	}
	else
		pdata->overflows++;

	return(true);
}

static void max44009_dump(const data_t *data, string_t output)
{
	max44009_private_data_t *pdata = data->private_data;

	assert(pdata);

	string_format_append(output, "overflows: %u, ", pdata->overflows);
	string_format_append(output, "mantissa: %u, ", pdata->mantissa);
	string_format_append(output, "exponent: %u", pdata->exponent);
};

enum
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
} asair_cmd;

enum
{
	asair_status_busy =		1 << 7,
	asair_status_ready =	1 << 3,
} asair_status;

typedef enum
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
	asair_private_data_t *pdata = data->private_data;

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
	log("asair_init: unknown device type");

	return(false);
}

static bool asair_detect(i2c_slave_t slave)
{
	uint8_t buffer[1];

	if(!i2c_send_1_receive(slave, asair_cmd_get_status, sizeof(buffer), buffer))
		return(false);

	if(!i2c_send_1(slave, asair_cmd_reset))
		return(false);

	return(true);
}

static bool asair_init(data_t *data)
{
	asair_private_data_t *pdata = data->private_data;

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
			log("asair_init: unknown device type");
			return(false);
		}

		pdata->state = asair_state_ready;
	}

	return(true);
}

static bool asair_poll(data_t *data)
{
	asair_private_data_t *pdata = data->private_data;
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
					log("asair_init: unknown device type");
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
				log("sensors: asair: poll error 1");
				return(false);
			}

			if((buffer[0] & asair_status_busy) || !(buffer[0] & asair_status_ready))
			{
				log("sensors: asair: poll error 2");
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
				log("sensors: asair: poll error 3");
				return(false);
			}

			if(buffer[0] & asair_status_busy)
			{
				log("sensors: asair: poll error 4");
				return(false);
			}

			if(!i2c_send_3(data->slave, asair_cmd_measure_0, asair_cmd_measure_1, asair_cmd_measure_2))
			{
				log("sensors: asair: poll error 5");
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
				log("sensors: asair: poll error 6");
				pdata->valid = false;
				return(false);
			}

			if(buffer[0] & asair_status_busy)
			{
				log("sensors: asair: poll error 7");
				pdata->valid = false;
				return(false);
			}

			pdata->raw_temperature = unsigned_20_bottom_be(&buffer[3]);
			pdata->raw_humidity = unsigned_20_top_be(&buffer[1]);

			data->values[sensor_type_temperature].value = ((200.f * pdata->raw_temperature) / 1048576.f) - 50.0f;
			data->values[sensor_type_temperature].stamp = time((time_t *)0);

			data->values[sensor_type_humidity].value = pdata->raw_humidity * 100.f / 1048576.f;
			data->values[sensor_type_humidity].stamp = time((time_t *)0);

			pdata->valid = true;
			pdata->state = asair_state_start_measure;

			break;
		}
	}

	return(true);
}

static void asair_dump(const data_t *data, string_t output)
{
	asair_private_data_t *pdata = data->private_data;

	assert(pdata);

	string_format_append(output, "state: %u, ", pdata->state);
	string_format_append(output, "type: %u, ", pdata->type);
	string_format_append(output, "valid: %u, ", (unsigned int)pdata->valid);
	string_format_append(output, "raw temperature: %u, ", pdata->raw_temperature);
	string_format_append(output, "raw humidity: %u", pdata->raw_temperature);
};

typedef enum
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

typedef enum
{
	tsl2561_cmd_address =	(1 << 0) | (1 << 1) | (1 << 2) | (1 << 3),
	tsl2561_cmd_block =		1 << 4,
	tsl2561_cmd_word =		1 << 5,
	tsl2561_cmd_clear =		1 << 6,
	tsl2561_cmd_cmd =		1 << 7,
} tsl2561_cmd_t;

typedef enum
{
	tsl2561_tim_integ_13ms	=	(0 << 1) | (0 << 0),
	tsl2561_tim_integ_101ms	=	(0 << 1) | (1 << 0),
	tsl2561_tim_integ_402ms	=	(1 << 1) | (0 << 0),
	tsl2561_tim_manual		=	1 << 3,
	tsl2561_tim_low_gain	=	0 << 4,
	tsl2561_tim_high_gain	=	1 << 4,
} tsl2561_timeint_t;

enum
{
	tsl2561_ctrl_power_off =	0x00,
	tsl2561_ctrl_power_on =		0x03,

	tsl2561_id_tsl2561 =		0x50,
	tsl2561_probe_threshold =	0x00,
};

enum
{
	tsl2561_autoranging_data_size = 4,
};

typedef enum
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
	{{	tsl2561_tim_integ_402ms,	tsl2561_tim_high_gain	},	{	0,		50000	},	65535,	{	0.48,	0	}},
	{{	tsl2561_tim_integ_402ms,	tsl2561_tim_low_gain	},	{	256,	50000	},	65535,	{	7.4,	0	}},
	{{	tsl2561_tim_integ_101ms,	tsl2561_tim_low_gain	},	{	256,	30000	},	37177,	{	28,		0	}},
	{{	tsl2561_tim_integ_13ms,		tsl2561_tim_low_gain	},	{	256,	65536	},	5047,	{	200,	0	}},
};

static bool tsl2561_write(i2c_slave_t slave, tsl2561_reg_t reg, unsigned int value)
{
	if(!i2c_send_2(slave, tsl2561_cmd_cmd | tsl2561_cmd_clear | (reg & tsl2561_cmd_address), value))
	{
		log("sensor: tsl2561: error 1");
		return(false);
	}

	return(true);
}

static bool tsl2561_read_byte(i2c_slave_t slave, tsl2561_reg_t reg, uint8_t *byte)
{
	if(!i2c_send_1_receive(slave, tsl2561_cmd_cmd | (reg & tsl2561_cmd_address), sizeof(*byte), byte))
	{
		log("sensor: tsl2561: error 2");
		return(false);
	}

	return(true);
}

static bool tsl2561_read_word(i2c_slave_t slave, tsl2561_reg_t reg, uint16_t *word)
{
	uint8_t buffer[2];

	if(!i2c_send_1_receive(slave, tsl2561_cmd_cmd | (reg & tsl2561_cmd_address), sizeof(buffer), buffer))
	{
		log("sensor: tsl2561: error 3");
		return(false);
	}

	*word = unsigned_16_le(buffer);

	return(true);
}

static bool tsl2561_write_check(i2c_slave_t slave, tsl2561_reg_t reg, unsigned int value)
{
	uint8_t rv;

	if(!tsl2561_write(slave, reg, value))
		return(false);

	if(!tsl2561_read_byte(slave, reg, &rv))
		return(false);

	if(value != rv)
		return(false);

	return(true);
}

static bool tsl2561_detect(i2c_slave_t slave)
{
	uint8_t regval;
	uint16_t word;

	if(!tsl2561_read_byte(slave, tsl2561_reg_id, &regval))
		return(false);

	if(regval != tsl2561_id_tsl2561)
		return(false);

	if(!tsl2561_read_word(slave, tsl2561_reg_threshlow, &word))
		return(false);

	if(word != tsl2561_probe_threshold)
		return(false);

	if(!tsl2561_read_word(slave, tsl2561_reg_threshhigh, &word))
		return(false);

	if(word != tsl2561_probe_threshold)
		return(false);

	if(!tsl2561_write_check(slave, tsl2561_reg_control, tsl2561_ctrl_power_off))
		return(false);

	if(tsl2561_write_check(slave, tsl2561_reg_id, 0x00)) // id register should not be writable
		return(false);

	return(true);
}

static bool tsl2561_init(data_t *data)
{
	tsl2561_private_data_t *pdata = data->private_data;
	uint8_t regval;

	assert(pdata);

	pdata->state = tsl2561_state_init;
	pdata->overflows = 0;
	pdata->scaling_up = 0;
	pdata->scaling_down = 0;
	pdata->scaling = tsl2561_autoranging_data_size - 1;
	pdata->channel[0] = 0;
	pdata->channel[1] = 0;

	if(!tsl2561_write_check(data->slave, tsl2561_reg_interrupt, 0x00)) // disable interrupts
		return(false);

	if(!tsl2561_write(data->slave, tsl2561_reg_control, tsl2561_ctrl_power_on))	// power up
		return(false);

	if(!tsl2561_read_byte(data->slave, tsl2561_reg_control, &regval))
		return(false);

	if((regval & 0x0f) != tsl2561_ctrl_power_on)
		return(false);

	return(true);
}

static bool tsl2561_poll(data_t *data)
{
	tsl2561_private_data_t *pdata = data->private_data;
	uint8_t buffer[2];
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
				log("sensor: tsl2561 poll error 1");
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

			if(!i2c_send_1_receive(data->slave, tsl2561_cmd_cmd | tsl2561_reg_data0, sizeof(buffer), buffer))
			{
				log("sensor: tsl2561 poll error 2");
				return(false);
			}

			pdata->channel[0] = unsigned_16_le(&buffer[0]);

			if(!i2c_send_1_receive(data->slave, tsl2561_cmd_cmd | tsl2561_reg_data1, sizeof(buffer), buffer))
			{
				log("sensor: tsl2561 poll error 3");
				return(false);
			}

			pdata->channel[1] = unsigned_16_le(&buffer[0]);

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

				value = (value * tsl2561_autoranging_data[pdata->scaling].correction.factor) + tsl2561_autoranging_data[pdata->scaling].correction.offset;

				if(value < 0)
					value = 0;
			}

			data->values[sensor_type_visible_light].value = value;
			data->values[sensor_type_visible_light].stamp = time((time_t *)0);

			break;
		}
	}

	return(true);
}

static void tsl2561_dump(const data_t *data, string_t output)
{
	tsl2561_private_data_t *pdata = data->private_data;

	assert(pdata);

	string_format_append(output, "state 0: %u, ", (unsigned int)pdata->state);
	string_format_append(output, "scaling: %u, ", pdata->scaling);
	string_format_append(output, "scaling up: %u, ", pdata->scaling_up);
	string_format_append(output, "scaling down: %u, ", pdata->scaling_down);
	string_format_append(output, "overflows: %u, ", pdata->overflows);
	string_format_append(output, "channel 0: %u, ", pdata->channel[0]);
	string_format_append(output, "channel 1: %u", pdata->channel[1]);
}

enum
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

	hdc1080_conf_rst =		0b1000000000000000,
	hdc1080_conf_reservd0 =	0b0100000000000000,
	hdc1080_conf_heat =		0b0010000000000000,
	hdc1080_conf_mode_two =	0b0001000000000000,
	hdc1080_conf_mode_one =	0b0000000000000000,
	hdc1080_conf_btst =		0b0000100000000000,
	hdc1080_conf_tres_11 =	0b0000010000000000,
	hdc1080_conf_tres_14 =	0b0000000000000000,
	hdc1080_conf_hres_8 =	0b0000001000000000,
	hdc1080_conf_hres_11 =	0b0000000100000000,
	hdc1080_conf_hres_14 =	0b0000000000000000,
	hdc1080_conf_reservd1 =	0b0000000011111111,
};

typedef enum
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

static bool hdc1080_send_16(i2c_slave_t slave, unsigned int reg, unsigned int word)
{
	uint8_t buffer[3];

	buffer[0] = reg;
	buffer[1] = (word & 0xff00) >> 8;
	buffer[2] = (word & 0x00ff) >> 0;

	return(i2c_send(slave, sizeof(buffer), buffer));
}

static bool hdc1080_detect(i2c_slave_t slave)
{
	uint8_t buffer[4];

	if(!i2c_send_1_receive(slave, hdc1080_reg_man_id, 2, buffer))
		return(false);

	if(unsigned_16_be(buffer) != hdc1080_man_id)
		return(false);

	if(!i2c_send_1_receive(slave, hdc1080_reg_dev_id, sizeof(buffer), buffer))
		return(false);

	if(unsigned_16_be(buffer) != hdc1080_dev_id)
		return(false);

	return(true);
}

static bool hdc1080_init(data_t *data)
{
	hdc1080_private_data_t *pdata = data->private_data;

	assert(pdata);

	pdata->state = hdc1080_state_init;

	if(!hdc1080_send_16(data->slave, hdc1080_reg_conf, hdc1080_conf_rst))
	{
		log("hdc1080: init failed");
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
	hdc1080_private_data_t *pdata = data->private_data;
	static const unsigned int conf =hdc1080_conf_tres_14 | hdc1080_conf_hres_14 | hdc1080_conf_mode_two;
	uint8_t buffer[4];

	assert(pdata);

	pdata->valid = false;

	switch(pdata->state)
	{
		case(hdc1080_state_init):
		{
			log("hdc1080: invalid state");
			pdata->state = hdc1080_state_reset;
			break;
		}

		case(hdc1080_state_reset):
		{
			if(!i2c_send_1_receive(data->slave, hdc1080_reg_conf, 2, buffer))
			{
				log("hdc1080: poll error 1");
				return(false);
			}

			if(unsigned_16_le(buffer) & hdc1080_conf_rst)
			{
				log("hdc1080: poll error 2");
				return(false);
			}

			if(!hdc1080_send_16(data->slave, hdc1080_reg_conf, conf))
			{
				log("hdc1080: poll error 3");
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
				log("hdc1080: poll error 4");
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
				log("hdc1080 poll error 5");
				return(false);
			}

			pdata->raw_temperature = unsigned_16_be(&buffer[0]);
			pdata->raw_humidity = unsigned_16_be(&buffer[2]);
			pdata->valid = true;

			data->values[sensor_type_temperature].value = ((pdata->raw_temperature * 165.0f) / (float)(1 << 16)) - 40.f;
			data->values[sensor_type_temperature].stamp = time((time_t *)0);
			data->values[sensor_type_humidity].value = (pdata->raw_humidity * 100.0f) / 65536.0f;
			data->values[sensor_type_humidity].stamp = time((time_t *)0);

			break;
		}
	}

	return(true);
}

static void hdc1080_dump(const data_t *data, string_t output)
{
	hdc1080_private_data_t *pdata = data->private_data;

	assert(pdata);

	string_format_append(output, "state: %u, ", pdata->state);
	string_format_append(output, "valid: %u, ", (unsigned int)pdata->valid);
	string_format_append(output, "raw temperature: %u, ", pdata->raw_temperature);
	string_format_append(output, "raw humidity: %u", pdata->raw_humidity);
}

typedef enum
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

enum
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

typedef enum
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
		log("sht3x: sht3x_send_command: error");
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
		log("sht3x: sht3x_receive_command: error");
		return(false);
	}

	crc_local = buffer[2];
	crc_remote = sht3x_crc8(2, &buffer[0]);

	if(crc_local != crc_remote)
	{
		log("sht3x: sht3x_receive_command: invalid crc");
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
		log("sht3x: sht3x_fetch_data: error");
		return(false);
	}

	crc_local = buffer[2];
	crc_remote = sht3x_crc8(2, &buffer[0]);

	if(crc_local != crc_remote)
	{
		log("sht3x: sht3x_fetch_data: invalid crc [0]");
		return(false);
	}

	crc_local = buffer[5];
	crc_remote = sht3x_crc8(2, &buffer[3]);

	if(crc_local != crc_remote)
	{
		log("sht3x: sht3x_fetch_data: invalid crc [1]");
		return(false);
	}

	*result1 = unsigned_16_be(&buffer[0]);
	*result2 = unsigned_16_be(&buffer[3]);

	return(true);
}

static bool sht3x_detect(i2c_slave_t slave)
{
	if(!sht3x_send_command(slave, sht3x_cmd_break))
	{
		log("sht3x: detect error");
		return(false);
	}

	return(true);
}

static bool sht3x_init(data_t *data)
{
	sht3x_private_data_t *pdata = data->private_data;

	assert(pdata);

	pdata->state = sht3x_state_init;
	pdata->valid = false;
	pdata->raw_temperature = 0;
	pdata->raw_humidity = 0;

	return(true);
}

static bool sht3x_poll(data_t *data)
{
	sht3x_private_data_t *pdata = data->private_data;
	unsigned int result, results[2];

	assert(pdata);

	switch(pdata->state)
	{
		case(sht3x_state_init):
		{
			if(!sht3x_send_command(data->slave, sht3x_cmd_reset))
			{
				log("sht3x: poll error 1");
				return(false);
			}

			pdata->state = sht3x_state_reset;

			break;
		}

		case(sht3x_state_reset):
		{
			if(!sht3x_receive_command(data->slave, sht3x_cmd_read_status, &result))
			{
				log("sht3x: poll error 2");
				return(false);
			}

			if((result & (sht3x_status_write_checksum | sht3x_status_command_status)) != 0x00)
			{
				log("sht3x: poll error 3");
				return(false);
			}

			if(!sht3x_send_command(data->slave, sht3x_cmd_clear_status))
			{
				log("sht3x: poll error 4");
				return(false);
			}

			pdata->state = sht3x_state_ready;

			break;
		}

		case(sht3x_state_ready):
		{
			if(!sht3x_receive_command(data->slave, sht3x_cmd_read_status, &result))
			{
				log("sht3x: poll error 5");
				return(false);
			}

			if((result & (sht3x_status_write_checksum | sht3x_status_command_status | sht3x_status_reset_detected)) != 0x00)
			{
				log("sht3x: poll error 6");
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
				log("sht3x: poll error 7");
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
				log("sht3x: poll error 8");
				return(false);
			}

			pdata->raw_temperature = results[0];
			pdata->raw_humidity = results[1];

			data->values[sensor_type_temperature].value = (((float)pdata->raw_temperature * 175.f) / ((1 << 16) - 1.0f)) - 45.0f;
			data->values[sensor_type_temperature].stamp = time((time_t *)0);
			data->values[sensor_type_humidity].value = ((float)pdata->raw_humidity * 100.0f) / ((1 << 16) - 1.0f);
			data->values[sensor_type_humidity].stamp = time((time_t *)0);

			pdata->valid = true;

			break;
		}
	}

	return(true);
}

static void sht3x_dump(const data_t *data, string_t output)
{
	sht3x_private_data_t *pdata = data->private_data;

	assert(pdata);

	string_format_append(output, "state: %u, ", pdata->state);
	string_format_append(output, "valid: %u, ", (unsigned int)pdata->valid);
	string_format_append(output, "raw temperature: %u, ", pdata->raw_temperature);
	string_format_append(output, "raw humidity: %u", pdata->raw_humidity);
}

enum
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

	bmx280_reg_ctrl_hum_osrs_h_skip =	0b00000000,
	bmx280_reg_ctrl_hum_osrs_h_1 =		0b00000001,
	bmx280_reg_ctrl_hum_osrs_h_2 =		0b00000010,
	bmx280_reg_ctrl_hum_osrs_h_4 =		0b00000011,
	bmx280_reg_ctrl_hum_osrs_h_8 =		0b00000100,
	bmx280_reg_ctrl_hum_osrs_h_16 =		0b00000101,

	bmx280_reg_status_measuring =		0b00001000,
	bmx280_reg_status_im_update =		0b00000001,

	bmx280_reg_ctrl_meas_osrs_t_skip =	0b00000000,
	bmx280_reg_ctrl_meas_osrs_t_1 =		0b00100000,
	bmx280_reg_ctrl_meas_osrs_t_2 =		0b01000000,
	bmx280_reg_ctrl_meas_osrs_t_4 =		0b01100000,
	bmx280_reg_ctrl_meas_osrs_t_8 =		0b10000000,
	bmx280_reg_ctrl_meas_osrs_t_16 =	0b10100000,
	bmx280_reg_ctrl_meas_osrs_p_skip =	0b00000000,
	bmx280_reg_ctrl_meas_osrs_p_1 =		0b00000100,
	bmx280_reg_ctrl_meas_osrs_p_2 =		0b00001000,
	bmx280_reg_ctrl_meas_osrs_p_4 =		0b00001100,
	bmx280_reg_ctrl_meas_osrs_p_8 =		0b00010000,
	bmx280_reg_ctrl_meas_osrs_p_16 =	0b00010100,
	bmx280_reg_ctrl_meas_mode_mask =	0b00000011,
	bmx280_reg_ctrl_meas_mode_sleep =	0b00000000,
	bmx280_reg_ctrl_meas_mode_forced =	0b00000010,
	bmx280_reg_ctrl_meas_mode_normal =	0b00000011,

	bmx280_reg_config_t_sb_05 =			0b00000000,
	bmx280_reg_config_t_sb_62 =			0b00100000,
	bmx280_reg_config_t_sb_125 =		0b01000000,
	bmx280_reg_config_t_sb_250 =		0b01100000,
	bmx280_reg_config_t_sb_500 =		0b10000000,
	bmx280_reg_config_t_sb_1000 =		0b10100000,
	bmx280_reg_config_t_sb_10000 =		0b11000000,
	bmx280_reg_config_t_sb_20000 =		0b11100000,
	bmx280_reg_config_filter_off =		0b00000000,
	bmx280_reg_config_filter_2 =		0b00000100,
	bmx280_reg_config_filter_4 =		0b00001000,
	bmx280_reg_config_filter_8 =		0b00001100,
	bmx280_reg_config_filter_16 =		0b00010000,
	bmx280_reg_config_spi3w_en =		0b00000001,
};

enum
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

typedef enum
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
	bmx280_private_data_t *pdata = data->private_data;
	uint8_t cal_data[bmx280_cal_size];
	uint8_t buffer[1];
	unsigned int e4, e5, e6;

	assert(pdata);

	if(!i2c_send_1_receive(data->slave, bmx280_reg_id, sizeof(buffer), buffer))
	{
		log("bmx280: error read otp data 1");
		return(false);
	}

	pdata->type = buffer[0];

	if(!i2c_send_1_receive(data->slave, bmx280_cal_base, sizeof(cal_data), cal_data))
	{
		log("bmx280: error read otp data 2");
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

static bool bmx280_detect(i2c_slave_t slave)
{
	uint8_t	buffer[1];

	if(!i2c_send_1_receive(slave, bmx280_reg_id, sizeof(buffer), buffer))
		return(false);

	if((buffer[0] != bmx280_reg_id_bmp280) && (buffer[0] != bmx280_reg_id_bme280))
		return(false);

	return(true);
}

static bool bmx280_init(data_t *data)
{
	bmx280_private_data_t *pdata = data->private_data;
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
		log("bmx280: init error 1");
		return(false);
	}

	if(!i2c_send_1_receive(data->slave, bmx280_reg_reset, sizeof(buffer), buffer))
	{
		log("bmx280: init error 2");
		return(false);
	}

	if(buffer[0] != 0x00)
	{
		log("bmx280: init error 3");
		return(false);
	}

	pdata->state = bmx280_state_reset;

	return(true);
}

static bool bmx280_poll(data_t *data)
{
	bmx280_private_data_t *pdata = data->private_data;
	uint8_t buffer[8];
	float var1, var2, airpressure, humidity;

	assert(pdata);

	switch(pdata->state)
	{
		case(bmx280_state_init):
		{
			log("bmx280: poll: invalid state");
			pdata->state = bmx280_state_reset;

			return(false);
		}

		case(bmx280_state_reset):
		{
			if(!bmx280_read_otp(data))
			{
				log("bmx280_init: cannot read OTP data");
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
				log("bmx280: poll error 1");
				return(false);
			}

			if((buffer[0] & bmx280_reg_ctrl_meas_mode_mask) != bmx280_reg_ctrl_meas_mode_sleep)
			{
				log("bmx280: poll error 2");
				return(false);
			}

			if(!i2c_send_2(data->slave, bmx280_reg_ctrl_hum, bmx280_reg_ctrl_hum_osrs_h_16))
			{
				log("bmx280: poll error 3");
				return(false);
			}

			if(!i2c_send_2(data->slave, bmx280_reg_config, bmx280_reg_config_filter_2))
			{
				log("bmx280: poll error 4");
				return(false);
			}

			if(!i2c_send_2(data->slave, bmx280_reg_ctrl_meas, bmx280_reg_ctrl_meas_osrs_t_16 | bmx280_reg_ctrl_meas_osrs_p_16 | bmx280_reg_ctrl_meas_mode_forced))
			{
				log("bmx280: poll error 5");
				return(false);
			}

			pdata->state = bmx280_state_measuring;

			break;
		}

		case(bmx280_state_measuring):
		{
			if(!i2c_send_1_receive(data->slave, bmx280_reg_adc, sizeof(buffer), buffer))
			{
				log("bmx280: poll error 6");
				return(false);
			}

			pdata->adc_airpressure = unsigned_20_top_be(&buffer[0]);
			pdata->adc_temperature = unsigned_20_top_be(&buffer[3]);
			pdata->adc_humidity = unsigned_16_be(&buffer[6]);

			var1 = ((pdata->adc_temperature / 16384.0f) - (pdata->t1 / 1024.0f)) * pdata->t2;
			var2 = ((pdata->adc_temperature / 131072.0f) - (pdata->t1 / 8192.0f)) * ((pdata->adc_temperature / 131072.0f) - (pdata->t1 / 8192.0f)) * pdata->t3;

			data->values[sensor_type_temperature].value = (var1 + var2) / 5120.0f;
			data->values[sensor_type_temperature].stamp = time((time_t *)0);

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
			data->values[sensor_type_airpressure].stamp = time((time_t *)0);

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
				data->values[sensor_type_humidity].stamp = time((time_t *)0);
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

static void bmx280_dump(const data_t *data, string_t output)
{
	bmx280_private_data_t *pdata = data->private_data;

	assert(pdata);

	string_format_append(output, "type: %02x, ", pdata->type);
	string_format_append(output, "state: %u, ", pdata->state);
	string_format_append(output, "adc temp: %u, ", pdata->adc_temperature);
	string_format_append(output, "adc pressure: %u, ", pdata->adc_airpressure);
	string_format_append(output, "adc humidity: %u, ", pdata->adc_humidity);
	string_format_append(output, "t_fine: %f, ", (double)pdata->t_fine);
	string_format_append(output, "t_fine_2: %f, ", (double)pdata->t_fine_2);
	string_format_append(output, "t1: %u, ", pdata->t1);
	string_format_append(output, "t2: %d, ", pdata->t2);
	string_format_append(output, "t3: %d, ", pdata->t3);
	string_format_append(output, "p1: %u, ", pdata->p1);
	string_format_append(output, "p2: %d, ", pdata->p2);
	string_format_append(output, "p3: %d, ", pdata->p3);
	string_format_append(output, "p4: %d, ", pdata->p4);
	string_format_append(output, "p5: %d, ", pdata->p5);
	string_format_append(output, "p6: %d, ", pdata->p6);
	string_format_append(output, "p7: %d, ", pdata->p7);
	string_format_append(output, "p8: %d, ", pdata->p8);
	string_format_append(output, "p9: %d, ", pdata->p9);
	string_format_append(output, "h1: %u, ", pdata->h1);
	string_format_append(output, "h2: %d, ", pdata->h2);
	string_format_append(output, "h3: %u, ", pdata->h3);
	string_format_append(output, "h4: %d, ", pdata->h4);
	string_format_append(output, "h5: %d, ", pdata->h5);
	string_format_append(output, "h6: %u", pdata->h6);
};

enum
{
	htu21_cmd_meas_temp_hold_master =		0xe3,
	htu21_cmd_meas_hum_hold_master =		0xe5,
	htu21_cmd_write_user =					0xe6,
	htu21_cmd_read_user =					0xe7,
	htu21_cmd_meas_temp_no_hold_master =	0xf3,
	htu21_cmd_meas_hum_no_hold_master =		0xf5,
	htu21_cmd_reset =						0xfe,

	htu21_user_reg_rh12_temp14 =			0b00000000,
	htu21_user_reg_rh8_temp12 =				0b00000001,
	htu21_user_reg_rh10_temp13 =			0b10000000,
	htu21_user_reg_rh11_temp11 =			0b10000001,
	htu21_user_reg_bat_stat =				0b01000000,
	htu21_user_reg_reserved =				0b00111000,
	htu21_user_reg_heater_enable =			0b00000100,
	htu21_user_reg_otp_reload_disable =		0b00000010,

	htu21_status_mask =						0b00000011,
	htu21_status_measure_temperature =		0b00000000,
	htu21_status_measure_humidity =			0b00000010,

	htu21_delay_reset =						2,
};

typedef enum
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
		log("htu21_get_data: error\n");
		return(false);
	}

	crc1 = buffer[2];
	crc2 = htu21_crc8(2, &buffer[0]);

	if(crc1 != crc2)
	{
		log("htu21_get_data: crc invalid\n");
		return(false);
	}

	*result = unsigned_16_be(buffer) & ~htu21_status_mask;

	return(true);
}

static bool htu21_detect(i2c_slave_t slave)
{
	uint8_t buffer[1];

	if(!i2c_send_1_receive(slave, htu21_cmd_read_user, sizeof(buffer), buffer))
		return(false);

	return(true);
}

static bool htu21_init(data_t *data)
{
	htu21_private_data_t *pdata = data->private_data;

	assert(pdata);

	pdata->state = htu21_state_init;

	pdata->raw_temperature = 0;
	pdata->raw_humidity = 0;

	return(true);
}

static bool htu21_poll(data_t *data)
{
	htu21_private_data_t *pdata = data->private_data;
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
				log("htu21: poll: error 1");
				break;
			}

			cmd[0] = htu21_cmd_write_user;
			cmd[1] &= (htu21_user_reg_reserved | htu21_user_reg_bat_stat);
			cmd[1] |= htu21_user_reg_rh11_temp11 | htu21_user_reg_otp_reload_disable;

			if(!i2c_send(data->slave, sizeof(cmd), cmd))
			{
				log("htu21: poll: error 2");
				break;
			}

			if(!i2c_send_1_receive(data->slave, htu21_cmd_read_user, sizeof(buffer), buffer))
			{
				log("htu21: poll: error 3");
				break;
			}

			buffer[0] &= ~(htu21_user_reg_reserved | htu21_user_reg_bat_stat);

			if(buffer[0] != (htu21_user_reg_rh11_temp11 | htu21_user_reg_otp_reload_disable))
			{
				log("htu21: poll: error 4");
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
				log("htu21 poll: error 5");
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
				log("htu21: poll: error 6");
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
			data->values[sensor_type_temperature].stamp = time((time_t *)0);
			data->values[sensor_type_humidity].value = humidity;
			data->values[sensor_type_humidity].stamp = data->values[sensor_type_temperature].stamp;

			pdata->state = htu21_state_finished;

			break;
		}
	}

	return(true);
}

static void htu21_dump(const data_t *data, string_t output)
{
	htu21_private_data_t *pdata = data->private_data;

	assert(pdata);

	string_format_append(output, "state: %u, ", pdata->state);
	string_format_append(output, "raw temperature: %u, ", pdata->raw_temperature);
	string_format_append(output, "raw humidity: %u", pdata->raw_humidity);
}

enum
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

enum
{
	veml7700_reg_id_id_1 =	0x81,
	veml7700_reg_id_id_2 =	0xc4,
};

enum
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

enum { veml7700_autoranging_data_size = 6 };

static const device_autoranging_data_t veml7700_autoranging_data[veml7700_autoranging_data_size] =
{
	{{	veml7700_conf_als_it_800,	veml7700_conf_als_gain_2 },		{ 0,	32768	}, 0, { 0.0036,	0 }},
	{{	veml7700_conf_als_it_800,	veml7700_conf_als_gain_1_8 },	{ 100,	32768	}, 0, { 0.0576,	0 }},
	{{	veml7700_conf_als_it_200,	veml7700_conf_als_gain_2 },		{ 100,	32768	}, 0, { 0.0144,	0 }},
	{{	veml7700_conf_als_it_200,	veml7700_conf_als_gain_1_8 },	{ 100,	32768	}, 0, { 0.2304,	0 }},
	{{	veml7700_conf_als_it_25,	veml7700_conf_als_gain_2 },		{ 100,	32768	}, 0, { 0.1152,	0 }},
	{{	veml7700_conf_als_it_25,	veml7700_conf_als_gain_1_8 },	{ 100,	65536	}, 0, { 1.8432, 0 }},
};

typedef enum
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

static bool veml7700_detect(i2c_slave_t slave)
{
	uint8_t buffer[2];

	if(!i2c_send_1_receive(slave, veml7700_reg_id, sizeof(buffer), buffer))
		return(false);

	if((buffer[0] != veml7700_reg_id_id_1) || (buffer[1] != veml7700_reg_id_id_2))
		return(false);

	return(true);
}

static bool veml7700_init(data_t *data)
{
	veml7700_private_data_t *pdata = data->private_data;

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
	veml7700_private_data_t *pdata = data->private_data;
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
				log("veml7700: poll: error 1");
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
				log("veml7700: poll: error 2");
				break;
			}

			pdata->raw_white = unsigned_16_le(buffer);

			if(!i2c_send_1_receive(data->slave, veml7700_reg_als, sizeof(buffer), buffer))
			{
				log("veml7700: poll: error 3");
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

			raw_lux = ((pdata->raw_als * veml7700_autoranging_data[pdata->scaling].correction.factor) + veml7700_autoranging_data[pdata->scaling].correction.offset);

			data->values[sensor_type_visible_light].value =
					(raw_lux * raw_lux * raw_lux * raw_lux * 6.0135e-13f)
					- (raw_lux * raw_lux * raw_lux * 9.3924e-09f)
					+ (raw_lux * raw_lux * 8.1488e-05f)
					+ (raw_lux * 1.0023e+00f);
			data->values[sensor_type_visible_light].stamp = time((time_t *)0);

			break;
		}
	}

	return(true);
}

static void veml7700_dump(const data_t *data, string_t output)
{
	veml7700_private_data_t *pdata = data->private_data;

	assert(pdata);

	string_format_append(output, "state: %u, ", pdata->state);
	string_format_append(output, "scaling: %u, ", pdata->scaling);
	string_format_append(output, "scaling up: %u, ", pdata->scaling_up);
	string_format_append(output, "scaling down: %u, ", pdata->scaling_down);
	string_format_append(output, "raw als: %u, ", pdata->raw_als);
	string_format_append(output, "raw als: %u, ", pdata->raw_white);
}

static const info_t info[sensor_size] =
{
	[sensor_bh1750] =
	{
		.name = "bh1750",
		.id = sensor_bh1750,
		.address = 0x23,
		.type = (1 << sensor_type_visible_light),
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
		.type = (1 << sensor_type_temperature),
		.precision = 1,
		.private_data_size = sizeof(tmp75_private_data_t),
		.detect_fn = tmp75_detect,
		.init_fn = tmp75_init,
		.poll_fn = tmp75_poll,
		.dump_fn = tmp75_dump,
	},
	[sensor_lm75] =
	{
		.name = "lm75",
		.id = sensor_lm75,
		.address = 0x48,
		.type = (1 << sensor_type_temperature),
		.precision = 1,
		.private_data_size = sizeof(lm75_private_data_t),
		.detect_fn = lm75_detect,
		.init_fn = lm75_init,
		.poll_fn = lm75_poll,
		.dump_fn = lm75_dump,
	},
	[sensor_opt3001] =
	{
		.name = "opt3001",
		.id = sensor_opt3001,
		.address = 0x45,
		.type = (1 << sensor_type_visible_light),
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
		.type = (1 << sensor_type_visible_light),
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
		.type = (1 << sensor_type_temperature) | (1 << sensor_type_humidity),
		.precision = 1,
		.private_data_size = sizeof(asair_private_data_t),
		.detect_fn = asair_detect,
		.init_fn = asair_init,
		.poll_fn = asair_poll,
		.dump_fn = asair_dump,
	},
	[sensor_tsl2561] =
	{
		.name = "tsl2561",
		.id = sensor_tsl2561,
		.address = 0x39,
		.type = (1 << sensor_type_visible_light),
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
		.type = (1 << sensor_type_temperature) | (1 << sensor_type_humidity),
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
		.type = (1 << sensor_type_temperature) | (1 << sensor_type_humidity),
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
		.type = (1 << sensor_type_temperature) | (1 << sensor_type_humidity) | (1 << sensor_type_airpressure),
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
		.type = (1 << sensor_type_temperature) | (1 << sensor_type_humidity),
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
		.type = (1 << sensor_type_visible_light),
		.precision = 2,
		.private_data_size = sizeof(veml7700_private_data_t),
		.detect_fn = veml7700_detect,
		.init_fn = veml7700_init,
		.poll_fn = veml7700_poll,
		.dump_fn = veml7700_dump,
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
	const run_parameters_t *run = (const run_parameters_t *)parameters;

	module = run->module;

	buses = i2c_buses(module);

	for(bus = i2c_bus_first; bus < buses; bus++)
	{
		for(sensor = sensor_first; sensor < sensor_size; sensor++)
		{
			infoptr = &info[sensor];

			if(i2c_find_slave(module, bus, infoptr->address))
			{
				stat_sensors_skipped[module]++;
				continue;
			}

			stat_sensors_probed[module]++;

			if(!i2c_probe_slave(module, bus, infoptr->address))
				continue;

			stat_sensors_found[module]++;

			if(!(slave = i2c_register_slave(infoptr->name, module, bus, infoptr->address)))
			{
				log_format("sensor: warning: cannot register sensor %s", infoptr->name);
				continue;
			}

			assert(infoptr->detect_fn);

			if(!infoptr->detect_fn(slave))
			{
				i2c_unregister_slave(&slave);
				continue;
			}

			new_data = (data_t *)util_memory_alloc_spiram(sizeof(*new_data));
			assert(new_data);

			for(type = sensor_type_first; type < sensor_type_size; type++)
			{
				new_data->values[type].value = 0;
				new_data->values[type].stamp = (time_t)0;
			}

			new_data->slave = slave;
			new_data->info = infoptr;

			new_data->private_data = (void *)0;
			new_data->next = (data_t *)0;

			assert(infoptr->init_fn);

			if(infoptr->private_data_size > 0)
				new_data->private_data = (void *)util_memory_alloc_spiram(infoptr->private_data_size);

			if(!infoptr->init_fn(new_data))
			{
				log_format("sensor: warning: failed to init sensor %s on bus %u", infoptr->name, bus);
				i2c_unregister_slave(&slave);
				if(new_data->private_data)
					free(new_data->private_data);
				free(new_data);
				continue;
			}

			stat_sensors_confirmed[module]++;

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
		vTaskDelete(0);

	for(;;)
	{
		stat_poll_run[module]++;

		for(dataptr = data_root; dataptr; dataptr = dataptr->next)
		{
			if(dataptr->info->poll_fn)
				if(dataptr->info->poll_fn(dataptr))
					stat_poll_ok[module]++;
				else
					stat_poll_error[module]++;
			else
				log_format("sensor: error: no poll function for sensor %s", dataptr->info->name);
		}

		util_sleep(1000);
	}

	util_abort("sensor: poll task returned");
}

void sensor_init(void)
{
	static run_parameters_t run[2] =
	{
		[i2c_module_0_fast] =
		{
			.module = i2c_module_0_fast,
		},
		[i2c_module_1_slow] =
		{
			.module = i2c_module_1_slow,
		},
	};

	assert(!inited);

	data_mutex = xSemaphoreCreateMutex();
	assert(data_mutex);

	inited = true;

	if(xTaskCreatePinnedToCore(run_sensors, "sensors 1", 3 * 1024, &run[i2c_module_0_fast], 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("sensor: xTaskCreatePinnedToNode sensors thread 0");

	util_sleep(100);

	if(xTaskCreatePinnedToCore(run_sensors, "sensors 2", 3 * 1024, &run[i2c_module_1_slow], 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("sensor: xTaskCreatePinnedToNode sensors thread 1");
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

	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "SENSOR info");

	if(!inited)
	{
		string_append_cstr(call->result, "\n--");
		return;
	}

	data_mutex_take();

	for(dataptr = data_root; dataptr; dataptr = dataptr->next)
	{
		slave = dataptr->slave;

		if(!slave)
		{
			string_append_cstr(call->result, "\nslave = NULL");
			continue;
		}

		if(!i2c_get_slave_info(slave, &module, &bus, &address, &name))
			string_append_cstr(call->result, "\n- unknown slave");
		else
		{
			string_format_append(call->result, "\n- %s@%u/%u/%x:", name, module, bus, address);

			for(type = sensor_type_first; type < sensor_type_size; type++)
				if(dataptr->info->type & (1 << type))
					string_format_append(call->result, " %s: %.*f %s", sensor_type_info[type].type,
							(int)dataptr->info->precision, (double)dataptr->values[type].value, sensor_type_info[type].unity);
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
	string_auto(time_string, 64);
	bool first_sensor;
	bool first_value;

	assert(call->parameter_count == 0);

	if(!inited)
		return;

	string_assign_cstr(call->result, "{");

	data_mutex_take();

	for(dataptr = data_root, first_sensor = true; dataptr; dataptr = dataptr->next)
	{
		slave = dataptr->slave;

		if(slave && i2c_get_slave_info(slave, &module, &bus, &address, &name))
		{
			string_append_cstr(call->result, first_sensor ? "" : ",");
			string_format_append(call->result,	"\n  \"%u-%u-%x\":", (unsigned int)module, (unsigned int)bus, address);
			string_append_cstr(call->result,	"\n  [");
			string_append_cstr(call->result,	"\n    {");
			string_format_append(call->result,	"\n      \"module\": %u,", (unsigned int)module);
			string_format_append(call->result,	"\n      \"bus\": %u,", (unsigned int)bus);
			string_format_append(call->result,	"\n      \"name\": \"%s\",", name);
			string_append_cstr(call->result,	"\n      \"values\":");
			string_append_cstr(call->result,	"\n      [");

			for(type = sensor_type_first, first_value = true; type < sensor_type_size; type++)
			{
				if(dataptr->info->type & (1 << type))
				{
					string_append_cstr(call->result, first_value ? "" : ",");
					string_append_cstr(call->result,	"\n        {");
					string_format_append(call->result,	"\n          \"type\": \"%s\",", sensor_type_info[type].type);
					string_format_append(call->result,	"\n          \"id\": %u,", dataptr->info->id);
					string_format_append(call->result,	"\n          \"address\": %u,", address);
					string_format_append(call->result,	"\n          \"unity\": \"%s\",", sensor_type_info[type].unity);
					string_format_append(call->result,	"\n          \"value\": %f,", (double)dataptr->values[type].value);
					string_format_append(call->result,	"\n          \"time\": %lld", dataptr->values[type].stamp);
					string_append_cstr(call->result,	"\n        }");

					first_value = false;
				}
			}

			string_append_cstr(call->result, "\n      ]");
			string_append_cstr(call->result, "\n    }");
			string_append_cstr(call->result, "\n  ]");

			first_sensor = false;
		}
	}

	string_append_cstr(call->result, "\n}");

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
	string_auto(time_string, 64);

	assert(call->parameter_count < 2);

	string_assign_cstr(call->result, "SENSOR dump");

	if(!inited)
	{
		string_append_cstr(call->result, "\n--");
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
			string_append_cstr(call->result, "\nslave = NULL");
			continue;
		}

		if(!i2c_get_slave_info(slave, &module, &bus, &address, &name))
			string_append_cstr(call->result, "\n- unknown slave");
		else
		{
			string_format_append(call->result, "\n- sensor %s at module %u, bus %u, address 0x%x", name, (unsigned int)module, (unsigned int)bus, address);
			string_append_cstr(call->result, "\n  values:");

			for(type = sensor_type_first; type < sensor_type_size; type++)
			{
				if(dataptr->info->type & (1 << type))
				{
					util_time_to_string(time_string, &dataptr->values[type].stamp);
					string_format_append(call->result, " %s=%.*f [%s]", sensor_type_info[type].type, (int)dataptr->info->precision, (double)dataptr->values[type].value,
						string_cstr(time_string));
				}
			}

			if(dataptr->info->dump_fn)
			{
				string_append_cstr(call->result, "\n  private data: ");
				dataptr->info->dump_fn(dataptr, call->result);
			}
		}
	}

	data_mutex_give();
}

void command_sensor_stats(cli_command_call_t *call)
{
	i2c_module_t module;

	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "SENSOR statistics");

	for(module = i2c_module_first; module < i2c_module_size; module++)
	{
		string_format_append(call->result, "\n- module %u", (unsigned int)module);
		string_format_append(call->result, "\n-  sensors skipped: %u", stat_sensors_skipped[module]);
		string_format_append(call->result, "\n-  sensors probed: %u", stat_sensors_probed[module]);
		string_format_append(call->result, "\n-  sensors found: %u", stat_sensors_found[module]);
		string_format_append(call->result, "\n-  sensors confirmed: %u", stat_sensors_confirmed[module]);
		string_format_append(call->result, "\n-  complete poll runs: %u", stat_poll_run[module]);
		string_format_append(call->result, "\n-  sensor poll succeeded: %u", stat_poll_ok[module]);
		string_format_append(call->result, "\n-  sensor poll failed: %u", stat_poll_error[module]);
	}
}
