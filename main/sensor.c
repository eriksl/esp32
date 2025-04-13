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

enum
{
	data_int_value_size = 5,
	data_float_value_size = 2,
};

typedef struct
{
	float value;
	time_t stamp;
} sensor_value_t;

typedef struct data_T
{
	i2c_slave_t slave;
	int int_value[data_int_value_size];
	float float_value[data_float_value_size];
	sensor_value_t values[sensor_type_size];
	const struct info_T *info;
	struct data_T *next;
} data_t;

typedef struct info_T
{
	const char *name;
	sensor_t id;
	unsigned int address;
	sensor_type_t type; // bitmask!
	unsigned int precision;
	bool (*const detect_fn)(data_t *);
	bool (*const init_fn)(data_t *);
	bool (*const poll_fn)(data_t *);
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

static unsigned int unsigned_16(unsigned int msb, unsigned int lsb)
{
	return(((msb & 0xff) << 8) | (lsb & 0xff));
}

typedef enum
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
} bh1750_opcode_t;

enum
{
	bh1750_start_measurement,
	bh1750_reading_measurement,
};

enum
{
	bh1750_int_raw_value = 0,
	bh1750_int_scaling = 1,
	bh1750_int_state = 2,
	bh1750_int_size,
	bh1750_autoranging_data_size = 4
};

_Static_assert((unsigned int)bh1750_int_size <= (unsigned int)data_int_value_size);

static const device_autoranging_data_t bh1750_autoranging_data[bh1750_autoranging_data_size] =
{
	{{	bh1750_opcode_one_hmode2,	254	},	{ 0,	50000 }, 0, { 0.13,	0,	}},
	{{	bh1750_opcode_one_hmode2,	69	},	{ 1000, 50000 }, 0, { 0.50,	0,	}},
	{{	bh1750_opcode_one_hmode2,	31	},	{ 1000, 50000 }, 0, { 1.10,	0,	}},
	{{	bh1750_opcode_one_lmode,	31	},	{ 1000, 65536 }, 0, { 2.40,	0,	}},
};

static bool bh1750_start_measuring(data_t *data)
{
	unsigned int timing;
	unsigned int opcode;

	opcode = bh1750_autoranging_data[data->int_value[bh1750_int_scaling]].data[0];
	timing = bh1750_autoranging_data[data->int_value[bh1750_int_scaling]].data[1];

	if(!i2c_send_1(data->slave, bh1750_opcode_change_meas_hi | ((timing >> 5) & 0b00000111)))
	{
		log("bh1750: warning: error sending change meas hi");
		return(false);
	}

	if(!i2c_send_1(data->slave, bh1750_opcode_change_meas_lo | ((timing >> 0) & 0b00011111)))
	{
		log("bh1750: warning: error sending change meas lo");
		return(false);
	}

	if(!i2c_send_1(data->slave, opcode))
	{
		log("bh1750: warning: error sending opcode");
		return(false);
	}

	return(true);
}

static bool bh1750_detect(data_t *data)
{
	uint8_t buffer[8];

	if(!i2c_receive(data->slave, sizeof(buffer), buffer))
		return(false);

	if((buffer[2] != 0xff) || (buffer[3] != 0xff) || (buffer[4] != 0xff) ||
			(buffer[5] != 0xff) || (buffer[6] != 0xff) || (buffer[7] != 0xff))
		return(false);

	return(true);
}

static bool bh1750_init(data_t *data)
{
	if(!i2c_send_1(data->slave, bh1750_opcode_poweron))
		return(false);

	if(!i2c_send_1(data->slave, bh1750_opcode_reset))
		return(false);

	data->int_value[bh1750_int_raw_value] = 0;
	data->int_value[bh1750_int_scaling] = 0;
	data->int_value[bh1750_int_state] = bh1750_start_measurement;
	data->values[sensor_type_visible_light].value = 0;
	data->values[sensor_type_visible_light].stamp = (time_t)0;

	return(true);
}

static bool bh1750_poll(data_t *data)
{
	float factor, offset;
	unsigned int scale_down_threshold, scale_up_threshold;
	int raw_value;
	int *scaling;
	uint8_t buffer[2];

	switch(data->int_value[bh1750_int_state])
	{
		case(bh1750_start_measurement):
		{
			if(!bh1750_start_measuring(data))
				return(false);

			data->int_value[bh1750_int_state] = bh1750_reading_measurement;

			break;
		}

		case(bh1750_reading_measurement):
		{
			scaling = &data->int_value[bh1750_int_scaling];

			scale_down_threshold =	bh1750_autoranging_data[*scaling].threshold.down;
			scale_up_threshold =	bh1750_autoranging_data[*scaling].threshold.up;
			factor =				bh1750_autoranging_data[*scaling].correction.factor;
			offset =				bh1750_autoranging_data[*scaling].correction.offset;

			data->int_value[bh1750_int_state] = bh1750_start_measurement;

			if(!i2c_receive(data->slave, 2, buffer))
			{
				log("bh1750: warning: error in receive data");
				return(false);
			}

			raw_value = (buffer[0] << 8) | (buffer[1] << 0);

			if((raw_value < scale_down_threshold) && (*scaling > 0))
				(*scaling)--;

			if((raw_value >= scale_up_threshold) && ((*scaling + 1) < bh1750_autoranging_data_size))
				(*scaling)++;

			if(raw_value < 0xffff)
			{
				data->int_value[bh1750_int_raw_value] = raw_value;
				data->values[sensor_type_visible_light].value = ((float)raw_value * factor) + offset;
				data->values[sensor_type_visible_light].stamp = time((time_t *)0);
			}
			else
				log_format("bh1750: warning: measurement out of range: %d", raw_value);

			break;
		}

		default:
		{
			log_format("bh1750: poll: invalid state: %d", data->int_value[bh1750_int_state]);
			return(false);
		}
	}

	return(true);
}

enum
{
	tmp75_int_raw_value_0 = 0,
	tmp75_int_raw_value_1,
	tmp75_int_size,
};

_Static_assert((unsigned int)tmp75_int_size <= (unsigned int)data_int_value_size);

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

static bool tmp75_detect(data_t *data)
{
	uint8_t buffer[2];

	if(!i2c_send_1_receive(data->slave, tmp75_reg_conf, 2, buffer))
		return(false);

	if((buffer[0] & tmp75_probe_conf_mask) != tmp75_probe_conf)
		return(false);

	if(!i2c_send_1_receive(data->slave, tmp75_reg_tlow, 2, buffer))
		return(false);

	if((buffer[0] != tmp75_probe_tl_h) || (buffer[1] != tmp75_probe_tl_l))
		return(false);

	if(!i2c_send_1_receive(data->slave, tmp75_reg_thigh, 2, buffer))
		return(false);

	if((buffer[0] != tmp75_probe_th_h) || (buffer[1] != tmp75_probe_th_l))
		return(false);

	if(i2c_send_1(data->slave, tmp75_probe_04))
		return(false);

	if(i2c_send_1(data->slave, tmp75_probe_a1))
		return(false);

	if(i2c_send_1(data->slave, tmp75_probe_a2))
		return(false);

	if(i2c_send_1(data->slave, tmp75_probe_aa))
		return(false);

	if(i2c_send_1(data->slave, tmp75_probe_ac))
		return(false);

	return(true);
}

static bool tmp75_init(data_t *data)
{
	uint8_t buffer[2];

	if(!i2c_send_2(data->slave, tmp75_reg_conf, tmp75_reg_conf_res_12 | tmp75_reg_conf_no_shut))
		return(false);

	if(!i2c_send_1_receive(data->slave, tmp75_reg_conf, 2, buffer))
		return(false);

	if(buffer[0] != (tmp75_reg_conf_res_12 | tmp75_reg_conf_no_shut))
		return(false);

	return(true);
}

static bool tmp75_poll(data_t *data)
{
	uint8_t buffer[2];
	float raw_temperature;

	if(!i2c_send_1_receive(data->slave, tmp75_reg_temp, 2, buffer))
	{
		log("sensor: error in poll tmp75");
		return(false);
	}

	data->int_value[tmp75_int_raw_value_0] = buffer[0];
	data->int_value[tmp75_int_raw_value_1] = buffer[1];
	raw_temperature = (buffer[0] << 8) | buffer[1];
	data->values[sensor_type_temperature].value = raw_temperature / 256.0F;
	data->values[sensor_type_temperature].stamp = time((time_t *)0);

	return(true);
}

enum
{
	lm75_int_raw_value_0 = 0,
	lm75_int_raw_value_1,
	lm75_int_size,
};

_Static_assert((unsigned int)lm75_int_size <= (unsigned int)data_int_value_size);

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

static bool lm75_detect(data_t *data)
{
	uint8_t buffer[2];
	i2c_module_t module;
	i2c_bus_t bus;
	unsigned int address;
	const char *name;

	i2c_get_slave_info(data->slave, &module, &bus, &address, &name);

	if(!i2c_send_1_receive(data->slave, lm75_reg_conf, 2, buffer))
		return(false);

	if(((buffer[0] & lm75_probe_conf_mask) != lm75_probe_conf))
		return(false);

	if(!i2c_send_1_receive(data->slave, lm75_reg_thyst, 2, buffer))
		return(false);

	if((buffer[0] != lm75_probe_thyst_h) || (buffer[1] != lm75_probe_thyst_l))
		return(false);

	if(!i2c_send_1_receive(data->slave, lm75_reg_tos, 2, buffer))
		return(false);

	if(((buffer[0] != lm75_probe_tos_1_h) || (buffer[1] != lm75_probe_tos_1_l)) && ((buffer[0] != lm75_probe_tos_2_h) || (buffer[1] != lm75_probe_tos_2_l)))
		return(false);

	return(true);
}

static bool lm75_init(data_t *data)
{
	uint8_t buffer[2];

	if(!i2c_send_2(data->slave, lm75_reg_conf, lm75_reg_conf_no_shutdown))
		return(false);

	if(!i2c_send_1_receive(data->slave, lm75_reg_conf, 2, buffer))
		return(false);

	if((buffer[0] & ~lm75_reg_conf_reserved) != lm75_reg_conf_no_shutdown)
		return(false);

	return(true);
}

static bool lm75_poll(data_t *data)
{
	uint8_t buffer[2];
	float raw_temperature;

	if(!i2c_send_1_receive(data->slave, lm75_reg_temp, 2, buffer))
	{
		log("lm75: poll error");
		return(false);
	}

	data->int_value[lm75_int_raw_value_0] = buffer[0];
	data->int_value[lm75_int_raw_value_1] = buffer[1];
	raw_temperature = (buffer[0] << 8) | buffer[1];
	data->values[sensor_type_temperature].value = raw_temperature / 256.0F;
	data->values[sensor_type_temperature].stamp = time((time_t *)0);

	return(true);
}

enum
{
	opt3001_int_raw_value_0 = 0,
	opt3001_int_raw_value_1,
	opt3001_int_size,
};

_Static_assert((unsigned int)opt3001_int_size <= (unsigned int)data_int_value_size);

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
} opt3001_conf;

static bool opt3001_detect(data_t *data)
{
	uint8_t buffer[4];
	unsigned int id;

	if(!i2c_send_1_receive(data->slave, opt3001_reg_id_manuf, 2, buffer))
		return(false);

	id = (buffer[0] << 8) | (buffer[1] << 0);

	if(id != opt3001_id_manuf_ti)
		return(false);

	if(!i2c_send_1_receive(data->slave, opt3001_reg_id_dev, 2, buffer))
		return(false);

	id = (buffer[0] << 8) | (buffer[1] << 0);

	if(id != opt3001_id_dev_opt3001)
		return(false);

	return(true);
}

static bool opt3001_init(data_t *data)
{
	uint8_t buffer[4];
	static const unsigned int config = opt3001_conf_range_auto | opt3001_conf_conv_time_800 | opt3001_conf_conv_mode_cont;
	unsigned int read_config;

	buffer[0] = opt3001_reg_conf;
	buffer[1] = (config & 0xff00) >> 8;
	buffer[2] = (config & 0x00ff) >> 0;

	if(!i2c_send(data->slave, 3, buffer))
		return(false);

	if(!i2c_send_1_receive(data->slave, opt3001_reg_conf, 2, buffer))
		return(false);

	read_config = ((buffer[0] << 8) | (buffer[1] << 0)) & (opt3001_conf_mask_exp | opt3001_conf_conv_mode | opt3001_conf_conv_time | opt3001_conf_range);

	if(read_config != config)
		return(false);

	return(true);
}

static bool opt3001_poll(data_t *data)
{
	uint8_t buffer[2];
	unsigned int config, exponent, mantissa;

	if(!i2c_send_1_receive(data->slave, opt3001_reg_conf, 2, buffer))
	{
		log("opt3001 poll: error 1");
		return(false);
	}

	config = (buffer[0] << 8) | (buffer[1] << 0);

	if(!(config & opt3001_conf_flag_ready))
		return(true);

	if(config & opt3001_conf_flag_ovf)
	{
		log("opt3001 poll: overflow");
		return(true);
	}

	if(!i2c_send_1_receive(data->slave, opt3001_reg_result, 2, buffer))
	{
		log("opt3001 poll: error 2");
		return(false);
	}

	exponent = (buffer[0] & 0xf0) >> 4;
	mantissa = ((buffer[0] & 0x0f) << 8) | buffer[1];

	data->int_value[opt3001_int_raw_value_0] = exponent;
	data->int_value[opt3001_int_raw_value_1] = mantissa;
	data->values[sensor_type_visible_light].value = 0.01F * (float)(1 << exponent) * (float)mantissa;
	data->values[sensor_type_visible_light].stamp = time((time_t *)0);

	return(true);
}

enum
{
	max44009_int_raw_value_0 = 0,
	max44009_int_raw_value_1,
	max44009_int_size,
};

_Static_assert((unsigned int)max44009_int_size <= (unsigned int)data_int_value_size);

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

static bool max44009_detect(data_t *data)
{
	uint8_t buffer[2];

	if(!i2c_send_1_receive(data->slave, max44009_reg_ints, 2, buffer))
		return(false);

	if((buffer[0] != max44009_probe_ints) || (buffer[1] != max44009_probe_ints))
		return(false);

	if(!i2c_send_1_receive(data->slave, max44009_reg_inte, 2, buffer))
		return(false);

	if((buffer[0] != max44009_probe_inte) || (buffer[1] != max44009_probe_inte))
		return(false);

	if(!i2c_send_1_receive(data->slave, max44009_reg_thresh_msb, 2, buffer))
		return(false);

	if((buffer[0] != max44009_probe_thresh_msb) || (buffer[1] != max44009_probe_thresh_msb))
		return(false);

	if(!i2c_send_1_receive(data->slave, max44009_reg_thresh_lsb, 2, buffer))
		return(false);

	if((buffer[0] != max44009_probe_thresh_lsb) || (buffer[1] != max44009_probe_thresh_lsb))
		return(false);

	if(!i2c_send_1_receive(data->slave, max44009_reg_thresh_timer, sizeof(buffer), buffer))
		return(false);

	if((buffer[0] != max44009_probe_thresh_timer) || (buffer[1] != max44009_probe_thresh_timer))
		return(false);

	return(true);
}

static bool max44009_init(data_t *data)
{
	uint8_t buffer[2];

	if(!i2c_send_2(data->slave, max44009_reg_conf, max44009_conf_cont))
	{
		log("sensors: max44009: init error 1");
		return(false);
	}

	if(!i2c_send_1_receive(data->slave, max44009_reg_conf, 2, buffer))
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
	uint8_t		buffer[2];
	int			exponent, mantissa;

	if(!i2c_send_1_receive(data->slave, max44009_reg_data_msb, 2, buffer))
	{
		log("sensors: max44009: poll error 1");
		return(false);
	}

	exponent =	(buffer[0] & 0xf0) >> 4;
	mantissa =	(buffer[0] & 0x0f) << 4;
	mantissa |=	(buffer[1] & 0x0f) << 0;

	data->int_value[max44009_int_raw_value_0] = exponent;
	data->int_value[max44009_int_raw_value_1] = mantissa;

	if(exponent == 0b1111)
		log("sensors: max44009: overflow");
	else
	{
		data->values[sensor_type_visible_light].value = (1 << exponent) * mantissa * 0.045;
		data->values[sensor_type_visible_light].stamp = time((time_t *)0);
	}

	return(true);
}

enum
{
	asair_int_raw_value_temp = 0,
	asair_int_raw_value_hum,
	asair_int_state,
	asair_int_valid,
	asair_int_type,
	asair_int_size,
};

_Static_assert((unsigned int)asair_int_size <= (unsigned int)data_int_value_size);

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
	asair_state_start_measure,
	asair_state_measuring,
	asair_state_measure_complete,
} asair_state_t;

static bool asair_detect(data_t *data)
{
	uint8_t buffer[1];

	if(!i2c_send_1_receive(data->slave, asair_cmd_get_status, sizeof(buffer), buffer))
		return(false);

	if(!i2c_send_1(data->slave, asair_cmd_reset))
		return(false);

	util_sleep(25);

	if(i2c_send_3(data->slave, asair_cmd_aht10_init_1, asair_cmd_aht10_init_2, asair_cmd_aht10_init_3))
	{
		data->int_value[asair_int_type] = 10;
		return(true);
	}

	if(i2c_send_3(data->slave, asair_cmd_aht20_init_1, asair_cmd_aht20_init_2, asair_cmd_aht20_init_3))
	{
		data->int_value[asair_int_type] = 20;
		return(true);
	}

	return(false);
}

static bool asair_init(data_t *data)
{
	uint8_t buffer[1];

	data->int_value[asair_int_raw_value_temp] = 0;
	data->int_value[asair_int_raw_value_hum] = 0;
	data->int_value[asair_int_state] = asair_state_init;
	data->int_value[asair_int_valid] = 0;

	if(!i2c_send_1_receive(data->slave, asair_cmd_get_status, sizeof(buffer), buffer))
	{
		log("sensors: asair: init error 1");
		return(false);
	}

	if(!(buffer[0] & asair_status_ready))
	{
		log("sensors: asair: init error 2");
		return(false);
	}

	return(true);
}

static bool asair_poll(data_t *data)
{
	uint8_t	buffer[8];

	switch(data->int_value[asair_int_state])
	{
		case(asair_state_init):
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

            data->int_value[asair_int_valid] = 0;
			data->int_value[asair_int_state] = asair_state_start_measure;

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
				data->int_value[asair_int_valid] = 0;
				return(false);
			}

			data->int_value[asair_int_state] = asair_state_measuring;

			break;
		}

		case(asair_state_measuring):
		{
			data->int_value[asair_int_state] = asair_state_measure_complete;

			break;
		}

		case(asair_state_measure_complete):
		{
			if(!i2c_send_1_receive(data->slave, asair_cmd_get_status, sizeof(buffer), buffer))
			{
				log("sensors: asair: poll error 6");
				data->int_value[asair_int_valid] = 0;
				return(false);
			}

			if(buffer[0] & asair_status_busy)
			{
				log("sensors: asair: poll error 7");
				data->int_value[asair_int_valid] = 0;
				return(false);
			}

            data->int_value[asair_int_raw_value_temp] =	((buffer[3] & 0x0f) << 16) | (buffer[4] << 8) | buffer[5];
			data->int_value[asair_int_raw_value_hum] =	((buffer[1] << 16) | (buffer[2] << 8) | (buffer[3] & 0xf0)) >> 4;

			data->values[sensor_type_temperature].value = ((200.f * data->int_value[asair_int_raw_value_temp]) / 1048576.f) - 50;
			data->values[sensor_type_temperature].stamp = time((time_t *)0);

			data->values[sensor_type_humidity].value = data->int_value[asair_int_raw_value_hum] * 100.f / 1048576.f;
			data->values[sensor_type_humidity].stamp = time((time_t *)0);

            data->int_value[asair_int_valid] = 1;
			data->int_value[asair_int_state] = asair_state_start_measure;

			break;
		}
	}

	return(true);
}

enum
{
	tsl2561_int_raw_value_ch0 = 0,
	tsl2561_int_raw_value_ch1,
	tsl2561_int_scaling,
	tsl2561_int_scaling_up,
	tsl2561_int_scaling_down,
	tsl2561_int_size,
};

_Static_assert((unsigned int)tsl2561_int_size <= (unsigned int)data_int_value_size);

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

static const device_autoranging_data_t tsl2561_autoranging_data[tsl2561_autoranging_data_size] =
{
	{{	tsl2561_tim_integ_402ms,	tsl2561_tim_high_gain	},	{	0,		50000	},	65535,	{	480000,		0	}},
	{{	tsl2561_tim_integ_402ms,	tsl2561_tim_low_gain	},	{	256,	50000	},	65535,	{	7400000,	0	}},
	{{	tsl2561_tim_integ_101ms,	tsl2561_tim_low_gain	},	{	256,	50000	},	31711,	{	28000000,	0	}},
	{{	tsl2561_tim_integ_13ms,		tsl2561_tim_low_gain	},	{	256,	50000	},	5047,	{	200000000,	0	}},
};

static bool tsl2561_write(data_t *data, tsl2561_reg_t reg, unsigned int value)
{
	if(!i2c_send_2(data->slave, tsl2561_cmd_cmd | tsl2561_cmd_clear | (reg & tsl2561_cmd_address), value))
	{
		log("sensor: tsl2561: error 1");
		return(false);
	}

	return(true);
}

static bool tsl2561_read_byte(data_t *data, tsl2561_reg_t reg, uint8_t *byte)
{
	if(!i2c_send_1_receive(data->slave, tsl2561_cmd_cmd | (reg & tsl2561_cmd_address), sizeof(*byte), byte))
	{
		log("sensor: tsl2561: error 2");
		return(false);
	}

	return(true);
}

static bool tsl2561_read_word(data_t *data, tsl2561_reg_t reg, uint16_t *word)
{
	uint8_t i2c_buffer[2];

	if(!i2c_send_1_receive(data->slave, tsl2561_cmd_cmd | (reg & tsl2561_cmd_address), sizeof(i2c_buffer), i2c_buffer))
	{
		log("sensor: tsl2561: error 3");
		return(false);
	}

	*word = (i2c_buffer[0] << 8) | i2c_buffer[1];

	return(true);
}

static bool tsl2561_write_check(data_t *data, tsl2561_reg_t reg, unsigned int value)
{
	uint8_t rv;

	if(!tsl2561_write(data, reg, value))
		return(false);

	if(!tsl2561_read_byte(data, reg, &rv))
		return(false);

	if(value != rv)
		return(false);

	return(true);
}

static bool tsl2561_start_measurement(data_t *data)
{
	unsigned int timeint, gain;

	timeint =	tsl2561_autoranging_data[data->int_value[tsl2561_int_scaling]].data[0];
	gain =		tsl2561_autoranging_data[data->int_value[tsl2561_int_scaling]].data[1];

	return(tsl2561_write_check(data, tsl2561_reg_timeint, timeint | gain));
}

static bool tsl2561_detect(data_t *data)
{
	uint8_t regval;
	uint16_t word;

	if(!tsl2561_read_byte(data, tsl2561_reg_id, &regval))
		return(false);

	if(regval != tsl2561_id_tsl2561)
		return(false);

	if(!tsl2561_read_word(data, tsl2561_reg_threshlow, &word))
		return(false);

	if(word != tsl2561_probe_threshold)
		return(false);

	if(!tsl2561_read_word(data, tsl2561_reg_threshhigh, &word))
		return(false);

	if(word != tsl2561_probe_threshold)
		return(false);

	if(!tsl2561_write_check(data, tsl2561_reg_control, tsl2561_ctrl_power_off))
		return(false);

	if(tsl2561_write_check(data, tsl2561_reg_id, 0x00)) // id register should not be writable
		return(false);

	return(true);
}

static bool tsl2561_init(data_t *data)
{
	uint8_t regval;

	if(!tsl2561_write_check(data, tsl2561_reg_interrupt, 0x00))				// disable interrupts
		return(false);

	if(!tsl2561_write(data, tsl2561_reg_control, tsl2561_ctrl_power_on))	// power up
		return(false);

	if(!tsl2561_read_byte(data, tsl2561_reg_control, &regval))
		return(false);

	if((regval & 0x0f) != tsl2561_ctrl_power_on)
		return(false);

	data->int_value[tsl2561_int_raw_value_ch0] = 0;
	data->int_value[tsl2561_int_raw_value_ch1] = 0;
	data->int_value[tsl2561_int_scaling] = 0;

	if(!tsl2561_start_measurement(data))
		return(false);

	return(true);
}

static bool tsl2561_poll(data_t *data)
{
	uint8_t i2c_buffer[4];
	unsigned int ch0, ch1;
	unsigned int overflow, scale_down_threshold, scale_up_threshold;

	scale_down_threshold =	tsl2561_autoranging_data[data->int_value[tsl2561_int_scaling]].threshold.down;
	scale_up_threshold =	tsl2561_autoranging_data[data->int_value[tsl2561_int_scaling]].threshold.up;
	overflow =				tsl2561_autoranging_data[data->int_value[tsl2561_int_scaling]].overflow;

	if(!i2c_send_1_receive(data->slave, tsl2561_cmd_cmd | tsl2561_reg_data0, sizeof(i2c_buffer), i2c_buffer))
	{
		log("sensor: tsl2561 poll error 1");
		return(false);
	}

	ch0 = (i2c_buffer[1] << 8) | i2c_buffer[0];
	ch1 = (i2c_buffer[3] << 8) | i2c_buffer[1];

	if((ch0 == 0) && (ch1 == 0))
	{
		log("sensor: tsl2561: malfunction");

		data->int_value[tsl2561_int_scaling] = tsl2561_autoranging_data_size - 1;
		data->int_value[tsl2561_int_scaling_up]++;

		if(!tsl2561_start_measurement(data))
		{
			log("sensor: tsl2561 poll error 2");
			return(false);
		}

		return(true);
	}

	if(((ch0 < scale_down_threshold) || (ch1 < scale_down_threshold)) && (data->int_value[tsl2561_int_scaling] > 0))
	{
		data->int_value[tsl2561_int_scaling]--;
		data->int_value[tsl2561_int_scaling_down]++;

		if(!tsl2561_start_measurement(data))
		{
			log("sensor: tsl2561 poll error 3");
			return(false);
		}

		return(true);
	}

	if((((ch0 >= scale_up_threshold) || (ch1 >= scale_up_threshold)) || (ch0 >= overflow) || (ch1 >= overflow)) &&
			((data->int_value[tsl2561_int_scaling] + 1) < tsl2561_autoranging_data_size))
	{
		data->int_value[tsl2561_int_scaling]++;
		data->int_value[tsl2561_int_scaling_up]++;

		if(!tsl2561_start_measurement(data))
		{
			log("sensor: tsl2561 poll error 4");
			return(false);
		}

		return(true);
	}

	float value, ratio;
	unsigned int factor_1000000;
	int offset_1000000;

	data->int_value[tsl2561_int_raw_value_ch0] = ch0;
	data->int_value[tsl2561_int_raw_value_ch1] = ch1;

	factor_1000000 = tsl2561_autoranging_data[data->int_value[tsl2561_int_scaling]].correction.factor;
	offset_1000000 = tsl2561_autoranging_data[data->int_value[tsl2561_int_scaling]].correction.offset;

	if(ch0 == 0)
		ratio = 0;
	else
		ratio = ch1 / ch0;

	if(ratio > 1.30f)
		value = -1;
	else
	{
		if(ratio >= 0.80f)
			value = (0.00146f * ch0) - (0.00112f * ch1);
		else
			if(ratio >= 0.61f)
				value = (0.0128f * ch0) - (0.0153f * ch1);
			else
				if(ratio >= 0.50f)
					value = (0.0224f * ch0) - (0.031f * ch1);
				else
					value = (0.0304f * ch0) - (0.062f * ch1 * (float)pow(ratio, 1.4f));

		value = ((value * factor_1000000) + offset_1000000) / 1000000.0f;

		if(value < 0)
			value = 0;
	}

	data->values[sensor_type_visible_light].value = value;
	data->values[sensor_type_visible_light].stamp = time((time_t *)0);

	return(true);
}

enum
{
	hdc1080_int_raw_temperature = 0,
	hdc1080_int_raw_humidity,
	hdc1080_int_valid,
	hdc1080_int_size,
};

_Static_assert((unsigned int)hdc1080_int_size <= (unsigned int)data_int_value_size);

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

static bool hdc1080_detect(data_t *data)
{
	uint8_t buffer[4];

	if(!i2c_send_1_receive(data->slave, hdc1080_reg_man_id, 2, buffer))
		return(false);

	if(unsigned_16(buffer[0], buffer[1]) != hdc1080_man_id)
		return(false);

	if(!i2c_send_1_receive(data->slave, hdc1080_reg_dev_id, sizeof(buffer), buffer))
		return(false);

	if(unsigned_16(buffer[0], buffer[1]) != hdc1080_dev_id)
		return(false);

	if(!i2c_send_2(data->slave, hdc1080_reg_conf, hdc1080_conf_rst))
		return(false);

	return(true);
}

static bool hdc1080_init(data_t *data)
{
	unsigned int conf;

	conf = hdc1080_conf_tres_14 | hdc1080_conf_hres_14 | hdc1080_conf_mode_two;

	if(!i2c_send_3(data->slave, hdc1080_reg_conf, (conf & 0xff00) >> 8, (conf & 0x00ff) >> 0))
	{
		log("sensors: hdc1080 error 1");
		return(false);
	}

	if(!i2c_send_1(data->slave, hdc1080_reg_data_temp))
	{
		log("sensors: hdc1080 error 2");
		return(false);
	}

	data->int_value[hdc1080_int_raw_temperature] = 0;
	data->int_value[hdc1080_int_raw_humidity] = 0;
	data->int_value[hdc1080_int_valid] = 0;

	return(true);
}

static bool hdc1080_poll(data_t *data)
{
	uint8_t buffer[4];

	if(!i2c_receive(data->slave, sizeof(buffer), buffer))
	{
		log("hdc1080 poll error 1");
		return(false);
	}

	if(!i2c_send_1(data->slave, hdc1080_reg_data_temp))
	{
		log("hdc1080 poll error 2");
		return(false);
	}

	data->int_value[hdc1080_int_raw_temperature] = unsigned_16(buffer[0], buffer[1]);
	data->int_value[hdc1080_int_raw_humidity] = unsigned_16(buffer[2], buffer[3]);
	data->int_value[hdc1080_int_valid] = 1;

	data->values[sensor_type_temperature].value = ((data->int_value[hdc1080_int_raw_temperature] * 165.0f) / (float)(1 << 16)) - 40.f;
	data->values[sensor_type_temperature].stamp = time((time_t *)0);

	data->values[sensor_type_humidity].value = (data->int_value[hdc1080_int_raw_humidity] * 100.0f) / 65536.0f;
	data->values[sensor_type_humidity].stamp = time((time_t *)0);

	return(true);
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
		.detect_fn = bh1750_detect,
		.init_fn = bh1750_init,
		.poll_fn = bh1750_poll,
	},
	[sensor_tmp75] =
	{
		.name = "tmp75",
		.id = sensor_tmp75,
		.address = 0x48,
		.type = (1 << sensor_type_temperature),
		.precision = 1,
		.detect_fn = tmp75_detect,
		.init_fn = tmp75_init,
		.poll_fn = tmp75_poll,
	},
	[sensor_lm75] =
	{
		.name = "lm75",
		.id = sensor_lm75,
		.address = 0x48,
		.type = (1 << sensor_type_temperature),
		.precision = 1,
		.detect_fn = lm75_detect,
		.init_fn = lm75_init,
		.poll_fn = lm75_poll,
	},
	[sensor_opt3001] =
	{
		.name = "opt3001",
		.id = sensor_opt3001,
		.address = 0x45,
		.type = (1 << sensor_type_visible_light),
		.precision = 2,
		.detect_fn = opt3001_detect,
		.init_fn = opt3001_init,
		.poll_fn = opt3001_poll,
	},
	[sensor_max44009] =
	{
		.name = "max44009",
		.id = sensor_max44009,
		.address = 0x4a,
		.type = (1 << sensor_type_visible_light),
		.precision = 2,
		.detect_fn = max44009_detect,
		.init_fn = max44009_init,
		.poll_fn = max44009_poll,
	},
	[sensor_asair] =
	{
		.name = "asair",
		.id = sensor_asair,
		.address = 0x38,
		.type = (1 << sensor_type_temperature) | (1 << sensor_type_humidity),
		.precision = 1,
		.detect_fn = asair_detect,
		.init_fn = asair_init,
		.poll_fn = asair_poll,
	},
	[sensor_tsl2561] =
	{
		.name = "tsl2561",
		.id = sensor_tsl2561,
		.address = 0x39,
		.type = (1 << sensor_type_visible_light),
		.precision = 2,
		.detect_fn = tsl2561_detect,
		.init_fn = tsl2561_init,
		.poll_fn = tsl2561_poll,
	},
	[sensor_hdc1080] =
	{
		.name = "hdc1080",
		.id = sensor_hdc1080,
		.address = 0x40,
		.type = (1 << sensor_type_temperature) | (1 << sensor_type_humidity),
		.precision = 1,
		.detect_fn = hdc1080_detect,
		.init_fn = hdc1080_init,
		.poll_fn = hdc1080_poll,
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
	unsigned int ix;
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

			new_data = (data_t *)util_memory_alloc_spiram(sizeof(*new_data));
			assert(new_data);

			for(type = sensor_type_first; type < sensor_type_size; type++)
			{
				new_data->values[type].value = 0;
				new_data->values[type].stamp = (time_t)0;
			}

			for(ix = 0; ix < data_int_value_size; ix++)
				new_data->int_value[ix] = 0;

			for(ix = 0; ix < data_float_value_size; ix++)
				new_data->float_value[ix] = 0;

			new_data->slave = slave;
			new_data->info = infoptr;
			new_data->next = (data_t *)0;

			assert(infoptr->detect_fn);

			if(!infoptr->detect_fn(new_data))
			{
				i2c_unregister_slave(&slave);
				free(new_data);
				continue;
			}

			stat_sensors_confirmed[module]++;

			assert(infoptr->init_fn);

			if(!infoptr->init_fn(new_data))
			{
				log_format("sensor: warning: failed to init sensor %s on bus %u", infoptr->name, bus);
				i2c_unregister_slave(&slave);
				free(new_data);
				continue;
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

	util_abort("sensor: vTaskSuspend returned");
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
	unsigned int ix;
	string_auto(time_string, 64);

	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "SENSOR dump");

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

			string_append_cstr(call->result, "\n  raw integer values:");
			for(ix = 0; ix < data_int_value_size; ix++)
				string_format_append(call->result, " %u=%d", ix, dataptr->int_value[ix]);

			string_append_cstr(call->result, "\n  raw float values:");

			for(ix = 0; ix < data_float_value_size; ix++)
				string_format_append(call->result, " %u=%.2f", ix, (double)dataptr->float_value[ix]);
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
