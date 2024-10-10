#include <stdint.h>
#include <stdbool.h>

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
	data_int_value_size = 4,
	data_float_value_size = 2,
};

typedef struct data_T
{
	i2c_slave_t slave;
	int int_value[data_int_value_size];
	float float_value[data_float_value_size];
	float value[sensor_type_size];
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
		.unity = "C",
	},
};

static bool inited = false;
static SemaphoreHandle_t data_mutex;
static data_t *data = (data_t *)0;

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

_Static_assert((unsigned int)bh1750_int_size < (unsigned int)data_int_value_size);

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

bool bh1750_detect(data_t *data)
{
	uint8_t buffer[8];

	if(!i2c_receive(data->slave, sizeof(buffer), buffer))
		return(false);

	if((buffer[2] != 0xff) || (buffer[3] != 0xff) || (buffer[4] != 0xff) ||
			(buffer[5] != 0xff) || (buffer[6] != 0xff) || (buffer[7] != 0xff))
		return(false);

	return(true);
}

bool bh1750_init(data_t *data)
{
	if(!i2c_send_1(data->slave, bh1750_opcode_poweron))
		return(false);

	if(!i2c_send_1(data->slave, bh1750_opcode_reset))
		return(false);

	data->int_value[bh1750_int_raw_value] = 0;
	data->int_value[bh1750_int_scaling] = 0;
	data->int_value[bh1750_int_state] = bh1750_start_measurement;
	data->value[sensor_type_visible_light] = 0;

	return(true);
}

bool bh1750_poll(data_t *data)
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

			if(raw_value == 0)
			{
				log("bh1750: warning: measurement unfinished");
				break;
			}

			if((raw_value < scale_down_threshold) && (*scaling > 0))
				(*scaling)--;

			if((raw_value >= scale_up_threshold) && ((*scaling + 1) < bh1750_autoranging_data_size))
				(*scaling)++;

			if(raw_value < 0xffff)
			{
				data->int_value[bh1750_int_raw_value] = raw_value;
				data->value[sensor_type_visible_light] = ((float)raw_value * factor) + offset;
			}
			else
				log_format("bh1750: warning: measurement out of range: %u", raw_value);

			break;
		}

		default:
		{
			log_format("bh1750: poll: invalid state: %u", data->int_value[bh1750_int_state]);
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

_Static_assert((unsigned int)tmp75_int_size < (unsigned int)data_int_value_size);

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
	data->value[sensor_type_temperature] = raw_temperature / 256.0;

	return(true);
}

enum
{
	lm75_int_raw_value_0 = 0,
	lm75_int_raw_value_1,
	lm75_int_size,
};

_Static_assert((unsigned int)lm75_int_size < (unsigned int)data_int_value_size);

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
	data->value[sensor_type_temperature] = raw_temperature / 256.0;

	return(true);
}

enum
{
	opt3001_int_raw_value_0 = 0,
	opt3001_int_raw_value_1,
	opt3001_int_size,
};

_Static_assert((unsigned int)opt3001_int_size < (unsigned int)data_int_value_size);

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
	data->value[sensor_type_visible_light] = 0.01 * (float)(1 << exponent) * (float)mantissa;

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
};

static void run_sensors(void *) // FIXME one thread per module parallel polling
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

	for(module = i2c_module_first; module < i2c_module_size; module++)
	{
		buses = i2c_buses(module);

		for(bus = i2c_bus_first; bus < buses; bus++)
		{
			for(sensor = sensor_first; sensor < sensor_size; sensor++)
			{
				infoptr = &info[sensor];

				if(i2c_find_slave(module, bus, infoptr->address))
				{
					log_format("sensor: warning: skip probe on bus %u, already present", (unsigned int)bus);
					continue;
				}

				if(!i2c_probe_slave(module, bus, infoptr->address))
					continue;

				if(!(slave = i2c_register_slave(infoptr->name, module, bus, infoptr->address)))
				{
					log_format("sensor: warning: cannot register sensor %s", infoptr->name);
					continue;
				}

				new_data = (data_t *)util_memory_alloc_spiram(sizeof(*new_data));
				assert(new_data);

				for(type = sensor_type_first; type < sensor_type_size; type++)
					new_data->value[type] = 0;

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
					log_format("sensor: warning: failed to detect sensor %s", infoptr->name);
					i2c_unregister_slave(&slave);
					free(new_data);
					continue;
				}

				assert(infoptr->init_fn);

				if(!infoptr->init_fn(new_data))
				{
					log_format("sensor: warning: failed to init sensor %s", infoptr->name);
					i2c_unregister_slave(&slave);
					free(new_data);
					continue;
				}

				data_mutex_take();

				if(!(dataptr = data))
					data = new_data;
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
	}

	for(;;)
	{
		for(dataptr = data; dataptr; dataptr = dataptr->next)
		{
			if(dataptr->info->poll_fn)
			{
				if(!dataptr->info->poll_fn(dataptr))
					log_format("sensor: warning: poll failed sensor %s", dataptr->info->name);
			}
			else
				log_format("sensor: error: no poll function for sensor %s", dataptr->info->name);
		}

		util_sleep(1000);
	}

	util_abort("sensor: vTaskSuspend returned");
}

void sensor_init(void)
{
	assert(!inited);

	data_mutex = xSemaphoreCreateMutex();
	assert(data_mutex);

	inited = true;

	if(xTaskCreatePinnedToCore(run_sensors, "sensors", 3 * 1024, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("sensor: xTaskCreatePinnedToNode sensors thread");
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

	for(dataptr = data; dataptr; dataptr = dataptr->next)
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
							dataptr->info->precision, dataptr->value[type], sensor_type_info[type].unity);
		}
	}

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

	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "SENSOR dump");

	data_mutex_take();

	for(dataptr = data; dataptr; dataptr = dataptr->next)
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
				if(dataptr->info->type & (1 << type))
					string_format_append(call->result, " %s=%.*f", sensor_type_info[type].type, dataptr->info->precision, dataptr->value[type]);

			string_append_cstr(call->result, "\n  raw integer values:");

			for(ix = 0; ix < data_int_value_size; ix++)
				string_format_append(call->result, " %u=%u", ix, dataptr->int_value[ix]);

			string_append_cstr(call->result, "\n  raw float values:");

			for(ix = 0; ix < data_float_value_size; ix++)
				string_format_append(call->result, " %u=%.2f", ix, dataptr->float_value[ix]);
		}
	}

	data_mutex_give();
}
