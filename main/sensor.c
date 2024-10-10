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

static const info_t info[sensor_size] =
{
	[sensor_bh1750] =
	{
		.name = "bh1750",
		.id = sensor_bh1750,
		.address = 0x23,
		.type = (1 << sensor_type_visible_light),
		.precision = 0,
		.detect_fn = bh1750_detect_fn,
		.init_fn = bh1750_init_fn,
		.poll_fn = bh1750_poll_fn,
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
