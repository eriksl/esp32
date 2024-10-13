#include <stdint.h>
#include <stdbool.h>

#include "i2c.h"
#include "string.h"
#include "info.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "io.h"

#include <assert.h>
#include <freertos/FreeRTOS.h>

typedef enum
{
	io_bus_i2c = 0,
	io_bus_first = io_bus_i2c,
	io_bus_size,
	io_bus_error = io_bus_size,
} io_bus_t;

typedef enum
{
	io_int_value_0 = 0,
	io_int_value_first = io_int_value_0,
	io_int_value_1,
	io_int_value_size,
	io_int_value_error = io_int_value_size,
} io_int_value_t;

struct io_data_T;

typedef struct
{
	io_id_t id;
	const char *name;
	io_bus_t bus;
	io_capabilities_t caps;
	unsigned int pins;
	unsigned int max_value;
	union
	{
		struct
		{
			unsigned int address;
		} i2c;
	};
	bool (*init_fn)(struct io_data_T *data);
	bool (*read_fn)(struct io_data_T *data, unsigned int pin, unsigned int *value);
	bool (*write_fn)(struct io_data_T *data, unsigned int pin, unsigned int value);
} io_info_t;

typedef struct io_data_T
{
	io_id_t id;
	union
	{
		struct
		{
			i2c_slave_t slave;
		} i2c;
		void *void_ptr;
	};
	int int_value[io_int_value_size];
	const io_info_t *info;
	struct io_data_T *next;
} io_data_t;

static bool inited = false;
static io_data_t *data_root = (io_data_t *)0;
static SemaphoreHandle_t data_mutex;
static unsigned int stat_probe_skipped;
static unsigned int stat_probe_tried;
static unsigned int stat_probe_found;

enum
{
	pcf8574_in_cache = 0,
	pcf8574_out_cache,
	pcf8574_size,
};

_Static_assert((unsigned int)pcf8574_size <= (unsigned int)io_int_value_size);

static inline void data_mutex_take(void)
{
	assert(xSemaphoreTake(data_mutex, portMAX_DELAY));
}

static inline void data_mutex_give(void)
{
	assert(xSemaphoreGive(data_mutex));
}

static bool pcf8574_init(io_data_t *dataptr)
{
	dataptr->int_value[pcf8574_in_cache] = 0xff;
	dataptr->int_value[pcf8574_out_cache] = 0xff;

	if(!i2c_send_1(dataptr->i2c.slave, 0xff))
	{
		log("io pcf8574 init: i2c send failed");
		return(false);
	}

	return(true);
}

static bool pcf8574_read(io_data_t *dataptr, unsigned int pin, unsigned int *value)
{
	uint8_t buffer[1];

	assert(dataptr);
	assert(dataptr->info);
	assert(pin < dataptr->info->pins);

	if(!i2c_receive(dataptr->i2c.slave, 1, buffer))
		return(false);

	*value = dataptr->int_value[pcf8574_in_cache] = (unsigned int)buffer[0];

	return(true);
}

static bool pcf8574_write(io_data_t *dataptr, unsigned int pin, unsigned int value)
{
	assert(dataptr);
	assert(dataptr->info);
	assert(pin < dataptr->info->pins);
	assert(value <= dataptr->info->max_value);

	if(value)
		dataptr->int_value[pcf8574_out_cache] &= ~(1 << pin);
	else
		dataptr->int_value[pcf8574_out_cache] |= (1 << pin);

	return(i2c_send_1(dataptr->i2c.slave, dataptr->int_value[pcf8574_out_cache]));
}

static const io_info_t info[io_id_size] =
{
	[io_id_pcf8574_26] =
	{
		.id = io_id_pcf8574_26,
		.name = "PCF8574 8-bit I/O expander",
		.caps = (1 << io_cap_input) | (1 << io_cap_output),
		.pins = 8,
		.max_value = 1,
		.bus = io_bus_i2c,
		.i2c.address = 0x26,
		.init_fn = pcf8574_init,
		.read_fn = pcf8574_read,
		.write_fn = pcf8574_write,
	},
	[io_id_pcf8574_3a] =
	{
		.id = io_id_pcf8574_3a,
		.name = "PCF8574 8-bit I/O expander",
		.caps = (1 << io_cap_input) | (1 << io_cap_output),
		.pins = 8,
		.max_value = 1,
		.bus = io_bus_i2c,
		.i2c.address = 0x3a,
		.init_fn = pcf8574_init,
		.read_fn = pcf8574_read,
		.write_fn = pcf8574_write,
	},
};

static io_data_t *find_io(unsigned int parameter_1, unsigned int parameter_2, unsigned int parameter_3)
{
	io_data_t *dataptr;

	data_mutex_take();

	for(dataptr = data_root; dataptr; dataptr = dataptr->next)
	{
		switch(dataptr->info->bus)
		{
			case(io_bus_i2c):
			{
				i2c_module_t module;
				i2c_bus_t bus;
				unsigned int address;
				const char *name;

				assert(parameter_1 < i2c_module_size);
				assert(parameter_2 < i2c_bus_size);
				assert(parameter_3 < 128);

				i2c_get_slave_info(dataptr->i2c.slave, &module, &bus, &address, &name);

				if((i2c_module_t)parameter_1 != module)
					continue;

				if(parameter_3 != address)
					continue;

				if(((i2c_bus_t)parameter_2 == i2c_bus_none) || (bus == i2c_bus_none))
					goto found;

				if((i2c_bus_t)parameter_2 == bus)
					goto found;

				break;
			}

			default:
			{
				log_format("io: find_io: bus %u unknown", dataptr->info->bus);
				break;
			}
		}
	}

found:
	data_mutex_give();
	return(dataptr);
}

void io_init(void)
{
	const io_info_t *infoptr;
	io_data_t *dataptr, *next;
	io_id_t id;
	i2c_module_t module;
	i2c_bus_t bus;
	i2c_slave_t slave;
	unsigned int buses;

	assert(!inited);

	data_mutex = xSemaphoreCreateMutex();
	assert(data_mutex);

	inited = true;

	for(id = io_id_first; id < io_id_size; id++)
	{
		infoptr = &info[id];

		assert(infoptr->id == id);

		switch(infoptr->bus)
		{
			case(io_bus_i2c):
			{
				for(module = i2c_module_first; module < i2c_module_size; module++)
				{
					buses = i2c_buses(module);

					for(bus = 0; bus < buses; bus++)
					{
						if(find_io((unsigned int)module, (unsigned int)bus, infoptr->i2c.address))
						{
							stat_probe_skipped++;
							continue;
						}

						stat_probe_tried++;

						if(!i2c_probe_slave(module, bus, infoptr->i2c.address))
							continue;

						if(!(slave = i2c_register_slave(infoptr->name, module, bus, infoptr->i2c.address)))
						{
							log_format("io: warning: cannot register io %s", infoptr->name);
							continue;
						}

						dataptr = (io_data_t *)util_memory_alloc_spiram(sizeof(*dataptr));
						assert(dataptr);

						dataptr->id = id;
						dataptr->i2c.slave = slave;
						dataptr->info = infoptr;
						dataptr->next = (io_data_t *)0;

						assert(infoptr->init_fn);

						if(!infoptr->init_fn(dataptr))
						{
							log_format("io: init %s failed", infoptr->name);
							continue;
						}

						stat_probe_found++;

						if(!data_root)
							data_root = dataptr;
						else
						{
							for(next = data_root; next->next; next = next->next)
								(void)0;

							assert(!next->next);

							next->next = dataptr;
						}
					}
				}

				break;
			}

			case(io_bus_error):
			{
				log("io: invalid io type in info");
				break;
			}
		}
	}
}

void command_io_dump(cli_command_call_t *call)
{
	const io_data_t *dataptr;
	i2c_module_t module;
	i2c_bus_t bus;
	unsigned int address, sequence;
	const char *name;
	io_int_value_t ix;

	assert(call->parameter_count == 0);
	assert(inited);

	string_assign_cstr(call->result, "IO DUMP");

	sequence = 0;

	data_mutex_take();

	for(dataptr = data_root; dataptr; dataptr = dataptr->next)
	{
		string_format_append(call->result, "\n- [%u]", sequence++);

		switch(dataptr->info->bus)
		{
			case(io_bus_i2c):
			{
				i2c_get_slave_info(dataptr->i2c.slave, &module, &bus, &address, &name);

				string_format_append(call->result, " IO %s at I2C:%u/%u/%#x", name, module, bus, address);

				break;
			}

			default:
			{
				string_format_append(call->result, " unknown IO type %u: %s", dataptr->info->bus, dataptr->info->name);
			}
		}

		string_append_cstr(call->result, ", int values:");

		for(ix = io_int_value_first; ix < io_int_value_size; ix++)
			string_format_append(call->result, " %u=%d", ix, dataptr->int_value[ix]);
	}

	data_mutex_give();
}

void command_io_stats(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "IO STATS");
	string_format_append(call->result, "\n- probe skipped: %u", stat_probe_skipped);
	string_format_append(call->result, "\n- probe tried: %u", stat_probe_tried);
	string_format_append(call->result, "\n- probe found: %u", stat_probe_found);
}

static io_data_t *get_data(unsigned int io)
{
	io_data_t *dataptr;
	unsigned int ix;

	data_mutex_take();

	for(ix = 0, dataptr = data_root; (ix < io) && dataptr; ix++, dataptr = dataptr->next)
		(void)0;

	data_mutex_give();

	return(dataptr);
}

static bool io_read_x(string_t result, io_data_t *dataptr, unsigned int pin, unsigned int *value)
{
	assert(dataptr);

	if(!(dataptr->info->caps & (1 << io_cap_input)))
	{
		if(result)
			string_append_cstr(result, "not input capable");
		return(false);
	}

	if(pin >= dataptr->info->pins)
	{
		if(result)
			string_format_append(result, "no such pin %u", pin);
		return(false);
	}

	if(*value > dataptr->info->max_value)
	{
		if(result)
			string_format_append(result, "value %u out of range", *value);
		return(false);
	}

	assert(dataptr->info->read_fn);

	if(!dataptr->info->read_fn(dataptr, pin, value))
	{
		if(result)
			string_append_cstr(result, "read failed");
		return(false);
	}

	return(true);
}

static bool io_write_x(string_t result, io_data_t *dataptr, unsigned int pin, unsigned int value)
{
	assert(dataptr);

	if(!(dataptr->info->caps & (1 << io_cap_output)))
	{
		if(result)
			string_append_cstr(result, "not output capable");
		return(false);
	}

	if(pin >= dataptr->info->pins)
	{
		if(result)
			string_format_append(result, "no such pin %u", pin);
		return(false);
	}

	if(value > dataptr->info->max_value)
	{
		if(result)
			string_format_append(result, "value %u out of range", value);
		return(false);
	}

	assert(dataptr->info->write_fn);

	if(!dataptr->info->write_fn(dataptr, pin, value))
	{
		if(result)
			string_append_cstr(result, "write failed");
		return(false);
	}

	return(true);
}

bool io_read(string_t result, unsigned int io, unsigned int pin, unsigned int *value)
{
	io_data_t *dataptr;

	if(!(dataptr = get_data(io)))
	{
		if(result)
			string_format_append(result, "no such I/O %u", io);
		return(false);
	}

	return(io_read_x(result, dataptr, pin, value));
}

bool io_write(string_t result, unsigned int io, unsigned int pin, unsigned int value)
{
	io_data_t *dataptr;

	if(!(dataptr = get_data(io)))
	{
		if(result)
			string_format_append(result, "no such I/O %u", io);
		return(false);
	}

	return(io_write_x(result, dataptr, pin, value));
}

void command_io_read(cli_command_call_t *call)
{
	unsigned int value;

	assert(call->parameter_count == 2);

	string_assign_cstr(call->result, "io-read: ");

	if(io_read(call->result, call->parameters[0].unsigned_int, call->parameters[1].unsigned_int, &value))
		string_format(call->result, "io-read %u/%u: %u OK", call->parameters[0].unsigned_int, call->parameters[1].unsigned_int, value);
}

void command_io_write(cli_command_call_t *call)
{
	assert(call->parameter_count == 3);

	string_assign_cstr(call->result, "io-write: ");

	if(io_write(call->result, call->parameters[0].unsigned_int, call->parameters[1].unsigned_int, call->parameters[2].unsigned_int))
		string_format(call->result, "io-write %u/%u: %u OK", call->parameters[0].unsigned_int, call->parameters[1].unsigned_int, call->parameters[2].unsigned_int);
}
