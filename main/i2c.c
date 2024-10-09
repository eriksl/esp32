#include <stdint.h>
#include <stdbool.h>

#include "i2c.h"
#include "string.h"
#include "info.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"

#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>

enum
{
	i2c_timeout_ms = 1000,
	i2c_bus_mux_address = 0x70,
};

typedef struct slave_T
{
	const char *name;
	i2c_master_dev_handle_t handle;
	i2c_module_t module;
	i2c_bus_t bus;
	unsigned int address;
	struct slave_T *next;
} slave_t;

typedef struct
{
	i2c_bus_t id;
	slave_t *slaves;
} bus_t;

typedef struct
{
	i2c_module_t id;
	const char *name;
	unsigned int sda;
	unsigned int scl;
	unsigned int speed;
	i2c_master_bus_handle_t handle;
	i2c_master_dev_handle_t mux_dev_handle;
	bus_t *bus[i2c_bus_size];
} module_t;

static const char *bus_name[i2c_bus_size] =
{
	[i2c_bus_none] = "none",
	[i2c_bus_0] = "bus 0",
	[i2c_bus_1] = "bus 1",
	[i2c_bus_2] = "bus 2",
	[i2c_bus_3] = "bus 3",
	[i2c_bus_4] = "bus 4",
	[i2c_bus_5] = "bus 5",
	[i2c_bus_6] = "bus 6",
	[i2c_bus_7] = "bus 7",
};

static module_t module_data[i2c_module_size] =
{
	[i2c_module_0_fast] =
	{
		.id = i2c_module_0_fast,
		.name = "module 0, fast",
		.sda = CONFIG_BSP_I2C0_SDA,
		.scl = CONFIG_BSP_I2C0_SCL,
		.speed = i2c_module_speed_fast,
		.handle = (i2c_master_bus_handle_t)0,
		.bus = { (bus_t *)0, },
	},
	[i2c_module_1_slow] =
	{
		.id = i2c_module_1_slow,
		.name = "module 1, slow",
		.sda = CONFIG_BSP_I2C1_SDA,
		.scl = CONFIG_BSP_I2C1_SCL,
		.speed = i2c_module_speed_slow,
		.handle = (i2c_master_bus_handle_t)0,
		.bus = { (bus_t *)0, },
	},
};

static bool inited = false;
static SemaphoreHandle_t data_mutex;

static inline void data_mutex_take(void)
{
	assert(xSemaphoreTake(data_mutex, portMAX_DELAY));
}

static inline void data_mutex_give(void)
{
	assert(xSemaphoreGive(data_mutex));
}

_Static_assert(i2c_bus_none == 0);
_Static_assert(i2c_bus_0 == 1);

static void set_mux(i2c_module_t module, i2c_bus_t bus)
{
	module_t *module_ptr;
	uint8_t reg[1];

	assert(module < i2c_module_size);
	assert(bus < i2c_bus_size);

	module_ptr = &module_data[module];

	if(!module_ptr->mux_dev_handle)
		return;

	if(bus == i2c_bus_none)
		reg[0] = 0;
	else
		reg[0] = (1 << (bus - i2c_bus_0));

	util_warn_on_esp_err("i2c_master_transmit mux", i2c_master_transmit(module_ptr->mux_dev_handle, reg, 1, i2c_timeout_ms));
}

static bool slave_check(slave_t *slave)
{
	module_t *module;
	bus_t *bus;
	slave_t *next;
	bool found;

	if(!slave)
	{
		log("i2c: check slave: slave address NULL");
		return(false);
	}

	data_mutex_take();

	if(slave->module >= i2c_module_size)
	{
		log_format("i2c: check slave: module id in slave struct out of bounds: %u", (unsigned int)slave->module);
		return(false);
	}

	if(slave->bus >= i2c_bus_size)
	{
		log_format("i2c: check slave: bus id in slave struct out of bounds: %u", (unsigned int)slave->bus);
		return(false);
	}

	module = &module_data[slave->module];

	if(!module)
	{
		log("i2c: check slave: module address NULL");
		return(false);
	}

	if(module->id >= i2c_module_size)
	{
		log_format("i2c: check slave: module id out of bounds: %u", (unsigned int)module->id);
		return(false);
	}

	if(!(bus = module->bus[slave->bus]))
	{
		log_format("i2c: check slave: bus unknown %u", slave->bus);
		goto error;
	}

	if(bus->id >= i2c_bus_size)
	{
		log_format("i2c: check slave: bus id out of bounds: %u", (unsigned int)bus->id);
		return(false);
	}

	if(bus->id != slave->bus)
	{
		log_format("i2c: check slave: bus->bus %u != slave->bus %u", slave->bus, bus->id);
		goto error;
	}

	if(!bus->slaves)
	{
		log_format("i2c: check slave: no slaves on this bus: %u", slave->bus);
		goto error;
	}

	if(bus->slaves->address == slave->address)
	{
		if(bus->slaves != slave)
		{
			log_format("i2c: check slave: slave address incorrect (1): %p vs %p", bus->slaves, slave);
			goto error;
		}

		goto finish;
	}

	for(next = bus->slaves->next, found = false; next->next; next = next->next)
	{
		if(next->next->address == slave->address)
		{
			found = true;
			break;
		}
	}

	if(!found)
	{
		log_format("i2c: check slave: slave %u not found", slave->address);
		goto error;
	}

	if(next->next != slave)
	{
		log_format("i2c: check slave: slave address incorrect (2): %p vs %p", next->next, slave);
		goto error;
	}

finish:
	data_mutex_give();
	return(true);

error:
	data_mutex_give();
	return(false);
}

void i2c_init(void)
{
	static const i2c_master_bus_config_t module_config[i2c_module_size] =
	{
		[i2c_module_0_fast] =
		{
			.i2c_port = 0,
			.sda_io_num = CONFIG_BSP_I2C0_SDA,
			.scl_io_num = CONFIG_BSP_I2C0_SCL,
			.clk_source = I2C_CLK_SRC_DEFAULT,
			.glitch_ignore_cnt = 7,
			.intr_priority = 0,
			.trans_queue_depth = 0,
			.flags =
			{
				.enable_internal_pullup = 1, // this is not necessary but suppresses a spurious warning
			},
		},
		[i2c_module_1_slow] =
		{
			.i2c_port = 1,
			.sda_io_num = CONFIG_BSP_I2C1_SDA,
			.scl_io_num = CONFIG_BSP_I2C1_SCL,
			.clk_source = I2C_CLK_SRC_DEFAULT,
			.glitch_ignore_cnt = 7,
			.intr_priority = 0,
			.trans_queue_depth = 0,
			.flags =
			{
				.enable_internal_pullup = 1, // this is not necessary but suppresses a spurious warning
			},
		},
	};
	i2c_module_t module;
	module_t *module_ptr;
	i2c_bus_t bus;
	bus_t *bus_ptr;
	unsigned int rv, buses;
	i2c_device_config_t dev_config_mux;

	assert(!inited);

	data_mutex = xSemaphoreCreateMutex();
	assert(data_mutex);

	for(module = i2c_module_first; module < i2c_module_size; module++)
	{
		module_ptr = &module_data[module];

		util_abort_on_esp_err("i2c_new_master_bus", i2c_new_master_bus(&module_config[module], &module_ptr->handle));

		rv = i2c_master_probe(module_ptr->handle, i2c_bus_mux_address, i2c_timeout_ms);

		if(rv != ESP_ERR_NOT_FOUND)
			util_warn_on_esp_err("i2c_master_probe mux", rv);

		if(rv == ESP_OK)
		{
			dev_config_mux.dev_addr_length = I2C_ADDR_BIT_LEN_7;
			dev_config_mux.device_address = i2c_bus_mux_address;
			dev_config_mux.scl_speed_hz = module_ptr->speed;
			dev_config_mux.scl_wait_us = 0;
			dev_config_mux.flags.disable_ack_check = 0;

			buses = i2c_bus_size;
			util_warn_on_esp_err("i2c master bus add device mux", i2c_master_bus_add_device(module_ptr->handle, &dev_config_mux, &module_ptr->mux_dev_handle));
		}
		else
		{
			buses = 1;
			module_ptr->mux_dev_handle = (i2c_master_dev_handle_t)0;
		}

		for(bus = i2c_bus_first; bus < buses; bus++)
		{
			bus_ptr = (bus_t *)util_memory_alloc_spiram(sizeof(*bus_ptr));
			assert(bus_ptr);
			bus_ptr->id = bus;
			bus_ptr->slaves = (slave_t *)0;
			module_ptr->bus[bus] = bus_ptr;
		}

		for(; bus < i2c_bus_size; bus++)
			module_ptr->bus[bus] = (bus_t *)0;
	}

	inited = true;
}

i2c_slave_t i2c_register_slave(const char *name, i2c_module_t module, i2c_bus_t bus, unsigned int address)
{
	slave_t *new_slave, *slave_ptr;
	i2c_device_config_t dev_config;
	i2c_master_dev_handle_t dev_handle;
	module_t *module_ptr;
	bus_t *bus_ptr;
	unsigned int rv;

	assert(inited);
	assert(module < i2c_module_size);
	assert(bus < i2c_bus_size);
	assert(address < 128);

	new_slave = (slave_t *)0;

	data_mutex_take();

	module_ptr = &module_data[module];

	dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
	dev_config.device_address = address;
	dev_config.scl_speed_hz = module_ptr->speed;
	dev_config.scl_wait_us = 0;
	dev_config.flags.disable_ack_check = 0;

	rv = i2c_master_bus_add_device(module_ptr->handle, &dev_config, &dev_handle);

	if(rv != ESP_OK)
	{
		util_warn_on_esp_err("i2c_master_bus_add_device", rv);
		goto error;
	}

	new_slave = (slave_t *)util_memory_alloc_spiram(sizeof(*new_slave));
	new_slave->name = name;
	new_slave->handle = dev_handle;
	new_slave->module = module;
	new_slave->bus = bus;
	new_slave->address = address;
	new_slave->next = (slave_t *)0;

	if(!(bus_ptr = module_ptr->bus[bus]))
	{
		log_format("i2c register slave: bus %u doesn't exist", bus);
		goto error;
	}

	if(!(slave_ptr = bus_ptr->slaves))
		bus_ptr->slaves = new_slave;
	else
	{
		for(; slave_ptr->next; slave_ptr = slave_ptr->next)
			(void)0;

		assert(slave_ptr);
		assert(!slave_ptr->next);

		slave_ptr->next = new_slave;
	}

	data_mutex_give();

	if(!slave_check(new_slave))
	{
		log_format("failed to register slave %u/%u/%u:%s", new_slave->module, new_slave->bus, new_slave->address, new_slave->name);
		i2c_unregister_slave((i2c_slave_t *)&new_slave);
		goto finish;
	}

	goto finish;

error:
	data_mutex_give();
finish:
	return((i2c_slave_t)new_slave);
}

bool i2c_unregister_slave(i2c_slave_t *slave)
{
	slave_t **_slave;
	module_t *module;
	bus_t *bus;
	slave_t *next;
	bool found;
	bool error;

	error = true;

	_slave = (slave_t **)slave;

	assert(_slave);
	assert(*_slave);
	assert((**_slave).module < i2c_module_size);
	assert((**_slave).bus < i2c_bus_size);

	util_abort_on_esp_err("i2c_master_bus_rm_device", i2c_master_bus_rm_device((**_slave).handle));

	data_mutex_take();

	module = &module_data[(**_slave).module];

	assert(module);
	assert(module->id < i2c_module_size);

	if(!(bus = module->bus[(**_slave).bus]))
	{
		log_format("i2c unregister slave: bus unknown %u", (**_slave).bus);
		goto finish;
	}

	assert(bus->id < i2c_bus_size);

	if(bus->id != (**_slave).bus)
	{
		log_format("i2c unregister slave: bus->bus %u != slave->bus %u", (**_slave).bus, bus->id);
		goto finish;
	}

	if(!bus->slaves)
	{
		log_format("i2c: unregister slave: no slaves on this bus: %u", (**_slave).bus);
		goto finish;
	}

	if(bus->slaves->address == (**_slave).address)
	{
		if(bus->slaves != *_slave)
		{
			log_format("i2c unregister slave: slave address incorrect (1): %p vs %p", bus->slaves, *_slave);
			goto finish;
		}

		bus->slaves = (**_slave).next;
		error = false;
		goto finish;
	}

	for(next = bus->slaves->next, found = false; next->next; next = next->next)
	{
		if(next->next->address == (**_slave).address)
		{
			found = true;
			break;
		}
	}

	if(!found)
	{
		log_format("i2c unregister slave: slave %u not found", (**_slave).address);
		goto finish;
	}

	if(next->next != *_slave)
	{
		log_format("i2c unregister slave: slave address incorrect (2): %p vs %p", next->next, *_slave);
		goto finish;
	}

	next->next = (**_slave).next;
	error = false;

finish:
	data_mutex_give();

	free(*_slave);
	*_slave = (slave_t *)0;

	return(error);
}

bool i2c_probe_slave(i2c_module_t module, i2c_bus_t bus, unsigned int address)
{
	unsigned int rv;

	assert(inited);
	assert(module < i2c_module_size);
	assert(bus < i2c_bus_size);
	assert(address < 128);

	set_mux(module, bus);

	rv = i2c_master_probe(module_data[module].handle, address, i2c_timeout_ms);

	if(rv != ESP_ERR_NOT_FOUND)
		util_warn_on_esp_err("i2c_master_probe", rv);

	return(rv == ESP_OK);
}

bool i2c_send(i2c_slave_t slave, unsigned int send_buffer_length, const uint8_t *send_buffer)
{
	slave_t *_slave = (slave_t *)slave;
	unsigned int rv;

	assert(inited);
	assert(_slave);

	if(send_buffer_length == 0)
		return(true);

	assert(send_buffer);

	if(!slave_check(_slave))
		return(false);

	set_mux(_slave->module, _slave->bus);

	util_warn_on_esp_err("i2c_master_transmit", rv = i2c_master_transmit(_slave->handle, send_buffer, send_buffer_length, i2c_timeout_ms));

	return(rv == ESP_OK);
}

bool i2c_send_1(i2c_slave_t slave, unsigned int byte)
{
	uint8_t buffer[1] = { byte };

	return(i2c_send(slave, 1, buffer));
}

bool i2c_receive(i2c_slave_t slave, unsigned int receive_buffer_size, uint8_t *receive_buffer)
{
	slave_t *_slave = (slave_t *)slave;
	unsigned int rv;

	assert(inited);
	assert(_slave);

	if(receive_buffer_size == 0)
		return(true);

	assert(receive_buffer);

	if(!slave_check(_slave))
		return(false);

	set_mux(_slave->module, _slave->bus);

	util_warn_on_esp_err("i2c_master_receive", rv = i2c_master_receive(_slave->handle, receive_buffer, receive_buffer_size, i2c_timeout_ms));

	return(rv == ESP_OK);
}

bool i2c_get_slave_info(i2c_slave_t slave, i2c_module_t *module, i2c_bus_t *bus, unsigned int *address, const char **name)
{
	slave_t *_slave = (slave_t *)slave;

	if(!slave_check(_slave))
		return(false);

	if(module)
		*module = _slave->module;

	if(bus)
		*bus = _slave->bus;

	if(address)
		*address = _slave->address;

	if(name)
		*name = _slave->name;

	return(true);
}

i2c_slave_t i2c_find_slave(i2c_module_t module, i2c_bus_t bus, unsigned int address)
{
	i2c_bus_t bus_index;
	module_t *moduleptr;
	bus_t *busptr;
	slave_t *slaveptr;

	data_mutex_take();

	assert(module < i2c_module_size);

	moduleptr = &module_data[module];
	assert(moduleptr);

	if(moduleptr->mux_dev_handle)
	{
		for(bus_index = i2c_bus_first; bus_index < i2c_bus_size; bus_index++)
		{
			if(!(busptr = moduleptr->bus[bus_index]))
				continue;

			for(slaveptr = busptr->slaves; slaveptr; slaveptr = slaveptr->next)
				if((address == slaveptr->address) && ((bus == i2c_bus_none) || (slaveptr->bus == i2c_bus_none) || (bus == slaveptr->bus)))
					goto found;
		}
	}
	else
	{
		busptr = moduleptr->bus[i2c_bus_none];
		assert(busptr);

		for(slaveptr = busptr->slaves; slaveptr; slaveptr = slaveptr->next)
			if(address == slaveptr->address)
				goto found;
	}

	slaveptr = (slave_t *)0;

found:
	data_mutex_give();
	return((i2c_slave_t)slaveptr);
}

unsigned int i2c_buses(i2c_module_t module)
{
	assert(module < i2c_module_size);

	return(module_data[module].mux_dev_handle ? i2c_bus_size : (i2c_bus_none + 1));
}

void command_i2c_info(cli_command_call_t *call)
{
	i2c_module_t module_index;
	i2c_bus_t bus_index;
	module_t *module;
	bus_t *bus;
	slave_t *slave;

	data_mutex_take();

	string_format(call->result, "I2C info");

	for(module_index = i2c_module_first; module_index < i2c_module_size; module_index++)
	{
		module = &module_data[module_index];

		string_format_append(call->result, "\n- module [%u]: \"%s\", sda=%u, scl=%u, speed=%u khz", module->id, module->name, module->sda, module->scl, module->speed / 1000);

		for(bus_index = i2c_bus_first; bus_index < i2c_bus_size; bus_index++)
		{
			if((bus = module->bus[bus_index]))
			{
				string_format_append(call->result, "\n-  i2c bus %u: mux %s", bus->id, bus_name[bus->id]);

				for(slave = bus->slaves; slave; slave = slave->next)
					string_format_append(call->result, "\n-   slave [0x%x]: name: %s, module: %u, bus: %u, handle: %p",
							slave->address, slave->name, slave->module, slave->bus, slave->handle);
			}
		}
	}

	data_mutex_give();
}
