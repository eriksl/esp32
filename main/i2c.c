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

#include <ulp_riscv.h>
#include <ulp_riscv_i2c.h>

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
	bool available;
	i2c_module_t id;
	const char *name;
	unsigned int sda;
	unsigned int scl;
	unsigned int speed;
	bool ulp;
} module_info_t;

typedef struct
{
	bool has_mux;
	unsigned int buses;
	unsigned int selected_bus;
	i2c_master_bus_handle_t handle;
	i2c_master_dev_handle_t mux_dev_handle;
	bus_t *bus[i2c_bus_size];
} module_data_t;

static const char *bus_name[i2c_bus_size] =
{
	[i2c_bus_none] = "root bus",
	[i2c_bus_0] = "mux bus 1",
	[i2c_bus_1] = "mux bus 2",
	[i2c_bus_2] = "mux bus 3",
	[i2c_bus_3] = "mux bus 4",
	[i2c_bus_4] = "mux bus 5",
	[i2c_bus_5] = "mux bus 6",
	[i2c_bus_6] = "mux bus 7",
	[i2c_bus_7] = "mux bus 8",
};

static const module_info_t module_info[i2c_module_size] =
{
	[i2c_module_0_fast] =
	{
#if((CONFIG_BSP_I2C0_SDA >= 0) && (CONFIG_BSP_I2C0_SCL >= 0))
		.available = true,
		.id = i2c_module_0_fast,
		.name = "module 0, on main CPU, 400 kHz", // FIXME: make bus speed configurable
		.sda = CONFIG_BSP_I2C0_SDA,
		.scl = CONFIG_BSP_I2C0_SCL,
		.speed = i2c_module_speed_fast,
		.ulp = false,
#else
		.available = false,
		.id = i2c_module_unavailable,
		.name = "module 0 unavailable",
		.sda = -1,
		.scl = -1,
		.speed = i2c_module_speed_none,
		.ulp = false,
#endif
	},

	[i2c_module_1_slow] =
	{
#if((CONFIG_BSP_I2C1_SDA >= 0) && (CONFIG_BSP_I2C1_SCL >= 0))
		.available = true,
		.id = i2c_module_1_slow,
		.name = "module 1, on main CPU, 100 kHz",
		.sda = CONFIG_BSP_I2C1_SDA,
		.scl = CONFIG_BSP_I2C1_SCL,
		.speed = i2c_module_speed_slow,
		.ulp = false,
#else
		.available = false,
		.id = i2c_module_unavailable,
		.name = "module 1 unavailable",
		.sda = -1,
		.scl = -1,
		.speed = i2c_module_speed_none,
		.ulp = false,
#endif
	},

	[i2c_module_2_ulp] =
	{
#if((CONFIG_BSP_I2C2_SDA >= 0) && (CONFIG_BSP_I2C2_SCL >= 0))
		.available = true,
		.id = i2c_module_2_ulp,
		.name = "module 2, on ULP, 400 kHz",
		.sda = CONFIG_BSP_I2C2_SDA,
		.scl = CONFIG_BSP_I2C2_SCL,
		.speed = i2c_module_speed_fast,
		.ulp = true,
#else
		.available = false,
		.id = i2c_module_unavailable,
		.name = "module 2 unavailable",
		.sda = -1,
		.scl = -1,
		.speed = i2c_module_speed_none,
		.ulp = false,
#endif
	},
};

static bool inited = false;
static SemaphoreHandle_t data_mutex;
static SemaphoreHandle_t module_mutex[i2c_module_size];
static module_data_t *module_data;

static inline void data_mutex_take(void)
{
	assert(xSemaphoreTake(data_mutex, portMAX_DELAY));
}

static inline void data_mutex_give(void)
{
	assert(xSemaphoreGive(data_mutex));
}

static inline void module_mutex_take(i2c_module_t module)
{
	assert(xSemaphoreTake(module_mutex[module], portMAX_DELAY));
}

static inline void module_mutex_give(i2c_module_t module)
{
	assert(xSemaphoreGive(module_mutex[module]));
}

_Static_assert(i2c_bus_none == 0);
_Static_assert(i2c_bus_0 == 1);

static bool i2c_send_receive_intern(bool lock, i2c_slave_t slave, unsigned int send_buffer_length, const uint8_t *send_buffer, unsigned int receive_buffer_size, uint8_t *receive_buffer);

static void set_mux(i2c_module_t module, i2c_bus_t bus)
{
	const module_info_t *info;
	module_data_t *data;
	uint8_t buffer_in[1];
	esp_err_t rv;

	assert(inited);
	assert(module < i2c_module_size);
	assert(bus < i2c_bus_size);

	info = &module_info[module];
	data = &module_data[module];

	assert(info->available);

	if(!data->has_mux)
		return;

	if(data->selected_bus == bus)
		return;

	assert(bus < data->buses);
	assert(info->ulp || data->mux_dev_handle);

	if(bus == i2c_bus_none)
		buffer_in[0] = 0;
	else
		buffer_in[0] = (1 << (bus - i2c_bus_0));

	if(info->ulp)
	{
		ulp_riscv_i2c_master_set_slave_addr(i2c_bus_mux_address);
		ulp_riscv_i2c_master_set_slave_reg_addr(buffer_in[0]);
		util_warn_on_esp_err("ulp_riscv_i2c_master_write_to_device", rv = ulp_riscv_i2c_master_write_to_device(buffer_in, sizeof(buffer_in)));
	}
	else
		util_warn_on_esp_err("i2c_master_transmit", rv = i2c_master_transmit(data->mux_dev_handle, buffer_in, sizeof(buffer_in), i2c_timeout_ms));

	if(rv == ESP_OK)
		data->selected_bus = bus;
}

static bool slave_check(slave_t *slave)
{
	const module_info_t *info;
	module_data_t *data;
	bus_t *bus;
	slave_t *search_slave;
	bool rv = false;

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

	info = &module_info[slave->module];
	data = &module_data[slave->module];

	assert(info->available);

	if(!info)
	{
		log("i2c: check slave: module address NULL");
		return(false);
	}

	if(slave->bus >= data->buses)
	{
		log_format("i2c: check slave: bus id in slave struct out of bounds: %u", (unsigned int)slave->bus);
		return(false);
	}

	if(info->id >= i2c_module_size)
	{
		log_format("i2c: check slave: module id out of bounds: %u", (unsigned int)info->id);
		return(false);
	}

	if(!(bus = data->bus[slave->bus]))
	{
		log_format("i2c: check slave: bus unknown %u", slave->bus);
		goto finish;
	}

	if(bus->id >= data->buses)
	{
		log_format("i2c: check slave: bus id out of bounds: %u", (unsigned int)bus->id);
		return(false);
	}

	if(bus->id != slave->bus)
	{
		log_format("i2c: check slave: bus->bus %u != slave->bus %u", slave->bus, bus->id);
		goto finish;
	}

	if(!bus->slaves)
	{
		log_format("i2c: check slave: no slaves on this bus: %u", slave->bus);
		goto finish;
	}

	for(search_slave = bus->slaves; search_slave; search_slave = search_slave->next)
		if(search_slave->address == slave->address)
			goto found;

	log_format("i2c: check slave: slave %#x not found", slave->address);
	goto finish;

found:
	if(search_slave != slave)
	{
		log_format("i2c: check slave: slave address incorrect (2): %p vs %p", search_slave, slave);
		goto finish;
	}

	rv = true;

finish:
	data_mutex_give();
	return(rv);
}

void i2c_init(void)
{
	static const i2c_master_bus_config_t main_i2c_module_config[i2c_module_size] =
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
#if((CONFIG_BSP_I2C2_SDA >= 0) && (CONFIG_BSP_I2C2_SCL >= 0))
	static const ulp_riscv_i2c_cfg_t ulp_i2c_module_config =
	{
		.i2c_pin_cfg =
		{
			.sda_io_num = CONFIG_BSP_I2C2_SDA,
			.scl_io_num = CONFIG_BSP_I2C2_SCL,
			.sda_pullup_en = false,
			.scl_pullup_en = false,
		},
		.i2c_timing_cfg =
		{
			.scl_low_period = 1.4,
			.scl_high_period = 0.3,
			.sda_duty_period = 1,
			.scl_start_period = 2,
			.scl_stop_period = 1.3,
			.i2c_trans_timeout = 20,
		},
	};
#endif

	i2c_module_t module;
	const module_info_t *info;
	module_data_t *data;
	i2c_bus_t bus;
	bus_t *bus_ptr;
	i2c_device_config_t dev_config_mux;
	uint8_t buffer_in[1];
	uint8_t buffer_out[1];

	assert(!inited);

	data_mutex = xSemaphoreCreateMutex();
	assert(data_mutex);

	for(module = i2c_module_first; module < i2c_module_size; module++)
	{
		module_mutex[module] = xSemaphoreCreateMutex();
		assert(module_mutex[module]);
	}

	module_data = util_memory_alloc_spiram(sizeof(*data) * i2c_module_size);
	assert(module_data);

	for(module = i2c_module_first; module < i2c_module_size; module++)
	{
		info = &module_info[module];

		if(!info->available)
			continue;

		module_mutex_take(module);

		data = &module_data[module];

		data->has_mux = false;

#if((CONFIG_BSP_I2C2_SDA >= 0) && (CONFIG_BSP_I2C2_SCL >= 0))
		if(info->ulp)
		{
			data->handle = nullptr;
			data->mux_dev_handle = nullptr;

			util_abort_on_esp_err("ulp_riscv_i2c_master_init", ulp_riscv_i2c_master_init(&ulp_i2c_module_config));
			ulp_riscv_i2c_master_set_slave_addr(i2c_bus_mux_address);
			ulp_riscv_i2c_master_set_slave_reg_addr(0xff);

			if((ulp_riscv_i2c_master_read_from_device(buffer_out, sizeof(buffer_out)) == ESP_OK) &&
					(ulp_riscv_i2c_master_read_from_device(buffer_out, sizeof(buffer_out)) == ESP_OK) &&
					(buffer_out[0] == 0xff))
			{
				ulp_riscv_i2c_master_set_slave_reg_addr(0x00);

				if((ulp_riscv_i2c_master_read_from_device(buffer_out, sizeof(buffer_out)) == ESP_OK) &&
						(ulp_riscv_i2c_master_read_from_device(buffer_out, sizeof(buffer_out)) == ESP_OK) &&
						(buffer_out[0] == 0x00))
					data->has_mux = true;
			}
		}
		else
#endif
		{
			util_abort_on_esp_err("i2c_new_master_bus", i2c_new_master_bus(&main_i2c_module_config[module], &data->handle));

			if(i2c_master_probe(data->handle, i2c_bus_mux_address, i2c_timeout_ms) == ESP_OK)
			{
				dev_config_mux.dev_addr_length = I2C_ADDR_BIT_LEN_7;
				dev_config_mux.device_address = i2c_bus_mux_address;
				dev_config_mux.scl_speed_hz = info->speed;
				dev_config_mux.scl_wait_us = 0;
				dev_config_mux.flags.disable_ack_check = 0;

				util_warn_on_esp_err("i2c master bus add device mux", i2c_master_bus_add_device(data->handle, &dev_config_mux, &data->mux_dev_handle));

				buffer_in[0] = 0xff;

				if((i2c_master_transmit_receive(data->mux_dev_handle, buffer_in, sizeof(buffer_in), buffer_out, sizeof(buffer_out), i2c_timeout_ms) == ESP_OK) &&
						(i2c_master_transmit_receive(data->mux_dev_handle, buffer_in, sizeof(buffer_in), buffer_out, sizeof(buffer_out), i2c_timeout_ms) == ESP_OK) &&
						(buffer_out[0] == 0xff))
				{
					buffer_in[0] = 0x00;

					if((i2c_master_transmit_receive(data->mux_dev_handle, buffer_in, sizeof(buffer_in), buffer_out, sizeof(buffer_out), i2c_timeout_ms) == ESP_OK) &&
							(i2c_master_transmit_receive(data->mux_dev_handle, buffer_in, sizeof(buffer_in), buffer_out, sizeof(buffer_out), i2c_timeout_ms) == ESP_OK) &&
							(buffer_out[0] == 0x00))
						data->has_mux = true;
				}

				if(!data->has_mux)
					i2c_master_bus_rm_device(data->mux_dev_handle);
			}

			if(!data->has_mux)
				data->mux_dev_handle = nullptr;
		}

		data->selected_bus = i2c_bus_invalid;
		data->buses = data->has_mux ? i2c_bus_size : 1;

		for(bus = i2c_bus_first; bus < i2c_bus_size; bus++)
			data->bus[bus] = nullptr;

		for(bus = i2c_bus_first; bus < data->buses; bus++)
		{
			bus_ptr = util_memory_alloc_spiram(sizeof(*bus_ptr));
			assert(bus_ptr);
			bus_ptr->id = bus;
			bus_ptr->slaves = nullptr;
			data->bus[bus] = bus_ptr;
		}

		module_mutex_give(module);
	}

	inited = true;
}

i2c_slave_t i2c_register_slave(const char *name, i2c_module_t module, i2c_bus_t bus, unsigned int address)
{
	slave_t *new_slave, *slave_ptr;
	i2c_device_config_t dev_config;
	i2c_master_dev_handle_t dev_handle;
	const module_info_t *info;
	module_data_t *data;
	bus_t *bus_ptr;
	unsigned int rv;

	assert(inited);
	assert(module < i2c_module_size);
	assert(bus < i2c_bus_size);
	assert(address < 128);

	new_slave = nullptr;

	data_mutex_take();

	info = &module_info[module];
	data = &module_data[module];

	assert(bus < data->buses);
	assert(info->available);

	if(info->ulp)
		dev_handle = nullptr;
	else
	{
		dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
		dev_config.device_address = address;
		dev_config.scl_speed_hz = info->speed;
		dev_config.scl_wait_us = 0;
		dev_config.flags.disable_ack_check = 0;

		module_mutex_take(module);
		rv = i2c_master_bus_add_device(data->handle, &dev_config, &dev_handle);
		module_mutex_give(module);

		if(rv != ESP_OK)
		{
			util_warn_on_esp_err("i2c_master_bus_add_device", rv);
			goto error;
		}
	}

	new_slave = util_memory_alloc_spiram(sizeof(*new_slave));
	new_slave->name = name;
	new_slave->handle = dev_handle;
	new_slave->module = module;
	new_slave->bus = bus;
	new_slave->address = address;
	new_slave->next = nullptr;

	if(!(bus_ptr = data->bus[bus]))
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
		log_format("failed to register slave %u/%u/%#x:%s", new_slave->module, new_slave->bus, new_slave->address, new_slave->name);
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
	const module_info_t *info;
	module_data_t *data;
	bus_t *bus;
	slave_t *slaveptr;
	bool error;

	error = true;

	_slave = (slave_t **)slave;

	assert(_slave);
	assert(*_slave);
	assert((**_slave).module < i2c_module_size);
	assert((**_slave).bus < i2c_bus_size);

	data_mutex_take();

	info = &module_info[(**_slave).module];
	data = &module_data[(**_slave).module];

	assert(info);
	assert(info->available);
	assert(info->id == (**_slave).module);
	assert((**_slave).bus < data->buses);

	if(info->ulp)
		assert(!(**_slave).handle);
	else
	{
		module_mutex_take((**_slave).module);
		util_abort_on_esp_err("i2c_master_bus_rm_device", i2c_master_bus_rm_device((**_slave).handle));
		module_mutex_give((**_slave).module);
	}

	if(!(bus = data->bus[(**_slave).bus]))
	{
		log_format("i2c unregister slave: bus unknown %u", (**_slave).bus);
		goto finish;
	}

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

	for(slaveptr = bus->slaves; slaveptr->next; slaveptr = slaveptr->next)
		if(slaveptr->next->address == (**_slave).address)
			goto found;

	log_format("i2c unregister slave: slave %#x not found", (**_slave).address);
	goto finish;

found:
	if(slaveptr->next != *_slave)
	{
		log_format("i2c unregister slave: slave address incorrect (2): %p vs %p", slaveptr->next, *_slave);
		goto finish;
	}

	slaveptr->next = (**_slave).next;
	error = false;

finish:
	data_mutex_give();

	free(*_slave);
	*_slave = (slave_t *)0;

	return(error);
}

bool i2c_probe_slave(i2c_module_t module, i2c_bus_t bus, unsigned int address, unsigned int probe_write_value, const char *probe_name)
{
	const module_info_t *info;
	module_data_t *data;
	uint8_t buffer_in[1];
	uint8_t buffer_out[1];
	bool success;
	i2c_slave_t slave;

	assert(inited);
	assert(module < i2c_module_size);
	assert(bus < i2c_bus_size);
	assert(address < 128);

	module_mutex_take(module);

	info = &module_info[module];
	data = &module_data[module];

	assert(info->available);
	assert(bus < data->buses);

	if(info->ulp)
	{
		success = false;

		// skip the probe if the slave cannot handle a one byte write
		if(probe_write_value == i2c_probe_no_write)
			success = true;
		else
		{
			if((slave = i2c_register_slave(probe_name, module, bus, address)))
			{
				set_mux(module, bus);
				buffer_in[0] = probe_write_value;
				success = i2c_send_receive_intern(false, slave, sizeof(buffer_in), buffer_in, sizeof(buffer_out), buffer_out);
				i2c_unregister_slave(&slave);
			}
		}
	}
	else
	{
		set_mux(module, bus);
		success = i2c_master_probe(data->handle, address, i2c_timeout_ms) == ESP_OK;
	}

	module_mutex_give(module);

	return(success);
}

bool i2c_send(i2c_slave_t slave, unsigned int send_buffer_length, const uint8_t *send_buffer)
{
	uint8_t receive_buffer[1];
	slave_t *_slave = (slave_t *)slave;
	const module_info_t *info;
	esp_err_t rv;

	assert(inited);
	assert(_slave);
	assert(send_buffer);

	info = &module_info[_slave->module];

	assert(info->available);

	if(send_buffer_length < 1)
	{
		log("i2c: i2c_send called with zero length data");
		return(false);
	}

	if(!slave_check(_slave))
	{
		log("i2c_send: slave_check failed");
		return(false);
	}

	module_mutex_take(_slave->module);
	set_mux(_slave->module, _slave->bus);

	if(info->ulp)
	{
		ulp_riscv_i2c_master_set_slave_addr(_slave->address);
		ulp_riscv_i2c_master_set_slave_reg_addr(send_buffer[0]);

		if(send_buffer_length < 2)
			rv = ulp_riscv_i2c_master_read_from_device(receive_buffer, sizeof(receive_buffer));
		else
			rv = ulp_riscv_i2c_master_write_to_device(send_buffer + 1, send_buffer_length - 1);

		if(rv != ESP_OK)
			util_warn_on_esp_err("ulp_riscv_i2c_master_write_to_device", rv);

	}
	else
		util_warn_on_esp_err("i2c_master_transmit", rv = i2c_master_transmit(_slave->handle, send_buffer, send_buffer_length, i2c_timeout_ms));

	module_mutex_give(_slave->module);

	return(rv == ESP_OK);
}

bool i2c_send_1(i2c_slave_t slave, unsigned int byte)
{
	uint8_t buffer[1];

	buffer[0] = byte;

	return(i2c_send(slave, sizeof(buffer), buffer));
}

bool i2c_send_2(i2c_slave_t slave, unsigned int byte_1, unsigned int byte_2)
{
	uint8_t buffer[2];

	buffer[0] = byte_1;
	buffer[1] = byte_2;

	return(i2c_send(slave, sizeof(buffer), buffer));
}

bool i2c_send_3(i2c_slave_t slave, unsigned int byte_1, unsigned int byte_2, unsigned int byte_3)
{
	uint8_t buffer[3];

	buffer[0] = byte_1;
	buffer[1] = byte_2;
	buffer[2] = byte_3;

	return(i2c_send(slave, sizeof(buffer), buffer));
}

bool i2c_receive(i2c_slave_t slave, unsigned int receive_buffer_size, uint8_t *receive_buffer)
{
	slave_t *_slave = (slave_t *)slave;
	const module_info_t *info;
	unsigned int rv;

	assert(inited);
	assert(_slave);
	assert(receive_buffer);

	info = &module_info[_slave->module];

	assert(info->available);

	if(info->ulp)
	{
		log("i2c: i2c_receive called for ULP I2C module");
		return(false);
	}

	if(receive_buffer_size == 0)
	{
		log("i2c: i2c_receive called with zero receive buffer size");
		return(true);
	}

	if(!slave_check(_slave))
		return(false);

	module_mutex_take(_slave->module);
	set_mux(_slave->module, _slave->bus);
	util_warn_on_esp_err("i2c_master_receive", rv = i2c_master_receive(_slave->handle, receive_buffer, receive_buffer_size, i2c_timeout_ms));
	module_mutex_give(_slave->module);

	return(rv == ESP_OK);
}

static bool i2c_send_receive_intern(bool lock, i2c_slave_t slave, unsigned int send_buffer_length, const uint8_t *send_buffer, unsigned int receive_buffer_size, uint8_t *receive_buffer)
{
	slave_t *_slave = (slave_t *)slave;
	const module_info_t *info;
	unsigned int rv;

	assert(inited);
	assert(_slave);
	assert(send_buffer);
	assert(receive_buffer);

	info = &module_info[_slave->module];

	assert(info->available);

	if((send_buffer_length == 0) || (receive_buffer_size == 0))
	{
		log("i2c: i2c_send_receive called with zero receive buffer size or zero send buffer size");
		return(true);
	}

	if(!slave_check(_slave))
		return(false);

	if(lock)
		module_mutex_take(_slave->module);

	set_mux(_slave->module, _slave->bus);

	if(info->ulp)
	{
		if(send_buffer_length != 1)
		{
			log("i2c: i2c_send_receive: send buffer length should be 1 when using ULP I2C");
			rv = ESP_ERR_NOT_FOUND;
			goto error;
		}

		ulp_riscv_i2c_master_set_slave_addr(_slave->address);
		ulp_riscv_i2c_master_set_slave_reg_addr(send_buffer[0]);
		rv = ulp_riscv_i2c_master_read_from_device(receive_buffer, receive_buffer_size);
	}
	else
		util_warn_on_esp_err("i2c_master_transmit_receive", rv =
				i2c_master_transmit_receive(_slave->handle, send_buffer, send_buffer_length, receive_buffer, receive_buffer_size, i2c_timeout_ms));

error:
	if(lock)
		module_mutex_give(_slave->module);

	return(rv == ESP_OK);
}

bool i2c_send_receive(i2c_slave_t slave, unsigned int send_buffer_length, const uint8_t *send_buffer, unsigned int receive_buffer_size, uint8_t *receive_buffer)
{
	return(i2c_send_receive_intern(true, slave, send_buffer_length, send_buffer, receive_buffer_size, receive_buffer));
}

bool i2c_send_1_receive(i2c_slave_t slave, unsigned int byte, unsigned int receive_buffer_size, uint8_t *receive_buffer)
{
	uint8_t send_buffer[1];

	send_buffer[0] = byte;

	return(i2c_send_receive_intern(true, slave, sizeof(send_buffer), send_buffer, receive_buffer_size, receive_buffer));
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
	module_data_t *data;
	bus_t *busptr;
	slave_t *slaveptr;

	data_mutex_take();

	assert(module < i2c_module_size);
	assert(i2c_module_available(module));

	data = &module_data[module];
	assert(data);

	for(bus_index = i2c_bus_first; bus_index < data->buses; bus_index++) // FIXME check
	{
		if(!(busptr = data->bus[bus_index]))
			continue;

		for(slaveptr = busptr->slaves; slaveptr; slaveptr = slaveptr->next)
			if((address == slaveptr->address) && ((bus == i2c_bus_none) || (slaveptr->bus == i2c_bus_none) || (bus == slaveptr->bus)))
				goto found;
	}

	slaveptr = nullptr;

found:
	data_mutex_give();
	return((i2c_slave_t)slaveptr);
}

bool i2c_module_available(i2c_module_t module)
{
	return(module_info[module].available);
}

unsigned int i2c_buses(i2c_module_t module)
{
	assert(module < i2c_module_size);

	return(module_data[module].buses);
}

bool i2c_ulp(i2c_module_t module)
{
	const module_info_t *info;

	assert(module < i2c_module_size);
	info = &module_info[module];

	assert(info->available);

	return(info->ulp);
}

bool i2c_slave_ulp(i2c_slave_t slave)
{
	slave_t *_slave = (slave_t *)slave;

	return(i2c_ulp(_slave->module));
}

void command_i2c_info(cli_command_call_t *call)
{
	i2c_module_t module_index;
	i2c_bus_t bus_index;
	const module_info_t *info;
	module_data_t *data;
	bus_t *bus;
	slave_t *slave;

	data_mutex_take();

	string_format(call->result, "I2C info");

	for(module_index = i2c_module_first; module_index < i2c_module_size; module_index++)
	{
		info = &module_info[module_index];

		if(info->available)
		{
			data = &module_data[module_index];

			string_format_append(call->result, "\n- module [%u]: \"%s\", sda=%u, scl=%u, speed=%u khz", info->id, info->name, info->sda, info->scl, info->speed / 1000);

			for(bus_index = i2c_bus_first; bus_index < data->buses; bus_index++)
			{
				if((bus = data->bus[bus_index]))
				{
					string_format_append(call->result, "\n-  i2c bus %u: %s", bus->id, bus_name[bus->id]);

					for(slave = bus->slaves; slave; slave = slave->next)
						string_format_append(call->result, "\n-   slave [0x%x]: name: %s, module: %u, bus: %u, handle: %p",
								slave->address, slave->name, slave->module, slave->bus, slave->handle);
				}
			}
		}
		else
			string_format_append(call->result, "\n- module [%u]: unavailable", info->id);
	}

	data_mutex_give();
}
