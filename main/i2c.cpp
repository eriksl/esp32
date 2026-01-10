#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "string.h"
#include "info.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "config.h"
#include "i2c.h"

#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>

#include <ulp_riscv.h>
#include <ulp_riscv_i2c.h>

enum
{
	i2c_address_shift = 1,
	i2c_write_flag = 0x00,
	i2c_read_flag = 0x01,

	i2c_timeout_ms = 1000,
	i2c_bus_mux_address = 0x70,
};

typedef struct slave_T
{
	const char *name;
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
	bool ulp;
} module_info_t;

typedef struct
{
	bool has_mux;
	unsigned int buses;
	unsigned int selected_bus;
	unsigned int speed_khz;
	i2c_master_bus_handle_t bus_handle;
	i2c_master_dev_handle_t device_handle;
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
	[i2c_module_0] =
	{
#if((CONFIG_BSP_I2C0_SDA >= 0) && (CONFIG_BSP_I2C0_SCL >= 0))
		.available = true,
		.id = i2c_module_0,
		.name = "module 0, on main CPU",
		.sda = CONFIG_BSP_I2C0_SDA,
		.scl = CONFIG_BSP_I2C0_SCL,
		.ulp = false,
#else
		.available = false,
		.id = i2c_module_unavailable,
		.name = "module 0 unavailable",
		.sda = -1,
		.scl = -1,
		.ulp = false,
#endif
	},

	[i2c_module_1] =
	{
#if((CONFIG_BSP_I2C1_SDA >= 0) && (CONFIG_BSP_I2C1_SCL >= 0))
		.available = true,
		.id = i2c_module_1,
		.name = "module 1, on main CPU",
		.sda = CONFIG_BSP_I2C1_SDA,
		.scl = CONFIG_BSP_I2C1_SCL,
		.ulp = false,
#else
		.available = false,
		.id = i2c_module_unavailable,
		.name = "module 1 unavailable",
		.sda = -1,
		.scl = -1,
		.ulp = false,
#endif
	},

	[i2c_module_2_ulp] =
	{
#if((CONFIG_BSP_I2C2_SDA >= 0) && (CONFIG_BSP_I2C2_SCL >= 0))
		.available = true,
		.id = i2c_module_2_ulp,
		.name = "module 2, on ULP",
		.sda = CONFIG_BSP_I2C2_SDA,
		.scl = CONFIG_BSP_I2C2_SCL,
		.ulp = true,
#else
		.available = false,
		.id = i2c_module_unavailable,
		.name = "module 2 unavailable",
		.sda = -1,
		.scl = -1,
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

static_assert(i2c_bus_none == 0);
static_assert(i2c_bus_0 == 1);

static bool slave_check(slave_t *slave) // FIXME
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
		log_format("i2c: check slave: module id in slave struct out of bounds: %d", slave->module);
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
		log_format("i2c: check slave: bus id in slave struct out of bounds: %d", slave->bus);
		return(false);
	}

	if(info->id >= i2c_module_size)
	{
		log_format("i2c: check slave: module id out of bounds: %d", info->id);
		return(false);
	}

	if(!(bus = data->bus[slave->bus]))
	{
		log_format("i2c: check slave: bus unknown %d", slave->bus);
		goto finish;
	}

	if(bus->id >= data->buses)
	{
		log_format("i2c: check slave: bus id out of bounds: %d", bus->id);
		return(false);
	}

	if(bus->id != slave->bus)
	{
		log_format("i2c: check slave: bus->bus %d != slave->bus %d", slave->bus, bus->id);
		goto finish;
	}

	if(!bus->slaves)
	{
		log_format("i2c: check slave: no slaves on this bus: %d", slave->bus);
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

//
// ULP controller implementation
//

static bool ll_ulp_send(unsigned int address, unsigned int size, const uint8_t *data, bool verbose)
{
	esp_err_t rv;
	uint8_t dummy[1];

	if(size == 0)
	{
		if(verbose)
			log_format("ll ulp send: address: 0x%02x: cannot send 0 bytes using ULP I2C", address);

		return(false);
	}

	ulp_riscv_i2c_master_set_slave_addr(address);
	ulp_riscv_i2c_master_set_slave_reg_addr(data[0]);

	if(size > 1)
	{
		rv = ulp_riscv_i2c_master_write_to_device(&data[1], size - 1);

		if(verbose && (rv != ESP_OK))
			util_warn_on_esp_err("ll ulp send: ulp_riscv_i2c_master_write_to_device", rv);
	}
	else
	{
		// workaround for writing only one byte, which the ULP I2C can't do
		// instead write one byte, then read one (dummy) byte, which it can do

		if(verbose)
			log_format("ll ulp send: address: 0x%02x, emulating one byte write by a write/read cycle", address);

		rv = ulp_riscv_i2c_master_read_from_device(dummy, sizeof(dummy));

		if(verbose && (rv != ESP_OK))
			util_warn_on_esp_err("ll ulp send: ulp_riscv_i2c_master_read_from_device", rv);
	}

	return(rv == ESP_OK);
}

static bool ll_ulp_receive(unsigned int address, unsigned int receive_buffer_size, uint8_t *receive_buffer, bool verbose)
{
	if(verbose)
		log_format("ll ulp receive: address 0x%02x: ULP I2C does not support reading without writing", address);

	return(false);
}

static bool ll_ulp_send_receive(unsigned int address, unsigned int send_buffer_length, const uint8_t *send_buffer,
		unsigned int receive_buffer_size, uint8_t *receive_buffer, bool verbose)
{
	esp_err_t rv;

	if(send_buffer_length != 1)
	{
		log_format("ll ulp send receive: address 0x%02x: ULP I2C can only send one byte in write/read transaction", address);
		return(false);
	}

	ulp_riscv_i2c_master_set_slave_addr(address);
	ulp_riscv_i2c_master_set_slave_reg_addr(send_buffer[0]);

	rv = ulp_riscv_i2c_master_read_from_device(receive_buffer, receive_buffer_size);

	if(verbose && (rv != ESP_OK))
	{
		log_format("ll ulp send_receive: address 0x%02x:", address);
		util_warn_on_esp_err("ll ulp send receive: ulp_riscv_i2c_master_read_from_device", rv);
	}

	return(rv == ESP_OK);
}

//
// main controller implementation
//

static bool ll_main_send(const module_info_t *info, module_data_t *data, unsigned int address_in, unsigned int send_buffer_length, const uint8_t *send_buffer, bool verbose)
{
	esp_err_t rv;
	i2c_operation_job_t i2c_operations[3];
	unsigned int current;
	unsigned int cooked_send_buffer_length = send_buffer_length + 1;
	uint8_t cooked_send_buffer[cooked_send_buffer_length];

	assert(info->available);
	assert(data->device_handle);

	cooked_send_buffer[0] = (uint8_t)((address_in << i2c_address_shift) | i2c_write_flag);
	memcpy(&cooked_send_buffer[1], send_buffer, send_buffer_length);

	current = 0;

	i2c_operations[current++].command = I2C_MASTER_CMD_START;

	i2c_operations[current].command = I2C_MASTER_CMD_WRITE;
	i2c_operations[current].write.ack_check = true;
	i2c_operations[current].write.data = cooked_send_buffer;
	i2c_operations[current].write.total_bytes = cooked_send_buffer_length;
	current++;

	i2c_operations[current++].command = I2C_MASTER_CMD_STOP;

	rv = i2c_master_execute_defined_operations(data->device_handle, i2c_operations, current, 20);

	if(verbose && (rv != ESP_OK))
		util_warn_on_esp_err("ll main send: module i2c_master_execute_defined_operations", rv);

	return(rv == ESP_OK);
}

static bool ll_main_receive(const module_info_t *info, module_data_t *data, unsigned int address_in, unsigned int receive_buffer_size, uint8_t *receive_buffer, bool verbose)
{
	uint8_t address;
	esp_err_t rv;
	i2c_operation_job_t i2c_operations[5];
	unsigned int current;

	assert(info->available);
	assert(data->device_handle);

	if((receive_buffer_size == 0) || !receive_buffer)
	{
		receive_buffer_size = 0;
		receive_buffer = nullptr;
	}

	// FIXME For the moment no zero size reads are permitted due to a bug in the ESP-IDF driver code.
	// When that's fixed, this code and the next comment can be removed.

	if((receive_buffer_size == 0) || !receive_buffer)
	{
		if(verbose)
			log_format("ll main receive: address 0x%02x: main I2C module cannot handle zero byte reads", address_in);

		return(false);
	}

	address = (uint8_t)((address_in << i2c_address_shift) | i2c_read_flag);

	current = 0;

	i2c_operations[current++].command = I2C_MASTER_CMD_START;

	i2c_operations[current].command = I2C_MASTER_CMD_WRITE;
	i2c_operations[current].write.ack_check = true;
	i2c_operations[current].write.data = &address;
	i2c_operations[current].write.total_bytes = 1;
	current++;

	if(receive_buffer_size > 1)
	{
		i2c_operations[current].command = I2C_MASTER_CMD_READ;
		i2c_operations[current].read.ack_value = I2C_ACK_VAL;
		i2c_operations[current].read.data = receive_buffer;
		i2c_operations[current].read.total_bytes = receive_buffer_size - 1;
		current++;
	}

	// The conditions are only relevant when zero byte read are permitted. For the moment they're not because of a bug in
	// the ESP-IDF driver code.
	i2c_operations[current].command = I2C_MASTER_CMD_READ;
	i2c_operations[current].read.ack_value = I2C_NACK_VAL;
	i2c_operations[current].read.data = receive_buffer ? &receive_buffer[receive_buffer_size - 1] : nullptr;
	i2c_operations[current].read.total_bytes = receive_buffer ? 1 : 0;
	current++;

	i2c_operations[current++].command = I2C_MASTER_CMD_STOP;

	rv = i2c_master_execute_defined_operations(data->device_handle, i2c_operations, current, 500);

	if(verbose && (rv != ESP_OK))
		util_warn_on_esp_err("ll main receive: i2c_master_defined_operations", rv);

	return(rv == ESP_OK);
}

static bool ll_main_send_receive(const module_info_t *info, module_data_t *data, unsigned int address_in, unsigned int send_buffer_length, const uint8_t *send_buffer,
		unsigned int receive_buffer_size, uint8_t *receive_buffer, bool verbose)
{
	esp_err_t rv;
	i2c_operation_job_t i2c_operations[7];
	unsigned int current;
	unsigned int cooked_send_buffer_length = send_buffer_length + 1;
	uint8_t cooked_send_buffer[cooked_send_buffer_length];
	uint8_t read_address;

	assert(info->available);
	assert(data->device_handle);

	cooked_send_buffer[0] = (uint8_t)((address_in << i2c_address_shift) | i2c_write_flag);
	read_address = (uint8_t)((address_in << i2c_address_shift) | i2c_read_flag);

	memcpy(&cooked_send_buffer[1], send_buffer, send_buffer_length);

	current = 0;

	i2c_operations[current++].command = I2C_MASTER_CMD_START;

	i2c_operations[current].command = I2C_MASTER_CMD_WRITE;
	i2c_operations[current].write.ack_check = true;
	i2c_operations[current].write.data = cooked_send_buffer;
	i2c_operations[current].write.total_bytes = cooked_send_buffer_length;
	current++;

	i2c_operations[current++].command = I2C_MASTER_CMD_START;

	i2c_operations[current].command = I2C_MASTER_CMD_WRITE;
	i2c_operations[current].write.ack_check = true;
	i2c_operations[current].write.data = &read_address;
	i2c_operations[current].write.total_bytes = 1;
	current++;

	if(receive_buffer_size > 0)
	{
		i2c_operations[current].command = I2C_MASTER_CMD_READ;
		i2c_operations[current].read.ack_value = I2C_ACK_VAL;
		i2c_operations[current].read.data = receive_buffer;
		i2c_operations[current].read.total_bytes = receive_buffer_size - 1;
		current++;
	}

	i2c_operations[current].command = I2C_MASTER_CMD_READ;
	i2c_operations[current].read.ack_value = I2C_NACK_VAL;
	i2c_operations[current].read.data = &receive_buffer[receive_buffer_size - 1];
	i2c_operations[current].read.total_bytes = 1;
	current++;

	i2c_operations[current++].command = I2C_MASTER_CMD_STOP;

	rv = i2c_master_execute_defined_operations(data->device_handle, i2c_operations, current, 500);

	if(verbose && (rv != ESP_OK))
		util_warn_on_esp_err("ll main send receive: i2c_master_defined_operations", rv);

	return(rv == ESP_OK);
}

//
// generic internal interface
//

static bool ll_send(const module_info_t *info, module_data_t *data, unsigned int address, unsigned int send_buffer_length, const uint8_t *send_buffer, bool verbose)
{
	bool success;
	assert(info->available);

	if(info->ulp)
		success = ll_ulp_send(address, send_buffer_length, send_buffer, verbose);
	else
		success = ll_main_send(info, data, address, send_buffer_length, send_buffer, verbose);

	return(success);
}

static bool ll_receive(const module_info_t *info, module_data_t *data, unsigned int address, unsigned int receive_buffer_size, uint8_t *receive_buffer, bool verbose)
{
	bool success;
	assert(info->available);

	if(info->ulp)
		success = ll_ulp_receive(address, receive_buffer_size, receive_buffer, verbose);
	else
		success = ll_main_receive(info, data, address, receive_buffer_size, receive_buffer, verbose);

	return(success);
}

static bool ll_send_receive(const module_info_t *info, module_data_t *data, unsigned int address, unsigned int send_buffer_length, const uint8_t *send_buffer,
		unsigned int receive_buffer_size, uint8_t *receive_buffer, bool verbose)
{
	bool success;

	assert(send_buffer_length > 0);
	assert(send_buffer);
	assert(receive_buffer_size > 0);
	assert(receive_buffer);

	if(info->ulp)
		success = ll_ulp_send_receive(address, send_buffer_length, send_buffer, receive_buffer_size, receive_buffer, verbose);
	else
		success = ll_main_send_receive(info, data, address, send_buffer_length, send_buffer, receive_buffer_size, receive_buffer, verbose);

	return(success);
}

static bool ll_probe(const module_info_t *info, module_data_t *data, unsigned int address)
{
	uint8_t buffer_in[1];
	uint8_t buffer_out[1];
	bool success;

	assert(info->available);
	assert(address < 128);

	success = false;

	// The ULP can't just write zero bytes. Implement the probe with a write-read-cyle which it can do.

	if(info->ulp)
	{
		buffer_in[0] = 0;
		success = ll_ulp_send_receive(address, sizeof(buffer_in), buffer_in, sizeof(buffer_out), buffer_out, false);
	}
	else
		success = ll_main_send(info, data, address, 0, nullptr, false);

	return(success);
}

static bool set_mux(i2c_module_t module, i2c_bus_t bus)
{
	const module_info_t *info;
	module_data_t *data;
	unsigned int bus_bits;
	bool success;
	uint8_t buffer_out[2];

	assert(inited);
	assert(module < i2c_module_size);
	assert(bus < i2c_bus_size);

	info = &module_info[module];
	data = &module_data[module];

	assert(info->available);

	if(!data->has_mux)
		return(false);

	assert(bus < data->buses);

	if(bus == i2c_bus_none)
		bus_bits = 0;
	else
		bus_bits = (1 << (bus - i2c_bus_0));

	buffer_out[0] = buffer_out[1] = (uint8_t)bus_bits;

	// ULP can't write one byte.
	// The implemented workaround in ll_send, writing one byte and reading one byte with a repeated start condition
	// doesn't work on the mux. The transaction is OK but the bus is not switched. So instead write the same data twice, that works.

	if(info->ulp)
		success = ll_ulp_send(i2c_bus_mux_address, 2, buffer_out, true);
	else
		success = ll_main_send(info, data, i2c_bus_mux_address, 1, buffer_out, true);

	if(success)
		data->selected_bus = bus;

	return(success);
}

//
// generic external interface
//

void i2c_init(void)
{
	static const i2c_master_bus_config_t main_i2c_module_config[i2c_module_size] =
	{
		[i2c_module_0] =
		{
			.i2c_port = 0,
			.sda_io_num = static_cast<gpio_num_t>(CONFIG_BSP_I2C0_SDA),
			.scl_io_num = static_cast<gpio_num_t>(CONFIG_BSP_I2C0_SCL),
			.clk_source = I2C_CLK_SRC_DEFAULT,
			.glitch_ignore_cnt = 7,
			.intr_priority = 0,
			.trans_queue_depth = 0,
			.flags =
			{
				.enable_internal_pullup = 0,
				.allow_pd = 0,
			},
		},
		[i2c_module_1] =
		{
			.i2c_port = 1,
			.sda_io_num = static_cast<gpio_num_t>(CONFIG_BSP_I2C1_SDA),
			.scl_io_num = static_cast<gpio_num_t>(CONFIG_BSP_I2C1_SCL),
			.clk_source = I2C_CLK_SRC_DEFAULT,
			.glitch_ignore_cnt = 7,
			.intr_priority = 0,
			.trans_queue_depth = 0,
			.flags =
			{
				.enable_internal_pullup = 0,
				.allow_pd = 0,
			},
		},
	};

	i2c_module_t module;
	const module_info_t *info;
	module_data_t *data;
	i2c_bus_t bus;
	bus_t *bus_ptr;
	uint8_t buffer_in[1];
	uint8_t buffer_out[1];
	uint32_t config_value;

	assert(!inited);

	data_mutex = xSemaphoreCreateMutex();
	assert(data_mutex);

	for(module = i2c_module_first; module < i2c_module_size; module = static_cast<i2c_module_t>(module + 1))
	{
		module_mutex[module] = xSemaphoreCreateMutex();
		assert(module_mutex[module]);
	}

	module_data = static_cast<module_data_t *>(util_memory_alloc_spiram(sizeof(*data) * i2c_module_size));
	assert(module_data);

	for(module = i2c_module_first; module < i2c_module_size; module = static_cast<i2c_module_t>(module + 1))
	{
		info = &module_info[module];

		if(!info->available)
			continue;

		module_mutex_take(module);

		data = &module_data[module];

		if(config_get_uint("i2c.%d.speed", config_value))
			data->speed_khz = config_value;
		else
			data->speed_khz = 100;

		if(info->ulp)
		{
			static const ulp_riscv_i2c_cfg_t ulp_i2c_module_config_slow =
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
					.scl_low_period = 5.0f,
					.scl_high_period = 5.0f,
					.sda_duty_period = 2.0f,
					.scl_start_period = 3.0f,
					.scl_stop_period = 6.0f,
					.i2c_trans_timeout = 20,
				},
			};

			static const ulp_riscv_i2c_cfg_t ulp_i2c_module_config_fast =
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
					.scl_low_period = 1.3f,
					.scl_high_period = 0.5f,
					.sda_duty_period = 1.0f,
					.scl_start_period = 2.0f,
					.scl_stop_period = 1.3f,
					.i2c_trans_timeout = 20,
				},
			};

			const ulp_riscv_i2c_cfg_t *ulp_i2c_module_config;

			data->bus_handle = nullptr;

			if(data->speed_khz >= 400)
			{
				data->speed_khz = 400;
				ulp_i2c_module_config = &ulp_i2c_module_config_fast;
			}
			else
			{
				data->speed_khz = 100;
				ulp_i2c_module_config = &ulp_i2c_module_config_slow;
			}

			util_abort_on_esp_err("ulp riscv i2c master init", ulp_riscv_i2c_master_init(ulp_i2c_module_config));
		}
		else
		{
			i2c_device_config_t device_config =
			{
				.dev_addr_length = I2C_ADDR_BIT_LEN_7,
				.device_address = I2C_DEVICE_ADDRESS_NOT_USED,
				.scl_speed_hz = data->speed_khz * 1000,
				.scl_wait_us = 0,
				.flags =
				{
					.disable_ack_check = 0,
				}
			};

			util_abort_on_esp_err("i2c new master bus", i2c_new_master_bus(&main_i2c_module_config[module], &data->bus_handle));

			util_warn_on_esp_err("i2c master bus add device", i2c_master_bus_add_device(data->bus_handle, &device_config, &data->device_handle));
		}

		data->has_mux = false;

		buffer_out[0] = 0xff;

		if(ll_probe(info, data, i2c_bus_mux_address) &&
					ll_send_receive(info, data, i2c_bus_mux_address, sizeof(buffer_out), buffer_out, sizeof(buffer_in), buffer_in, true) &&
					(buffer_in[0] == 0xff))
		{
			buffer_out[0] = 0x00;
			if(ll_send_receive(info, data, i2c_bus_mux_address, sizeof(buffer_out), buffer_out, sizeof(buffer_in), buffer_in, true) &&
					(buffer_in[0] == 0x00))
				data->has_mux = true;
		}

		data->selected_bus = i2c_bus_invalid;
		data->buses = data->has_mux ? i2c_bus_size : 1;

		for(bus = i2c_bus_first; bus < i2c_bus_size; bus = static_cast<i2c_bus_t>(bus + 1))
			data->bus[bus] = nullptr;

		for(bus = i2c_bus_first; bus < data->buses; bus = static_cast<i2c_bus_t>(bus + 1))
		{
			bus_ptr = static_cast<bus_t *>(util_memory_alloc_spiram(sizeof(*bus_ptr)));
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
	const module_info_t *info;
	module_data_t *data;
	bus_t *bus_ptr;

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

	new_slave = static_cast<slave_t *>(util_memory_alloc_spiram(sizeof(*new_slave)));
	new_slave->name = name;
	new_slave->module = module;
	new_slave->bus = bus;
	new_slave->address = address;
	new_slave->next = nullptr;

	if(!(bus_ptr = data->bus[bus]))
	{
		log_format("i2c register slave: bus %d doesn't exist", bus);
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
		log_format("failed to register slave %d/%d/%#x:%s", new_slave->module, new_slave->bus, new_slave->address, new_slave->name);
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

	if(!(bus = data->bus[(**_slave).bus]))
	{
		log_format("i2c unregister slave: bus unknown %d", (**_slave).bus);
		goto finish;
	}

	if(bus->id != (**_slave).bus)
	{
		log_format("i2c unregister slave: bus->bus %d != slave->bus %d", (**_slave).bus, bus->id);
		goto finish;
	}

	if(!bus->slaves)
	{
		log_format("i2c: unregister slave: no slaves on this bus: %d", (**_slave).bus);
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

bool i2c_probe_slave(i2c_module_t module, i2c_bus_t bus, unsigned int address)
{
	bool success;
	const module_info_t *info;
	module_data_t *data;

	assert(inited);
	assert(module < i2c_module_size);
	assert(bus < i2c_bus_size);
	assert(address < 128);

	module_mutex_take(module);

	info = &module_info[module];
	data = &module_data[module];

	assert(info->available);
	assert(bus < data->buses);

	set_mux(module, bus);
	success = ll_probe(info, data, address);
	module_mutex_give(module);

	return(success);
}

bool i2c_send(i2c_slave_t slave, unsigned int send_buffer_length, const uint8_t *send_buffer)
{
	slave_t *_slave = (slave_t *)slave;
	const module_info_t *info;
	module_data_t *data;
	bool success;

	assert(inited);
	assert(_slave);

	info = &module_info[_slave->module];
	data = &module_data[_slave->module];

	assert(info->available);

	if(!slave_check(_slave)) // FIXME
	{
		log("i2c send: slave_check failed");
		return(false);
	}

	module_mutex_take(_slave->module);
	set_mux(_slave->module, _slave->bus);
	success = ll_send(info, data, _slave->address, send_buffer_length, send_buffer, true);
	module_mutex_give(_slave->module);

	return(success);
}

bool i2c_receive(i2c_slave_t slave, unsigned int receive_buffer_size, uint8_t *receive_buffer)
{
	slave_t *_slave = (slave_t *)slave;
	const module_info_t *info;
	module_data_t *data;
	bool success;

	assert(inited);
	assert(_slave);
	assert(receive_buffer);
	assert(receive_buffer_size > 0);

	info = &module_info[_slave->module];
	data = &module_data[_slave->module];

	assert(info->available);

	if(!slave_check(_slave))
		return(false);

	module_mutex_take(_slave->module);
	set_mux(_slave->module, _slave->bus);
	success = ll_receive(info, data, _slave->address, receive_buffer_size, receive_buffer, true);
	module_mutex_give(_slave->module);

	return(success);
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

bool i2c_send_receive(i2c_slave_t slave, unsigned int send_buffer_length, const uint8_t *send_buffer,
		unsigned int receive_buffer_size, uint8_t *receive_buffer)
{
	slave_t *_slave = (slave_t *)slave;
	const module_info_t *info;
	module_data_t *data;
	bool success;

	assert(inited);
	assert(_slave);
	assert(send_buffer);
	assert(receive_buffer);

	info = &module_info[_slave->module];
	data = &module_data[_slave->module];

	assert(info->available);

	if(!slave_check(_slave))
		return(false);

	module_mutex_take(_slave->module);

	set_mux(_slave->module, _slave->bus);

	success = ll_send_receive(info, data, _slave->address, send_buffer_length, send_buffer, receive_buffer_size, receive_buffer, true);

	module_mutex_give(_slave->module);

	return(success);
}

bool i2c_send_1_receive(i2c_slave_t slave, unsigned int byte, unsigned int receive_buffer_size, uint8_t *receive_buffer)
{
	uint8_t send_buffer[1];

	send_buffer[0] = byte;

	return(i2c_send_receive(slave, sizeof(send_buffer), send_buffer, receive_buffer_size, receive_buffer));
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

	for(bus_index = i2c_bus_first; bus_index < data->buses; bus_index = static_cast<i2c_bus_t>(bus_index + 1)) // FIXME check
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

	if(!inited)
	{
		string_assign_cstr(call->result, "I2C init not complete, come back later");
		return;
	}

	data_mutex_take();

	string_format(call->result, "I2C info");

	for(module_index = i2c_module_first; module_index < i2c_module_size; module_index = static_cast<i2c_module_t>(module_index + 1))
	{
		info = &module_info[module_index];

		if(info->available)
		{
			data = &module_data[module_index];

			string_format_append(call->result, "\n- module [%d]: \"%s\", sda: %u, scl: %u, speed: %u khz", info->id, info->name, info->sda, info->scl, data->speed_khz);

			for(bus_index = i2c_bus_first; bus_index < data->buses; bus_index = static_cast<i2c_bus_t>(bus_index + 1))
			{
				if((bus = data->bus[bus_index]))
				{
					string_format_append(call->result, "\n-  i2c bus %d: %s", bus->id, bus_name[bus->id]);

					for(slave = bus->slaves; slave; slave = slave->next)
						string_format_append(call->result, "\n-   slave [0x%x]: name: %s, module: %d, bus: %d",
								slave->address, slave->name, slave->module, slave->bus);
				}
			}
		}
		else
			string_format_append(call->result, "\n- module [%d]: unavailable", info->id);
	}

	data_mutex_give();
}

void command_i2c_speed(cli_command_call_t *call)
{
	const module_info_t *info;
	module_data_t *data;
	i2c_module_t module_index;
	unsigned int speed;

	if(!inited)
	{
		string_assign_cstr(call->result, "I2C init not complete, come back later");
		return;
	};

	assert(call->parameter_count <= 2);

	if(call->parameter_count == 2)
	{
		uint32_t config_value;

		data_mutex_take();

		module_index = (i2c_module_t)call->parameters[0].unsigned_int;
		assert(module_index < i2c_module_size);
		speed = call->parameters[1].unsigned_int;

		info = &module_info[module_index];

		if(!info->available)
		{
			data_mutex_give();
			string_format(call->result, "I2C module #%u unavailable", (unsigned int)module_index);
			return;
		}

		data = &module_data[module_index];
		data->speed_khz = config_value = speed;

		config_set_uint("i2c.%d.speed", config_value);

		data_mutex_give();
	}

	data_mutex_take();

	string_format(call->result, "I2C speed");

	for(module_index = i2c_module_first; module_index < i2c_module_size; module_index = static_cast<i2c_module_t>(module_index + 1))
	{
		info = &module_info[module_index];

		if(info->available)
		{
			data = &module_data[module_index];

			string_format_append(call->result, "\n- module [%d]: \"%s\", speed: %u khz", info->id, info->name, data->speed_khz);
		}
		else
			string_format_append(call->result, "\n- module [%d]: unavailable", info->id);
	}

	data_mutex_give();
}
