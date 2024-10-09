#pragma once

enum
{
	i2c_module_speed_slow = 100000,
	i2c_module_speed_fast = 400000,
};

typedef enum
{
	i2c_module_0_fast = 0,
	i2c_module_first = i2c_module_0_fast,
	i2c_module_1_slow,
	i2c_module_size,
	i2c_module_error = i2c_module_size,
} i2c_module_t;

typedef enum
{
	i2c_bus_none = 0,
	i2c_bus_first = i2c_bus_none,
	i2c_bus_0,
	i2c_bus_1,
	i2c_bus_2,
	i2c_bus_3,
	i2c_bus_4,
	i2c_bus_5,
	i2c_bus_6,
	i2c_bus_7,
	i2c_bus_size,
	i2c_bus_error = i2c_bus_size,
} i2c_bus_t;

struct i2c_opaque_t {};

typedef struct i2c_opaque_t *i2c_slave_t;
typedef const struct i2c_opaque_t *const_i2c_slave_t;

void i2c_init(void);
unsigned int i2c_buses(i2c_module_t module);
i2c_slave_t i2c_register_slave(const char *name, i2c_module_t module, i2c_bus_t bus, unsigned int address);
bool i2c_unregister_slave(i2c_slave_t *slave);
bool i2c_probe_slave(i2c_module_t module, i2c_bus_t bus, unsigned int address);
bool i2c_get_slave_info(i2c_slave_t slave, i2c_module_t *module, i2c_bus_t *bus, unsigned int *address, const char **name);
i2c_slave_t i2c_find_slave(i2c_module_t module, i2c_bus_t bus, unsigned int address);
bool i2c_send(i2c_slave_t slave, unsigned int send_buffer_length, const uint8_t *send_buffer);
bool i2c_send_1(i2c_slave_t slave, unsigned int byte);
bool i2c_receive(i2c_slave_t slave, unsigned int read_buffer_size, uint8_t *read_buffer);
