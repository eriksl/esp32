#pragma once

enum
{
	i2c_module_speed_slow = 100000,
	i2c_module_speed_fast = 400000,
	i2c_module_speed_ulp = 400000,
	i2c_module_speed_none = 0,
};

typedef enum
{
	i2c_module_0_fast = 0,
	i2c_module_first = i2c_module_0_fast,
	i2c_module_1_slow,
	i2c_module_2_ulp,
	i2c_module_size,
	i2c_module_error = i2c_module_size,
	i2c_module_unavailable = i2c_module_size,
} i2c_module_t;

typedef enum
{
	i2c_bus_none = 0,
	i2c_bus_first = i2c_bus_none,
	i2c_bus_0,
	i2c_bus_mux_first = i2c_bus_0,
	i2c_bus_1,
	i2c_bus_2,
	i2c_bus_3,
	i2c_bus_4,
	i2c_bus_5,
	i2c_bus_6,
	i2c_bus_7,
	i2c_bus_size,
	i2c_bus_invalid = i2c_bus_size,
} i2c_bus_t;

enum
{
	i2c_probe_no_write = 0xffff,
};

struct i2c_opaque_t {} __attribute__((aligned(sizeof(int))));

typedef struct i2c_opaque_t *i2c_slave_t;
typedef const struct i2c_opaque_t *const_i2c_slave_t;

void i2c_init(void);
bool i2c_module_available(i2c_module_t module);
unsigned int i2c_buses(i2c_module_t module);
bool i2c_ulp(i2c_module_t module);
bool i2c_slave_ulp(i2c_slave_t slave);
i2c_slave_t i2c_register_slave(const char *name, i2c_module_t module, i2c_bus_t bus, unsigned int address);
bool i2c_unregister_slave(i2c_slave_t *slave);
bool i2c_probe_slave(i2c_module_t module, i2c_bus_t bus, unsigned int address, unsigned int probe_write_value, const char *probe_name);
bool i2c_get_slave_info(i2c_slave_t slave, i2c_module_t *module, i2c_bus_t *bus, unsigned int *address, const char **name);
i2c_slave_t i2c_find_slave(i2c_module_t module, i2c_bus_t bus, unsigned int address);
bool i2c_send(i2c_slave_t slave, unsigned int send_buffer_length, const uint8_t *send_buffer);
bool i2c_send_1(i2c_slave_t slave, unsigned int byte);
bool i2c_send_2(i2c_slave_t slave, unsigned int byte_1, unsigned int byte_2);
bool i2c_send_3(i2c_slave_t slave, unsigned int byte_1, unsigned int byte_2, unsigned int byte_3);
bool i2c_receive(i2c_slave_t slave, unsigned int read_buffer_size, uint8_t *read_buffer);
bool i2c_send_receive(i2c_slave_t slave, unsigned int send_buffer_length, const uint8_t *send_buffer, unsigned int receive_buffer_size, uint8_t *receive_buffer);
bool i2c_send_1_receive(i2c_slave_t slave, unsigned int byte, unsigned int receive_buffer_size, uint8_t *receive_buffer);
