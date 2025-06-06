#include <stdint.h>
#include <stdbool.h>

#include "i2c.h"
#include "string.h"
#include "info.h"
#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "mcpwm.h"
#include "ledpwm.h"
#include "pdm.h"
#include "ledpixel.h"
#include "io.h"

#include <assert.h>
#include <freertos/FreeRTOS.h>

typedef enum
{
	io_bus_none = 0,
	io_bus_apb,
	io_bus_first = io_bus_apb,
	io_bus_i2c,
	io_bus_size,
	io_bus_error = io_bus_size,
} io_bus_t;

typedef enum
{
	io_int_value_0 = 0,
	io_int_value_first = io_int_value_0,
	io_int_value_1,
	io_int_value_2,
	io_int_value_3,
	io_int_value_size,
	io_int_value_error = io_int_value_size,
} io_int_value_t;

struct io_data_T;

typedef struct io_info_T
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
		struct
		{
			lp_t instance;
		} ledpixel;
	};
	void (*info_fn)(const struct io_data_T *data, string_t result);
	bool (*detect_fn)(const struct io_info_T *info, unsigned int module, unsigned int bus, unsigned int address);
	bool (*init_fn)(struct io_data_T *data);
	bool (*read_fn)(struct io_data_T *data, unsigned int pin, unsigned int *value);
	bool (*write_fn)(struct io_data_T *data, unsigned int pin, unsigned int value);
	void (*pin_info_fn)(const struct io_data_T *data, unsigned int pin, string_t result);
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
	};
	int int_value[io_int_value_size];
	const io_info_t *info;
	struct io_data_T *next;
} io_data_t;

static bool inited = false;
static io_data_t *data_root = (io_data_t *)0;
static SemaphoreHandle_t data_mutex;
static const char *cap_to_string[io_cap_size] =
{
	[io_cap_input] = "input",
	[io_cap_output] = "output",
};

static unsigned int stat_i2c_detect_skipped;
static unsigned int stat_i2c_detect_tried;
static unsigned int stat_i2c_detect_found;

static inline void data_mutex_take(void)
{
	assert(xSemaphoreTake(data_mutex, portMAX_DELAY));
}

static inline void data_mutex_give(void)
{
	assert(xSemaphoreGive(data_mutex));
}

enum
{
	esp32_mcpwm_pin_0 = 0,
	esp32_mcpwm_pin_1,
	esp32_mcpwm_pin_2,
	esp32_mcpwm_pin_3,
	esp32_mcpwm_pin_size
};

_Static_assert((unsigned int)esp32_mcpwm_pin_size <= (unsigned int)io_int_value_size);
_Static_assert((unsigned int)esp32_mcpwm_pin_0 == (unsigned int)mpt_16bit_150hz_0);
_Static_assert((unsigned int)esp32_mcpwm_pin_1 == (unsigned int)mpt_16bit_150hz_1);
_Static_assert((unsigned int)esp32_mcpwm_pin_2 == (unsigned int)mpt_16bit_2400hz_0);
_Static_assert((unsigned int)esp32_mcpwm_pin_3 == (unsigned int)mpt_16bit_2400hz_1);

static void esp32_mcpwm_info(const io_data_t *dataptr, string_t result)
{
	assert(inited);
	assert(dataptr);
}

static bool esp32_mcpwm_init(io_data_t *dataptr)
{
	mcpwm_t handle;
	bool rv = false;

	assert(inited);
	assert(dataptr);

	for(handle = mpt_first; handle < mpt_size; handle++)
		if((dataptr->int_value[handle] = mcpwm_open(handle, "I/O MC-PWM")))
			rv = true;

	return(rv);
}

static bool esp32_mcpwm_write(io_data_t *dataptr, unsigned int pin, unsigned int value)
{
	assert(inited);
	assert(dataptr);
	assert(pin < dataptr->info->pins);
	assert(pin < mpt_size);
	assert(value <= dataptr->info->max_value);

	if(!dataptr->int_value[pin])
		return(false);

	mcpwm_set((mcpwm_t)pin, value);

	return(true);
}

static void esp32_mcpwm_pin_info(const io_data_t *dataptr, unsigned int pin, string_t result)
{
	assert(inited);
	assert(dataptr);
	assert(result);
	assert(pin < dataptr->info->pins);
	assert(pin < mpt_size);

	if(dataptr->int_value[pin])
		string_format_append(result, "MC-PWM channel %u duty: %u", pin, mcpwm_get(pin));
	else
		string_append_cstr(result, "pin unvailable on this board");
}

enum
{
	esp32_ledpwm_pin_0 = 0,
	esp32_ledpwm_pin_1,
	esp32_ledpwm_pin_2,
	esp32_ledpwm_pin_3,
	esp32_ledpwm_pin_size
};

_Static_assert((unsigned int)esp32_ledpwm_pin_size <= (unsigned int)io_int_value_size);
_Static_assert((unsigned int)esp32_ledpwm_pin_0 == (unsigned int)lpt_14bit_5khz_notify);
_Static_assert((unsigned int)esp32_ledpwm_pin_1 == (unsigned int)lpt_14bit_5khz_lcd_spi_2);
_Static_assert((unsigned int)esp32_ledpwm_pin_2 == (unsigned int)lpt_14bit_5khz_lcd_spi_3);
_Static_assert((unsigned int)esp32_ledpwm_pin_3 == (unsigned int)lpt_14bit_120hz);

static void esp32_ledpwm_info(const io_data_t *dataptr, string_t result)
{
	assert(inited);
	assert(dataptr);
}

static bool esp32_ledpwm_init(io_data_t *dataptr)
{
	ledpwm_t handle;
	bool rv = false;

	assert(inited);
	assert(dataptr);

	for(handle = lpt_first; handle < lpt_size; handle++)
		if((dataptr->int_value[handle] = ledpwm_open(handle, "I/O LED-PWM")))
			rv = true;

	return(rv);
}

static bool esp32_ledpwm_write(io_data_t *dataptr, unsigned int pin, unsigned int value)
{
	assert(inited);
	assert(dataptr);
	assert(pin < dataptr->info->pins);
	assert(pin < lpt_size);
	assert(value <= dataptr->info->max_value);

	if(!dataptr->int_value[pin])
		return(false);

	ledpwm_set((ledpwm_t)pin, value);

	return(true);
}

static void esp32_ledpwm_pin_info(const io_data_t *dataptr, unsigned int pin, string_t result)
{
	assert(inited);
	assert(dataptr);
	assert(result);
	assert(pin < dataptr->info->pins);
	assert(pin < lp_size);

	if(dataptr->int_value[pin])
		string_format_append(result, "LED-PWM channel %u duty: %u", pin, ledpwm_get(pin));
	else
		string_append_cstr(result, "pin unvailable on this board");
}

enum
{
	esp32_pdm_pin_0 = 0,
	esp32_pdm_pin_1,
	esp32_pdm_pin_2,
	esp32_pdm_pin_3,
	esp32_pdm_pin_size
};

_Static_assert((unsigned int)esp32_pdm_pin_size <= (unsigned int)io_int_value_size);
_Static_assert((unsigned int)esp32_pdm_pin_0 == pdm_8bit_150khz_0);
_Static_assert((unsigned int)esp32_pdm_pin_1 == pdm_8bit_150khz_1);
_Static_assert((unsigned int)esp32_pdm_pin_2 == pdm_8bit_150khz_2);
_Static_assert((unsigned int)esp32_pdm_pin_3 == pdm_8bit_150khz_3);

static void esp32_pdm_info(const io_data_t *dataptr, string_t result)
{
	assert(inited);
	assert(dataptr);
}

static bool esp32_pdm_init(io_data_t *dataptr)
{
	pdm_t handle;
	bool rv = false;

	assert(inited);
	assert(dataptr);

	for(handle = pdm_first; handle < pdm_size; handle++)
		if((dataptr->int_value[handle] = pdm_channel_open(handle, "I/O PDM")))
			rv = true;

	return(rv);
}

static bool esp32_pdm_write(io_data_t *dataptr, unsigned int pin, unsigned int value)
{
	assert(inited);
	assert(dataptr);
	assert(pin < dataptr->info->pins);
	assert(pin < pdm_size);
	assert(value <= dataptr->info->max_value);

	if(!dataptr->int_value[pin])
		return(false);

	pdm_channel_set((pdm_t)pin, value);

	return(true);
}

static void esp32_pdm_pin_info(const io_data_t *dataptr, unsigned int pin, string_t result)
{
	assert(inited);
	assert(dataptr);
	assert(result);
	assert(pin < dataptr->info->pins);
	assert(pin < pdm_size);

	if(dataptr->int_value[pin])
		string_format_append(result, "PDM channel %u density: %u", pin, pdm_channel_get(pin));
	else
		string_append_cstr(result, "pin unvailable on this board");
}

enum
{
	esp32_ledpixel_int_value_open = 0,
};

_Static_assert((unsigned int)esp32_ledpixel_int_value_open < (unsigned int)io_int_value_size);

static void esp32_ledpixel_info(const io_data_t *dataptr, string_t result)
{
	assert(inited);
	assert(dataptr);
}

static bool esp32_ledpixel_init(io_data_t *dataptr)
{
	bool rv;

	assert(inited);
	assert(dataptr);

	rv = ledpixel_open(dataptr->info->ledpixel.instance, "I/O ledpixel");

	dataptr->int_value[esp32_ledpixel_int_value_open] = rv ? 1 : 0;

	return(rv);
}

static bool esp32_ledpixel_write(io_data_t *dataptr, unsigned int pin, unsigned int value)
{
	assert(inited);
	assert(dataptr);
	assert(pin < dataptr->info->pins);
	assert(value <= dataptr->info->max_value);

	if(!dataptr->int_value[esp32_ledpixel_int_value_open])
		return(false);

	ledpixel_set(dataptr->info->ledpixel.instance, pin, (value & 0x00ff0000) >> 16, (value & 0x0000ff00) >> 8, (value & 0x000000ff) >> 0);
	ledpixel_flush(dataptr->info->ledpixel.instance);

	return(true);
}

static void esp32_ledpixel_pin_info(const io_data_t *dataptr, unsigned int pin, string_t result)
{
	assert(inited);
	assert(dataptr);
	assert(result);
	assert(pin < dataptr->info->pins);

	if(dataptr->int_value[esp32_ledpixel_int_value_open])
		string_format_append(result, "LEDpixel instance %u", dataptr->info->ledpixel.instance);
	else
		string_append_cstr(result, "pin unvailable on this board");
}

enum
{
	pcf8574_int_value_cache_in = 0,
	pcf8574_int_value_cache_out,
	pcf8574_int_value_size,
};

_Static_assert((unsigned int)pcf8574_int_value_size <= (unsigned int)io_int_value_size);

static void pcf8574_info(const io_data_t *dataptr, string_t result)
{
	string_append_cstr(result, "\npin cache");
	string_format_append(result, "\n- input %#02x", (unsigned int)dataptr->int_value[pcf8574_int_value_cache_in]);
	string_format_append(result, "\n- output %#02x", (unsigned int)dataptr->int_value[pcf8574_int_value_cache_out]);
}

static bool pcf8574_detect(const io_info_t *info, unsigned int module, unsigned int bus, unsigned int address)
{
	assert(inited);

	return(i2c_probe_slave(module, bus, address, 0xff, info->name));
}

static bool pcf8574_init(io_data_t *dataptr)
{
	assert(inited);

	dataptr->int_value[pcf8574_int_value_cache_in] = 0xff;
	dataptr->int_value[pcf8574_int_value_cache_out] = 0xff;

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

	assert(inited);

	assert(dataptr);
	assert(dataptr->info);
	assert(pin < dataptr->info->pins);

	if(!i2c_receive(dataptr->i2c.slave, 1, buffer))
		return(false);

	dataptr->int_value[pcf8574_int_value_cache_in] = buffer[0];

	*value = !!(buffer[0] & (1 << pin));

	return(true);
}

static bool pcf8574_write(io_data_t *dataptr, unsigned int pin, unsigned int value)
{
	assert(dataptr);
	assert(dataptr->info);
	assert(pin < dataptr->info->pins);
	assert(value <= dataptr->info->max_value);

	assert(inited);

	if(value)
		dataptr->int_value[pcf8574_int_value_cache_out] &= ~(1 << pin);
	else
		dataptr->int_value[pcf8574_int_value_cache_out] |= (1 << pin);

	return(i2c_send_1(dataptr->i2c.slave, dataptr->int_value[pcf8574_int_value_cache_out]));
}

static void pcf8574_pin_info(const io_data_t *dataptr, unsigned int pin, string_t result)
{
	string_format_append(result, "binary I/O, current I/O value: %u/%u",
			(unsigned int)!(dataptr->int_value[pcf8574_int_value_cache_in] & (1 << pin)),
			(unsigned int)!(dataptr->int_value[pcf8574_int_value_cache_out] & (1 << pin)));
}

static const io_info_t info[io_id_size] =
{
	[io_id_esp32_mcpwm] =
	{
		.id = io_id_esp32_mcpwm,
		.name = "ESP32 MC-PWM 16 bits",
		.caps = (1 << io_cap_output),
		.pins = esp32_mcpwm_pin_size,
		.max_value = 65535,
		.bus = io_bus_apb,
		.info_fn = esp32_mcpwm_info,
		.detect_fn = nullptr,
		.init_fn = esp32_mcpwm_init,
		.read_fn = (void *)0,
		.write_fn = esp32_mcpwm_write,
		.pin_info_fn = esp32_mcpwm_pin_info,
	},
	[io_id_esp32_ledpwm] =
	{
		.id = io_id_esp32_ledpwm,
		.name = "ESP32 LED-PWM 14 bits",
		.caps = (1 << io_cap_output),
		.pins = esp32_ledpwm_pin_size,
		.max_value = 16383,
		.bus = io_bus_apb,
		.info_fn = esp32_ledpwm_info,
		.detect_fn = nullptr,
		.init_fn = esp32_ledpwm_init,
		.read_fn = (void *)0,
		.write_fn = esp32_ledpwm_write,
		.pin_info_fn = esp32_ledpwm_pin_info,
	},
	[io_id_esp32_pdm] =
	{
		.id = io_id_esp32_pdm,
		.name = "ESP32 PDM 8 bits",
		.caps = (1 << io_cap_output),
		.pins = esp32_pdm_pin_size,
		.max_value = 255,
		.bus = io_bus_apb,
		.info_fn = esp32_pdm_info,
		.detect_fn = nullptr,
		.init_fn = esp32_pdm_init,
		.read_fn = (void *)0,
		.write_fn = esp32_pdm_write,
		.pin_info_fn = esp32_pdm_pin_info,
	},
	[io_id_esp32_ledpixel_0] =
	{
		.id = io_id_esp32_ledpixel_0,
		.name = "ESP32 LEDpixel 0",
		.caps = (1 << io_cap_output),
		.pins = ledpixel_leds_size,
		.max_value = 0x00ffffff,
		.bus = io_bus_apb,
		.ledpixel.instance = lp_0_notify,
		.info_fn = esp32_ledpixel_info,
		.detect_fn = nullptr,
		.init_fn = esp32_ledpixel_init,
		.read_fn = (void *)0,
		.write_fn = esp32_ledpixel_write,
		.pin_info_fn = esp32_ledpixel_pin_info,
	},
	[io_id_esp32_ledpixel_1] =
	{
		.id = io_id_esp32_ledpixel_1,
		.name = "ESP32 LEDpixel 1",
		.caps = (1 << io_cap_output),
		.pins = ledpixel_leds_size,
		.max_value = 0x00ffffff,
		.bus = io_bus_apb,
		.ledpixel.instance = lp_1,
		.info_fn = esp32_ledpixel_info,
		.detect_fn = nullptr,
		.init_fn = esp32_ledpixel_init,
		.read_fn = (void *)0,
		.write_fn = esp32_ledpixel_write,
		.pin_info_fn = esp32_ledpixel_pin_info,
	},
	[io_id_esp32_ledpixel_2] =
	{
		.id = io_id_esp32_ledpixel_2,
		.name = "ESP32 LEDpixel 2",
		.caps = (1 << io_cap_output),
		.pins = ledpixel_leds_size,
		.max_value = 0x00ffffff,
		.bus = io_bus_apb,
		.ledpixel.instance = lp_2,
		.info_fn = esp32_ledpixel_info,
		.detect_fn = nullptr,
		.init_fn = esp32_ledpixel_init,
		.read_fn = (void *)0,
		.write_fn = esp32_ledpixel_write,
		.pin_info_fn = esp32_ledpixel_pin_info,
	},
	[io_id_esp32_ledpixel_3] =
	{
		.id = io_id_esp32_ledpixel_3,
		.name = "ESP32 LEDpixel 3",
		.caps = (1 << io_cap_output),
		.pins = ledpixel_leds_size,
		.max_value = 0x00ffffff,
		.bus = io_bus_apb,
		.ledpixel.instance = lp_3,
		.info_fn = esp32_ledpixel_info,
		.detect_fn = nullptr,
		.init_fn = esp32_ledpixel_init,
		.read_fn = (void *)0,
		.write_fn = esp32_ledpixel_write,
		.pin_info_fn = esp32_ledpixel_pin_info,
	},
	[io_id_pcf8574_26] =
	{
		.id = io_id_pcf8574_26,
		.name = "PCF8574 8-bit I/O expander",
		.caps = (1 << io_cap_input) | (1 << io_cap_output),
		.pins = 8,
		.max_value = 1,
		.bus = io_bus_i2c,
		.i2c.address = 0x26,
		.info_fn = pcf8574_info,
		.detect_fn = pcf8574_detect,
		.init_fn = pcf8574_init,
		.read_fn = pcf8574_read,
		.write_fn = pcf8574_write,
		.pin_info_fn = pcf8574_pin_info,
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
		.info_fn = pcf8574_info,
		.detect_fn = pcf8574_detect,
		.init_fn = pcf8574_init,
		.read_fn = pcf8574_read,
		.write_fn = pcf8574_write,
		.pin_info_fn = pcf8574_pin_info,
	},
};

static io_data_t *find_io(io_bus_t bus, unsigned int parameter_1, unsigned int parameter_2, unsigned int parameter_3)
{
	io_data_t *dataptr;

	data_mutex_take();

	for(dataptr = data_root; dataptr; dataptr = dataptr->next)
	{
		if(bus != dataptr->info->bus)
			continue;

		switch(dataptr->info->bus)
		{
			case(io_bus_apb):
			{
				goto found;
			}

			case(io_bus_i2c):
			{
				i2c_module_t i2c_module;
				i2c_bus_t i2c_bus;
				unsigned int i2c_address;
				const char *name;

				assert(parameter_1 < i2c_module_size);
				assert(parameter_2 < i2c_bus_size);
				assert(parameter_3 < 128);

				i2c_get_slave_info(dataptr->i2c.slave, &i2c_module, &i2c_bus, &i2c_address, &name);

				if((i2c_module_t)parameter_1 != i2c_module)
					continue;

				if(parameter_3 != i2c_address)
					continue;

				if(((i2c_bus_t)parameter_2 == i2c_bus_none) || (i2c_bus == i2c_bus_none))
					goto found;

				if((i2c_bus_t)parameter_2 == i2c_bus)
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
			case(io_bus_apb):
			{
				if(infoptr->detect_fn && !infoptr->detect_fn(infoptr, 0, 0, 0))
					continue;

				dataptr = (io_data_t *)util_memory_alloc_spiram(sizeof(*dataptr));
				assert(dataptr);

				dataptr->id = id;
				dataptr->info = infoptr;
				dataptr->next = (io_data_t *)0;

				assert(infoptr->init_fn);

				if(!infoptr->init_fn(dataptr))
					continue;

				if(!data_root)
					data_root = dataptr;
				else
				{
					for(next = data_root; next->next; next = next->next)
						(void)0;

					assert(!next->next);

					next->next = dataptr;
				}

				break;
			}

			case(io_bus_i2c):
			{
				for(module = i2c_module_first; module < i2c_module_size; module++)
				{
					if(!i2c_module_available(module))
						continue;

					buses = i2c_buses(module);

					for(bus = 0; bus < buses; bus++)
					{
						if(find_io(infoptr->bus, (unsigned int)module, (unsigned int)bus, infoptr->i2c.address))
						{
							stat_i2c_detect_skipped++;
							continue;
						}

						stat_i2c_detect_tried++;

						if(infoptr->detect_fn && !infoptr->detect_fn(infoptr, module, bus, infoptr->i2c.address))
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

						stat_i2c_detect_found++;

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

			case(io_bus_none):
			{
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

static void io_info_x(string_t result, const io_data_t *dataptr)
{
	io_capabilities_t cap;

	assert(inited);
	assert(result);
	assert(dataptr);

	string_append_cstr(result, dataptr->info->name);
	string_format_append(result, "\n- id: %u", (unsigned int)dataptr->info->id);
	string_format_append(result, "\n- pins: %u", dataptr->info->pins);
	string_format_append(result, "\n- max value per pin: %u", dataptr->info->max_value);
	string_append_cstr(result, "\n- capabilities:");

	for(cap = io_cap_first; cap < io_cap_size; cap++)
		if(dataptr->info->caps & (1 << cap))
			string_format_append(result, " %s", cap_to_string[cap]);

	string_append_cstr(result, "\n- extra device info: ");

	if(dataptr->info->info_fn)
		dataptr->info->info_fn(dataptr, result);
}

static bool io_read_x(string_t result, io_data_t *dataptr, unsigned int pin, unsigned int *value)
{
	assert(inited);
	assert(dataptr);
	assert(value);

	if(!(dataptr->info->caps & (1 << io_cap_input)) || !dataptr->info->read_fn)
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
	assert(inited);
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

static void io_pin_info_x(string_t result, const io_data_t *dataptr, unsigned int pin)
{
	if(dataptr->info->pin_info_fn)
		dataptr->info->pin_info_fn(dataptr, pin, result);
}

bool io_info(string_t result, unsigned int io)
{
	const io_data_t *dataptr;

	assert(inited);
	assert(result);

	if(!(dataptr = get_data(io)))
	{
		string_format_append(result, "no such I/O %u", io);
		return(false);
	}

	io_info_x(result, dataptr);

	return(true);
}

bool io_read(string_t result, unsigned int io, unsigned int pin, unsigned int *value)
{
	io_data_t *dataptr;

	assert(inited);

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

	assert(inited);

	if(!(dataptr = get_data(io)))
	{
		if(result)
			string_format_append(result, "no such I/O %u", io);
		return(false);
	}

	return(io_write_x(result, dataptr, pin, value));
}

bool io_pin_info(string_t result, unsigned int io, unsigned int pin)
{
	io_data_t *dataptr;

	assert(inited);

	if(!(dataptr = get_data(io)))
	{
		if(result)
			string_format_append(result, "no such I/O %u", io);
		return(false);
	}

	if(pin >= dataptr->info->pins)
	{
		if(result)
			string_format_append(result, "no such pin %u", pin);
		return(false);
	}

	io_pin_info_x(result, dataptr, pin);

	return(true);
}

void command_io_dump(cli_command_call_t *call)
{
	const io_data_t *dataptr;
	i2c_module_t module;
	i2c_bus_t bus;
	unsigned int address, sequence;
	const char *name;
	unsigned int pin;

	assert(inited);
	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "I/O DUMP");

	sequence = 0;

	data_mutex_take();

	for(dataptr = data_root; dataptr; dataptr = dataptr->next)
	{
		string_format_append(call->result, "\n[%u]: ", sequence++);

		io_info_x(call->result, dataptr);

		switch(dataptr->info->bus)
		{
			case(io_bus_apb):
			{
				string_append_cstr(call->result, "\nbus info\n- APB device");
				break;
			}

			case(io_bus_i2c):
			{
				i2c_get_slave_info(dataptr->i2c.slave, &module, &bus, &address, &name);
				string_format_append(call->result, "\nbus info\n- I2C device %s at %u/%u/%#x", name, module, bus, address);

				break;
			}

			default:
			{
				string_format_append(call->result, " unknown IO type %u: %s", dataptr->info->bus, dataptr->info->name);
			}
		}
		string_append_cstr(call->result, "\npins:");

		for(pin = 0; pin < dataptr->info->pins; pin++)
		{
			string_format_append(call->result, "\n- pin %u: ", pin);
			io_pin_info_x(call->result, dataptr, pin);
		}
	}

	data_mutex_give();
}

void command_io_stats(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 0);

	string_assign_cstr(call->result, "IO STATS");
	string_assign_cstr(call->result, "\n- detecting");
	string_format_append(call->result, "\n-  skipped: %u", stat_i2c_detect_skipped);
	string_format_append(call->result, "\n-  tried: %u", stat_i2c_detect_tried);
	string_format_append(call->result, "\n-  found: %u", stat_i2c_detect_found);
}

void command_io_read(cli_command_call_t *call)
{
	unsigned int value;

	assert(inited);
	assert(call->parameter_count == 2);

	string_assign_cstr(call->result, "io-read: ");

	if(io_read(call->result, call->parameters[0].unsigned_int, call->parameters[1].unsigned_int, &value))
		string_format(call->result, "io-read %u/%u: %u OK", call->parameters[0].unsigned_int, call->parameters[1].unsigned_int, value);
}

void command_io_write(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 3);

	string_assign_cstr(call->result, "io-write: ");

	if(io_write(call->result, call->parameters[0].unsigned_int, call->parameters[1].unsigned_int, call->parameters[2].unsigned_int))
		string_format(call->result, "io-write %u/%u: %u OK", call->parameters[0].unsigned_int, call->parameters[1].unsigned_int, call->parameters[2].unsigned_int);
}
