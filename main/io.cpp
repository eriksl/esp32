#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "log.h"
#include "util.h"
#include "cli-command.h"
#include "mcpwm.h"
#include "ledpwm.h"
#include "pdm.h"
#include "ledpixel.h"
#include "io.h"
#include "info.h"
#include "i2c.h"
#include "exception.h"

#include <string>
#include <format>

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
	io_capabilities_t caps;
	unsigned int pins;
	unsigned int max_value;
	io_bus_t bus;
	union
	{
		struct
		{
			unsigned int address;
		} i2c;
		struct
		{
			Ledpixel::lp_t instance;
		} ledpixel;
	} instance;
	void (*info_fn)(const struct io_data_T *data, std::string &result);
	bool (*detect_fn)(const struct io_info_T *info, unsigned int module, unsigned int bus, unsigned int address);
	bool (*init_fn)(struct io_data_T *data);
	bool (*read_fn)(struct io_data_T *data, unsigned int pin, unsigned int *value);
	bool (*write_fn)(struct io_data_T *data, unsigned int pin, unsigned int value);
	void (*pin_info_fn)(const struct io_data_T *data, unsigned int pin, std::string &result);
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
static const char *cap_to_string[io_cap_size] = // FIXME
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

static_assert((unsigned int)esp32_mcpwm_pin_size <= (unsigned int)io_int_value_size);
static_assert((unsigned int)esp32_mcpwm_pin_0 == (unsigned int)mpt_16bit_150hz_0);
static_assert((unsigned int)esp32_mcpwm_pin_1 == (unsigned int)mpt_16bit_150hz_1);
static_assert((unsigned int)esp32_mcpwm_pin_2 == (unsigned int)mpt_16bit_2400hz_0);
static_assert((unsigned int)esp32_mcpwm_pin_3 == (unsigned int)mpt_16bit_2400hz_1);

static void esp32_mcpwm_info(const io_data_t *dataptr, std::string &result)
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

	for(handle = mpt_first; handle < mpt_size; handle = static_cast<mcpwm_t>(handle + 1))
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

static void esp32_mcpwm_pin_info(const io_data_t *dataptr, unsigned int pin, std::string &result)
{
	assert(inited);
	assert(dataptr);
	assert(pin < dataptr->info->pins);
	assert(pin < mpt_size);

	if(dataptr->int_value[pin])
		result += std::format("MC-PWM channel {:d} duty: {:d}", pin, mcpwm_get(static_cast<mcpwm_t>(pin)));
	else
		result += "pin unvailable on this board";
}

enum
{
	esp32_ledpwm_pin_0 = 0,
	esp32_ledpwm_pin_1,
	esp32_ledpwm_pin_2,
	esp32_ledpwm_pin_3,
	esp32_ledpwm_pin_size
};

static_assert((unsigned int)esp32_ledpwm_pin_size <= (unsigned int)io_int_value_size);
static_assert((unsigned int)esp32_ledpwm_pin_0 == (unsigned int)LedPWM::lpt_14bit_5khz_notify);
static_assert((unsigned int)esp32_ledpwm_pin_1 == (unsigned int)LedPWM::lpt_14bit_5khz_lcd_spi_2);
static_assert((unsigned int)esp32_ledpwm_pin_2 == (unsigned int)LedPWM::lpt_14bit_5khz_lcd_spi_3);
static_assert((unsigned int)esp32_ledpwm_pin_3 == (unsigned int)LedPWM::lpt_14bit_120hz);

static void esp32_ledpwm_info(const io_data_t *, std::string &)
{
	assert(inited);
}

static bool esp32_ledpwm_init(io_data_t *dataptr)
{
	LedPWM::ledpwm_t handle;

	assert(inited);
	assert(dataptr);

	for(handle = LedPWM::lpt_first; handle < LedPWM::lpt_size; handle = static_cast<LedPWM::ledpwm_t>(handle + 1))
	{
		dataptr->int_value[handle] = 0;

		try
		{
			LedPWM::get().open(handle, "I/O LED-PWM");
		}
		catch(const transient_exception &)
		{
			return(false);
		}

		dataptr->int_value[handle] = 1;
	}

	return(true);
}

static bool esp32_ledpwm_write(io_data_t *dataptr, unsigned int pin, unsigned int value)
{
	assert(inited);
	assert(dataptr);
	assert(pin < dataptr->info->pins);
	assert(pin < LedPWM::lpt_size);
	assert(value <= dataptr->info->max_value);

	if(!dataptr->int_value[pin])
		return(false);

	LedPWM::get().set(static_cast<LedPWM::ledpwm_t>(pin), value);

	return(true);
}

static void esp32_ledpwm_pin_info(const io_data_t *dataptr, unsigned int pin, std::string &result)
{
	assert(inited);
	assert(dataptr);
	assert(pin < dataptr->info->pins);
	assert(pin < Ledpixel::lp_size);

	if(dataptr->int_value[pin])
		result += std::format("LED-PWM channel {:d} duty: {:d}", pin, LedPWM::get().get(static_cast<LedPWM::ledpwm_t>(pin)));
	else
		result += "pin unvailable on this board";
}

enum
{
	esp32_pdm_pin_0 = 0,
	esp32_pdm_pin_1,
	esp32_pdm_pin_2,
	esp32_pdm_pin_3,
	esp32_pdm_pin_size
};

static_assert((unsigned int)esp32_pdm_pin_size <= (unsigned int)io_int_value_size);
static_assert((unsigned int)esp32_pdm_pin_0 == pdm_8bit_150khz_0);
static_assert((unsigned int)esp32_pdm_pin_1 == pdm_8bit_150khz_1);
static_assert((unsigned int)esp32_pdm_pin_2 == pdm_8bit_150khz_2);
static_assert((unsigned int)esp32_pdm_pin_3 == pdm_8bit_150khz_3);

static void esp32_pdm_info(const io_data_t *, std::string &)
{
	assert(inited);
}

static bool esp32_pdm_init(io_data_t *dataptr)
{
	pdm_t handle;
	bool rv = false;

	assert(inited);
	assert(dataptr);

	for(handle = pdm_first; handle < pdm_size; handle = static_cast<pdm_t>(handle + 1))
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

static void esp32_pdm_pin_info(const io_data_t *dataptr, unsigned int pin, std::string &result)
{
	assert(inited);
	assert(dataptr);
	assert(pin < dataptr->info->pins);
	assert(pin < pdm_size);

	if(dataptr->int_value[pin])
		result += std::format("PDM channel {:d} density: {:d}", pin, pdm_channel_get(static_cast<pdm_t>(pin)));
	else
		result += "pin unvailable on this board";
}

enum
{
	esp32_ledpixel_int_value_open = 0,
};

static_assert((unsigned int)esp32_ledpixel_int_value_open < (unsigned int)io_int_value_size);

static void esp32_ledpixel_info(const io_data_t *, std::string &)
{
	assert(inited);
}

static bool esp32_ledpixel_init(io_data_t *dataptr)
{
	assert(inited);
	assert(dataptr);

	dataptr->int_value[esp32_ledpixel_int_value_open] = 0;

	try
	{
		Ledpixel::get().open(dataptr->info->instance.ledpixel.instance, "I/O ledpixel");
	}
	catch(const transient_exception &)
	{
		return(false);
	}

	dataptr->int_value[esp32_ledpixel_int_value_open] = 1;

	return(true);
}

static bool esp32_ledpixel_write(io_data_t *dataptr, unsigned int pin, unsigned int value)
{
	assert(inited);
	assert(dataptr);
	assert(pin < dataptr->info->pins);
	assert(value <= dataptr->info->max_value);

	if(!dataptr->int_value[esp32_ledpixel_int_value_open])
		return(false);

	Ledpixel::get().set(dataptr->info->instance.ledpixel.instance, pin, (value & 0x00ff0000) >> 16, (value & 0x0000ff00) >> 8, (value & 0x000000ff) >> 0);
	Ledpixel::get().flush(dataptr->info->instance.ledpixel.instance);

	return(true);
}

static void esp32_ledpixel_pin_info(const io_data_t *dataptr, unsigned int pin, std::string &result)
{
	assert(inited);
	assert(dataptr);
	assert(pin < dataptr->info->pins);

	if(dataptr->int_value[esp32_ledpixel_int_value_open])
		result += std::format("LEDpixel instance {:d}", static_cast<unsigned int>(dataptr->info->instance.ledpixel.instance));
	else
		result += "pin unvailable on this board";
}

enum
{
	pcf8574_int_value_cache_in = 0,
	pcf8574_int_value_cache_out,
	pcf8574_int_value_size,
};

static_assert((unsigned int)pcf8574_int_value_size <= (unsigned int)io_int_value_size);

static void pcf8574_info(const io_data_t *dataptr, std::string &result)
{
	result += "\npin cache";
	result += std::format("\n- input  {:#02x}", dataptr->int_value[pcf8574_int_value_cache_in]);
	result += std::format("\n- output {:#02x}", dataptr->int_value[pcf8574_int_value_cache_out]);
}

static bool pcf8574_detect(const io_info_t *info, unsigned int module, unsigned int bus, unsigned int address)
{
	assert(inited);

	return(i2c_probe_slave(static_cast<i2c_module_t>(module), static_cast<i2c_bus_t>(bus), address));
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

static void pcf8574_pin_info(const io_data_t *dataptr, unsigned int pin, std::string &result)
{
	result += std::format("binary I/O, current I/O value: {:d}/{:d}",
			!(dataptr->int_value[pcf8574_int_value_cache_in] & (1 << pin)),
			!(dataptr->int_value[pcf8574_int_value_cache_out] & (1 << pin)));
}

static const io_info_t info[io_id_size] =
{
	[io_id_esp32_mcpwm] =
	{
		.id = io_id_esp32_mcpwm,
		.name = "ESP32 MC-PWM 16 bits",
		.caps = static_cast<io_capabilities_t>((1 << io_cap_output)),
		.pins = esp32_mcpwm_pin_size,
		.max_value = 65535,
		.bus = io_bus_apb,
		.instance = {},
		.info_fn = esp32_mcpwm_info,
		.detect_fn = nullptr,
		.init_fn = esp32_mcpwm_init,
		.read_fn = nullptr,
		.write_fn = esp32_mcpwm_write,
		.pin_info_fn = esp32_mcpwm_pin_info,
	},
	[io_id_esp32_ledpwm] =
	{
		.id = io_id_esp32_ledpwm,
		.name = "ESP32 LED-PWM 14 bits",
		.caps = static_cast<io_capabilities_t>((1 << io_cap_output)),
		.pins = esp32_ledpwm_pin_size,
		.max_value = 16383,
		.bus = io_bus_apb,
		.instance = {},
		.info_fn = esp32_ledpwm_info,
		.detect_fn = nullptr,
		.init_fn = esp32_ledpwm_init,
		.read_fn = nullptr,
		.write_fn = esp32_ledpwm_write,
		.pin_info_fn = esp32_ledpwm_pin_info,
	},
	[io_id_esp32_pdm] =
	{
		.id = io_id_esp32_pdm,
		.name = "ESP32 PDM 8 bits",
		.caps = static_cast<io_capabilities_t>((1 << io_cap_output)),
		.pins = esp32_pdm_pin_size,
		.max_value = 255,
		.bus = io_bus_apb,
		.instance = {},
		.info_fn = esp32_pdm_info,
		.detect_fn = nullptr,
		.init_fn = esp32_pdm_init,
		.read_fn = nullptr,
		.write_fn = esp32_pdm_write,
		.pin_info_fn = esp32_pdm_pin_info,
	},
	[io_id_esp32_ledpixel_0] =
	{
		.id = io_id_esp32_ledpixel_0,
		.name = "ESP32 LEDpixel 0",
		.caps = static_cast<io_capabilities_t>((1 << io_cap_output)),
		.pins = Ledpixel::ledpixel_leds_size,
		.max_value = 0x00ffffff,
		.bus = io_bus_apb,
		.instance
		{
			.ledpixel =
			{
				.instance = Ledpixel::lp_0_notify,
			},
		},
		.info_fn = esp32_ledpixel_info,
		.detect_fn = nullptr,
		.init_fn = esp32_ledpixel_init,
		.read_fn = nullptr,
		.write_fn = esp32_ledpixel_write,
		.pin_info_fn = esp32_ledpixel_pin_info,
	},
	[io_id_esp32_ledpixel_1] =
	{
		.id = io_id_esp32_ledpixel_1,
		.name = "ESP32 LEDpixel 1",
		.caps = static_cast<io_capabilities_t>((1 << io_cap_output)),
		.pins = Ledpixel::ledpixel_leds_size,
		.max_value = 0x00ffffff,
		.bus = io_bus_apb,
		.instance
		{
			.ledpixel
			{
				.instance = Ledpixel::lp_1,
			},
		},
		.info_fn = esp32_ledpixel_info,
		.detect_fn = nullptr,
		.init_fn = esp32_ledpixel_init,
		.read_fn = nullptr,
		.write_fn = esp32_ledpixel_write,
		.pin_info_fn = esp32_ledpixel_pin_info,
	},
	[io_id_esp32_ledpixel_2] =
	{
		.id = io_id_esp32_ledpixel_2,
		.name = "ESP32 LEDpixel 2",
		.caps = static_cast<io_capabilities_t>((1 << io_cap_output)),
		.pins = Ledpixel::ledpixel_leds_size,
		.max_value = 0x00ffffff,
		.bus = io_bus_apb,
		.instance
		{
			.ledpixel
			{
				.instance = Ledpixel::lp_2,
			},
		},
		.info_fn = esp32_ledpixel_info,
		.detect_fn = nullptr,
		.init_fn = esp32_ledpixel_init,
		.read_fn = nullptr,
		.write_fn = esp32_ledpixel_write,
		.pin_info_fn = esp32_ledpixel_pin_info,
	},
	[io_id_esp32_ledpixel_3] =
	{
		.id = io_id_esp32_ledpixel_3,
		.name = "ESP32 LEDpixel 3",
		.caps = static_cast<io_capabilities_t>((1 << io_cap_output)),
		.pins = Ledpixel::ledpixel_leds_size,
		.max_value = 0x00ffffff,
		.bus = io_bus_apb,
		.instance
		{
			.ledpixel
			{
				.instance = Ledpixel::lp_3,
			},
		},
		.info_fn = esp32_ledpixel_info,
		.detect_fn = nullptr,
		.init_fn = esp32_ledpixel_init,
		.read_fn = nullptr,
		.write_fn = esp32_ledpixel_write,
		.pin_info_fn = esp32_ledpixel_pin_info,
	},
	[io_id_pcf8574_26] =
	{
		.id = io_id_pcf8574_26,
		.name = "PCF8574 8-bit I/O expander",
		.caps = static_cast<io_capabilities_t>((1 << io_cap_input) | (1 << io_cap_output)),
		.pins = 8,
		.max_value = 1,
		.bus = io_bus_i2c,
		.instance
		{
			.i2c
			{
				.address = 0x26,
			},
		},
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
		.caps = static_cast<io_capabilities_t>((1 << io_cap_input) | (1 << io_cap_output)),
		.pins = 8,
		.max_value = 1,
		.bus = io_bus_i2c,
		.instance
		{
			.i2c
			{
				.address = 0x3a,
			},
		},
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
				log_format("io: find_io: bus %d unknown", dataptr->info->bus);
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

	for(id = io_id_first; id < io_id_size; id = static_cast<io_id_t>(id + 1))
	{
		infoptr = &info[id];

		assert(infoptr->id == id);

		switch(infoptr->bus)
		{
			case(io_bus_apb):
			{
				if(infoptr->detect_fn && !infoptr->detect_fn(infoptr, 0, 0, 0))
					continue;

				dataptr = new io_data_t;

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
				for(module = i2c_module_first; module < i2c_module_size; module = static_cast<i2c_module_t>(module + 1))
				{
					if(!i2c_module_available(module))
						continue;

					buses = i2c_buses(module);

					for(bus = i2c_bus_first; bus < buses; bus = static_cast<i2c_bus_t>(bus + 1))
					{
						if(find_io(infoptr->bus, (unsigned int)module, (unsigned int)bus, infoptr->instance.i2c.address))
						{
							stat_i2c_detect_skipped++;
							continue;
						}

						stat_i2c_detect_tried++;

						if(infoptr->detect_fn && !infoptr->detect_fn(infoptr, module, bus, infoptr->instance.i2c.address))
							continue;

						if(!(slave = i2c_register_slave(infoptr->name, module, bus, infoptr->instance.i2c.address)))
						{
							log_format("io: warning: cannot register io %s", infoptr->name);
							continue;
						}

						dataptr = new io_data_t;

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

static void io_info_x(std::string &result, const io_data_t *dataptr)
{
	io_capabilities_t cap;

	assert(inited);
	assert(dataptr);

	result += dataptr->info->name;;
	result += std::format("\n- id: {:d}", static_cast<unsigned int>(dataptr->info->id));
	result += std::format("\n- pins: {:d}", dataptr->info->pins);
	result += std::format("\n- max value per pin: {:d}", dataptr->info->max_value);

	result += "\n- capabilities:";

	for(cap = io_cap_first; cap < io_cap_size; cap = static_cast<io_capabilities_t>(cap + 1))
		if(dataptr->info->caps & (1 << cap))
			result += std::format(" {}", cap_to_string[cap]);

	result += "\n- extra device info: ";

	if(dataptr->info->info_fn)
		dataptr->info->info_fn(dataptr, result);
}

static bool io_read_x(std::string &result, io_data_t *dataptr, unsigned int pin, unsigned int *value)
{
	assert(inited);
	assert(dataptr);
	assert(value);

	if(!(dataptr->info->caps & (1 << io_cap_input)) || !dataptr->info->read_fn)
	{
		result += "not input capable";
		return(false);
	}

	if(pin >= dataptr->info->pins)
	{
		result += std::format("no such pin {:d}", pin);
		return(false);
	}

	if(!dataptr->info->read_fn(dataptr, pin, value))
	{
		result += "read failed";
		return(false);
	}

	return(true);
}

static bool io_write_x(std::string &result, io_data_t *dataptr, unsigned int pin, unsigned int value)
{
	assert(inited);
	assert(dataptr);

	if(!(dataptr->info->caps & (1 << io_cap_output)))
	{
		result += "not output capable";
		return(false);
	}

	if(pin >= dataptr->info->pins)
	{
		result += std::format("no such pin {:d}", pin);
		return(false);
	}

	if(value > dataptr->info->max_value)
	{
		result += std::format("value {:d} out of range", value);
		return(false);
	}

	assert(dataptr->info->write_fn);

	if(!dataptr->info->write_fn(dataptr, pin, value))
	{
		result += "write failed";
		return(false);
	}

	return(true);
}

static void io_pin_info_x(std::string &result, const io_data_t *dataptr, unsigned int pin)
{
	if(dataptr->info->pin_info_fn)
		dataptr->info->pin_info_fn(dataptr, pin, result);
}

bool io_info(std::string &result, unsigned int io)
{
	const io_data_t *dataptr;

	assert(inited);

	if(!(dataptr = get_data(io)))
	{
		result += std::format("no such I/O {:d}", io);
		return(false);
	}

	io_info_x(result, dataptr);

	return(true);
}

bool io_read(std::string &result, unsigned int io, unsigned int pin, unsigned int *value)
{
	io_data_t *dataptr;

	assert(inited);

	if(!(dataptr = get_data(io)))
	{
		result += std::format("no such I/O {:d}", io);
		return(false);
	}

	return(io_read_x(result, dataptr, pin, value));
}

bool io_write(std::string &result, unsigned int io, unsigned int pin, unsigned int value)
{
	io_data_t *dataptr;

	assert(inited);

	if(!(dataptr = get_data(io)))
	{
		result += std::format("no such I/O {:d}", io);
		return(false);
	}

	return(io_write_x(result, dataptr, pin, value));
}

bool io_pin_info(std::string &result, unsigned int io, unsigned int pin)
{
	io_data_t *dataptr;

	assert(inited);

	if(!(dataptr = get_data(io)))
	{
		result += std::format("no such I/O {:d}", io);
		return(false);
	}

	if(pin >= dataptr->info->pins)
	{
		result += std::format("no such pin {:d}", pin);
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

	call->result = "I/O DUMP";

	sequence = 0;

	data_mutex_take();

	for(dataptr = data_root; dataptr; dataptr = dataptr->next)
	{
		call->result += std::format("\n[{:d}]: ", sequence);

		sequence++;

		io_info_x(call->result, dataptr);

		switch(dataptr->info->bus)
		{
			case(io_bus_apb):
			{
				call->result += "\nbus info\n- APB device";
				break;
			}

			case(io_bus_i2c):
			{
				i2c_get_slave_info(dataptr->i2c.slave, &module, &bus, &address, &name);
				call->result += std::format("\nbus info\n- I2C device {} at {:d}/{:d}/{:#x}", name, static_cast<unsigned int>(module), static_cast<unsigned int>(bus), address);

				break;
			}

			default:
			{
				call->result += std::format(" unknown IO type {:d}: {}", static_cast<unsigned int>(dataptr->info->bus), dataptr->info->name);
			}
		}
		call->result += "\npins:";

		for(pin = 0; pin < dataptr->info->pins; pin++)
		{
			call->result += std::format("\n- pin {:d}: ", pin);
			io_pin_info_x(call->result, dataptr, pin);
		}
	}

	data_mutex_give();
}

void command_io_stats(cli_command_call_t *call)
{
	assert(inited);
	assert(call->parameter_count == 0);

	call->result = "IO STATS";
	call->result += "\n- detecting";
	call->result += std::format("\n-  skipped: {:d}", stat_i2c_detect_skipped);
	call->result += std::format("\n-  tried: {:d}", stat_i2c_detect_tried);
	call->result += std::format("\n-  found: {:d}", stat_i2c_detect_found);
}

void command_io_read(cli_command_call_t *call)
{
	unsigned int value;
	std::string result;

	assert(inited);
	assert(call->parameter_count == 2);

	if(io_read(result, call->parameters[0].unsigned_int, call->parameters[1].unsigned_int, &value))
		call->result = std::format("io-read {:d}/{:d}: {:d} OK", call->parameters[0].unsigned_int, call->parameters[1].unsigned_int, value);
	else
		call->result = std::format("io-read {:d}/{:d}: {:d}: {}", call->parameters[0].unsigned_int, call->parameters[1].unsigned_int, value, result);
}

void command_io_write(cli_command_call_t *call)
{
	std::string result;

	assert(inited);
	assert(call->parameter_count == 3);

	if(io_write(result, call->parameters[0].unsigned_int, call->parameters[1].unsigned_int, call->parameters[2].unsigned_int))
		call->result = std::format("io-write {:d}/{:d}: {:d} OK", call->parameters[0].unsigned_int, call->parameters[1].unsigned_int, call->parameters[2].unsigned_int);
	else
		call->result = std::format("io-write {:d}/{:d}: {:d}: {}", call->parameters[0].unsigned_int, call->parameters[1].unsigned_int, call->parameters[2].unsigned_int, result);
}
