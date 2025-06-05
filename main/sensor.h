#pragma once

typedef enum
{
	sensor_bh1750 = 0,
	sensor_first = sensor_bh1750,
	sensor_tmp75,
	sensor_opt3001,
	sensor_max44009,
	sensor_asair,
	sensor_apds9930,
	sensor_tsl2561,
	sensor_hdc1080,
	sensor_sht3x,
	sensor_bmx280,
	sensor_htu21,
	sensor_veml7700,
	sensor_bme680,
	sensor_apds9960,
	sensor_tsl2591,
	sensor_tsl2591_28, // this must come after sensor_tsl2591
	sensor_am2320,
	sensor_size,
	sensor_error = sensor_size,
	sensor_none = sensor_size,
} sensor_t;

typedef enum
{
	sensor_type_visible_light,
	sensor_type_first = sensor_type_visible_light,
	sensor_type_temperature,
	sensor_type_humidity,
	sensor_type_airpressure,
	sensor_type_size,
	sensor_type_error = sensor_type_size,
} sensor_type_t;

static_assert(sensor_type_size < 32); // used as bitmask

void sensor_init(void);
