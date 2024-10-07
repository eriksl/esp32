#pragma once

typedef enum
{
	sensor_bh1750 = 0,
	sensor_first = sensor_bh1750,
	sensor_size,
	sensor_error = sensor_size,
	sensor_none = sensor_size,
} sensor_t;

typedef enum
{
	sensor_type_none = 0,
	sensor_type_visible_light = (1 << 0),
} sensor_type_t;

void sensor_init(void);
