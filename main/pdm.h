#pragma once

typedef enum
{
	pdm_8bit_150khz_0 = 0,
	pdm_first = pdm_8bit_150khz_0,
	pdm_8bit_150khz_1,
	pdm_8bit_150khz_2,
	pdm_8bit_150khz_3,
	pdm_size,
	pdm_error = pdm_size,
} pdm_t;

bool pdm_channel_open(pdm_t handle, const char *owner);
void pdm_channel_set(pdm_t handle, unsigned int density);
unsigned int pdm_channel_get(pdm_t handle);

void pdm_init(void);
