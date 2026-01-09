#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
	mpt_16bit_150hz_0 = 0,
	mpt_16bit_150hz_1,
	mpt_16bit_2400hz_0,
	mpt_16bit_2400hz_1,
	mpt_size,
	mpt_error = mpt_size,
	mpt_first = mpt_16bit_150hz_0,
} mcpwm_t;

bool mcpwm_open(mcpwm_t handle, const char *owner);
void mcpwm_set(mcpwm_t handle, unsigned int duty);
unsigned int mcpwm_get(mcpwm_t handle);

#ifdef __cplusplus
}
#endif

void mcpwm_init(void);
