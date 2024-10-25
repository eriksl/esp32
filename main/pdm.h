#pragma once

struct pdm_opaque_t {} __attribute__((aligned(sizeof(uint64_t))));

typedef struct pdm_opaque_t *pdm_t;
typedef const struct pdm_opaque_t *const_pdm_t;

void pdm_init(void);
pdm_t pdm_channel_new(unsigned int gpio, const char *name);
void pdm_channel_set(pdm_t channel, unsigned int density);
unsigned int pdm_channel_get(const const_pdm_t channel);
