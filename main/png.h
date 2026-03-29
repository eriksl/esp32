#pragma once

#include <stdbool.h>

typedef struct
{
	unsigned int width;
	unsigned int height;
	unsigned int row_size;
	unsigned int image_size;
	void *file;
	void *ctx;
} png_handle_t;

#ifdef __cplusplus
extern "C" {
#endif
png_handle_t *png_open(const char *filename);
bool png_decode_row(png_handle_t *, unsigned int, uint8_t *);
void png_close(png_handle_t **);
#ifdef __cplusplus
}
#endif
