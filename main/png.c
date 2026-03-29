#include <spng.h>
#include <stdio.h>

#include "png.h"

enum
{
	decode_format = SPNG_FMT_RGB8,
};

static png_handle_t *PNG_free(png_handle_t **handle)
{
	if(handle && *handle)
	{
		if((**handle).file)
		{
			fclose((FILE *)(**handle).file);
			(**handle).file = nullptr;
		}

		if((**handle).ctx)
		{
			spng_ctx_free((spng_ctx *)(**handle).ctx);
			(**handle).ctx = nullptr;
		}

		free(*handle);
		*handle = nullptr;
	}

	return(nullptr);
}

static png_handle_t *PNG_init(const char *filename)
{
	png_handle_t *handle;
	struct spng_ihdr ihdr;

	if(!(handle = malloc(sizeof(png_handle_t))))
		return(nullptr);

	handle->file = nullptr;
	handle->ctx = nullptr;

	if(!(handle->file = fopen(filename, "r")))
		return(PNG_free(&handle));

	if(!(handle->ctx = spng_ctx_new(0)))
		return(PNG_free(&handle));

	spng_set_crc_action((spng_ctx *)handle->ctx, SPNG_CRC_ERROR, SPNG_CRC_DISCARD);
	spng_set_chunk_limits((spng_ctx *)handle->ctx, 1024, 1024);
	spng_set_png_file((spng_ctx *)handle->ctx, (FILE *)handle->file);

	if(!!(spng_get_ihdr((spng_ctx *)handle->ctx, &ihdr)))
		return(PNG_free(&handle));

	if(!!(spng_decoded_image_size((spng_ctx *)handle->ctx, decode_format, &handle->image_size)))
		return(PNG_free(&handle));

	handle->width = ihdr.width;
	handle->height = ihdr.height;
	handle->row_size = handle->image_size / ihdr.height;

	return(handle);
}

png_handle_t *png_open(const char *filename)
{
	png_handle_t *handle;

	if(!(handle = PNG_init(filename)))
		return(nullptr);

	if(spng_decode_image((spng_ctx *)handle->ctx, nullptr, 0, decode_format, SPNG_DECODE_PROGRESSIVE))
	{
		PNG_free(&handle);
		return(nullptr);
	}

	return((png_handle_t *)handle);
}

bool png_decode_row(png_handle_t *handle, unsigned int size, uint8_t *buffer)
{
	if(handle->height == 0)
		return(false);

	if(size < handle->row_size)
		return(false);

	if(spng_decode_row((spng_ctx *)handle->ctx, buffer, size))
		return(false);

	return(true);
}

void png_close(png_handle_t **handle)
{
	PNG_free(handle);
}
