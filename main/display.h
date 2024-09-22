#pragma once

#include "string.h"

typedef enum
{
	dt_no_display = 0,
	dt_spi_generic,
	dt_type_first = dt_spi_generic,
	dt_error,
	dt_size = dt_error,
} display_type_t;

enum
{
	display_colours = 8,
};

typedef struct
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
} display_rgb_t;

static_assert(sizeof(display_rgb_t) == 3);

extern const display_rgb_t display_colour_map[display_colours];
extern unsigned int display_pixel_buffer_size;
extern uint8_t *display_pixel_buffer;

typedef struct
{
	int interface_index;
	int x_size;
	int y_size;
	int x_offset;
	int y_offset;
	int x_mirror;
	int y_mirror;
	int rotate;
	int invert;
} display_init_parameters_t;

enum
{
	font_basic_glyphs_size = 256,
	font_extra_glyphs_size = 128,
	font_cols_size = 16,
	font_rows_size = 32,
};

typedef enum
{
	font_magic_word = 0xf0bdf11e,
} font_magic_word_t;

typedef struct __attribute__((packed))
{
	uint32_t codepoint;
	uint16_t row[font_rows_size];
} font_glyph_t;

typedef struct __attribute__((packed))
{
	font_magic_word_t magic_word:32;
	uint8_t checksum[32];
	struct
	{
		uint32_t width;
		uint32_t height;
	} raw;
	struct
	{
		uint32_t width;
		uint32_t height;
	} net;
	uint32_t extra_glyphs;
	font_glyph_t basic_glyph[font_basic_glyphs_size];
	font_glyph_t extra_glyph[font_extra_glyphs_size];
} font_t;

typedef struct
{
	const char *name;
	bool (*init_fn)(const display_init_parameters_t *parameters, unsigned int *buffer_size);
	void (*bright_fn)(unsigned int percentage);
	void (*write_fn)(const font_t *font, unsigned int row, unsigned int line_unicode_length, const uint32_t *line_unicode);
	void (*clear_fn)(void);
	void (*scroll_fn)(unsigned int lines);
} display_info_t;

void display_init(void);
bool display_brightness(unsigned int percentage);
bool display_write(bool scroll, const uint8_t *line);
bool display_clear(void);

bool display_spi_generic_init(const display_init_parameters_t *parameters, unsigned int *buffer_size);
void display_spi_generic_bright(unsigned int percentage);
void display_spi_generic_write(const font_t *font, unsigned int row, unsigned int line_unicode_length, const uint32_t *line_unicode);
void display_spi_generic_clear(void);
