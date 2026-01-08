#pragma once

#include "string.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
	dt_no_display = 0,
	dt_spi_generic,
	dt_type_first = dt_spi_generic,
	dt_ra8875,
	dt_error,
	dt_size = dt_error,
} display_type_t;

typedef enum
{
	dc_black = 0,
	dc_blue,
	dc_green,
	dc_cyan,
	dc_red,
	dc_purple,
	dc_yellow,
	dc_white,
	dc_error,
	dc_size = dc_error,
} display_colour_t;

typedef struct
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
} display_rgb_t;

static_assert(sizeof(display_rgb_t) == 3);

extern const display_rgb_t display_colour_map[dc_size];

typedef struct
{
	int interface_index;
	int x_size;
	int y_size;
	int flip;
	int invert;
	int rotate;
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
	bool (*init_fn)(const display_init_parameters_t *parameters);
	void (*bright_fn)(unsigned int percentage);
	void (*write_fn)(const font_t *font, display_colour_t fg, display_colour_t bg,
				unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y,
				unsigned int line_unicode_length, const uint32_t *line_unicode);
	void (*clear_fn)(display_colour_t);
	void (*box_fn)(display_colour_t, unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y);
	void (*plot_line_fn)(unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int rgb_pixels_length, const display_rgb_t *pixels);
	void (*set_layer_fn)(unsigned int layer);
	void (*show_layer_fn)(unsigned int layer);
} display_info_t;

unsigned int display_image_x_size(void);
unsigned int display_image_y_size(void);

bool display_spi_generic_init(const display_init_parameters_t *parameters);
void display_spi_generic_bright(unsigned int percentage);
void display_spi_generic_write(const font_t *font, display_colour_t fg, display_colour_t bg,
		unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y,
		unsigned int line_unicode_length, const uint32_t *line_unicode);
void display_spi_generic_clear(display_colour_t);
void display_spi_generic_box(display_colour_t, unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y);
void display_spi_generic_plot_line(unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int rgb_pixels_length, const display_rgb_t *pixels);

bool display_ra8875_init(const display_init_parameters_t *parameters);
void display_ra8875_bright(unsigned int percentage);
void display_ra8875_write(const font_t *font, display_colour_t fg, display_colour_t bg,
		unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y,
		unsigned int line_unicode_length, const uint32_t *line_unicode);
void display_ra8875_clear(display_colour_t);
void display_ra8875_box(display_colour_t, unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y);
void display_ra8875_plot_line(unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int rgb_pixels_length, const display_rgb_t *pixels);
void display_ra8875_set_layer(unsigned int layer);
void display_ra8875_show_layer(unsigned int layer);
#ifdef __cplusplus
}
#endif

void display_init(void);
