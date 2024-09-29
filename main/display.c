#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <mbedtls/sha256.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <freertos/FreeRTOS.h>

#include "string.h"
#include "log.h"
#include "util.h"
#include "display.h"
#include "config.h"
#include "cli-command.h"

static_assert(sizeof(font_glyph_t) == 68);
static_assert(offsetof(font_t, basic_glyph) == 56);
static_assert(offsetof(font_t, extra_glyph) == 17464);

enum
{
	unicode_buffer_size = 64,
	display_page_lines_size = 12,
	page_border_size = 3,
	page_text_offset = 1,
};

typedef enum
{
	dv_start = 0,
	dv_type = dv_start,
	dv_if,
	dv_x_size,
	dv_y_size,
	dv_x_mirror,
	dv_y_mirror,
	dv_rotate,
	dv_invert,
	dv_error,
	dv_size = dv_error,
} dv_t;

typedef enum
{
	dpt_text,
	dpt_image,
} display_page_type_t;

typedef struct display_page_T
{
	struct display_page_T *next;
	string_t name;
	time_t expiry;
	display_page_type_t type;
	union
	{
		struct
		{
			string_t line[display_page_lines_size];
		} text;
		struct
		{
			string_t filename;
		} image;
	};
} display_page_t;

static const char * const display_variable[dv_size][3] =
{
	[dv_type] =		{	"type",				"display.type",		"display type, 0 = generic SPI LCD",	},
	[dv_if] =		{	"interface",		"display.if",		"interface, 0 = SPI2, 1 = SPI3",		},
	[dv_x_size] =	{	"x size",			"display.x.size",	"x size (width)",						},
	[dv_y_size] =	{	"y size",			"display.y.size",	"y size (height)",						},
	[dv_x_mirror] = {	"x mirror",			"display.x.mirror",	"x mirror (optional)",					},
	[dv_y_mirror] = {	"y mirror",			"display.y.mirror",	"y mirror (optional)",					},
	[dv_rotate] =	{	"rotate",			"display.rotate",	"rotate display (optional)",			},
	[dv_invert] =	{	"invert",			"display.invert",	"invert display (optional)",			},
};

static const display_info_t info[dt_size] =
{
	[dt_no_display] =
	{
		.name = "No display",
		.init_fn = (void *)0,
		.bright_fn = (void *)0,
		.write_fn = (void *)0,
		.clear_fn = (void *)0,
		.box_fn = (void *)0,
		.scroll_fn = (void *)0,
	},
	[dt_spi_generic] =
	{
		.name = "Generic SPI LCD display",
		.init_fn = display_spi_generic_init,
		.bright_fn = display_spi_generic_bright,
		.write_fn = display_spi_generic_write,
		.clear_fn = display_spi_generic_clear,
		.box_fn = display_spi_generic_box,
		.scroll_fn = (void *)0,
	},
};
const display_rgb_t display_colour_map[dc_size] =
{
	[dc_black] =	{	0x00,	0x00,	0x00	},
	[dc_blue] =		{	0x00,	0x00,	0xff	},
	[dc_green] =	{	0x00,	0x88,	0x00	},
	[dc_cyan] =		{	0x00,	0xaa,	0xaa	},
	[dc_red] =		{	0xff,	0x00,	0x00	},
	[dc_purple] =	{	0xff,	0x00,	0xff	},
	[dc_yellow] =	{	0xff,	0xbb,	0x00	},
	[dc_white] =	{	0xff,	0xff,	0xff	},
};

static display_page_t *display_pages = (display_page_t *)0;
static display_type_t display_type = dt_no_display;
static font_t *font = (font_t *)0;
static bool font_valid = false;
static uint32_t *unicode_buffer = (uint32_t *)0;
static unsigned int columns, rows;
static unsigned int x_size, y_size;
static QueueHandle_t log_display_queue;
static bool log_mode = true;
static SemaphoreHandle_t page_data_mutex;
static unsigned int display_log_y;

static inline void page_data_mutex_take(void)
{
	assert(xSemaphoreTake(page_data_mutex, portMAX_DELAY));
}

static inline void page_data_mutex_give(void)
{
	assert(xSemaphoreGive(page_data_mutex));
}

static unsigned int utf8_to_unicode(const uint8_t *src, unsigned int dst_size, uint32_t *dst)
{
	enum
	{
		u8p_state_base = 0,
		u8p_state_utf8_byte_3 = 1,
		u8p_state_utf8_byte_2 = 2,
		u8p_state_utf8_byte_1 = 3,
		u8p_state_done = 4
	} state;

	unsigned int src_current;
	unsigned int src_index;
	unsigned int dst_index;
	unsigned int unicode;

	for(src_index = 0, dst_index = 0, state = u8p_state_base, unicode = 0; src && dst && (dst_index < dst_size) && src[src_index]; src_index++)
	{
		src_current = src[src_index];

		switch(state)
		{
			case u8p_state_base:
			{
				if((src_current & 0xe0) == 0xc0) // first of two bytes (11 bits)
				{
					unicode = src_current & 0x1f;
					state = u8p_state_utf8_byte_1;
					continue;
				}
				else
					if((src_current & 0xf0) == 0xe0) // first of three bytes (16 bits)
					{
						unicode = src_current & 0x0f;
						state = u8p_state_utf8_byte_2;
						continue;
					}
					else
						if((src_current & 0xf8) == 0xf0) // first of four bytes (21 bits)
						{
							unicode = src_current & 0x07;
							state = u8p_state_utf8_byte_3;
							continue;
						}
						else
							if((src_current & 0x80) == 0x80)
							{
								log_format("utf8 parser: invalid utf8, bit 7 set: %x %c\n", src_current, (int)src_current);
								unicode = '*';
							}
							else
								unicode = src_current & 0x7f;

				break;
			}

			case u8p_state_utf8_byte_3 ... u8p_state_utf8_byte_1:
			{
				if((src_current & 0xc0) == 0x80) // following bytes
				{
					unicode = (unicode << 6) | (src_current & 0x3f);

					if(++state != u8p_state_done)
						continue;
				}
				else
				{
					log_format("utf8 parser: invalid utf8, no prefix on following byte, state: %u: %x %c\n", state, src_current, (int)src_current);
					unicode = '*';
				}

				break;
			}

			default:
			{
				log_format("utf8 parser: invalid state %u\n", state);
				unicode = '*';
			}
		}

		dst[dst_index++] = unicode;
		unicode = 0;
		state = u8p_state_base;
	}

	return(dst_index);
}

static void page_free(display_page_t **page)
{
	unsigned int ix;

	if((**page).type == dpt_text)
	{
		for(ix = 0; ix < display_page_lines_size; ix++)
			if((**page).text.line[ix])
				string_free(&(**page).text.line[ix]);
	}
	else
		string_free(&(**page).image.filename);

	string_free(&(**page).name);

	*page = (display_page_t *)0;
}

static void page_add(display_page_t *new_page)
{
	display_page_t root, *before, *current;

	assert(new_page);

	page_data_mutex_take();

	root.next = display_pages;

	for(before = &root, current = root.next; before && current; before = before->next, current = current->next)
		if(string_equal_string(current->name, new_page->name))
			break;

	if(current && string_equal_string(current->name, new_page->name))
	{
		new_page->next = current->next;

		if(before == &root)
			display_pages = new_page;
		else
			before->next = new_page;

		page_free(&current);
	}
	else
	{
		new_page->next = (display_page_t *)0;

		if(before == &root)
			display_pages = new_page;
		else
			before->next = new_page;
	}

	page_data_mutex_give();
}

static void page_erase(const const_string_t name)
{
	display_page_t root, *before, *current;

	assert(name);

	page_data_mutex_take();

	root.next = display_pages;

	for(before = &root, current = root.next; before && current; before = before->next, current = current->next)
		if(string_equal_string(current->name, name))
			break;

	if(current && string_equal_string(current->name, name))
	{
		if(before == &root)
			display_pages = current->next;
		else
			before->next = current->next;

		page_free(&current);
	}

	page_data_mutex_give();
}

static void page_add_text(const const_string_t name, unsigned int lifetime, const const_string_t contents)
{
	string_auto(line_string, 64);
	unsigned int ix, line;
	display_page_t *new_page;
	uint8_t previous, current;

	new_page = util_memory_alloc_spiram(sizeof(*new_page));

	new_page->name = string_dup(name);
	new_page->type = dpt_text;
	new_page->expiry = time((time_t *)0) + lifetime;

	current = '\0';
	previous = '\0';
	line = 0;
	string_clear(line_string);

	for(ix = 0; (ix < string_length(contents)) && (line < display_page_lines_size); ix++)
	{
		previous = current;
		current = string_at(contents, ix);

		if((ix > 0) && (previous == '\\') && (current == 'n'))
		{
			if(string_at_back(line_string) == 'n')
				string_pop_back(line_string);

			if(string_at_back(line_string) == '\\')
				string_pop_back(line_string);

			goto new_line;
		}

		if(current == '\n')
		{
			if(string_at_back(line_string) == '\n')
				string_pop_back(line_string);

			goto new_line;
		}

		string_append(line_string, current);
		continue;

new_line:
		new_page->text.line[line++] = string_dup(line_string);
		string_clear(line_string);
		previous = '\0';
		current = '\0';
	}

	if(string_length(line_string) > 0)
		new_page->text.line[line++] = string_dup(line_string);

	for(; line < display_page_lines_size; line++)
		new_page->text.line[line] = (string_t)0;

	page_add(new_page);
}

static void page_add_image(const const_string_t name, unsigned int lifetime, const const_string_t filename)
{
	display_page_t *new_page;

	new_page = util_memory_alloc_spiram(sizeof(*new_page));

	new_page->name = string_dup(name);
	new_page->type = dpt_image;
	new_page->expiry = time((time_t *)0) + lifetime;
	new_page->image.filename = string_dup(filename);

	page_add(new_page);
}

static bool load_font(const char *fontname)
{
	string_auto(pathfont, 32);
	int fd;
	mbedtls_sha256_context hash_ctx;
	uint8_t our_hash[32];
	uint8_t their_hash[32];

	string_assign_cstr(pathfont, "/littlefs/");
	string_append_cstr(pathfont, fontname);

	if((fd = open(string_cstr(pathfont), O_RDONLY, 0)) < 0)
	{
		log_format("display: failed to open font %s", string_cstr(pathfont));
		goto error;
	}

	if(!font)
		font = util_memory_alloc_spiram(sizeof(font_t));

	if(read(fd, font, sizeof(*font)) != sizeof(*font))
	{
		log_format("display: failed to read font %s", string_cstr(pathfont));
		goto error;
	}

	close(fd);
	fd = -1;

	if(font->magic_word != font_magic_word)
	{
		log_format("display: font file magic word invalid: %x", font->magic_word);
		goto error;
	}

	memcpy(their_hash, font->checksum, sizeof(their_hash));
	memset(font->checksum, 0, sizeof(font->checksum));

	mbedtls_sha256_init(&hash_ctx);
	mbedtls_sha256_starts(&hash_ctx, /* no SHA-224 */ 0);
	mbedtls_sha256_update(&hash_ctx, (const uint8_t *)font, sizeof(*font));
	mbedtls_sha256_finish(&hash_ctx, our_hash);
	mbedtls_sha256_free(&hash_ctx);

	if(memcmp(our_hash, their_hash, sizeof(our_hash)))
	{
		log("display: font file invalid checksum");
		goto error;
	}

	columns = (x_size - (2 * page_border_size)) / font->net.width;
	rows = (y_size - page_text_offset - (2 * page_border_size)) / font->net.height;

	return(true);

error:
	if(fd >= 0)
		close(fd);

	if(font)
		free(font);

	columns = 0;
	rows = 0;

	return(false);
}

static bool clear(display_colour_t bg)
{
	if((display_type == dt_no_display) || !info[display_type].clear_fn)
		return(false);

	info[display_type].clear_fn(bg);

	return(true);
}

static bool box(display_colour_t colour, unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y)
{
	if((display_type == dt_no_display) || !info[display_type].box_fn)
		return(false);

	info[display_type].box_fn(colour, from_x, from_y, to_x, to_y);

	return(true);
}


static void run_display_log(void *)
{
	unsigned int entry;
	time_t stamp;
	struct tm tm;
	char entry_text[64];
	char log_line[64];
	unsigned int unicode_length;

	assert(log_display_queue);

	display_log_y = 0;

	for(;;)
	{
		assert(xQueueReceive(log_display_queue, &entry, portMAX_DELAY));

		if(font_valid && log_mode && (display_type != dt_no_display) && info[display_type].write_fn)
		{
			log_get_entry(entry, &stamp, sizeof(entry_text), entry_text);
			localtime_r(&stamp, &tm);
			strftime(log_line, sizeof(log_line), "%H:%M:%S ", &tm);
			strlcat(log_line, entry_text, sizeof(log_line));
			unicode_length = utf8_to_unicode((uint8_t *)log_line, unicode_buffer_size, unicode_buffer);

			if(info[display_type].scroll_fn && ((display_log_y + font->net.height) >= y_size))
			{
				info[display_type].scroll_fn(1);
				display_log_y = y_size - font->net.height - 1;
			}

			info[display_type].write_fn(font, dc_white, dc_black, 0, display_log_y, x_size - 1, display_log_y + font->net.height - 1, unicode_length, unicode_buffer);

			display_log_y += font->net.height;

			if(!info[display_type].scroll_fn)
			{
				if((display_log_y + font->net.height) > y_size)
					display_log_y = 0;

				box(dc_black, 0, display_log_y, x_size - 1, display_log_y + font->net.height - 1);
			}
		}
	}

	util_abort("run_display_log returns");
}

static void run_display_info(void *)
{
	display_page_t *display_pages_current, *display_pages_next;
	display_colour_t current_colour;
	unsigned int row, y;
	int pad, chop;
	unsigned int unicode_length;
	time_t stamp;
	struct tm tm;
	char stamp_line[16];
	char title_line[64];
	void (*write_fn)(const font_t *font, display_colour_t fg, display_colour_t bg,
				unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y,
				unsigned int line_unicode_length, const uint32_t *line_unicode);

	display_pages_current = display_pages;
	current_colour = dc_black + 1;

	for(;;)
	{
		if(!font_valid || (display_type == dt_no_display) || (!(write_fn = info[display_type].write_fn)))
			goto next2;

		page_data_mutex_take();

		if(!display_pages_current)
		{
			display_pages_current = display_pages;
			current_colour = dc_black + 1;

			if(!display_pages_current)
			{
				if(!log_mode)
				{
					font_valid = load_font("font_small");
					log_mode = true;
					clear(dc_black);
					display_log_y = 0;
				}

				goto next1;
			}
		}

		if(log_mode)
		{
			log_mode = false;

			if(!((font_valid = load_font("font_big"))))
				goto next1;

			current_colour = dc_black + 1;
		}

		assert(page_border_size > 0);

		box(current_colour,	0,										0,										x_size - 1,				page_border_size - 1);
		box(current_colour,	(x_size - 1) - (page_border_size - 1),	0,										x_size - 1,				y_size - 1);
		box(current_colour,	0,										(y_size - 1) - (page_border_size - 1),	x_size - 1,				y_size - 1);
		box(current_colour,	0,										0,										page_border_size - 1,	y_size - 1);

		stamp = time((time_t *)0);
		localtime_r(&stamp, &tm);
		strftime(stamp_line, sizeof(stamp_line), "%d/%m %H:%M", &tm);

		if(strlen(stamp_line) > columns)
		{
			chop = string_length_utf8(display_pages_current->name);

			if(chop > columns)
			{
				chop = columns;
				pad = 0;
			}
			else
				pad = columns - string_length_utf8(display_pages_current->name);

			assert(pad >= 0);

			snprintf(title_line, sizeof(title_line), "%.*s%*s",
					chop, string_cstr(display_pages_current->name), pad, "");
		}
		else
		{
			chop = string_length_utf8(display_pages_current->name);

			if((chop + strlen(stamp_line)) > columns)
				chop = columns - strlen(stamp_line);

			pad = columns - strlen(stamp_line) - chop;

			assert(chop >= 0);
			assert(pad >= 0);

			snprintf(title_line, sizeof(title_line), "%.*s%*s%s",
				chop, string_cstr(display_pages_current->name), pad, "", stamp_line);
		}

		unicode_length = utf8_to_unicode((uint8_t *)title_line, unicode_buffer_size, unicode_buffer);
		write_fn(font, dc_white, current_colour,
				page_border_size, page_border_size, (x_size - 1) - page_border_size, page_text_offset + page_border_size + (font->net.height - 1),
				unicode_length, unicode_buffer);

		y = font->net.height + page_border_size + page_text_offset;

		for(row = 0; (row < display_page_lines_size) && display_pages_current->text.line[row]; row++)
		{
			if((y + font->net.height) > y_size)
				break;

			unicode_length = utf8_to_unicode((uint8_t *)string_cstr(display_pages_current->text.line[row]), unicode_buffer_size, unicode_buffer);
			write_fn(font, dc_black, dc_white,
					page_border_size, y, (x_size - 1) - page_border_size, y + (font->net.height - 1),
					unicode_length, unicode_buffer);

			y += font->net.height;
		}

		box(dc_white, page_border_size, y, (x_size - 1) - page_border_size, (y_size - 1) - page_border_size);

		current_colour++;

		if((current_colour == dc_white) || (current_colour == dc_black))
			current_colour++;

		if(current_colour >= dc_size)
			current_colour = dc_black + 1;

		if(time((time_t *)0) > display_pages_current->expiry)
		{
			display_pages_next = display_pages_current->next;
			page_data_mutex_give();
			page_erase(display_pages_current->name);
			page_data_mutex_take();
			display_pages_current = display_pages_next;
		}
		else
			display_pages_current = display_pages_current->next;

next1:
		page_data_mutex_give();
next2:
		util_sleep(2000);
	}

	util_abort("run_display_info returns");
}

static void display_info(string_t output)
{
	uint32_t value;
	unsigned int found;
	dv_t ix;
	unsigned int line;
	display_page_t *current_page;
	string_auto(datetime, 32);

	string_assign_cstr(output, "DISPLAY configuration:");

	for(ix = dv_start, found = 0; ix < dv_size; ix++)
	{
		if(config_get_uint_cstr(display_variable[ix][1], &value))
		{
			found++;
			string_format_append(output, "\n- %s: %u", display_variable[ix][0], (unsigned int)value);
		}
	}

	if(found == 0)
	{
		string_assign_cstr(output, "\n- no display configuration found");
		return;
	}

	if(!font_valid)
	{
		string_assign_cstr(output, "\n- no display font loaded");
		return;
	}

	string_format_append(output, "\nDISPLAY current type %s, font info: ", info[display_type].name);
	string_format_append(output, "\n- magic word: %08x", font->magic_word);
	string_format_append(output, "\n- raw width: %u", (unsigned int)font->raw.width);
	string_format_append(output, "\n- raw height: %u", (unsigned int)font->raw.height);
	string_format_append(output, "\n- net width: %u", (unsigned int)font->net.width);
	string_format_append(output, "\n- net height: %u", (unsigned int)font->net.height);
	string_format_append(output, "\n- basic glyphs: %u", font_basic_glyphs_size);
	string_format_append(output, "\n- extra glyphs: %u", (unsigned int)font->extra_glyphs);
	string_format_append(output, "\n- columns: %u", columns);
	string_format_append(output, "\n- rows: %u", rows);

	string_append_cstr(output, "\nPAGES:");

	page_data_mutex_take();

	for(current_page = display_pages; current_page; current_page = current_page->next)
	{
		util_time_to_string(datetime, &current_page->expiry);

		if(current_page->type == dpt_text)
		{
			string_format_append(output, "\ntext: %s [%s]", string_cstr(current_page->name), string_cstr(datetime));

			for(line = 0; (line < display_page_lines_size) && current_page->text.line[line]; line++)
				string_format_append(output, "\n- [%u]: %s", line, string_cstr(current_page->text.line[line]));
		}
		else
		{
			if(current_page->type == dpt_image)
				string_format_append(output, "\nimage: %s, file: %s [%s]", string_cstr(current_page->name), string_cstr(current_page->image.filename), string_cstr(datetime));
			else
				string_format_append(output, "\ninvalid type: %u", current_page->type);
		}
	}

	page_data_mutex_give();
}

void display_init(void)
{
	uint32_t type, value;

	static display_init_parameters_t display_init_parameters =
	{
		.interface_index = -1,
		.x_size = -1,
		.y_size = -1,
		.x_mirror = -1,
		.y_mirror = -1,
		.rotate = -1,
		.invert = -1,
	};

	if(!config_get_uint_cstr(display_variable[dv_type][1], &type))
		return;

	if((type + dt_type_first) >= dt_size)
	{
		log_format("display init: unknown display type: %u", (unsigned int)type);
		return;
	}

	display_type = (display_type_t)type + dt_type_first;

	if(config_get_uint_cstr(display_variable[dv_if][1], &value))
		display_init_parameters.interface_index = (int)value;

	if(config_get_uint_cstr(display_variable[dv_x_size][1], &value))
		display_init_parameters.x_size = x_size = (int)value;

	if(config_get_uint_cstr(display_variable[dv_y_size][1], &value))
		display_init_parameters.y_size = y_size = (int)value;

	if(config_get_uint_cstr(display_variable[dv_x_mirror][1], &value))
		display_init_parameters.x_mirror = (int)value;

	if(config_get_uint_cstr(display_variable[dv_y_mirror][1], &value))
		display_init_parameters.y_mirror = (int)value;

	if(config_get_uint_cstr(display_variable[dv_rotate][1], &value))
		display_init_parameters.rotate = (int)value;

	if(config_get_uint_cstr(display_variable[dv_invert][1], &value))
		display_init_parameters.invert = (int)value;

	assert(info[display_type].init_fn);

	if(!info[display_type].init_fn(&display_init_parameters))
	{
		display_type = dt_no_display;
		return;
	}

	if(!(font_valid = load_font("font_small")))
	{
		log("display: load font failed");
		return;
	}

	unicode_buffer = (uint32_t *)util_memory_alloc_spiram(sizeof(uint32_t) * unicode_buffer_size);

	clear(dc_black);

	log_get_display_queue(&log_display_queue);
	assert(log_display_queue);

	page_data_mutex = xSemaphoreCreateMutex();
	assert(page_data_mutex);

	if(xTaskCreatePinnedToCore(run_display_log, "display-log", 3072, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("display: xTaskCreatePinnedToNode display log");

	if(xTaskCreatePinnedToCore(run_display_info, "display-info", 3072, (void *)0, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("display: xTaskCreatePinnedToNode display run");
}

unsigned int display_image_x_size(void)
{
	if((display_type == dt_no_display) || !font_valid || !font)
		return(0);

	return(x_size - (2 * page_border_size));
}

unsigned int display_image_y_size(void)
{
	if((display_type == dt_no_display) || !font_valid || !font)
		return(0);

	return(y_size - (2 * page_border_size) - page_text_offset - font->net.height);
}

static bool brightness(unsigned int percentage)
{
	if((display_type == dt_no_display) || !info[display_type].bright_fn)
		return(false);

	info[display_type].bright_fn(percentage);

	return(true);
}

void command_display_brightness(cli_command_call_t *call)
{
	assert(call->parameter_count == 1);

	if(brightness(call->parameters[0].unsigned_int))
		string_assign_cstr(call->result, "set brightness: ok");
	else
		string_assign_cstr(call->result, "set brightness: no display");
}

void command_display_configure(cli_command_call_t *call)
{
	dv_t ix;

	assert((call->parameter_count <= 8));

	if(call->parameter_count == 0)
	{
		display_info(call->result);
		return;
	}

	if(call->parameters[0].unsigned_int >= dt_size)
	{
		string_assign_cstr(call->result, "display-configure: invalid display type, choose type as:");
		string_append_cstr(call->result, "\n- 0: generic SPI LCD");

		return;
	}

	if(call->parameter_count < 4)
	{
		string_assign_cstr(call->result, "display-configure: at least 4 parameters required:");

		for(ix = dv_start; ix < dv_size; ix++)
			string_format_append(call->result, "\n- %u: %s", (unsigned int)ix + 1U, display_variable[ix][2]);

		return;
	}

	config_erase_wildcard_cstr("display.");

	for(ix = dv_start; (ix < dv_size) && (ix < call->parameter_count); ix++)
		config_set_uint_cstr(display_variable[ix][1], call->parameters[ix].unsigned_int);

	display_info(call->result);
}

void command_display_erase(cli_command_call_t *call)
{
	config_erase_wildcard_cstr("display.");

	assert(call->parameter_count == 0);

	display_info(call->result);
}

void command_display_info(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	display_info(call->result);
}

void command_display_page_add_text(cli_command_call_t *call)
{
	assert(call->parameter_count == 3);

	page_add_text(call->parameters[0].string, call->parameters[1].unsigned_int, call->parameters[2].string);

	string_format(call->result, "display-page-add-text added \"%s\"", string_cstr(call->parameters[0].string));
}

void command_display_page_add_image(cli_command_call_t *call)
{
	assert(call->parameter_count == 3);

	page_add_image(call->parameters[0].string, call->parameters[1].unsigned_int, call->parameters[2].string);

	string_format(call->result, "display-page-add-image added \"%s\"", string_cstr(call->parameters[0].string));
}

void command_display_page_remove(cli_command_call_t *call)
{
	assert(call->parameter_count == 1);

	page_erase(call->parameters[0].string);

	string_format(call->result, "display-page-remove removed \"%s\"", string_cstr(call->parameters[0].string));
}
