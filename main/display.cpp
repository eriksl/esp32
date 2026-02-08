#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "log.h"
#include "util.h"
#include "config.h"
#include "cli-command.h"
#include "display.h"
#include "crypt.h"
#include "exception.h"

#include <assert.h>
#include <unistd.h>
#include <fcntl.h>
#include <freertos/FreeRTOS.h>
#include <png.h>
#include <zlib.h>
#include <esp_timer.h>
#include <errno.h>

#include <csetjmp>
#include <string>
#include <deque>
#include <vector>
#include <format>

static_assert(sizeof(font_glyph_t) == 68);
static_assert(offsetof(font_t, basic_glyph) == 56);
static_assert(offsetof(font_t, extra_glyph) == 17464);

enum
{
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
	dv_flip,
	dv_invert,
	dv_rotate,
	dv_error,
	dv_size = dv_error,
} dv_t;

typedef enum
{
	dpt_text,
	dpt_image,
	dpt_none,
	dpt_size,
	dpt_error = dpt_size,
	dpt_first = dpt_text,
	dpt_last = dpt_image,
} display_page_type_t;

typedef struct
{
	std::string name;
	time_t expiry;
	display_page_type_t type;
	display_colour_t colour;
	struct
	{
		std::deque<std::string> lines;
	} text;
	struct
	{
		unsigned int length;
		std::string filename;
	} image;
} display_page_t;

typedef enum
{
	user_png_io_ptr_magic_word = 0x1234abcdU,
} user_png_iot_ptr_magic_word_t;

typedef struct
{
	user_png_iot_ptr_magic_word_t magic_word;
	int fd;
} user_png_io_ptr_t;

static const char * const display_variable[dv_size][3] =
{
	[dv_type] =		{	"type",				"display.type",		"display type, 0 = generic SPI LCD",	},
	[dv_if] =		{	"interface",		"display.if",		"interface, 0 = SPI2, 1 = SPI3",		},
	[dv_x_size] =	{	"x size",			"display.x.size",	"x size (width)",						},
	[dv_y_size] =	{	"y size",			"display.y.size",	"y size (height)",						},
	[dv_flip] =		{	"flip",				"display.flip",		"flip display (optional)",				},
	[dv_invert] =	{	"invert",			"display.invert",	"invert display (optional)",			},
	[dv_rotate] =	{	"rotate",			"display.rotate",	"rotate display (optional)",			},
};

static const display_info_t info[dt_size] =
{
	[dt_no_display] =
	{
		.name = "No display",
		.init_fn = nullptr,
		.bright_fn = nullptr,
		.write_fn = nullptr,
		.clear_fn = nullptr,
		.box_fn = nullptr,
		.plot_line_fn = nullptr,
		.set_layer_fn = nullptr,
		.show_layer_fn = nullptr,
	},
	[dt_spi_generic] =
	{
		.name = "Generic SPI LCD display",
		.init_fn = display_spi_generic_init,
		.bright_fn = display_spi_generic_bright,
		.write_fn = display_spi_generic_write,
		.clear_fn = display_spi_generic_clear,
		.box_fn = display_spi_generic_box,
		.plot_line_fn = display_spi_generic_plot_line,
		.set_layer_fn = nullptr,
		.show_layer_fn = nullptr,
	},
	[dt_ra8875] =
	{
		.name = "SPI LCD display based on RA8875",
		.init_fn = display_ra8875_init,
		.bright_fn = display_ra8875_bright,
		.write_fn = display_ra8875_write,
		.clear_fn = display_ra8875_clear,
		.box_fn = display_ra8875_box,
		.plot_line_fn = display_ra8875_plot_line,
		.set_layer_fn = display_ra8875_set_layer,
		.show_layer_fn = display_ra8875_show_layer,
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

static bool inited = false;
static display_colour_t next_colour = dc_black;
static std::vector<display_page_t> display_pages;
static display_type_t display_type = dt_no_display;
static font_t *font = nullptr;
static bool font_valid = false;
static unsigned int display_columns, display_rows;
static unsigned int x_size, y_size;
static QueueHandle_t log_display_queue = nullptr;
static bool log_mode = true;
static SemaphoreHandle_t page_data_mutex;
static unsigned int display_log_y;
static unsigned int stat_display_show = 0;
static unsigned int stat_skipped_incomplete_images = 0;

static inline void page_data_mutex_take(void)
{
	assert(page_data_mutex);
	xSemaphoreTake(page_data_mutex, portMAX_DELAY);
}

static inline void page_data_mutex_give(void)
{
	assert(page_data_mutex);
	xSemaphoreGive(page_data_mutex);
}

static unsigned int string_length_utf8(std::string_view in)
{
	std::string_view::const_iterator it;
	unsigned int count;

	for(it = in.begin(), count = 0; it != in.end(); it++)
		if(static_cast<unsigned int>(*it) < 128)
			count++;

	return(count);
}

static void utf8_to_unicode(std::string_view src, std::deque<uint32_t> &dst)
{
	typedef enum
	{
		u8p_state_base = 0,
		u8p_state_utf8_byte_3 = 1,
		u8p_state_utf8_byte_2 = 2,
		u8p_state_utf8_byte_1 = 3,
		u8p_state_done = 4
	} state_t;

	state_t state = u8p_state_base ;
	uint32_t unicode;
	std::string_view::const_iterator src_it;
	unicode = 0;
	dst.clear();

	for(src_it = src.begin(); src_it != src.end(); src_it++)
	{
		switch(state)
		{
			case u8p_state_base:
			{
				if((static_cast<unsigned int>(*src_it) & 0xe0) == 0xc0) // first of two bytes (11 bits)
				{
					unicode = static_cast<unsigned int>(*src_it) & 0x1f;
					state = u8p_state_utf8_byte_1;
					continue;
				}
				else
					if((static_cast<unsigned int>(*src_it) & 0xf0) == 0xe0) // first of three bytes (16 bits)
					{
						unicode = static_cast<unsigned int>(*src_it) & 0x0f;
						state = u8p_state_utf8_byte_2;
						continue;
					}
					else
						if((static_cast<unsigned int>(*src_it) & 0xf8) == 0xf0) // first of four bytes (21 bits)
						{
							unicode = static_cast<unsigned int>(*src_it) & 0x07;
							state = u8p_state_utf8_byte_3;
							continue;
						}
						else
							if((static_cast<unsigned int>(*src_it) & 0x80) == 0x80)
							{
								Log::get() << std::format("utf8 parser: invalid utf8, bit 7 set: {:#x} '{:c}'\n", *src_it, *src_it);
								unicode = '*';
							}
							else
								unicode = static_cast<unsigned int>(*src_it) & 0x7f;

				break;
			}

			case u8p_state_utf8_byte_3 ... u8p_state_utf8_byte_1:
			{
				if((static_cast<unsigned int>(*src_it) & 0xc0) == 0x80) // following bytes
				{
					unicode = (unicode << 6) | (static_cast<unsigned int>(*src_it) & 0x3f);

					state = static_cast<state_t>(state + 1);

					if(state != u8p_state_done)
						continue;
				}
				else
				{
					Log::get() << std::format("utf8 parser: invalid utf8, no prefix on following byte, state: {:d}: {:#x} {:c}\n",
							static_cast<unsigned int>(state), *src_it, *src_it);
					unicode = '*';
				}

				break;
			}

			default:
			{
				Log::get() << std::format("utf8 parser: invalid state {:d}\n", static_cast<unsigned int>(state));
				unicode = '*';
			}
		}

		dst.push_back(unicode);
		unicode = 0;
		state = u8p_state_base;
	}
}

static void page_init(display_page_t &page)
{
	assert(inited);

	page.name.clear();
	page.type = dpt_none;
	page.expiry = 0;
	page.text.lines.clear();
	page.image.filename.clear();
	page.image.length = 0;
}

static void page_erase(int page)
{
	assert(inited);
	assert((page >= 0) && (page < display_pages.size()));

	if((display_pages[page].type == dpt_image) && unlink(display_pages[page].image.filename.c_str()))
		Log::get() << std::format("display: page erase: unlink image {} failed", display_pages[page].image.filename);

	display_pages.erase(display_pages.begin() + page);
}

static int page_find(std::string_view name)
{
	int page;

	assert(inited);

	for(page = 0; page < display_pages.size(); page++)
		if(display_pages[page].name == name)
			return(page);

	return(-1);
}

static bool page_add_text(std::string_view name, unsigned int lifetime, std::string_view contents)
{
	int page;
	display_page_t *page_ptr;
	std::string line;
	std::string_view::const_iterator contents_it;

	assert(inited);

	if((page = page_find(name)) < 0)
	{
		display_page_t new_page;

		new_page.colour = next_colour;
		next_colour = static_cast<display_colour_t>(next_colour + 1);
		if(next_colour >= dc_white)
			next_colour = dc_first;

		display_pages.push_back(new_page);
		page = display_pages.size() - 1;
	}
	else
		if((display_pages[page].type == dpt_image) && !display_pages[page].image.filename.empty() && unlink(display_pages[page].image.filename.c_str()))
			Log::get() << std::format("display: page add text: unlink image {} failed", display_pages[page].image.filename);

	page_ptr = &display_pages[page];

	page_init(*page_ptr);
	page_ptr->name = name;
	page_ptr->type = dpt_text;
	page_ptr->expiry = (lifetime > 0) ? time(nullptr) + lifetime : 0;

	for(contents_it = contents.begin(); contents_it != contents.end(); contents_it++)
	{
		switch(*contents_it)
		{
			case('\\'):
			{
				if(((contents_it + 1) == contents.end()) || (*(contents_it + 1) != 'n'))
				{
					line.append(1, *contents_it);
					continue;
				}

				contents_it++;
				page_ptr->text.lines.push_back(line);

				line.clear();

				break;
			}

			case('\n'):
			{
				page_ptr->text.lines.push_back(line);

				line.clear();

				break;
			}

			default:
			{
				line.append(1, *contents_it);

				continue;
			}
		}
	}

	return(true);
}

static bool page_add_image(std::string_view name, unsigned int lifetime, std::string_view filename, unsigned int length)
{
	int page;
	display_page_t *page_ptr;

	assert(inited);

	if((page = page_find(name)) < 0)
	{
		display_page_t new_page;

		new_page.colour = next_colour;
		next_colour = static_cast<display_colour_t>(next_colour + 1);
		if(next_colour >= dc_white)
			next_colour = dc_first;

		display_pages.push_back(new_page);
		page = display_pages.size() - 1;
	}
	else
		if((display_pages[page].type == dpt_image) &&
					!display_pages[page].image.filename.empty() &&
					(display_pages[page].image.filename != filename) &&
					unlink(display_pages[page].image.filename.c_str()))
			Log::get() << std::format("display: page add image: unlink image {} failed", display_pages[page].image.filename);

	page_ptr = &display_pages[page];
	page_init(*page_ptr);
	page_ptr->name = name;
	page_ptr->type = dpt_image;
	page_ptr->expiry = (lifetime > 0) ? time(nullptr) + lifetime : 0;
	page_ptr->image.filename = filename;
	page_ptr->image.length = length;

	return(true);
}

static bool load_font(std::string_view fontname)
{
	int fd;
	std::string pathfont;
	std::string our_hash;
	std::string their_hash;

	pathfont = std::string("/littlefs/");
	pathfont.append(fontname);

	if((fd = open(pathfont.c_str(), O_RDONLY, 0)) < 0)
	{
		Log::get() << std::format("display: failed to open font {}", pathfont);
		goto error;
	}

	if(!font)
		font = new font_t;

	if(read(fd, font, sizeof(*font)) != sizeof(*font))
	{
		Log::get() << std::format("display: failed to read font {}", pathfont);
		goto error;
	}

	close(fd);
	fd = -1;

	if(font->magic_word != font_magic_word)
	{
		Log::get() << std::format("display: font file magic word invalid: {:#x}", static_cast<unsigned int>(font->magic_word));
		goto error;
	}

	their_hash.resize(32);

	memcpy(their_hash.data(), font->checksum, their_hash.size());
	memset(font->checksum, 0, sizeof(font->checksum));

	our_hash = Crypt::sha256(std::string_view(reinterpret_cast<const char *>(font), sizeof(*font)));

	if(our_hash != their_hash)
	{
		Log::get() << "display: font file invalid checksum";
		goto error;
	}

	display_columns = (x_size - (2 * page_border_size)) / font->net.width;
	display_rows = (y_size - page_text_offset - (2 * page_border_size)) / font->net.height;

	return(true);

error:
	if(fd >= 0)
		close(fd);

	if(font)
	{
		delete font;
		font = nullptr;
	}

	display_columns = 0;
	display_rows = 0;

	return(false);
}

static bool clear(display_colour_t bg)
{
	assert(inited);

	if((display_type == dt_no_display) || !info[display_type].clear_fn)
		return(false);

	info[display_type].clear_fn(bg);

	return(true);
}

static bool box(display_colour_t colour, unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y)
{
	assert(inited);

	if((display_type == dt_no_display) || !info[display_type].box_fn)
		return(false);

	info[display_type].box_fn(colour, from_x, from_y, to_x, to_y);

	return(true);
}

static bool plot_line(unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int length_pixels, const png_bytep pixels)
{
	assert(inited);

	if((display_type == dt_no_display) || !info[display_type].plot_line_fn)
		return(false);

	info[display_type].plot_line_fn(from_x, from_y, to_x, length_pixels, (const display_rgb_t *)pixels);

	return(true);
}

static void run_display_log(void *)
{
	unsigned int entry;
	std::string entry_text;
	time_t stamp;
	std::deque<uint32_t> unicode_buffer;

	display_log_y = 0;

	for(;;)
	{
		if(!log_display_queue && !(log_display_queue = Log::get().get_display_queue()))
			continue;

		assert(xQueueReceive(log_display_queue, &entry, portMAX_DELAY));

		if(font_valid && log_mode && (display_type != dt_no_display) && info[display_type].write_fn)
		{
			Log::get().get_entry(entry, stamp, entry_text);
			utf8_to_unicode(util_time_to_string("{:%H:%M:%S}", stamp) + " " + entry_text, unicode_buffer);

			info[display_type].write_fn(font, dc_white, dc_black, 0, display_log_y, x_size - 1, display_log_y + font->net.height - 1, unicode_buffer);

			display_log_y += font->net.height;

			if((display_log_y + font->net.height) > y_size)
				display_log_y = 0;

			box(dc_black, 0, display_log_y, x_size - 1, display_log_y + font->net.height - 1);
		}
	}

	util_abort("run_display_log returns");
}

static png_voidp user_png_malloc(png_structp struct_ptr, png_alloc_size_t size)
{
	png_voidp ptr;

	ptr = malloc(size);

	return(ptr);
}

static void user_png_free(png_structp png_ptr, png_voidp ptr)
{
	if(ptr)
		free(ptr);
}

static void user_read_data(png_structp png_ptr, png_bytep data, size_t length)
{
	const user_png_io_ptr_t *io_ptr;
	ssize_t read_length;

	io_ptr = (const user_png_io_ptr_t *)png_get_io_ptr(png_ptr);

	assert(io_ptr->magic_word == user_png_io_ptr_magic_word);

	errno = 0;
	read_length = read(io_ptr->fd, data, length);

	if(length != read_length)
	{
		Log::get().log_errno(errno, std::format("display: user_read_data: read error: requested: {:d}, received: {:d}, fd: {:d}", length, read_length, io_ptr->fd));
		png_error(png_ptr, "read error");
	}
}

static void user_error(png_structp png_ptr, png_const_charp msg)
{
	Log::get() << std::format("fatal error in libpng: {}", msg);
	png_longjmp(png_ptr, 1);
}

static void user_warning(png_structp png_ptr, png_const_charp msg)
{
	if(strcmp(msg, "IDAT: Too much image data"))
		Log::get() << std::format("warning in libpng: {}", msg);
}

static void run_display_info(void *)
{
	// all static due to prevent clobbering by longjmp

	static int current_page;
	static unsigned int current_layer;
	static unsigned int row, y1, y2;
	static unsigned int image_x_size, image_y_size, row_bytes, colour_type, bit_depth;
	static int pad, chop;
	static std::string stamp_string;
	static std::string name_tmp;
	static std::string title_line;
	static std::string libpng_buffer;
	static void (*write_fn)(const font_t *font, display_colour_t fg, display_colour_t bg,
				unsigned int from_x, unsigned int from_y, unsigned int to_x, unsigned int to_y,
				const std::deque<uint32_t> &line_unicode);
	static png_structp png_ptr = nullptr;
	static png_infop info_ptr = nullptr;
	static png_bytep row_pointer = nullptr;
	static int fd = -1;
	static user_png_io_ptr_t user_io_ptr;
	static uint64_t time_start, time_spent;
	static bool fastskip;
	static struct stat statb;
	static std::deque<uint32_t> unicode_buffer;

	static const png_color_16 default_background =
	{
		.index = 255,
		.red = 0,
		.green = 0,
		.blue = 0,
		.gray = 0,
	};

	current_layer = 0;

	for(;;)
	{
		current_page = 0;

		for(;;)
		{
			page_data_mutex_take();

			fastskip = false;

			if(!font_valid || (display_type == dt_no_display) || (!(write_fn = info[display_type].write_fn)))
				goto next;

			if(display_pages.empty())
			{
				if(!log_mode)
				{
					font_valid = load_font("font_small");
					log_mode = true;
					clear(dc_black);
					display_log_y = 0;
				}

				goto next;
			}

			if(current_page >= display_pages.size())
				current_page = 0;

			if(log_mode)
			{
				if(!((font_valid = load_font("font_big"))))
					goto next;

				log_mode = false;
				current_page = 0;
			}

			time_start = esp_timer_get_time();

			if(info[display_type].set_layer_fn)
				info[display_type].set_layer_fn((current_layer + 1) % 2);

			box(display_pages[current_page].colour,	0,										0,										x_size - 1,				page_border_size - 1);
			box(display_pages[current_page].colour,	(x_size - 1) - (page_border_size - 1),	0,										x_size - 1,				y_size - 1);
			box(display_pages[current_page].colour,	0,										(y_size - 1) - (page_border_size - 1),	x_size - 1,				y_size - 1);
			box(display_pages[current_page].colour,	0,										0,										page_border_size - 1,	y_size - 1);

			stamp_string = util_time_to_string("{:%d/%m %H:%M}", time(nullptr));

			if(stamp_string.length() > display_columns)
			{
				chop = string_length_utf8(display_pages[current_page].name);

				if(chop > display_columns)
				{
					chop = display_columns;
					pad = 0;
				}
				else
					pad = display_columns - string_length_utf8(display_pages[current_page].name);

				assert(pad >= 0);

				name_tmp = display_pages[current_page].name;
				std::replace(name_tmp.begin(), name_tmp.end(), '_', ' ');

				if(chop >= name_tmp.length())
					title_line = name_tmp;
				else
					title_line = name_tmp.substr(0, chop);

				title_line.append(pad, ' ');
			}
			else
			{
				chop = string_length_utf8(display_pages[current_page].name);
;
				if((chop + stamp_string.length()) > display_columns)
					chop = display_columns - stamp_string.length();

				pad = display_columns - stamp_string.length() - chop;

				assert(chop >= 0);
				assert(pad >= 0);

				name_tmp = display_pages[current_page].name;
				std::replace(name_tmp.begin(), name_tmp.end(), '_', ' ');

				if(chop >= name_tmp.length())
					title_line = name_tmp;
				else
					title_line = name_tmp.substr(0, chop);

				title_line.append(pad, ' ');
				title_line.append(stamp_string);
			}

			utf8_to_unicode(title_line, unicode_buffer);
			write_fn(font, dc_white, display_pages[current_page].colour,
					page_border_size, page_border_size, (x_size - 1) - page_border_size, page_text_offset + page_border_size + (font->net.height - 1), unicode_buffer);

			switch(display_pages[current_page].type)
			{
				case(dpt_text):
				{
					y1 = font->net.height + page_border_size + page_text_offset;

					for(row = 0; row < display_pages[current_page].text.lines.size(); row++)
					{
						y2 = y1 + (font->net.height - 1);

						if(y2 > y_size - page_border_size)
							y2 = y_size - page_border_size;

						utf8_to_unicode(display_pages[current_page].text.lines[row], unicode_buffer);

						write_fn(font, dc_black, dc_white,
								page_border_size, y1, (x_size - 1) - page_border_size, y2,
								unicode_buffer);

						y1 += font->net.height;
					}

					if(y1 < ((y_size - 1) - page_border_size))
						box(dc_white, page_border_size, y1, (x_size - 1) - page_border_size, (y_size - 1) - page_border_size);

					break;
				}

				case(dpt_image):
				{
					row_pointer = nullptr;

					if(stat(display_pages[current_page].image.filename.c_str(), &statb))
					{
						Log::get() << std::format("display: cannot stat image file: {}", display_pages[current_page].image.filename);
						fastskip = true;
						goto skip;
					}

					if(statb.st_size != display_pages[current_page].image.length)
					{
						stat_skipped_incomplete_images++;
						fastskip = true;
						goto skip;
					}

					if((fd = open(display_pages[current_page].image.filename.c_str(), O_RDONLY, 0)) < 0)
					{
						Log::get() << std::format("display: cannot open image file: {}", display_pages[current_page].image.filename);
						fastskip = true;
						goto skip;
					}

					libpng_buffer.resize(8);

					if(read(fd, libpng_buffer.data(), libpng_buffer.size()) != libpng_buffer.size())
					{
						Log::get() << std::format("display: cannot read signature: {}", display_pages[current_page].image.filename);
						fastskip = true;
						goto skip;
					}

					if(png_sig_cmp(reinterpret_cast<png_const_bytep>(libpng_buffer.data()), 0, 8))
					{
						Log::get() << std::format("display: invalid PNG signature: {}", display_pages[current_page].image.filename);
						fastskip = true;
						goto skip;
					}

					png_ptr = png_create_read_struct_2(PNG_LIBPNG_VER_STRING, (png_voidp)0, user_error, user_warning, (png_voidp)0, user_png_malloc, user_png_free);
					assert(png_ptr);

					info_ptr = png_create_info_struct(png_ptr);
					assert(info_ptr);

					if(setjmp(png_jmpbuf(png_ptr)))
					{
						fastskip = true;
						goto abort;
					}

					user_io_ptr.magic_word = user_png_io_ptr_magic_word;
					user_io_ptr.fd = fd;

					png_set_read_fn(png_ptr, (png_bytep)&user_io_ptr, user_read_data);
					png_set_sig_bytes(png_ptr, 8);
					png_read_info(png_ptr, info_ptr);
					png_get_IHDR(png_ptr, info_ptr, (png_uint_32 *)0, (png_uint_32 *)0, (int *)0, (int *)0, (int *)0, (int *)0, (int *)0);
					png_set_background(png_ptr, &default_background, PNG_BACKGROUND_GAMMA_SCREEN, 0, 1);
					colour_type = png_get_color_type(png_ptr, info_ptr);
					bit_depth = png_get_bit_depth(png_ptr, info_ptr);
					if (colour_type == PNG_COLOR_TYPE_GRAY || colour_type == PNG_COLOR_TYPE_GRAY_ALPHA)
						png_set_gray_to_rgb(png_ptr);
					if((colour_type == PNG_COLOR_TYPE_GRAY) && (bit_depth < 8))
						png_set_expand_gray_1_2_4_to_8(png_ptr);
					if(colour_type == PNG_COLOR_TYPE_PALETTE)
						png_set_palette_to_rgb(png_ptr);
					if(png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS))
						png_set_tRNS_to_alpha(png_ptr);
					png_set_packing(png_ptr);
					png_read_update_info(png_ptr, info_ptr);
					image_x_size = png_get_image_width(png_ptr, info_ptr);
					image_y_size = png_get_image_height(png_ptr, info_ptr);
					row_bytes = png_get_rowbytes(png_ptr, info_ptr);
					assert(row_bytes == (image_x_size * 3));
					row_pointer = static_cast<png_bytep>(new uint8_t[row_bytes]);

					for(row = 0; row < image_y_size; row++)
					{
						y1 = page_border_size + page_text_offset + (font->net.height - 1) + row;

						if((y1 + page_border_size) > y_size)
							break;

						png_read_row(png_ptr, row_pointer, nullptr);
						plot_line(page_border_size, page_border_size + page_text_offset + (font->net.height - 1) + row, (x_size - 1) - page_border_size, image_x_size, row_pointer);
					}

					png_read_end(png_ptr, (png_infop)0);

abort:
					assert(png_ptr);
					assert(info_ptr);

					png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);

					assert(!png_ptr);
					assert(!info_ptr);

					if(row_pointer)
					{
						delete [] row_pointer;
						row_pointer = nullptr;
					}

skip:
					if(fd >= 0)
					{
						close(fd);
						fd = -1;
					}

					break;
				}

				default:
				{
					Log::get() << std::format("display: unknown page type: {:d}", static_cast<unsigned int>(display_pages[current_page].type));
					break;
				}
			}

			if((display_pages[current_page].expiry > 0) && (time(nullptr) > display_pages[current_page].expiry))
			{
				page_erase(current_page);
				goto next;
			}

			current_layer = (current_layer + 1) % 2;

			if(info[display_type].show_layer_fn)
				info[display_type].show_layer_fn(current_layer);

			time_spent = esp_timer_get_time() - time_start;
			stat_display_show = time_spent / 1000ULL;

next:
			page_data_mutex_give();
			util_sleep(fastskip ? 100 : 8000);

			current_page++;
		}
	}

	util_abort("run_display_info returns");
}

static void display_info(std::string &output)
{
	uint32_t value;
	unsigned int found;
	dv_t dv;
	unsigned int line, page;
	std::string datetime;

	if(!inited)
	{
		output = "No displays configured";
		return;
	}

	output = "DISPLAY configuration:";

	for(dv = dv_start, found = 0; dv < dv_size; dv = static_cast<dv_t>(dv + 1))
	{
		try
		{
			value = Config::get().get_int(display_variable[dv][1]);
			found++;
			output += std::format("\n- {}: {:d}", display_variable[dv][0], value);
		}
		catch(const transient_exception &)
		{
		}
	}

	if(found == 0)
	{
		output += "\n- no display configuration found";
		return;
	}

	output += std::format("\nDISPLAY current type {}, ", info[display_type].name);

	if(!font_valid)
		output += "no display font loaded";
	else
	{
		output += "font info: ";
		output += std::format("\n- magic word: {:#x}", static_cast<unsigned int>(font->magic_word));
		output += std::format("\n- raw width: {:d}", static_cast<unsigned int>(font->raw.width));
		output += std::format("\n- raw height: {:d}", static_cast<unsigned int>(font->raw.height));
		output += std::format("\n- net width: {:d}", static_cast<unsigned int>(font->net.width));
		output += std::format("\n- net height: {:d}", static_cast<unsigned int>(font->net.height));
		output += std::format("\n- basic glyphs: {:d}", static_cast<unsigned int>(font_basic_glyphs_size));
		output += std::format("\n- extra glyphs: {:d}", static_cast<unsigned int>(font->extra_glyphs));
		output += std::format("\n- columns: {:d}", display_columns);
		output += std::format("\n- rows: {:d}", display_rows);

		output += "\nPAGES:";

		for(page = 0; page < display_pages.size(); page++)
		{
			if(display_pages[page].expiry > 0)
				datetime = util_time_to_string("{:%d/%m %H:%M}", display_pages[page].expiry);
			else
				datetime = "<infinite>";

			output += std::format("\n- PAGE {:d}: \"{}\", expiry: {}, colour: {:d}, type: ",
					page, display_pages[page].name, datetime, static_cast<unsigned int>(display_pages[page].colour));

			switch(display_pages[page].type)
			{
				case(dpt_text):
				{
					output += "text, contents:";

					for(line = 0; line < display_pages[page].text.lines.size(); line++)
						output += std::format("\n-   {:d}: {}", line, display_pages[page].text.lines[line]);

					break;
				}

				case(dpt_image):
				{
					output += std::format("image, file: {} ({:d}k)",
							display_pages[page].image.filename, display_pages[page].image.length / 1024);
					break;
				}

				default:
				{
					util_abort("display_pages[page].type invalid");
				}
			}
		}

		output += "\nSTATS:";
		output += std::format("\n- display draw time: {:d} ms", stat_display_show);
		output += std::format("\n- incomplete images skipped: {:d}", stat_skipped_incomplete_images);
	}
}

static bool brightness(unsigned int percentage)
{
	assert(inited);

	if((display_type == dt_no_display) || !info[display_type].bright_fn)
		return(false);

	info[display_type].bright_fn(percentage);

	return(true);
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

	return(y_size - ((2 * page_border_size) + page_text_offset + font->net.height - 1));
}

void command_display_brightness(cli_command_call_t *call)
{
	assert(call->parameter_count == 1);

	if(brightness(call->parameters[0].unsigned_int))
		call->result = "set brightness: ok";
	else
		call->result = "set brightness: no display";
}

void command_display_configure(cli_command_call_t *call)
{
	dv_t ix;

	assert((call->parameter_count <= 9));

	if(call->parameter_count == 0)
	{
		display_info(call->result);
		return;
	}

	if(call->parameters[0].unsigned_int >= dt_size)
	{
		call->result = "display-configure: invalid display type, choose type as:";
		call->result += "\n- 0: generic SPI LCD";

		return;
	}

	if(call->parameter_count < 4)
	{
		call->result = "display-configure: at least 4 parameters required:";

		for(ix = dv_start; ix < dv_size; ix = static_cast<dv_t>(ix + 1))
			call->result += std::format("\n- {:d}: {}", ix + 1, display_variable[ix][2]);

		return;
	}

	Config::get().erase_wildcard("display.");

	for(ix = dv_start; (ix < dv_size) && (ix < call->parameter_count); ix = static_cast<dv_t>(ix + 1))
		Config::get().set_int(display_variable[ix][1], call->parameters[ix].unsigned_int);

	display_info(call->result);
}

void command_display_erase(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	Config::get().erase_wildcard("display.");

	page_data_mutex_take();
	display_info(call->result);
	page_data_mutex_give();
}

void command_display_info(cli_command_call_t *call)
{
	assert(call->parameter_count == 0);

	page_data_mutex_take();
	display_info(call->result);
	page_data_mutex_give();
}

void command_display_page_add_text(cli_command_call_t *call)
{
	bool rv;

	assert(call->parameter_count == 3);

	page_data_mutex_take();
	rv = page_add_text(call->parameters[0].str, call->parameters[1].unsigned_int, call->parameters[2].str);
	page_data_mutex_give();

	call->result = std::format("display-page-add-text{}added \"{}\"", rv ? " " : " not ", call->parameters[0].str);
}

void command_display_page_add_image(cli_command_call_t *call)
{
	bool rv;

	assert(call->parameter_count == 4);

	page_data_mutex_take();
	rv = page_add_image(call->parameters[0].str, call->parameters[1].unsigned_int, call->parameters[2].str, call->parameters[3].unsigned_int);
	page_data_mutex_give();

	call->result = std::format("display-page-add-image{}added \"{}\"", rv ? " " : " not ", call->parameters[0].str);
}

void command_display_page_remove(cli_command_call_t *call)
{
	int page;

	assert(call->parameter_count == 1);

	page_data_mutex_take();
	page = page_find(call->parameters[0].str);

	if(page < 0)
	{
		page_data_mutex_give();
		call->result = std::format("display-page-remove not found \"{}\"", call->parameters[0].str);
		return;
	}

	page_erase(page);
	page_data_mutex_give();

	call->result = std::format("display-page-remove removed \"{}\"", call->parameters[0].str);
}

void display_init(void)
{
	uint32_t type;

	static display_init_parameters_t display_init_parameters =
	{
		.interface_index = -1,
		.x_size = -1,
		.y_size = -1,
		.flip = -1,
		.invert = -1,
		.rotate = -1,
	};

	try
	{
		type = Config::get().get_int(display_variable[dv_type][1]);
	}
	catch(const transient_exception &e)
	{
		return;
	}

	if((dt_type_first + type) >= dt_size)
	{
		Log::get() << std::format("display init: unknown display type: {:d}", static_cast<unsigned int>(type));
		return;
	}

	display_type = static_cast<display_type_t>(dt_type_first + type);

	try
	{
		display_init_parameters.interface_index = Config::get().get_int(display_variable[dv_if][1]);
	}
	catch(const transient_exception &)
	{
	}

	try
	{
		display_init_parameters.x_size = x_size = Config::get().get_int(display_variable[dv_x_size][1]);
	}
	catch(const transient_exception &)
	{
	}

	try
	{
		display_init_parameters.y_size = y_size = Config::get().get_int(display_variable[dv_y_size][1]);
	}
	catch(const transient_exception &)
	{
	}

	try
	{
		display_init_parameters.flip = Config::get().get_int(display_variable[dv_flip][1]);
	}
	catch(const transient_exception &)
	{
	}

	try
	{
		display_init_parameters.invert = Config::get().get_int(display_variable[dv_invert][1]);
	}
	catch(const transient_exception &)
	{
	}

	try
	{
		display_init_parameters.rotate = Config::get().get_int(display_variable[dv_rotate][1]);
	}
	catch(const transient_exception &)
	{
	}

	assert(info[display_type].init_fn);

	if(!info[display_type].init_fn(&display_init_parameters))
	{
		display_type = dt_no_display;
		return;
	}

	page_data_mutex = xSemaphoreCreateMutex();
	assert(page_data_mutex);

	inited = true;

	if(!(font_valid = load_font("font_small")))
	{
		Log::get() << "display: load font failed";
		return;
	}

	clear(dc_black);
	brightness(75);

	if(xTaskCreatePinnedToCore(run_display_log, "display-log", 4 * 1024, nullptr, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("display: xTaskCreatePinnedToNode display log");

	if(xTaskCreatePinnedToCore(run_display_info, "display-info", 5 * 1024, nullptr, 1, (TaskHandle_t *)0, 1) != pdPASS)
		util_abort("display: xTaskCreatePinnedToNode display run");
}
