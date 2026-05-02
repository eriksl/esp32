#include "display.h"
#include "display-module.h"
#include "display-spi.h"
#include "display-spi-generic.h"
#include "display-ra8875.h"
#include "png.h"

#include "crypt.h"
#include "exception.h"
#include "magic_enum/magic_enum.hpp"

#include <vector>
#include <thread>
#include <chrono>

#include <unistd.h>
#include <fcntl.h>

#include <esp_pthread.h>

Display* Display::singleton = nullptr;

const Display::Colourmap Display::colourmap
{
	{ colour_t::black,	{ 0x00, 0x00, 0x00 }},
	{ colour_t::blue,	{ 0x00, 0x00, 0xff }},
	{ colour_t::green,	{ 0x00, 0x88, 0x00 }},
	{ colour_t::cyan,	{ 0x00, 0xaa, 0xaa }},
	{ colour_t::red,	{ 0xff, 0x00, 0x00 }},
	{ colour_t::purple,	{ 0xff, 0x00, 0xff }},
	{ colour_t::yellow,	{ 0xff, 0xbb, 0x00 }},
	{ colour_t::white,	{ 0xff, 0xff, 0xff }},
};

Display::Page::Page(Log &log_in, type_t type_in, Display::colour_t colour_in)
		: log(log_in), type(type_in), colour(colour_in)
{
	this->clear();
}

Display::Page::~Page()
{
	if((this->type == Page::type_t::image) && !this->image.filename.empty() && unlink(this->image.filename.c_str()))
		this->log << std::format("display: ~Page: unlink image {} failed", this->image.filename);
}

void Display::Page::clear()
{
	this->text.lines.clear();
	this->image.length = 0;
	this->image.filename.clear();
}

Display::Display(Config &config_in, Log &log_in, Util& util_in, SPI& spi_in, LedPWM& ledpwm_in)
		:
	config(config_in), log(log_in), util(util_in), spi(spi_in), ledpwm(ledpwm_in)
{
	int type, interface, x_size, y_size;
	bool flip, invert, rotate;
	esp_err_t rv;

	this->singleton = this;
	this->mode = mode_t::init_log;
	this->log_line = 0;
	this->next_colour = this->colour_first();
	this->pages.version = 0;

	this->fontinfo.valid = false;
	this->fontinfo.magic_word = 0;
	this->fontinfo.raw.width = 0;
	this->fontinfo.raw.height = 0;
	this->fontinfo.net.width = 0;
	this->fontinfo.net.height = 0;
	this->fontinfo.extra_glyphs = 0;
	this->fontinfo.columns = 0;
	this->fontinfo.rows = 0;

	if(!this->config_get("display.type", type))
		return;

	if(!this->config_get("display.if", interface))
		return;

	if(!this->config_get("display.size.x", x_size))
		return;

	if(!this->config_get("display.size.y", y_size))
		return;

	if(!this->config_get("display.flip", flip))
		flip = false;

	if(!this->config_get("display.invert", invert))
		invert = false;

	if(!this->config_get("display.rotate", rotate))
		rotate = false;

	if((!this->module) && (type == magic_enum::enum_integer(DisplayModuleGenericSPI::type)))
	{
		try
		{
			this->module = std::make_unique<DisplayModuleGenericSPI>(this->config, this->log, this->util, this->spi, this->ledpwm, interface, x_size, y_size, flip, invert, rotate);
		}
		catch(transient_exception &e)
		{
			this->log << std::format("Display: init DisplayModuleGenericSPI: {}", e.what());
		}
	}

	if(!(this->module) && (type == magic_enum::enum_integer(DisplayModuleRA8875::type)))
	{
		try
		{
			this->module = std::make_unique<DisplayModuleRA8875>(this->config, this->log, this->util, this->spi, this->ledpwm, interface, x_size, y_size, flip, invert, rotate);
		}
		catch(transient_exception &e)
		{
			this->log << std::format("Display: init DisplayModuleRA8875: {}", e.what());
		}
	}

	if(!this->module)
	{
		this->log << std::format("Display: invalid display type: {:d} or display cannot be initialised", type);
		return;
	}

	this->brightness(75);

	esp_pthread_cfg_t thread_config = esp_pthread_get_default_config();

	thread_config.thread_name = "display info";
	thread_config.pin_to_core = 1;
	thread_config.stack_size = 5 * 1024;
	thread_config.prio = 1;

	if((rv = esp_pthread_set_cfg(&thread_config)) != ESP_OK)
		throw(hard_exception(this->log.esp_string_error(rv, "esp_pthread_set_cfg")));

	std::thread run_thread([this]() { this->run_thread(); });
	run_thread.detach();
}

Display::~Display()
{
}

Display::colour_t Display::colour_first()
{
	return(Display::colour_t::blue);
}

Display::colour_t Display::colour_next(Display::colour_t colour_in)
{
	auto colour = magic_enum::enum_cast<Display::colour_t>(magic_enum::enum_integer(colour_in) + 1);

	if(!colour.has_value() || (colour.value() == Display::colour_t::white))
		return(Display::colour_first());

	return(colour.value());
}

Display::colour_t Display::colour_from_int(int colour_int)
{
	auto colour = magic_enum::enum_cast<Display::colour_t>(colour_int);

	if(!colour.has_value())
		throw(hard_exception("Display::colour: out of range"));

	return(colour.value());
}

int Display::string_length_utf8(std::string_view in)
{
    int count = 0;

	for(auto i : in)
		if((static_cast<unsigned int>(i) & 0x80) == 0)
			count++;

    return(count);
}

void Display::utf8_to_unicode(std::string_view src_str, std::deque<unsigned int> &dst)
{
	static constexpr const std::byte byte_0x07{0x07};
	static constexpr const std::byte byte_0x0f{0x0f};
	static constexpr const std::byte byte_0x1f{0x1f};
	static constexpr const std::byte byte_0x3f{0x3f};
	static constexpr const std::byte byte_0x7f{0x7f};
	static constexpr const std::byte byte_0x80{0x80};
	static constexpr const std::byte byte_0xc0{0xc0};
	static constexpr const std::byte byte_0xe0{0xe0};
	static constexpr const std::byte byte_0xf0{0xf0};
	static constexpr const std::byte byte_0xf8{0xf8};

	enum state_t
	{
		u8p_state_base = 0,
		u8p_state_utf8_byte_3 = 1,
		u8p_state_utf8_byte_2 = 2,
		u8p_state_utf8_byte_1 = 3,
		u8p_state_done = 4
	};

	state_t state = u8p_state_base ;
	unsigned int unicode;
	unicode = 0;
	dst.clear();

	for(auto src_char : src_str)
	{
		std::byte src{src_char};

		switch(state)
		{
			case u8p_state_base:
			{
				if((src & byte_0xe0) == byte_0xc0) // first of two bytes (11 bits)
				{
					unicode = std::to_integer<unsigned int>(src & byte_0x1f);
					state = u8p_state_utf8_byte_1;
					continue;
				}
				else
					if((src & byte_0xf0) == byte_0xe0) // first of three bytes (16 bits)
					{
						unicode = std::to_integer<unsigned int>(src & byte_0x0f);
						state = u8p_state_utf8_byte_2;
						continue;
					}
					else
						if((src & byte_0xf8) == byte_0xf0) // first of four bytes (21 bits)
						{
							unicode = std::to_integer<unsigned int>(src & byte_0x07);
							state = u8p_state_utf8_byte_3;
							continue;
						}
						else
							if((src & byte_0x80) == byte_0x80)
							{
								this->log << std::format("utf8 parser: invalid utf8, bit 7 set: {:#x} '{:c}'\n", src_char, src_char);
								unicode = '*';
							}
							else
								unicode = std::to_integer<unsigned int>(src & byte_0x7f);

				break;
			}

			case u8p_state_utf8_byte_3 ... u8p_state_utf8_byte_1:
			{
				if((src & byte_0xc0) == byte_0x80) // following bytes
				{
					unicode = (unicode << 6) | std::to_integer<unsigned int>(src & byte_0x3f);

					state = static_cast<state_t>(state + 1);

					if(state != u8p_state_done)
						continue;
				}
				else
				{
					this->log << std::format("utf8 parser: invalid utf8, no prefix on following byte, state: {:d}: {:#x} {:c}\n",
							static_cast<unsigned int>(state), src_char, src_char);
					unicode = '*';
				}

				break;
			}

			default:
			{
				this->log << std::format("utf8 parser: invalid state {:d}\n", static_cast<unsigned int>(state));
				unicode = '*';
			}
		}

		dst.push_back(unicode);
		unicode = 0;
		state = u8p_state_base;
	}
}

bool Display::config_get(const std::string& id, int& value)
{
	int rv;

	try
	{
		rv = this->config.get_int(id);
		value = rv;
	}
	catch(const transient_exception &)
	{
		return(false);
	}

	return(true);
}

bool Display::config_get(const std::string& id, bool& value)
{
	int rv;

	try
	{
		rv = this->config.get_int(id);
		value = (rv != 0);
	}
	catch(const transient_exception &)
	{
		return(false);
	}

	return(true);
}

void Display::load_font(std::string_view fontname)
{
	int fd;
	std::string pathfont = std::format("/littlefs/{}", fontname);
	std::string our_hash;
	std::string their_hash;

	if((fd = open(pathfont.c_str(), O_RDONLY, 0)) < 0)
		throw(transient_exception(std::format("Display: failed to open font {}", pathfont)));

	this->fontinfo.valid = false;
	this->fontinfo.magic_word = 0;
	this->fontinfo.raw.width = 0;
	this->fontinfo.raw.height = 0;
	this->fontinfo.net.width = 0;
	this->fontinfo.net.height = 0;
	this->fontinfo.extra_glyphs = 0;
	this->fontinfo.columns = 0;
	this->fontinfo.rows = 0;

	if((!this->fontstructptr) && !(this->fontstructptr = std::make_unique<font_t>()))
	{
		close(fd);
		throw(hard_exception("Display: out of memory"));
	}

	if(read(fd, this->fontstructptr.get(), sizeof(font_t)) != sizeof(font_t))
	{
		close(fd);
		throw(transient_exception(std::format("Display: failed to read font {}", pathfont)));
	}

	close(fd);

	if(this->fontstructptr->magic_word != this->fontstruct_magic_word)
	{
		this->fontstructptr.reset();
		throw(transient_exception(std::format("Display: font file magic word invalid: {:#x}", static_cast<unsigned int>(this->fontstructptr->magic_word))));
	}

	their_hash.resize(32);

	memcpy(their_hash.data(), this->fontstructptr->checksum, their_hash.size());
	memset(this->fontstructptr->checksum, 0, sizeof(font_t::checksum));

	our_hash = Crypt::sha256(std::string_view(reinterpret_cast<const char *>(this->fontstructptr.get()), sizeof(font_t)));

	if(our_hash != their_hash)
	{
		this->fontstructptr.reset();
		throw(transient_exception("Display: font file invalid checksum"));
	}

	this->fontinfo.valid = true;
	this->fontinfo.magic_word = this->fontstructptr->magic_word;
	this->fontinfo.raw.width = this->fontstructptr->raw.width;
	this->fontinfo.raw.height = this->fontstructptr->raw.height;
	this->fontinfo.net.width = this->fontstructptr->net.width;
	this->fontinfo.net.height = this->fontstructptr->net.height;
	this->fontinfo.extra_glyphs =this->fontstructptr->extra_glyphs;
	this->fontinfo.columns = (this->display_x_size() - (2 * this->page_border_size)) / this->fontinfo.net.width;
	this->fontinfo.rows = (this->display_y_size() - this->page_text_offset - (2 * this->page_border_size)) / this->fontinfo.net.height;
}

std::string Display::make_title(std::string_view title)
{
	int pad, chop;
	std::string stamp_string = Util::get().time_to_string(time(nullptr), "{:%d/%m %H:%M}");
	std::string tag;

	chop = string_length_utf8(title);

	if(stamp_string.length() > this->fontinfo.columns)
		stamp_string.clear();

	if((chop + stamp_string.length()) > this->fontinfo.columns)
		chop = this->fontinfo.columns - stamp_string.length();

	pad = this->fontinfo.columns - stamp_string.length() - chop;

	if(chop < 0)
		throw(hard_exception("Display::make_title: chop < 0"));

	if(pad < 0)
		throw(hard_exception("Display::make_title: pad < 0"));

	tag = title;
	std::replace(tag.begin(), tag.end(), '_', ' ');

	if(chop < tag.length())
		tag = tag.substr(0, chop);

	tag.append(pad, ' ');
	tag.append(stamp_string);

	return(tag);
}

Display::Page& Display::add_page(std::scoped_lock<std::mutex>&, const std::string& name, Display::Page::type_t type, int lifetime)
{
	PagesData::iterator it;

	if((it = this->pages.data.find(name)) != this->pages.data.end())
	{
		if(it->second.type == type)
			it->second.clear();
		else
		{
			this->pages.data.erase(name);
			this->pages.version++;
			it = this->pages.data.end();
		}
	}

	if(it == this->pages.data.end())
	{
		this->pages.data.try_emplace(name, this->log, type, this->next_colour);
		this->pages.version++;

		if((it = this->pages.data.find(name)) == this->pages.data.end())
			throw(hard_exception("Display::add_page: key not found"));

		this->next_colour = this->colour_next(this->next_colour);
	}

	it->second.expiry = (lifetime > 0) ? time(nullptr) + lifetime : 0;

	return(it->second);
}

void Display::clear(colour_t colour)
{
	if(!this->module)
		throw(hard_exception("Display: no display active"));

	const auto colourmap_it = this->colourmap.find(colour);

	if(colourmap_it == this->colourmap.end())
		throw(hard_exception("Display::clear: unknown colour"));

	this->module->clear(colourmap_it->second.r, colourmap_it->second.g, colourmap_it->second.b);
}

void Display::box(const Display::box_rgb_args_t& args)
{
	if(!this->module)
		throw(hard_exception("Display: no display active"));

	this->module->box(args);
}

void Display::box(const Display::box_colour_args_t& args)
{
	const auto colourmap_it = this->colourmap.find(args.colour);

	if(colourmap_it == this->colourmap.end())
		throw(hard_exception("Display::box: unknown colour"));

	this->box({ .r = colourmap_it->second.r, .g = colourmap_it->second.g, .b = colourmap_it->second.b, .geometry = args.geometry });
}

void Display::plot(const Display::plot_args_t& args)
{
	if(!this->module)
		throw(hard_exception("Display: no display active"));

	if(this->module->set_window(args.window))
		this->module->plot(args.length, args.pixels);
}

void Display::set_active_layer(int layer)
{
	if(!this->module)
		throw(hard_exception("Display: no display active"));

	this->module->set_active_layer(layer);
}

void Display::show_layer(int layer)
{
	if(!this->module)
		throw(hard_exception("Display: no display active"));

	this->module->show_layer(layer);
}

void Display::write(const write_args_t& args)
{
	const font_glyph_t *glyph;
	colour_t new_colour;
	int current_x_offset;
	int column, row, stride;
	int current_glyph;
	unsigned int glyph_row;
	rgb_t fg_rgb;
	rgb_t bg_rgb;
	std::deque<unsigned int> unicode_line;
	std::vector<rgb_t> pixels;
	geometry_t window;

	if(!this->module)
		throw(hard_exception("Display::write: no display module"));

	if(!this->fontinfo.valid)
		throw(hard_exception("Display::write: no font loaded"));

	if(!this->colourmap.contains(args.fg) || !this->colourmap.contains(args.bg))
		throw(hard_exception("Display::write: invalid colour argument"));

	fg_rgb = this->colourmap.at(args.fg);
	bg_rgb = this->colourmap.at(args.bg);

	if((args.at_line < 0) || (args.y_offset < 0) || (args.x_pad < 0) || (args.y_pad < 0))
		throw(hard_exception("Display::write: invalid position argument"));

	window.from_x = args.x_pad;
	window.to_x = this->display_x_size() - 1 - args.x_pad;

	if((window.from_x >= this->display_x_size()) || (window.to_x < 0))
		return;

	window.from_y = args.y_pad + args.y_offset + (args.at_line * this->fontinfo.net.height);
	window.to_y = window.from_y + this->fontinfo.net.height - 1;

	if((window.from_y >= (this->display_y_size() - args.y_pad)) || (window.to_y >= (this->display_y_size() - args.y_pad)))
		return;

	stride = window.to_x - window.from_x + 1;
	pixels.resize(stride * (window.to_y - window.from_y + 1));

	this->utf8_to_unicode(args.text, unicode_line);
	current_x_offset = 0;

	for(const auto& unicode : unicode_line)
	{
		if(current_x_offset >= (window.to_x - window.from_x))
			break;

		if((unicode >= 0xf800) && (unicode < 0xf808)) // abuse private use unicode codepoints for foreground colours
		{
			new_colour = this->colour_from_int(unicode - 0xf800);

			if(!this->colourmap.contains(new_colour))
				this->log << std::format("Display::write: foreground colour out of range: {:d}", static_cast<unsigned int>(new_colour));
			else
				fg_rgb = this->colourmap.at(new_colour);

			continue;
		}

		if((unicode >= 0xf808) && (unicode < 0xf810)) // abuse private use unicode codepoints for background colours
		{
			new_colour = this->colour_from_int(unicode - 0xf808);

			if(!this->colourmap.contains((new_colour)))
				this->log << std::format("Display::write: background colour out of range: {:d}", static_cast<unsigned int>(new_colour));
			else
				bg_rgb = this->colourmap.at(new_colour);

			continue;
		}

		glyph = nullptr;

		if(unicode < this->fontstruct_basic_glyphs)
			glyph = &this->fontstructptr->basic_glyph[unicode];
		else
		{
			for(current_glyph = 0; current_glyph < this->fontinfo.extra_glyphs; current_glyph++)
			{
				glyph = &this->fontstructptr->extra_glyph[current_glyph];

				if(glyph->codepoint == unicode)
					break;
			}

			if(current_glyph >= this->fontinfo.extra_glyphs)
				glyph = nullptr;
		}

		if(!glyph)
			continue;

		for(row = 0; row < this->fontinfo.net.height; row++)
		{
			glyph_row = glyph->row[row];

			for(column = 0; (column < this->fontinfo.net.width) && ((current_x_offset + column) < (window.to_x - window.from_x)); column++)
				pixels.at((row * stride) + current_x_offset + column) = (glyph_row & (1 << column)) ? fg_rgb : bg_rgb;
		}

		current_x_offset += this->fontinfo.net.width;
	}

	this->plot({
		.window = window,
		.length = static_cast<int>(pixels.size()),
		.pixels = pixels.data(),
	});

	if((window.from_x += current_x_offset) <= window.to_x)
		this->box({ .r = bg_rgb.r, .g = bg_rgb.g, .b = bg_rgb.b, .geometry = window });
}

void Display::run_thread()
{
	std::string entry_text;
	time_t stamp;
	int data_version;
	int row, from_x, from_y, to_x, to_y, max_x;
	int x_size, y_size;
	uint64_t time_start, time_spent;
	struct stat statb;
	png_handle_t *handle;
	PagesData::const_iterator page;
	int active_layer = 0;
	std::vector<rgb_t> row_buffer; // = rgb_composite_t = byte[3]

	data_version = 0;

	try
	{
		for(;;)
		{
			try
			{
				switch(this->mode)
				{
					case(mode_t::init_log):
					{
						try
						{
							this->load_font("font_small");
							this->clear(Display::colour_t::black);
							this->log_line = 0;
							this->mode = mode_t::log;
						}
						catch(const transient_exception &e)
						{
						}

						break;
					}

					case(mode_t::init_info):
					{
						try
						{
							std::scoped_lock<std::mutex> pages_lock(this->pages_mutex);
							this->load_font("font_big");
							this->pages.version = 0;
							page = this->pages.data.begin();
							active_layer = 0;
							this->set_active_layer(1);
							this->show_layer(0);
							this->clear(colour_t::black);
							this->mode = mode_t::info;
						}
						catch(const transient_exception &)
						{
						}

						break;
					}

					case(mode_t::log):
					{
						if(!this->log.get_entry(stamp, entry_text))
						{ // pages_lock
							std::scoped_lock<std::mutex> pages_lock(this->pages_mutex);

							if(this->pages.data.size() > 0)
								this->mode = mode_t::init_info;

							throw(transient_exception());
						}

						this->write({
							.fg = Display::colour_t::white,
							.bg = Display::colour_t::black,
							.at_line = this->log_line,
							.text = this->util.time_to_string(stamp, "{:%H:%M:%S}") + " " + entry_text,
						});

						if(++this->log_line >= this->fontinfo.rows)
							this->log_line = 0;

						this->box({
								.colour = colour_t::black,
								.geometry =
								{
									.from_x = 0,
									.from_y = this->log_line * this->fontinfo.net.height,
									.to_x = this->display_x_size() - 1,
									.to_y = ((this->log_line + 1) * this->fontinfo.net.height) - 1,
								},
						});

						break;
					}

					case(mode_t::info):
					{
						{ // pages lock
							std::scoped_lock<std::mutex> pages_lock(this->pages_mutex);

							if(this->pages.version != data_version)
							{
								data_version = this->pages.version;
								page = this->pages.data.begin();
							}
							else
							{
								if(page == this->pages.data.end())
									page = this->pages.data.begin();
								else
								{
									page++;

									if(page == this->pages.data.end())
										page = this->pages.data.begin();
								}
							}

							if((page != this->pages.data.end()) && (page->second.expiry > 0) && (time(nullptr) > page->second.expiry))
								page = this->pages.data.erase(page);

							if(page == this->pages.data.end())
							{
								this->mode = mode_t::init_log;
								throw(transient_exception());
							}

							time_start = esp_timer_get_time();

							x_size = this->display_x_size();
							y_size = this->display_y_size();

							this->box({
								.colour = page->second.colour,
								.geometry =
								{
									.from_x = 0,
									.from_y = 0,
									.to_x =	page_border_size - 1,
									.to_y = y_size - 1,
								},
							});

							this->box({
								.colour = page->second.colour,
								.geometry =
								{
									.from_x = 0,
									.from_y = 0,
									.to_x = x_size - 1,
									.to_y = this->page_border_size - 1,
								},
							});

							this->box({
								.colour = page->second.colour,
								.geometry =
								{
									.from_x = x_size - 1 - this->page_border_size + 1,
									.from_y = 0,
									.to_x = x_size - 1,
									.to_y =	y_size - 1,
								},
							});

							this->box({
								.colour = page->second.colour,
								.geometry =
								{
									.from_x = 0,
									.from_y = y_size - 1 - page_border_size + 1,
									.to_x = x_size - 1,
									.to_y = y_size - 1,
								},
							});

							this->box({
								.colour = page->second.colour,
								.geometry =
								{
									.from_x = 0,
									.from_y = this->page_border_size + this->fontinfo.net.height,
									.to_x = x_size - 1,
									.to_y = this->page_border_size + this->fontinfo.net.height + this->page_text_offset - 1,
								},
							});

							this->write({
								.fg = colour_t::white,
								.bg = page->second.colour,
								.at_line = 0,
								.y_offset = 0,
								.x_pad = this->page_border_size,
								.y_pad = this->page_border_size,
								.text = this->make_title(page->first),
							});

							switch(page->second.type)
							{
								case(Page::type_t::text):
								{

									for(row = 0; row < page->second.text.lines.size(); row++)
									{
										this->write({
											.fg = colour_t::black,
											.bg = colour_t::white,
											.at_line = row + 1,
											.y_offset = this->page_text_offset,
											.x_pad = this->page_border_size,
											.y_pad = this->page_border_size,
											.text = page->second.text.lines[row],
										});
									}

									from_y = ((row + 1) * this->fontinfo.net.height) + this->page_border_size + this->page_text_offset;

									if(from_y <= (y_size - 1 - this->page_border_size))
										this->box({
											.colour = colour_t::white,
											.geometry =
											{
												.from_x = this->page_border_size,
												.from_y = from_y,
												.to_x = x_size - 1 - this->page_border_size,
												.to_y = y_size - 1 - this->page_border_size,
											},
										});

									break;
								}

								case(Page::type_t::image):
								{
									if(stat(page->second.image.filename.c_str(), &statb))
										throw(transient_exception(std::format("cannot stat image file: {}", page->second.image.filename)));

									if(statb.st_size != page->second.image.length)
									{
										this->stats["skipped incomplete images"]++;
										throw(transient_exception());
									}

									if(!(handle = png_open(page->second.image.filename.c_str())))
										throw(transient_exception(std::format("error in png_open(\"{}\")", page->second.image.filename)));

									row_buffer.resize(handle->width);

									max_x = x_size - 1 - this->page_border_size;
									from_y = this->page_border_size + this->fontinfo.net.height + this->page_text_offset;
									from_x = this->page_border_size;
									to_y = y_size - 1 - this->page_border_size;
									to_x = this->page_border_size + handle->width;

									if(to_x > max_x)
										to_x = max_x;

									for(row = 0; row < handle->height; row++, from_y++)
									{
										if(from_y > to_y)
											break;

										if(!png_decode_row(handle, handle->row_size, reinterpret_cast<uint8_t *>(row_buffer.data())))
											break;

										this->plot({
												.window =
												{
													.from_x = from_x,
													.from_y = from_y,
													.to_x = to_x,
													.to_y = from_y,
												},
												.length = static_cast<int>(handle->width),
												.pixels = row_buffer.data(),
										});

										to_x++;

										if(to_x <= max_x)
											this->box({
												.colour = page->second.colour,
												.geometry =
												{
													.from_x = to_x,
													.from_y = from_y,
													.to_x = max_x,
													.to_y = from_y,
												},
											});
									}

									if(from_y < to_y)
										this->box({
											.colour = page->second.colour,
											.geometry =
											{
												.from_x = this->page_border_size,
												.from_y = from_y,
												.to_x = x_size - 1 - this->page_border_size,
												.to_y = to_y,
											},
										});

									png_close(&handle);

									break;
								}

								default:
								{
									throw(hard_exception(std::format("display: unknown page type: {:d}", static_cast<unsigned int>(page->second.type))));
									break;
								}
							}
						}

						this->show_layer(active_layer);
						active_layer = (active_layer + 1) % 2;
						this->set_active_layer(active_layer);

						time_spent = esp_timer_get_time() - time_start;
						this->stats["display show time"] = time_spent / 1000ULL;

						std::this_thread::sleep_for(std::chrono::milliseconds(8000));

						break;
					}

					default:
					{
						throw(hard_exception("invalid mode"));
						break;
					}
				}
			}
			catch(const transient_exception &e)
			{
				std::string what(e.what());

				if(what.length() > 0)
					this->log << std::format("Display info: {}", what);

				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
		}
	}
	catch(const transient_exception &e)
	{
		this->log.abort(std::format("Display::run_thread: unhandled transient exception: {}", e.what()));
	}
	catch(const hard_exception &e)
	{
		this->log.abort(std::format("Display::run_thread: unhandled hard exception: {}", e.what()));
	}
	catch(const std::exception &e)
	{
		this->log.abort(std::format("Display::run_thread: unhandled std::exception: {}", e.what()));
	}
	catch(...)
	{
		this->log.abort(std::format("Display::run_thread: unhandled unknown exception"));
	}

	this->log.abort("Display::run_thread: main loop returns");
}

Display& Display::get()
{
	if(!Display::singleton)
		throw(hard_exception("Display not active"));

	return(*Display::singleton);
}

std::string Display::info()
{
	std::string out, datetime;
	int page_index, line_index;
	std::scoped_lock<std::mutex> pages_lock(this->pages_mutex);

	if(!this->module)
	{
		out += "\nno display active";
		return(out);
	}

	out  = std::format("DISPLAY information for display: {}, dimensions {:d} x {:d} using interface {}", this->module->_name(), this->display_x_size(), this->display_y_size(), this->module->_interface());
	out += std::format("\n- configuration: flip: {}, invert {}, rotate {}", Util::yesno(this->module->flip), Util::yesno(this->module->invert), Util::yesno(this->module->rotate));

	if(!this->fontinfo.valid)
	{
		out += "\nno display font loaded";
		return(out);
	}

	out += "\nfont info: ";
	out += std::format("\n- magic word: {:#x}", this->fontinfo.magic_word);
	out += std::format("\n- raw width: {:d}", this->fontinfo.raw.width);
	out += std::format("\n- raw height: {:d}", this->fontinfo.raw.height);
	out += std::format("\n- net width: {:d}", this->fontinfo.net.width);
	out += std::format("\n- net height: {:d}", this->fontinfo.net.height);
	out += std::format("\n- basic glyphs: {:d}", this->fontstruct_basic_glyphs);
	out += std::format("\n- extra glyphs: {:d}/{:d}", this->fontinfo.extra_glyphs, this->fontstruct_max_extra_glyphs);
	out += std::format("\n- columns: {:d}", this->fontinfo.columns);
	out += std::format("\n- rows: {:d}", this->fontinfo.rows);

	out += "\npages:";

	page_index = 0;

	for(const auto& page : this->pages.data)
	{
		if(page.second.expiry > 0)
			datetime = this->util.time_to_string(page.second.expiry, "{:%d/%m %h:%m}");
		else
			datetime = "<infinite>";

		out += std::format("\n- PAGE {:d}: \"{}\", expiry: {}, colour: {:s}, type: ",
				page_index, page.first, datetime, magic_enum::enum_name(page.second.colour));

		switch(page.second.type)
		{
			case(Page::type_t::text):
			{
				out += "text, contents:";

				line_index = 0;

				for(const auto& line : page.second.text.lines)
				{
					out += std::format("\n-   {:d}: {}", line_index, line);
					line_index++;
				}

				break;
			}

			case(Page::type_t::image):
			{
				out += std::format("image, file: {} ({:d}k)", page.second.image.filename, page.second.image.length / 1024);
				break;
			}

			default:
			{
				out += std::format("invalid type: {:d}", magic_enum::enum_integer(page.second.type));
				break;
			}
		}

		page_index++;
	}

	out += "\nSTATS:";

	for(const auto& stat : this->stats)
		out += std::format("\n- {:12}: {:d}", std::format("{}:", stat.first), stat.second);

	return(out);
}

int Display::display_x_size()
{
	if(!this->module)
		throw(hard_exception("Display: no display active"));

	return(this->module->display_x_size());
}

int Display::display_y_size()
{
	if(!this->module)
		throw(hard_exception("Display: no display active"));

	return(this->module->display_y_size());
}

int Display::image_x_size()
{
	if(!this->fontinfo.valid)
		return(0);

	return(this->display_x_size() - (2 * this->page_border_size));
}

int Display::image_y_size()
{
	if(!this->fontinfo.valid)
		return(0);

	return(this->display_y_size() - ((2 * this->page_border_size) + this->page_text_offset + this->fontinfo.net.height - 1));
}

void Display::configure(int type, int interface_index, int __x_size, int __y_size, bool flip_, bool invert_, bool rotate_)
{
	if(!magic_enum::enum_contains<Display::module_id_t>(type))
		throw(transient_exception("display::configure: invalid type"));

	this->config.erase_wildcard("display.");
	this->config.set_int("display.type", type);
	this->config.set_int("display.if", interface_index);
	this->config.set_int("display.size.x", __x_size);
	this->config.set_int("display.size.y", __y_size);
	this->config.set_int("display.flip", flip_ ? 1 : 0);
	this->config.set_int("display.invert", invert_ ? 1 : 0);
	this->config.set_int("display.rotate", rotate_ ? 1 : 0);
}

void Display::erase()
{
	this->config.erase_wildcard("display.");
}

void Display::brightness(int value)
{
	if(!this->module)
		throw(hard_exception("Display: no display active"));

	this->module->brightness(value);
}

void Display::add_text_page(const std::string& name, int lifetime, std::string_view contents)
{
	auto contents_it = contents.begin();
	std::string line;
	std::scoped_lock<std::mutex> pages_lock(this->pages_mutex);
	auto& page = this->add_page(pages_lock, name, Page::type_t::text, lifetime);

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
				page.text.lines.push_back(line);

				line.clear();

				break;
			}

			case('\n'):
			{
				page.text.lines.push_back(line);

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
}

void Display::add_image_page(const std::string& name, int lifetime, std::string_view filename, int length)
{
	std::scoped_lock<std::mutex> pages_lock(this->pages_mutex);
	auto& page = this->add_page(pages_lock, name, Page::type_t::image, lifetime);

	page.image.filename = filename;
	page.image.length = length;
}

bool Display::remove_page(const std::string& name)
{
	std::scoped_lock<std::mutex> pages_lock(this->pages_mutex);
	PagesData::const_iterator it;

	if((this->pages.data.find(name)) == this->pages.data.end())
		return(false);

	this->pages.data.erase(it);
	this->pages.version++;

	return(true);
}
