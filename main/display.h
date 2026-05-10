#pragma once

#include "config.h"
#include "log.h"
#include "util.h"
#include "spi.h"
#include "ledpwm.h"

#include <cstdint>
#include <mutex>
#include <deque>
#include <map>
#include <vector>
#include <memory>

class DisplayModule;

class Display
{
	public:

		explicit Display() = delete;
		explicit Display(Display &) = delete;
		explicit Display(Display &&) = delete;
		explicit Display(Config&, Log&, Util&, SPI&, LedPWM&);
		Display& operator =(const Display &) = delete;
		~Display();

		enum class module_id_t
		{
			none = 0,
			generic_spi = 1,
			ra8875 = 2,
			lt7381 = 3,
		};

		enum class colour_t
		{
			black = 0,
			blue,
			green,
			cyan,
			red,
			purple,
			yellow,
			white,
		};

		enum class mode_t
		{
			init_log,
			init_info,
			log,
			info,
		};

		struct __attribute__((packed)) rgb_t
		{
			unsigned char r;
			unsigned char g;
			unsigned char b;
		};

		struct __attribute__((packed)) rgb_composite_t
		{
			union
			{
				rgb_t as_struct;
				unsigned char as_array[3];
			};
		};
		static_assert(sizeof(rgb_t) == 3);
		static_assert(sizeof(rgb_composite_t) == 3);
		static_assert(offsetof(rgb_t, r) == 0);
		static_assert(offsetof(rgb_t, g) == 1);
		static_assert(offsetof(rgb_t, b) == 2);
		static_assert(offsetof(rgb_composite_t, as_struct) == 0);
		static_assert(offsetof(rgb_composite_t, as_array) == 0);

		struct geometry_t
		{
			int from_x = 0;
			int from_y = 0;
			int to_x = 0;
			int to_y = 0;
		};

		struct box_colour_args_t
		{
			colour_t colour = colour_t::black;
			geometry_t geometry;
		};

		struct box_rgb_args_t
		{
			int r = 0;
			int g = 0;
			int b = 0;
			geometry_t geometry;
		};

		struct plot_args_t
		{
			geometry_t window;
			int length;
			const rgb_t* pixels;
		};

		struct write_args_t
		{
			colour_t fg = colour_t::white;
			colour_t bg = colour_t::black;
			int at_line = 0;
			int y_offset = 0;
			int x_pad = 0;
			int y_pad = 0;
			std::string_view text = "";
		};

	private:

		static constexpr const int page_border_size = 3;
		static constexpr const int page_text_offset = 1;

		static constexpr const int fontstruct_magic_word = 0xf0bdf11e;
		static constexpr const int fontstruct_cols_size = 16;
		static constexpr const int fontstruct_rows_size = 32;
		static constexpr const int fontstruct_basic_glyphs = 256;
		static constexpr const int fontstruct_max_extra_glyphs = 128;

		struct __attribute__((packed)) font_glyph_t
		{
			std::uint32_t codepoint;
			std::uint16_t row[fontstruct_rows_size];
		};

		struct __attribute__((packed)) font_t
		{
			std::uint32_t magic_word;
			std::uint8_t checksum[32];
			struct
			{
				std::uint32_t width;
				std::uint32_t height;
			} raw;
			struct
			{
				std::uint32_t width;
				std::uint32_t height;
			} net;
			std::uint32_t extra_glyphs;
			font_glyph_t basic_glyph[fontstruct_basic_glyphs];
			font_glyph_t extra_glyph[fontstruct_max_extra_glyphs];
		};

		struct fontinfo_wxh_t
		{
			int width;
			int height;
		};

		struct fontinfo_t
		{
			bool valid;
			unsigned int magic_word;
			fontinfo_wxh_t raw;
			fontinfo_wxh_t net;
			int extra_glyphs;
			int columns;
			int rows;
		};

		class Page
		{
			public:

				enum class type_t : unsigned char
				{
					none,
					text,
					image,
				};

				Page() = delete;
				Page(const Page &) = delete;
				Page(const Page &&) = delete;
				Page& operator =(const Page &) = delete;
				Page(Log&, type_t type, Display::colour_t colour);
				~Page();

				Log &log;
				time_t expiry;
				type_t type;
				colour_t colour;
				struct
				{
					std::deque<std::string> lines;
				} text;
				struct
				{
					unsigned int length;
					std::string filename;
				} image;

				void clear();
		};

		using Colourmap = std::map<colour_t, rgb_t>;
		static const Colourmap colourmap;

		using PagesData = std::map<std::string, Page>;
		using Pages = struct
		{
			int version;
			PagesData data;
		};
		Pages pages;

		using Stats = std::map<std::string, int>;
		Stats stats;

		static Display* singleton;
		Config& config;
		Log& log;
		Util& util;
		SPI& spi;
		LedPWM& ledpwm;
		std::unique_ptr<DisplayModule> module;
		mode_t mode;
		int log_line;
		std::unique_ptr<font_t> fontstructptr;
		fontinfo_t fontinfo;
		colour_t next_colour;
		std::mutex pages_mutex;

		static colour_t colour_first();
		static colour_t colour_next(Display::colour_t colour_in);
		static colour_t colour_from_int(int colour_int);
		static int string_length_utf8(std::string_view);

		bool config_get(const std::string& id, int&);
		bool config_get(const std::string& id, bool&);
		void utf8_to_unicode(std::string_view src, std::deque<unsigned int> &dst);
		void load_font(std::string_view fontname);
		std::string make_title(std::string_view title);
		Page& add_page(std::scoped_lock<std::mutex>&, const std::string &name, Display::Page::type_t type, int lifetime);
		void clear(colour_t colour);
		void box(const box_colour_args_t&);
		void box(const box_rgb_args_t&);
		void plot(const plot_args_t&);
		void set_active_layer(int);
		void show_layer(int);
		void write(const write_args_t&);
		void run_thread();

	public:

		Display& get();
		std::string info();
		int display_x_size();
		int display_y_size();
		int image_x_size();
		int image_y_size();
		void configure(int type, int interface_index, int x_size, int y_size, bool flip, bool invert, bool rotate, bool blinvert);
		void erase();
		void brightness(int);
		void add_text_page(const std::string& name, int lifetime, std::string_view contents);
		void add_image_page(const std::string& name, int lifetime, std::string_view filename, int length);
		bool remove_page(const std::string& name);
};
