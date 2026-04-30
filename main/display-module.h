#pragma once

class DisplayModule
{
	friend class Display;

	protected:

		explicit DisplayModule() = delete;
		explicit DisplayModule(DisplayModule&) = delete;
		explicit DisplayModule(DisplayModule&&) = delete;
		explicit DisplayModule(Config&, Log&, Util&, int x_size, int y_size, bool flip, bool invert, bool rotate);
		DisplayModule& operator =(const DisplayModule&) = delete;

		Config& config;
		Log& log;
		Util& util;
		int x_size, y_size;
		bool flip, invert, rotate;

		virtual std::string _name() = 0;
		virtual std::string _interface() = 0;
		virtual void _brightness(int percentage) = 0;
		virtual void _clear(int r, int g, int b) = 0;
		virtual void _box(const Display::box_rgb_args_t&) = 0;
		virtual void _set_window(const Display::geometry_t&) = 0;
		virtual void _plot(int length, const Display::rgb_t* pixel) = 0;
		virtual void _set_active_layer(int) = 0;
		virtual void _show_layer(int) = 0;

		int display_x_size();
		int display_y_size();
		void brightness(int percentage);
		void clear(int r, int g, int b);
		void box(const Display::box_rgb_args_t&);
		bool set_window(Display::geometry_t);
		void plot(int length, const Display::rgb_t* pixels);
		void set_active_layer(int);
		void show_layer(int);
};
