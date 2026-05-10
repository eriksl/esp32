#pragma once

class DisplayModuleLT7381 final : public DisplayModuleSPI
{
	public:

		explicit DisplayModuleLT7381() = delete;
		explicit DisplayModuleLT7381(DisplayModuleLT7381 &) = delete;
		explicit DisplayModuleLT7381(DisplayModuleLT7381 &&) = delete;
		DisplayModuleLT7381& operator =(const DisplayModuleLT7381 &) = delete;
		explicit DisplayModuleLT7381(Config&, Log&, Util&, SPI&, LedPWM&, int module_index, int x_size, int y_size, bool flip, bool invert, bool rotate, bool blinvert);

		static constexpr const Display::module_id_t type = Display::module_id_t::lt7381;

	private:

		int active_page;

		int read_status();
		int read_register(unsigned char reg);
		void write_register(unsigned char reg, unsigned char data);
		void check_register(unsigned char reg, unsigned char data);
		void write_and_check_register(unsigned char reg, unsigned char data);
		std::string pll_status(unsigned int reg1, unsigned int reg2, unsigned int &freq);
		void set_main_window_position(int x, int y);
		void set_active_window(int from_x, int from_y, int to_x, int to_y, int page);
		void box(int r, int g, int b, int from_x, int from_y, int to_x, int to_y, int page);

	protected:

		std::string _name() override;
		void _brightness(int percentage) override;
		void _clear(int r, int g, int b) override;
		void _box(const Display::box_rgb_args_t&) override;
		void _set_window(const Display::geometry_t&) override;
		void _plot(int length, const Display::rgb_t* pixel) override;
		void _set_active_layer(int) override;
		void _show_layer(int) override;
};
