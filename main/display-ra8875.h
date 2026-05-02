#pragma once

class DisplayModuleRA8875 final : public DisplayModuleSPI
{
	public:

		explicit DisplayModuleRA8875() = delete;
		explicit DisplayModuleRA8875(DisplayModuleRA8875 &) = delete;
		explicit DisplayModuleRA8875(DisplayModuleRA8875 &&) = delete;
		DisplayModuleRA8875& operator =(const DisplayModuleRA8875 &) = delete;
		explicit DisplayModuleRA8875(Config&, Log&, Util&, SPI&, LedPWM&, int module_index, int x_size, int y_size, bool flip, bool invert, bool rotate);

		static constexpr const Display::module_id_t type = Display::module_id_t::ra8875;

	private:

		int active_layer;

		int read_status();
		void write_register(unsigned char reg, unsigned char data);

		void set_fgcolour(unsigned int r, unsigned int g, unsigned int b);
		void set_bgcolour(unsigned int r, unsigned int g, unsigned int b);

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
