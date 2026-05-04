#pragma once

class DisplayModuleGenericSPI final : public DisplayModuleSPI
{
	public:

		explicit DisplayModuleGenericSPI() = delete;
		explicit DisplayModuleGenericSPI(DisplayModuleGenericSPI &) = delete;
		explicit DisplayModuleGenericSPI(DisplayModuleGenericSPI &&) = delete;
		explicit DisplayModuleGenericSPI(Config&, Log&, Util &, SPI&, LedPWM&, int module, int x_size, int y_size, bool flip, bool invert, bool rotate, bool blinvert);
		DisplayModuleGenericSPI& operator =(const DisplayModuleGenericSPI &) = delete;

		static constexpr const Display::module_id_t type = Display::module_id_t::generic_spi;

	protected:

		std::string _name() override;
		void _brightness(int percentage) override;
		void _clear(int r, int g, int b) override;
		void _box(const Display::box_rgb_args_t&) override;
		void _set_window(const Display::geometry_t&) override;
		void _plot(int length, const Display::rgb_t* pixels) override;
		void _set_active_layer(int) override;
		void _show_layer(int) override;

	private:

		void send_command(unsigned char);
		void send_command_data_1b(unsigned char, unsigned char);
		void send_command_data_2w(unsigned char, unsigned int word1, unsigned int word2);
		void set_window(int from_x, int from_y, int to_x, int to_y);
		void box(int r, int g, int b, int from_x, int from_y, int to_x, int to_y);

		int dc_gpio;
		LedPWM::Channel ledpwm_channel;
};
