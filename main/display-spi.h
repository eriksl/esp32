#pragma once

#include <cstdint>

class DisplayModuleSPI : public DisplayModule
{
	public:

		explicit DisplayModuleSPI() = delete;
		explicit DisplayModuleSPI(DisplayModuleSPI&) = delete;
		explicit DisplayModuleSPI(DisplayModuleSPI&&) = delete;
		explicit DisplayModuleSPI(Config&, Log&, Util&, SPI&, LedPWM&, int module_index, int x_size, int y_size, bool flip, bool invert, bool rotate, bool blinvert);
		DisplayModuleSPI& operator =(const DisplayModuleSPI&) = delete;

		virtual std::string _name() override = 0;
		virtual std::string _interface() override;
		virtual void _brightness(int percentage) override = 0;
		virtual void _clear(int r, int g, int b) override = 0;
		virtual void _box(const Display::box_rgb_args_t&) override = 0;
		virtual void _set_window(const Display::geometry_t&) override = 0;
		virtual void _plot(int size, const Display::rgb_t* pixel) override = 0;
		virtual void _set_active_layer(int) override = 0;
		virtual void _show_layer(int) override = 0;

	private:

		SPI::Module& spi_module;

	protected:

		class SPIDataBuffer
		{
			public:

				explicit SPIDataBuffer() = delete;
				explicit SPIDataBuffer(const SPIDataBuffer &) = delete;
				explicit SPIDataBuffer(const SPIDataBuffer &&) = delete;
				SPIDataBuffer& operator =(const SPIDataBuffer&) = delete;
				explicit SPIDataBuffer(DisplayModuleSPI&, int size);

			private:

				SPI::data_t leader;
				DisplayModuleSPI& parent;
				int length;
				int size;
				unsigned char* buffer;

			public:

				void set_leader(SPI::rodata_t);
				void clear_leader();
				void push(SPI::rodata_t in);
				void flush();
		};

		SPIDataBuffer spi_data_buffer;

		int speed();
		void speed(int);

		void transfer(const SPI::transfer_t&);
		void set_leader(SPI::rodata_t);
		void clear_leader();
		void push(SPI::rodata_t in);
		void flush();
};
