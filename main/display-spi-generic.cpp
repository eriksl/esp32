#include "display.h"
#include "display-module.h"
#include "display-spi.h"
#include "display-spi-generic.h"

DisplayModuleGenericSPI::DisplayModuleGenericSPI(Config& config_in, Log& log_in, Util& util_in, SPI& spi_in,
			int module_in, int x_size_in, int y_size_in, bool flip_in, bool invert_in, bool rotate_in) :
		DisplayModuleSPI(config_in, log_in, util_in, spi_in, module_in, x_size_in, y_size_in, flip_in, invert_in, rotate_in)
{
}

std::string DisplayModuleGenericSPI::_name()
{
	return("Generic SPI");
}

void DisplayModuleGenericSPI::_brightness(int brightness)
{
	// FIXME
}

void DisplayModuleGenericSPI::_clear(int r, int g, int b)
{
	// FIXME
}

void DisplayModuleGenericSPI::_box(const Display::box_rgb_args_t&)
{
	// FIXME
}

void DisplayModuleGenericSPI::_set_window(const Display::geometry_t&)
{
	// FIXME
}

void DisplayModuleGenericSPI::_plot(int length, const Display::rgb_t* pixels)
{
	// FIXME
}

void DisplayModuleGenericSPI::_set_active_layer(int)
{
	// FIXME
}

void DisplayModuleGenericSPI::_show_layer(int)
{
	// FIXME
}
