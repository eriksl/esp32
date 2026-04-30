#include "display.h"
#include "display-module.h"

#include "exception.h"

DisplayModule::DisplayModule(Config& config_in, Log& log_in, Util& util_in, int x_size_in, int y_size_in, bool flip_in, bool invert_in, bool rotate_in) :
		config(config_in), log(log_in), util(util_in), x_size(x_size_in), y_size(y_size_in), flip(flip_in), invert(invert_in), rotate(rotate_in)
{
}

int DisplayModule::display_x_size()
{
	return(this->x_size);
}

int DisplayModule::display_y_size()
{
	return(this->y_size);
}

void DisplayModule::brightness(int percentage)
{
	if((percentage < 0) || (percentage > 100))
		throw(transient_exception("DisplayModule::brightness: value out of range"));

	this->_brightness(percentage);
}

void DisplayModule::clear(int r, int g, int b)
{
	this->_clear(r, g, b);
}

void DisplayModule::box(const Display::box_rgb_args_t& args)
{
	const Display::geometry_t& geo = args.geometry;

	if((geo.from_x < 0) || (geo.from_y < 0) || (geo.to_x < 0) || (geo.to_y < 0) ||
				(geo.to_x < geo.from_x) || (geo.to_y < geo.from_y))
	{
		this->log << std::format("box out of range: {:d} {:d} {:d} {:d}",
				geo.from_x,
				geo.from_y,
				geo.to_x,
				geo.to_y);
		throw(hard_exception("DisplayModule::box: invalid argument"));
	}

	this->_box(args);
}

bool DisplayModule::set_window(Display::geometry_t geo)
{
	if((geo.from_x < 0) || (geo.from_y < 0) || (geo.to_x < 0) || (geo.to_y < 0) || (geo.to_x < geo.from_x) || (geo.to_y < geo.from_y))
		throw(hard_exception("DisplayModule::set_window: invalid argument"));

	if((geo.from_x >= this->x_size) || (geo.from_y >= this->y_size))
		return(false);

	if(geo.to_x >= this->x_size)
		geo.to_x = this->x_size - 1;

	if(geo.to_y >= this->y_size)
		geo.to_y = this->y_size - 1;

	this->_set_window(geo);

	return(true);
}

void DisplayModule::plot(int length, const Display::rgb_t* pixels)
{
	this->_plot(length, pixels);
}

void DisplayModule::set_active_layer(int layer)
{
	this->_set_active_layer(layer);
}

void DisplayModule::show_layer(int layer)
{
	this->_show_layer(layer);
}
