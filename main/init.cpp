extern "C"
{
[[noreturn]] void app_main();
}

#include "exception.h"
#include "config.h"
#include "util.h"
#include "console.h"
#include "ledpixel.h"
#include "ledpwm.h"
#include "notify.h"
#include "log.h"
#include "system.h"
#include "pdm.h"
#include "mcpwm.h"
#include "fs.h"
#include "ramdisk.h"
#include "bt.h"
#include "command.h"

#include "display.h"
#include "i2c.h"
#include "io.h"
#include "sensor.h"
#include "tcp.h"
#include "udp.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void app_main()
{
	std::string exception_text;

	try
	{
		Config config("config");
		Util util(config);
		Console console(config);
		Ledpixel ledpixel;
		LedPWM ledpwm;
		Notify notify;
		notify.run();
		notify.notify(Notify::Notification::sys_booting);
		Log log(console, util);
		System system(log);
		PDM pdm(log);
		MCPWM mcpwm(log);
		Ramdisk::Root ramdisk(log, "/ramdisk", system.get_initial_free_spiram() / 2);
		FS fs(log, ramdisk);
		BT bt(log, config);
		Command command(config, console, ledpixel, ledpwm, notify, log, system, util, pdm, mcpwm, fs, bt);
		console.set(&command);
		bt.set(&command);
		wlan_init();
		net_udp_init();
		net_tcp_init();
		display_init();
		i2c_init();
		io_init();
		sensor_init();
		bt.run();
		console.run();
		command.run();
		notify.notify(Notify::Notification::sys_booting_finished);
		vTaskSuspend(NULL);
		throw("vTaskSuspend returned");
	}
	catch(const hard_exception &e)
	{
		exception_text = std::string("init: hard exception not handled: ") + e.what();
	}
	catch(const transient_exception &e)
	{
		exception_text = std::string("init: transient exception not handled: ") + e.what();
	}
	catch(const std::exception &e)
	{
		exception_text = std::string("init: std exception not handled: ") + e.what();
	}
	catch(const char *e)
	{
		exception_text = std::string("init: char exception not handled: ") + e;
	}
	catch(...)
	{
		exception_text = "init: default exception not handled";
	}

	Console::emergency_wall(exception_text);

	vTaskSuspend(NULL);
	for(;;);
}
