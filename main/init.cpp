extern "C"
{
void app_main(void);
}

#include "exception.h"
#include "config.h"
#include "console.h"
#include "ledpixel.h"
#include "ledpwm.h"
#include "notify.h"
#include "log.h"
#include "system.h"
#include "pdm.h"
#include "mcpwm.h"
#include "command.h"

#include "cli.h"
#include "alias.h"
#include "bt.h"
#include "display.h"
#include "fs.h"
#include "i2c.h"
#include "io.h"
#include "ramdisk.h"
#include "sensor.h"
#include "tcp.h"
#include "udp.h"
#include "util.h"
#include "wlan.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void app_main(void)
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
		Command command(config, console, ledpixel, ledpwm, notify, log, system, util, pdm, mcpwm);
		fs_init();
		ramdisk_init(system.get_initial_free_spiram() / 2);
		alias_init();
		cli_init();
		bt_init();
		wlan_init();
		net_udp_init();
		net_tcp_init();
		display_init();
		i2c_init();
		io_init();
		sensor_init();
		console.run();
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
