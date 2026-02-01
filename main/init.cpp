#include "exception.h"

extern "C"
{
__attribute__((noreturn)) void app_main(void);
}

#include "config.h"
#include "command.h"

#include "cli.h"
#include "log.h"
#include "alias.h"
#include "bt.h"
#include "console.h"
#include "display.h"
#include "fs.h"
#include "i2c.h"
#include "info.h"
#include "io.h"
#include "ledpixel.h"
#include "ledpwm.h"
#include "mcpwm.h"
#include "notify.h"
#include "perftest.h"
#include "pdm.h"
#include "process.h"
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
	try
	{
		console_init_1();
		Config config("config");
		Command command(config);
		ledpixel_init();
		ledpwm_init();
		notify_init();
		notify(notify_sys_booting);
		info_init();
		process_init();
		log_init();
		util_init();
		pdm_init();
		mcpwm_init();
		fs_init();
		ramdisk_init(initial_free_spiram / 2);
		alias_init();
		cli_init();
		bt_init();
		wlan_init();
		net_udp_init();
		net_tcp_init();
		perftest_init();
		console_init_2();
		display_init();
		i2c_init();
		io_init();
		sensor_init();
		notify(notify_sys_booting_finished);
		vTaskSuspend(NULL);
		throw("vTaskSuspend returned");
	}
	catch(const hard_exception &e)
	{
		std::string text;

		text = std::string("init: hard exception not handled: ") + e.what();
		util_abort(text.c_str());
		for(;;);
	}
	catch(const transient_exception &e)
	{
		std::string text;

		text = std::string("init: transient exception not handled: ") + e.what();
		util_abort(text.c_str());
		for(;;);
	}
	catch(const std::exception &e)
	{
		std::string text;

		text = std::string("init: std exception not handled: ") + e.what();
		util_abort(text.c_str());
		for(;;);
	}
	catch(const char *e)
	{
		std::string text;

		text = std::string("init: char exception not handled: ") + e;
		util_abort(text.c_str());
		for(;;);
	}
	catch(...)
	{
		util_abort("init: default exception not handled");
		for(;;);
	}
}
