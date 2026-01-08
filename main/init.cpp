#include <stdint.h>
#include <stdbool.h>

extern "C"
{
#include "string.h"
}

extern "C++"
{
#include "cli.h"
}

extern "C"
{
#include "log.h"
#include "console.h"
#include "config.h"
#include "util.h"
#include "fs.h"
#include "wlan.h"
#include "info.h"
#include "perftest.h"
#include "ramdisk.h"
#include "notify.h"
#include "display.h"
#include "ledpwm.h"
#include "mcpwm.h"
#include "i2c.h"
#include "sensor.h"
#include "io.h"
#include "pdm.h"
#include "ledpixel.h"
#include "process.h"

__attribute__((noreturn)) void app_main(void);
}

#include "udp.h"
#include "tcp.h"
#include "bt.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void app_main(void)
{
	try
	{
		console_init_1();
		ledpixel_init();
		ledpwm_init();
		notify_init();
		notify(notify_sys_booting);
		info_init();
		process_init();
		string_module_init();
		log_init();
		config_init();
		util_init();
		pdm_init();
		mcpwm_init();
		fs_init();
		ramdisk_init(initial_free_spiram / 2);
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
	catch(const char *e)
	{
		log_format("exception: %s", e);
		util_abort("init: exception caught");
		for(;;);
	}
	catch(...)
	{
		util_abort("init: exception caught");
		for(;;);
	}
}
