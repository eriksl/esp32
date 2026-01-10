#include <stdint.h>
#include <stdbool.h>

extern "C"
{
__attribute__((noreturn)) void app_main(void);
}

#include "string.h"
#include "cli.h"
#include "log.h"
#include "bt.h"
#include "config.h"
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
