#include <stdint.h>
#include <stdbool.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "string.h"
#include "cli.h"
#include "bt.h"
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

__attribute__((noreturn)) void app_main(void);

void app_main(void)
{
	console_init_1();
	ledpixel_init();
	ledpwm_init();
	notify_init();
	notify(notify_sys_booting);
	info_init();
	string_module_init();
	log_init();
	config_init();
	util_init();
	pdm_init();
	mcpwm_init();
	fs_init();
	ramdisk_init();
	cli_init();
	bt_init();
	wlan_init();
	perftest_init();
	console_init_2();
	display_init();
	i2c_init();
	io_init();
	sensor_init();
	notify(notify_sys_booting_finished);
	vTaskSuspend(NULL);
	util_abort("init: vTaskSuspend returned");
	for(;;);
}
