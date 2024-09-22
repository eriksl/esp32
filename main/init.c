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

void app_main(void)
{
	notify_init();
	notify(notify_sys_booting);
	console_init_1();
	info_init();
	string_module_init();
	log_init();
	config_init();
	util_init();
	fs_init();
	ramdisk_init();
	cli_init();
	bt_init();
	wlan_init();
	perftest_init();
	console_init_2();
	display_init();
	notify(notify_sys_booting_finished);
	vTaskSuspend(NULL);
	util_abort("init: vTaskSuspend returned");
}
