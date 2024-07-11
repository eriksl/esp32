#include <stdint.h>
#include <stdbool.h>

#include "string.h"
#include "cli-command.h"
#include "cli.h"
#include "bt.h"
#include "main.h"
#include "log.h"
#include "console.h"
#include "config.h"
#include "util.h"
#include "fs.h"
#include "wlan.h"
#include "info.h"
#include "perftest.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs.h>
#include <nvs_flash.h>

void app_main(void)
{
	console_init_1();
	info_init();
	string_module_init();
	log_init();
	config_init();
	util_init();
	fs_init();
	cli_init();
	bt_init();
	wlan_init();
	perftest_init();
	console_init_2();

	run_main();
}
