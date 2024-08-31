#include <stdint.h>
#include <stdbool.h>

#include "string.h"
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
#include "ramdisk.h"
#include "notify.h"

void app_main(void)
{
	notify_init();
	notify(0, 0xff, 0x00, 0x00);
	notify(1, 0xff, 0x00, 0x00);
	notify(2, 0xff, 0x00, 0x00);
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
#if 0
	perftest_init();
#endif
	console_init_2();
	notify(0, 0x00, 0x01, 0x00);
	notify(2, 0x00, 0x00, 0x00);

	run_main();
}
