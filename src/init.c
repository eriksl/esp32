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

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs.h>
#include <nvs_flash.h>

void app_main(void)
{
	string_module_init();
	log_init();
	config_init();
	util_init();
	cli_init();
	bt_init();
	console_init();

	run_main();
}
