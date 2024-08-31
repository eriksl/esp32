#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "string.h"
#include "log.h"
#include "util.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void run_main(void)
{
	vTaskSuspend(NULL);
	util_abort("main: vTaskSuspend returned");
}
