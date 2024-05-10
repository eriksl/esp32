#ifndef _bt_h_
#define _bt_h_

#include "cli.h"

#include <esp_check.h>

esp_err_t bt_init(void);
void bt_send(cli_buffer_t *);

#endif
