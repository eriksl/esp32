#pragma once

void wlan_init(void);
void wlan_tcp_send(const cli_buffer_t *cli_buffer);
void wlan_udp_send(const cli_buffer_t *cli_buffer);
