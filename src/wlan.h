#pragma once

void wlan_init(void);
void wlan_tcp_send(const cli_buffer_t *cli_buffer);

void wlan_command_client_config(cli_command_call_t *call);
void wlan_command_info(cli_command_call_t *call);
void wlan_command_ip_info(cli_command_call_t *call);
