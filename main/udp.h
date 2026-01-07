#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
void net_udp_send(const cli_buffer_t *cli_buffer);
#ifdef __cplusplus
}
#endif

void net_udp_init(void); // C++
