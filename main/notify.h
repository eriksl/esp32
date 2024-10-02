#pragma once

typedef enum
{
	notify_sys_booting = 0,
	notify_sys_booting_finished,
	notify_net_init,
	notify_net_associating,
	notify_net_associating_finished,
	notify_net_ipv4_acquired,
	notify_net_ipv6_ll_active,
	notify_net_ipv6_slaac_acquired,
	notify_net_ipv6_static_active,
	notify_net_ap_mode_init,
	notify_net_ap_mode_idle,
	notify_net_ap_mode_associated,
	notify_error,
	notify_size = notify_error,
} notify_t;

bool notify_init(void);
bool notify(notify_t);
