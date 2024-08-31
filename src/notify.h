#pragma once

enum
{
	notify_state_size = 3,
};

void notify_init(void);
void notify(unsigned int state, unsigned int r, unsigned g, unsigned int b);
