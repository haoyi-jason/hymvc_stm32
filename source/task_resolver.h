#ifndef _TASK_RESOLVER_
#define _TASK_RESOLVER_


void resolver_task_init();
float resolver_get_speed(uint8_t id);
float resolver_get_position(uint8_t id);
float resolver_get_position_deg(uint8_t id)

#endif