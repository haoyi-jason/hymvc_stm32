#ifndef _TASK_RESOLVER_
#define _TASK_RESOLVER_
#include "arm_math.h"


#define RAD2DEGG        180./PI

// fault bits
enum _FAULT_BITS{
  ERR_CONFIG_PAR,
  ERR_PHASE_OS,
  ERR_VELOCITY_OS,
  ERR_TRACKING,
  ERR_DOS_MISMATCH,
  ERR_DOS_OVERRANGE,
  ERR_BELOW_LOS,
  ERR_INPUT_CLIPPED
};

void resolver_task_init();
float resolver_get_speed(uint8_t id);
float resolver_get_position(uint8_t id);
float resolver_get_position_deg(uint8_t id);
uint16_t resolver_get_position_raw(uint8_t id);
void resolver_task_stop();
uint8_t lastError(uint8_t id);
#endif