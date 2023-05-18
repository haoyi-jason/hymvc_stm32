#ifndef _SYSTEM_VALID
#define _SYSTEM_VALID

#include "app_config.h"

enum _STOW_STATE_e
{
  STOW_NONE,
  STOW_LOCK_INIT,
  STOW_LOCKING,
  STOW_LOCK_DONE,
  STOW_UNLOCK_INIT,
  STOW_UNLOCKING,
  STOW_UNLOCK_DONE,
  STOW_TIMEOUT,
  STOW_CHECK_FAIL,
};

typedef struct _state_machine_s{
  uint8_t state;
  uint8_t nextState;
  uint8_t subState;
}_stow_state_t;

void validStow(uint8_t activity,_stow_control_t *pctrl);

#endif