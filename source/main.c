#include "ch.h"
#include "hal.h"
#include "task_analog_input.h"
#include "task_dac.h"
#include "task_resolver.h"
#include "task_communication.h"

int main(void)
{
  halInit();
  chSysInit();;
  task_communication_init();
  while(1){
    chThdSleepMilliseconds(100);
  }
}