#include "ch.h"
#include "hal.h"
#include "task_canopen.h"
#include "shell.h"
#include "app_config.h"
static long counter = 0;


int main(void)
{
  halInit();
  chSysInit();
//  task_communication_init();
//  tpcmdh_taskInit();
//  tdmotc_algorithm_task_init();

  task_canopen_init();
  while(1){
    chThdSleepMilliseconds(100);
  }
}

