#include "ch.h"
#include "hal.h"
#include "task_canopen.h"
#include "shell.h"
#include "app_config.h"
#include "task_baseoperation.h"
static long counter = 0;


int main(void)
{
  halInit();
  chSysInit();

//  task_canopen_init();
  task_baseOperationInit();
  while(1){
    chThdSleepMilliseconds(100);
  }
}

