#include "ch.h"
#include "hal.h"
#include "task_analog_input.h"
#include "task_dac.h"
#include "task_resolver.h"
#include "task_communication.h"
#include "task_dual_motor_ctrl.h"
#include "task_mbmaster.h"
#include "task_dac.h"

static long speed = 0;
static long pos = 0;
static long counter = 0;
static float output_sequence[4] = {-1.5f, 0.0f, 1.5f, 0.0f};
static uint8_t sequence_index = 0;
int main(void)
{
  halInit();
  chSysInit();;
  task_communication_init();
  //dmotc_algorithm_task_init();

  modbus_master_WriteCtrl(1, 3);
  chThdSleepMilliseconds(1000);
  while(1){
    if(!(counter % 500))
    {
      modbus_master_WriteCtrl(1, 3);
      sequence_index %= 4;
      analog_output_set_voltage(0, &output_sequence[sequence_index]);
      sequence_index++;
      
      counter = 0;
    }
    speed = modbus_master_ReadSpeed();
    pos = modbus_master_ReadPosition();
    counter++;
    chThdSleepMilliseconds(10);
  }
}