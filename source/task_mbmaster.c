#include "ch.h"
#include "hal.h"
#include "mbport.h"
#include "mbm.h"

#define EV_REGISTER_READ        EVENT_MASK(0)
#define EV_REGISTER_WRITE       EVENT_MASK(1)

struct _runTime{
  thread_t *self;
  uint8_t state;
};

static struct _runTime runTime, *mbmRuntime;

static THD_WORKING_AREA(waMBMaster,1024);
static THD_FUNCTION(procMBMaster ,p)
{
  xMBHandle xMBMaster;
  eMBErrorCode eStatus, eStatus2;
  
  USHORT usNRegs[5];
  USHORT cntr=0;
  if(MB_ENOERR == (eStatus = eMBMSerialInit(&xMBMaster,MB_RTU,0,9600,MB_PAR_NONE))){
    while(!chThdShouldTerminateX()){
        eStatus = MB_ENOERR;
        if(MB_ENOERR != (eStatus2 = eMBMWriteSingleRegister(xMBMaster,1,0,cntr++))){
          eStatus = eStatus2;
        }
           
       chThdSleepMilliseconds(1000);
    }
  }
  eMBMClose(&xMBMaster);
  
  
}

void modbus_master_task_init()
{
  mbmRuntime = &runTime;
  runTime.self = chThdCreateStatic(waMBMaster,sizeof(waMBMaster),NORMALPRIO,procMBMaster,NULL);
}