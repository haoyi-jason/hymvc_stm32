#include "ch.h"
#include "hal.h"
#include "mbport.h"
#include "mbm.h"
#include "modbus_config.h"

#define EV_REGISTER_READ        EVENT_MASK(0)
#define EV_REGISTER_WRITE_1     EVENT_MASK(1)
#define EV_REGISTER_WRITE_2     EVENT_MASK(2)

struct _runTime{
  thread_t *self;
  uint8_t state;
  long     speed_act;
  long     pos_act;
  uint32_t signal_mask;
  uint16_t ctrl_1;
  uint16_t ctrl_2;
  uint16_t status_1;
  uint16_t status_2;
  virtual_timer_t vtModbus;
};

static struct _runTime runTime, *mbmRuntime;

static void modbus_SignalPeriod(void *arg)
{
  chSysLockFromISR();
  chEvtSignalI(runTime.self,EV_REGISTER_READ);
  chVTSetI(&runTime.vtModbus, TIME_MS2I(100),modbus_SignalPeriod,NULL);
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waMBMaster,1024);
static THD_FUNCTION(procMBMaster ,p)
{
  xMBHandle xMBMaster;
  static eMBErrorCode eStatus, eStatus2;
    
  USHORT usNRegs[5];
  USHORT cntr=0;
  USHORT buffer[4] = {0,0,0,0};

  uint16_t _counter_read = 0;

  chVTObjectInit(&runTime.vtModbus);
  chVTSetI(&runTime.vtModbus, TIME_MS2I(100),modbus_SignalPeriod,NULL);
  
  if(MB_ENOERR == (eStatus = eMBMSerialInit(&xMBMaster,MB_RTU,0,MODBUS_BAUDRATE,MB_PAR_NONE))){
    while(!chThdShouldTerminateX())
    {
      eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
      if(evt & EV_REGISTER_WRITE_1){
        if(MB_ENOERR != (eStatus2 = eMBMWriteSingleRegister(xMBMaster,
                                                            MODBUS_ADD_INV1,
                                                            MODBUS_REG_ADD_CTRL,
                                                            runTime.ctrl_1)))
        {
          /*Write error*/
          eStatus = eStatus2;
        }
      }

      if(evt & EV_REGISTER_WRITE_2){
        if(MB_ENOERR != (eStatus2 = eMBMWriteSingleRegister(xMBMaster,
                                                            MODBUS_ADD_INV2,
                                                            MODBUS_REG_ADD_CTRL,                                                            
                                                            runTime.ctrl_2)))
        {
          /*Write error*/
          eStatus = eStatus2;
        }
      }
      
      if(evt & EV_REGISTER_READ)
      {
        eStatus = MB_ENOERR;
        if(!(_counter_read % 10))
        {
          if(MB_ENOERR != (eStatus2 = eMBMReadHoldingRegisters(xMBMaster,
                                                               MODBUS_ADD_INV1,
                                                               MODBUS_REG_ADD_STATUS,
                                                               1,
                                                               buffer)))
          {
            /*Reading error*/
            eStatus = eStatus2;
          }
          else
          {
            /*Read*/
            runTime.status_1 = buffer[0];
          }

          if(MB_ENOERR != (eStatus2 = eMBMReadHoldingRegisters(xMBMaster,
                                                               MODBUS_ADD_INV2,
                                                               MODBUS_REG_ADD_STATUS,
                                                               1,
                                                               buffer)))
          {
            /*Reading error*/
            eStatus = eStatus2;
          }
          else
          {
            /*Read*/
            runTime.status_2 = buffer[0];
          }

          /*Reset counter*/
          _counter_read = 0;
        }

        if(MB_ENOERR != (eStatus2 = eMBMReadHoldingRegisters(xMBMaster,
                                                             MODBUS_ADD_INV1,
                                                             MODBUS_REG_ADD_POS_ACT,
                                                             2,
                                                             buffer)))
        {
          /*Reading error*/
          eStatus = eStatus2;
        }
        else
        {
          /*Read*/
          runTime.pos_act = (((long)buffer[0] << 16) | (long)buffer[1]);
        }

        if(MB_ENOERR != (eStatus2 = eMBMReadHoldingRegisters(xMBMaster,
                                                             MODBUS_ADD_INV1,
                                                             MODBUS_REG_ADD_SPEED_ACT,
                                                             2,
                                                             buffer)))
        {
          /*Reading error*/
          eStatus = eStatus2;
        }
        else
        {
          /*Read*/
          runTime.speed_act = (((int32_t)buffer[0] << 16) | (int32_t)buffer[1]);
        }

        _counter_read++;
      }

       chThdSleepMilliseconds(50);
    }
  }
  eMBMClose(&xMBMaster);
}

void modbus_master_task_init()
{
  mbmRuntime = &runTime;
  runTime.self = chThdCreateStatic(waMBMaster,sizeof(waMBMaster),NORMALPRIO,procMBMaster,NULL);
}

int8_t modbus_master_WriteCtrl(uint16_t id, uint16_t value)
{
  int8_t ret = -1;

  if(id == 1)
  {
    runTime.ctrl_1 = value;
    chSysLockFromISR();
    chEvtSignalI(runTime.self,EV_REGISTER_WRITE_1);
    chSysUnlockFromISR();
    ret = 0;
  }
  else if(id == 2)
  {
    runTime.ctrl_2 = value;
    chSysLockFromISR();
    chEvtSignalI(runTime.self,EV_REGISTER_WRITE_2);
    chSysUnlockFromISR();
    ret = 0;
  }
  else
  {
    /*Bad id*/
    ret = -1;
  }

  return ret;
}

uint16_t modbus_master_ReadStatus(uint16_t id)
{
  if(id == 1)
  {

  }
  else if(id == 2)
  {

  }
  else
  {
    /*Bad id*/
  }
}

long modbus_master_ReadSpeed(void)
{
  return runTime.speed_act;
}

long modbus_master_ReadPosition(void)
{
  return runTime.pos_act;
}

