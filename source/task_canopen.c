/*
file name :     task_canopen.c
author:         Jason Yeh
description:
  
  canopen main task for dual axis motor controller

*/

#include "ch.h"
#include "hal.h"

#include <stdlib.h>

/* canopen includes */
#include "canopen.h"
#include "od.h"
#include "301/co_driver.h"
#include "storage/co_storage.h"
#include "storage/co_storageEeprom.h"

#include "task_pos_cmd_handler.h"
#include "task_dual_motor_ctrl.h"

#include "task_canopen.h"
#include "task_analog_input.h"
#include "task_dac.h"
#include "task_resolver.h"
#include "app_config.h"
#include "ad76_drv.h"
#include "ad57_drv.h"
#include "digital_io.h"

#include "task_systemvalid.h"
#include "shell.h"
#include "chprintf.h"

#include "pid/nested_pid.h"


static void cmd_report(BaseSequentialStream *chp, int argc, char *argv[]); 
static void cmd_log(BaseSequentialStream *chp, int argc, char *argv[]); 
static void sys_log();
static void sys_logEx(_nested_pid_t *pid);
static void load_pid_param();
static void load_pid_param_dummy();
static void load_pid_param_ex();

static THD_WORKING_AREA(waShell, 512);
static const ShellCommand commands[] = {
  {"who", cmd_report},
  {"log", cmd_log},
  {NULL, NULL}
};

static SerialConfig serialCfg = {
  115200
};
static const ShellConfig shell_cfg = {
  (BaseSequentialStream *)&CONSOLE,
  commands
};

/* Default values for CO_CANopenInit() */
#define NMT_CONTROL                     (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)
#define FIRST_HB_TIME                   500
#define SDO_SRV_TIMEOUT_TIME            1000
#define SDO_CLI_TIMEOUT_TIME            500
#define SDO_CLI_BLOCK                   false
#define OD_STATUS_BITS                  NULL

#define DEG2RAD                         3.1415926/180.

#define CO_CONFIGURE_STORAGE_ENABLE     1

/* CAN masks for identifiers */
#define CANID_MASK                              0x07FF  /*!< CAN standard ID mask */
#define FLAG_RTR                                0x8000  /*!< RTR flag, part of identifier */

#define NOF_EXTENSION   4

#if(CO_CONFIGURE_STORAGE_ENABLE) & CO_CONFIG_STORAGE_ENABLE
 static CO_storage_entry_t storageEntries[] = {
    {
      .addr = &OD_PERSIST_COMM,
      .len = sizeof(OD_PERSIST_COMM),
      .subIndexOD = 2,
      .attr = CO_storage_cmd | CO_storage_restore,
      .addrNV = NULL
    }
  };
#endif

static  OD_extension_t OD_extension[NOF_EXTENSION] = 
{
  {
    .object = NULL,
    .read = OD_readOriginal,
    .write = OD_writeOriginal,
    .flagsPDO = {0xff,0xff,0xff,0xff}
  },
  {
    .object = NULL,
    .read = OD_readOriginal,
    .write = OD_writeOriginal,
    .flagsPDO = {0xff,0xff,0xff,0xff}
  },
  {
    .object = NULL,
    .read = OD_readOriginal,
    .write = OD_writeOriginal,
    .flagsPDO = {0xff,0xff,0xff,0xff}
  },
  {
    .object = NULL,
    .read = OD_readOriginal,
    .write = OD_writeOriginal,
    .flagsPDO = {0xff,0xff,0xff,0xff}
  }
};

static   CO_LSS_address_t lssAddress = {
    .identity = {
      .vendorID = VENDER_ID,
      .productCode = PROEUCT_CODE,
      .revisionNumber = REVISION_NUMBER,
      .serialNumber = SERIAL_NUMVER
    }
  };


typedef struct{
  thread_t *appThread;
  systime_t runtime;
  CO_t *CO;
  uint32_t co_heap_usage;
  CO_NMT_reset_cmd_t nmt_reset_cmd;
  virtual_timer_t vt_periodic;
  bool appStarted;
  uint8_t co_opState;
  _pid_config_t pidAZ;
  _pid_config_t pidEL;
  _dac_config dacConfig;
  uint8_t nodeId;
  bool appAutoStart;
  bool pidAutoStart;
  struct{
    uint8_t *pvFlag;
    uint8_t *spFlag;
    uint8_t *diFlag;
    uint8_t *doFlag;
    uint8_t *daFlag;
    uint8_t *statusFlag; // 6041
  }cosFlags;
#if(CO_CONFIGURE_STORAGE_ENABLE) & CO_CONFIG_STORAGE_ENABLE
  CO_storage_t storage;
#endif
  uint32_t mcStatus;
  uint16_t stowCommand;
  uint8_t stowValidState;
  _stow_control_t stowControl;
  bool localLog;
  uint32_t logIntervalMs;
  virtual_timer_t vtLog;
  _nested_pid_t pidConfig[2];
}_runTime_t;

static _runTime_t runTime, *pCORunTime;

static _storageModule_t storageModule = {
  &I2CD2,
  0x50,
  32,
  128,
  2
};

static CANConfig cfg500K = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_BRP(5) | CAN_BTR_TS1(10) | CAN_BTR_TS2(1) | CAN_BTR_SJW(1)
};

// can config for 250K baud
//static CANConfig cfg250K = {
//  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
//  CAN_BTR_BRP(11) | CAN_BTR_TS1(10) | CAN_BTR_TS2(1) | CAN_BTR_SJW(1)
//};

static void updateStatus(uint8_t mask, bool set);


void testcan(CANDriver *dev)
{
  CANTxFrame ptx;
  uint8_t i;
  canStop(dev);
  canStart(dev, &cfg500K);
  ptx.SID = 0x702;
//  ptx.IDE = CAN_IDE_STD;
  ptx.RTR = CAN_RTR_DATA;
  ptx.DLC = 1;
  
  for(i=0;i<8;i++){
    canTransmit(dev,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
    chThdSleepMilliseconds(1000);
    ptx.data8[0]++;
  }
  canStop(dev);
  
}


//void sdo_callback(void *object)
//{
//  CO_SDOserver_t *sdo = (CO_SDOserver_t*)object;
//  
//}

void nmt_callback(CO_NMT_internalState_t state)
{
  if(runTime.appThread != NULL){
    chSysLock();
    //chEvtSignalI(runTime.appThread,EV_NMT_STATE_CHANGE);
    chSysUnlock();
  }
//  switch(nmt->operatingState){
//    
//  }
}

static ODR_t OD_Control_Input_6040(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten)
{
  if(stream == NULL || buf == NULL || countWritten == NULL){
    return ODR_DEV_INCOMPAT;
  }
  
  ODR_t odr = OD_writeOriginal(stream, buf, count, countWritten);
  
  if(odr == ODR_OK){  
    if(stream->subIndex == 0x01){
      uint16_t v = CO_getUint16(buf);
      chSysLock();
      if(v == 1){
        chEvtSignal(runTime.appThread,EV_PID_STOP);
      }
      else if(v == 2){
        chEvtSignal(runTime.appThread,EV_PID_START);
      }
      chSysUnlock();
    }
    else if(stream->subIndex == 0x02){
      uint16_t v = CO_getUint16(buf);
      uint16_t msk = v & 0x03;
      if(v == 0x01){
        runTime.stowCommand = ACT_UNLOCK;
        chSysLock();
        chEvtSignal(runTime.appThread,EV_AZ_STOW_LOCK);
        chSysUnlock();
      }
      else if(v == 0x02){
        runTime.stowCommand = ACT_LOCK;
        chSysLock();
        chEvtSignal(runTime.appThread,EV_AZ_STOW_LOCK);
        chSysUnlock();
      }
      msk = v & 0x0C;
      if(v == 0x04){
        runTime.stowCommand = ACT_UNLOCK;
        chSysLock();
        chEvtSignal(runTime.appThread,EV_EL_STOW_LOCK);
        chSysUnlock();
      }
      else if(v == 0x08){
        runTime.stowCommand = ACT_LOCK;
        chSysLock();
        chEvtSignal(runTime.appThread,EV_EL_STOW_LOCK);
        chSysUnlock();
      }
    }
    // subindex 0x03, log control
    else if(stream->subIndex == 0x03){
      uint16_t v = CO_getUint16(buf);
      if(v == 0x01){
        
        chSysLock();
        chEvtSignal(runTime.appThread,EV_LOCAL_LOG_START);
        chSysUnlock();
      }
      else if(v == 0x02){
        chSysLock();
        chEvtSignal(runTime.appThread,EV_LOCAL_LOG_STOP);
        chSysUnlock();
      }
    }
    else if(stream->subIndex == 0x04){ // mode
      uint16_t v = CO_getUint16(buf);
      if( v == 0 || v ==1){ // speed mode
        nested_pid_set_mode(AXIS_AZ, v);
        nested_pid_set_mode(AXIS_EL, v);
        //runTime.pidConfig[AXIS_AZ].speedMode = (v == 0)?true:false;
        //runTime.pidConfig[AXIS_EL].speedMode = (v == 0)?true:false;
      }
    }
  }
  return odr;
}


static ODR_t OD_Remote_SP_6063(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten)
{
  if(stream == NULL || buf == NULL || countWritten == NULL){
    return ODR_DEV_INCOMPAT;
  }
  
  ODR_t odr = OD_writeOriginal(stream, buf, count, countWritten);
  if(odr == ODR_OK){
    uint8_t mode_readback;
    float fv;
    OD_get_f32(OD_ENTRY_H6063,stream->subIndex,&fv,true);
    
    
    switch(stream->subIndex){
    case 1:
      nested_pid_set_pos_command(AXIS_AZ,fv);
      break;
    case 2:
//      if(runTime.pidAZ.stowState != STOW_UNKNOW){
        nested_pid_set_spd_command(AXIS_AZ,fv);
//      }
      break;
    case 3:
      nested_pid_set_pos_command(AXIS_EL,fv);
      break;
    case 4:
      nested_pid_set_spd_command(AXIS_EL,fv);
      break;
    case 5: // joystick az
      if(runTime.pidAZ.joystickMode){
        // put command to d/a port
        nested_pid_set_spd_command(AXIS_AZ,fv);
      }
      break;
    case 6: // joystick el
      if(runTime.pidEL.joystickMode){
        nested_pid_set_spd_command(AXIS_EL,fv);
      }
      break;
    default:break;
    }  
  }
  return odr;
}

static ODR_t OD_write_digital_6300(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten)
{
  if(stream == NULL || buf == NULL || countWritten == NULL){
    return ODR_DEV_INCOMPAT;
  }
  
  ODR_t odr = OD_writeOriginal(stream, buf, count, countWritten);
  if(odr == ODR_OK){
    if(stream->subIndex == 0x01){
      uint16_t v = CO_getUint16(buf);
      digital_set_iso_outw(0,v);
    }  
  }
  return odr;
}

static ODR_t OD_write_ao_6411_raw(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten)
{
  ODR_t odr = OD_writeOriginal(stream, buf, count, countWritten);
  if(odr == ODR_OK){
    if(stream == NULL || buf == NULL || countWritten == NULL){
      return ODR_DEV_INCOMPAT;
    }
    
    if((stream->subIndex > 0) && (stream->subIndex < 5)){
      uint16_t v = CO_getUint16(buf);
      analog_output_set_data(stream->subIndex-1,(int16_t*)&v);
    }  
  }
  return odr;
}

static ODR_t OD_write_ao_6413_real(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten)
{
  ODR_t odr = OD_writeOriginal(stream, buf, count, countWritten);
  if(odr == ODR_OK){
    if(stream == NULL || buf == NULL || countWritten == NULL){
      return ODR_DEV_INCOMPAT;
    }
    
    if((stream->subIndex > 0) && (stream->subIndex < 5)){
      float v;
      memmove(&v,buf,sizeof(float));
      analog_output_set_voltage(stream->subIndex-1,(float*)&v);
    }  
  }
  return odr;
}

//static bool sysValidStow(uint8_t action, _pid_config_t *config)
//{
//  // valid stow state
//  _stow_control_t stowCtrl;
//  stowCtrl.ctrl_map = &config->ctrl_map;
//  stowCtrl.stow_activity = action;
//  if(validStow(&stowCtrl)){
//    config->stowValid = true;
//    config->stowState = (action == ACT_LOCK)?STOW_LOCKED:STOW_UNLOCKED;
//  }
//  else{
//    config->stowValid = false;
//    config->stowState = STOW_UNKNOW;
//  }
//  
//  return (true);
//}

static void pid_run_ex(uint8_t axis, uint16_t mode)
{
  if(axis < NOF_AXES){
    load_pid_param_ex();
    uint16_t mode;
    OD_get_u16(OD_ENTRY_H6040,0x04,&mode,true);
    
    runTime.pidConfig[axis].speedMode = (mode==0)?true:false;
    nested_pid_init(axis,&runTime.pidConfig[axis]);
  }
}

static void pid_stop_ex(uint8_t axis)
{
  nested_pid_stop(axis);
}

static void pid_run(uint8_t axis, uint16_t mode)
{
  load_pid_param();
  _pid_config_t *cfg = (axis==0)?&runTime.pidAZ:&runTime.pidEL;
  tdmotc_SetPID(axis,TDMOTC_PID_P,TDMOTC_PID_ID_P,cfg->pid_cfg_pos.kp);
  tdmotc_SetPID(axis,TDMOTC_PID_P,TDMOTC_PID_ID_I,cfg->pid_cfg_pos.ki);
  tdmotc_SetPID(axis,TDMOTC_PID_P,TDMOTC_PID_ID_D,cfg->pid_cfg_pos.kd);
  
  tdmotc_Start(axis,mode);
  // set sp at first start
  float sp = resolver_get_position(axis);
  if(mode == TDMOTC_MODE_P){
    tpcmdh_SetPosCmd(axis,sp);
  }
  if(mode == TDMOTC_MODE_P2){
    tdmotc_SetPosCmd2(axis,sp);
  }
}

static void pid_stop(uint8_t axis)
{
  tdmotc_Stop(axis);
}

static void start_pid()
{
  uint16_t mode = 0;
  
  if(!runTime.pidAZ.stowValid) return;
  if(!runTime.pidEL.stowValid) return;
  
  OD_get_u16(OD_ENTRY_H6040,0x04,&mode,true);
  
  runTime.pidConfig[AXIS_AZ].speedMode = (mode==0)?true:false;
  runTime.pidConfig[AXIS_EL].speedMode = (mode==0)?true:false;
  
//  tpcmdh_taskInit(&runTime.pidAZ,&runTime.pidEL);
//  tdmotc_algorithm_task_init(&runTime.pidAZ,&runTime.pidEL,mode);
//  tdmotc_Start(AXIS_AZ, mode);
  nested_pid_init(AXIS_AZ, &runTime.pidConfig[0]);
}

static void stop_pid()
{
//  tdmotc_Stop(AXIS_AZ);
//  tdmotc_Stop(AXIS_EL);
//  tdmotc_algorithm_task_stop();
//  tpcmdh_taskStop();
  
  
  updateStatus(STA_SYS_PIDAZ_READY,0);
  updateStatus(STA_SYS_PIDEL_READY,0);
  
}

static void od_set_status(uint32_t status)
{
    OD_entry_t *od = OD_find(OD,0x6041); // status word
    if(od != NULL){
      OD_set_u32(od,0x0,status,true);
      OD_requestTPDO(runTime.cosFlags.diFlag,1);
    }
}

static void updateStatus(uint8_t mask, bool set)
{
    uint32_t sta = runTime.mcStatus;
    if(set){
      sta |= (1 << mask);
    }
    else{
      sta  &= ~(1 << mask);
    }
    if(sta != runTime.mcStatus){
      runTime.mcStatus = sta;
      od_set_status(runTime.mcStatus);
      OD_requestTPDO(runTime.cosFlags.statusFlag,1);
    }
  
}

static void logTimer(void *arg)
{
  chSysLockFromISR();
  chEvtSignalI(runTime.appThread, EV_LOCAL_LOG_EVT);
  if(runTime.localLog){
    chVTSetI(&runTime.vtLog,TIME_MS2I(runTime.logIntervalMs),logTimer,NULL);
  }
  chSysUnlockFromISR();
}

static void load_pid_param_dummy()
{
  _nested_pid_t *pid = &runTime.pidConfig[0];
  
  pid->spd.param.kp = 1.0;
  pid->spd.param.ki = 0;
  pid->spd.param.kd = 0;
  
  pid->spd.input_range.min = -10;
  pid->spd.input_range.max = 10; // rpm
  pid->spd.input_range.offset = 0.;
  pid->spd.output_clamp.min = -10;
  pid->spd.output_clamp.max = 10;
  pid->spd.output_clamp.offset = 0.;
    
  pid->spd.cal.pv = 0.;
  pid->spd.cal.sp = 0.;
  pid->spd.cal.out = 0.;
  pid->spd.cal.err = 0.;
  
  pid->spd.getPv = resolver_get_speed;
  pid->spd.getSp = NULL;
  pid->spd.clamp = NULL;
  pid->spd.driveOutput = 0; 
  pid->spd.update = NULL;

  pid->pos.param.kp = 1.0;
  pid->pos.param.ki = 0;
  pid->pos.param.kd = 0;
  
  pid->pos.input_range.min = -180;
  pid->pos.input_range.max = 180; 
  pid->pos.output_clamp.min = -10;
  pid->pos.output_clamp.max = 10;
    
  pid->pos.cal.pv = 0.;
  pid->pos.cal.sp = 0.;
  pid->pos.cal.out = 0.;
  pid->pos.cal.err = 0.;
  
  pid->pos.getPv = resolver_get_position;
  pid->pos.getSp = NULL;
  pid->pos.clamp = NULL;
  pid->pos.driveOutput = 0; 
  pid->pos.update = NULL;
  
  // control map
  pid->controlMap.axis_id = AXIS_AZ;
  pid->controlMap.servo_en[0] = 0;
  pid->controlMap.servo_on[0] = 1;
  pid->controlMap.servo_en[1] = 2;
  pid->controlMap.servo_on[1] = 3;
  pid->controlMap.servo_cmd[0] = 0; 
  pid->controlMap.servo_cmd[0] = 1;
  
  pid->controlMap.pv_pos = 0;
  pid->controlMap.pv_spd = 0;
  pid->controlMap.sp_pos = 0;
  pid->controlMap.sp_spd = 0;
  
  pid->cycleTimeUs = 5000;
  pid->stream = NULL;
  
}

static void load_pid_param_ex()
{
  _nested_pid_t *pid;
#ifdef OD_ENTRY_H60A0
  pid = &runTime.pidConfig[0];
  OD_get_f32(OD_ENTRY_H60A0,0x04,&pid->pos.param.kp,true);
  OD_get_f32(OD_ENTRY_H60A0,0x05,&pid->pos.param.ki,true);
  OD_get_f32(OD_ENTRY_H60A0,0x06,&pid->pos.param.kd,true);
  OD_get_f32(OD_ENTRY_H60A0,0x0e,&pid->pos.input_range.min,true);
  OD_get_f32(OD_ENTRY_H60A0,0x0f,&pid->pos.input_range.max,true);
  OD_get_f32(OD_ENTRY_H60A0,0x10,&pid->pos.input_range.offset,true);
  OD_get_f32(OD_ENTRY_H60A0,0x11,&pid->pos.output_clamp.min,true);
  OD_get_f32(OD_ENTRY_H60A0,0x12,&pid->pos.output_clamp.max,true);
  OD_get_f32(OD_ENTRY_H60A0,0x13,&pid->pos.output_clamp.offset,true);
  // convert degree setting to radian
  pid->pos.input_range.min *= DEG2RAD;
  pid->pos.input_range.max *= DEG2RAD;
  pid->pos.input_range.offset *= DEG2RAD;
  

  OD_get_f32(OD_ENTRY_H60A0,0x07,&pid->spd.param.kp,true);
  OD_get_f32(OD_ENTRY_H60A0,0x08,&pid->spd.param.ki,true);
  OD_get_f32(OD_ENTRY_H60A0,0x09,&pid->spd.param.kd,true);
  OD_get_f32(OD_ENTRY_H60A0,0x11,&pid->spd.input_range.min,true);
  OD_get_f32(OD_ENTRY_H60A0,0x12,&pid->spd.input_range.max,true);
  OD_get_f32(OD_ENTRY_H60A0,0x13,&pid->spd.input_range.offset,true);
  OD_get_f32(OD_ENTRY_H60A0,0x1,&pid->spd.output_clamp.min,true);
  OD_get_f32(OD_ENTRY_H60A0,0x2,&pid->spd.output_clamp.max,true);
  OD_get_f32(OD_ENTRY_H60A0,0x3,&pid->spd.output_clamp.offset,true);
    pid->pos.cal.pv = 0.;
    pid->pos.cal.sp = 0.;
    pid->pos.cal.out = 0.;
    pid->pos.cal.err = 0.;
    pid->pos.getPv = resolver_get_position;
    pid->pos.getSp = NULL;
    pid->pos.clamp = NULL;
    pid->pos.driveOutput = 0; 
    pid->pos.update = NULL;

    pid->spd.cal.pv = 0.;
    pid->spd.cal.sp = 0.;
    pid->spd.cal.out = 0.;
    pid->spd.cal.err = 0.;
    pid->spd.getPv = resolver_get_speed;
    pid->spd.getSp = NULL;
    pid->spd.clamp = NULL;
    pid->spd.driveOutput = 0; 
    pid->spd.update = NULL;
    
    
    // control map
    pid->controlMap.axis_id = AXIS_AZ;

#ifdef OD_ENTRY_H2005
    OD_get_u8(OD_ENTRY_H2005,0x01,&pid->controlMap.servo_en[0],true);
    OD_get_u8(OD_ENTRY_H2005,0x02,&pid->controlMap.servo_on[0],true);
    OD_get_u8(OD_ENTRY_H2005,0x03,&pid->controlMap.servo_en[1],true);
    OD_get_u8(OD_ENTRY_H2005,0x04,&pid->controlMap.servo_on[1],true);
    OD_get_u8(OD_ENTRY_H2005,0x10,&pid->controlMap.controlMode,true);
#endif
    
#ifdef OD_ENTRY_H2007
    OD_get_u8(OD_ENTRY_H2007,0x01,&pid->controlMap.servo_cmd[0],true);
    OD_get_u8(OD_ENTRY_H2007,0x02,&pid->controlMap.servo_cmd[1],true);
#endif    
    
    pid->controlMap.pv_pos = 0;
    pid->controlMap.pv_spd = 0;
    pid->controlMap.sp_pos = 0;
    pid->controlMap.sp_spd = 0;
    pid->cycleTimeUs = 5000;
    pid->stowValid = false;
    pid->stowState = STOW_UNKNOW;
    pid->mcStatus = 0xAA;
    pid->joystickMode = false;
    pid->stream = (BaseSequentialStream*)&CONSOLE;
    pid->axisId = AXIS_AZ;
    
#endif

  pid = &runTime.pidConfig[1];
#ifdef OD_ENTRY_H60A1
  OD_get_f32(OD_ENTRY_H60A1,0x04,&pid->pos.param.kp,true);
  OD_get_f32(OD_ENTRY_H60A1,0x05,&pid->pos.param.ki,true);
  OD_get_f32(OD_ENTRY_H60A1,0x06,&pid->pos.param.kd,true);
  OD_get_f32(OD_ENTRY_H60A1,0x0e,&pid->pos.input_range.min,true);
  OD_get_f32(OD_ENTRY_H60A1,0x0f,&pid->pos.input_range.max,true);
  OD_get_f32(OD_ENTRY_H60A1,0x10,&pid->pos.input_range.offset,true);
  OD_get_f32(OD_ENTRY_H60A1,0x11,&pid->pos.output_clamp.min,true);
  OD_get_f32(OD_ENTRY_H60A1,0x12,&pid->pos.output_clamp.max,true);
  OD_get_f32(OD_ENTRY_H60A1,0x13,&pid->pos.output_clamp.offset,true);
  // convert degree setting to radian
  pid->pos.input_range.min *= DEG2RAD;
  pid->pos.input_range.max *= DEG2RAD;
  pid->pos.input_range.offset *= DEG2RAD;
  

  OD_get_f32(OD_ENTRY_H60A1,0x07,&pid->spd.param.kp,true);
  OD_get_f32(OD_ENTRY_H60A1,0x08,&pid->spd.param.ki,true);
  OD_get_f32(OD_ENTRY_H60A1,0x09,&pid->spd.param.kd,true);
  OD_get_f32(OD_ENTRY_H60A1,0x11,&pid->spd.input_range.min,true);
  OD_get_f32(OD_ENTRY_H60A1,0x12,&pid->spd.input_range.max,true);
  OD_get_f32(OD_ENTRY_H60A1,0x13,&pid->spd.input_range.offset,true);
  OD_get_f32(OD_ENTRY_H60A1,0x1,&pid->spd.output_clamp.min,true);
  OD_get_f32(OD_ENTRY_H60A1,0x2,&pid->spd.output_clamp.max,true);
  OD_get_f32(OD_ENTRY_H60A1,0x3,&pid->spd.output_clamp.offset,true);
    pid->pos.cal.pv = 0.;
    pid->pos.cal.sp = 0.;
    pid->pos.cal.out = 0.;
    pid->pos.cal.err = 0.;
    pid->pos.getPv = resolver_get_position;
    pid->pos.getSp = NULL;
    pid->pos.clamp = NULL;
    pid->pos.driveOutput = 0; 
    pid->pos.update = NULL;

    pid->spd.cal.pv = 0.;
    pid->spd.cal.sp = 0.;
    pid->spd.cal.out = 0.;
    pid->spd.cal.err = 0.;
    pid->spd.getPv = resolver_get_speed;
    pid->spd.getSp = NULL;
    pid->spd.clamp = NULL;
    pid->spd.driveOutput = 0; 
    pid->spd.update = NULL;
    
    
    // control map
    pid->controlMap.axis_id = AXIS_EL;
    
#ifdef OD_ENTRY_H2005
    OD_get_u8(OD_ENTRY_H2005,0x05,&pid->controlMap.servo_en[0],true);
    OD_get_u8(OD_ENTRY_H2005,0x06,&pid->controlMap.servo_on[0],true);
    OD_get_u8(OD_ENTRY_H2005,0x07,&pid->controlMap.servo_en[1],true);
    OD_get_u8(OD_ENTRY_H2005,0x08,&pid->controlMap.servo_on[1],true);
    OD_get_u8(OD_ENTRY_H2005,0x11,&pid->controlMap.controlMode,true);
#endif
    
#ifdef OD_ENTRY_H2007
    OD_get_u8(OD_ENTRY_H2007,0x03,&pid->controlMap.servo_cmd[0],true);
    OD_get_u8(OD_ENTRY_H2007,0x04,&pid->controlMap.servo_cmd[1],true);
#endif    
    
    pid->controlMap.pv_pos = 0;
    pid->controlMap.pv_spd = 0;
    pid->controlMap.sp_pos = 0;
    pid->controlMap.sp_spd = 0;
    pid->cycleTimeUs = 5000;
    pid->stowValid = false;
    pid->stowState = STOW_UNKNOW;
    pid->mcStatus = 0xAA;
    pid->joystickMode = false;
    pid->stream = (BaseSequentialStream*)&CONSOLE;
    pid->axisId = AXIS_EL;

#endif

  
}

static void load_pid_param()
{
    float f32;
    // corse gain for DAC
    OD_entry_t *od = NULL;

    od = OD_find(OD,0x60a0); // azimuth 
    if(od != NULL){                
      OD_get_f32(od,0xa,&f32,true);
      runTime.pidAZ.tqbc_boundary.out_min_set = f32;
      runTime.pidAZ.tqbc_boundary.out_min_2set = f32;
      OD_get_f32(od,0xb,&f32,true);
      runTime.pidAZ.tqbc_boundary.out_max_set = f32;
      runTime.pidAZ.tqbc_boundary.out_max_2set = f32;

      OD_get_f32(od,0xc,&f32,true);
      runTime.pidAZ.tqbc_boundary.gain_set = f32;
      runTime.pidAZ.tqbc_boundary.gain_2set = f32;

      OD_get_f32(od,0xd,&f32,true);
      runTime.pidAZ.tqbc_boundary.zcp_set = f32;
      runTime.pidAZ.tqbc_boundary.zcp_2set = f32;

      OD_get_f32(od,0x1,&f32,true);
      runTime.pidAZ.torque_max = f32;
      OD_get_f32(od,0x3,&f32,true);
      runTime.pidAZ.rpm_max = f32;
      OD_get_f32(od,0xb,&f32,true);
      runTime.pidAZ.output_max = f32;

      OD_get_f32(od,0x4,&f32,true);
      runTime.pidAZ.pid_cfg_pos.kp = f32;
      OD_get_f32(od,0x5,&f32,true);
      runTime.pidAZ.pid_cfg_pos.ki = f32;
      OD_get_f32(od,0x6,&f32,true);
      runTime.pidAZ.pid_cfg_pos.kd = f32;
      OD_get_f32(od,0x7,&f32,true);
      runTime.pidAZ.pid_cfg_speed.kp = f32;
      OD_get_f32(od,0x8,&f32,true);
      runTime.pidAZ.pid_cfg_speed.ki = f32;
      OD_get_f32(od,0x9,&f32,true);
      runTime.pidAZ.pid_cfg_speed.kd = f32;
      OD_get_f32(od,0x0e,&f32,true);
      runTime.pidAZ.pos_min = f32;
      OD_get_f32(od,0x0f,&f32,true);
      runTime.pidAZ.pos_max = f32;
      OD_get_f32(od,0x10,&f32,true);
      runTime.pidAZ.home_offset = f32;
      runTime.pidAZ.axisId = AXIS_AZ;
    }
    

    od = OD_find(OD,0x60a1); // elevator 
    if(od != NULL){                
      OD_get_f32(od,0xa,&f32,true);
      runTime.pidEL.tqbc_boundary.out_min_set = f32;
      runTime.pidEL.tqbc_boundary.out_min_2set = f32;
      OD_get_f32(od,0xb,&f32,true);
      runTime.pidEL.tqbc_boundary.out_max_set = f32;
      runTime.pidEL.tqbc_boundary.out_max_2set = f32;

      OD_get_f32(od,0xc,&f32,true);
      runTime.pidEL.tqbc_boundary.gain_set = f32;
      runTime.pidEL.tqbc_boundary.gain_2set = f32;

      OD_get_f32(od,0xd,&f32,true);
      runTime.pidEL.tqbc_boundary.zcp_set = f32;
      runTime.pidEL.tqbc_boundary.zcp_2set = f32;

      OD_get_f32(od,0x4,&f32,true);
      runTime.pidEL.pid_cfg_pos.kp = f32;
      OD_get_f32(od,0x5,&f32,true);
      runTime.pidEL.pid_cfg_pos.ki = f32;
      OD_get_f32(od,0x6,&f32,true);
      runTime.pidEL.pid_cfg_pos.kd = f32;
      OD_get_f32(od,0x7,&f32,true);
      runTime.pidEL.pid_cfg_speed.kp = f32;
      OD_get_f32(od,0x8,&f32,true);
      runTime.pidEL.pid_cfg_speed.ki = f32;
      OD_get_f32(od,0x9,&f32,true);
      runTime.pidEL.pid_cfg_speed.kd = f32;
      OD_get_f32(od,0x0e,&f32,true);
      runTime.pidEL.pos_min = f32;
      OD_get_f32(od,0x0f,&f32,true);
      runTime.pidEL.pos_max = f32;
      OD_get_f32(od,0x10,&f32,true);
      runTime.pidEL.home_offset = f32;
      runTime.pidEL.axisId = AXIS_EL;
    }
}

void load_app_param()
{
  // load app parameters
//  if(storageInitError != 0){
//    // load program defaults
//    for(uint8_t i=0;i<AD57_CHANNELS;i++){
//      runTime.dacConfig.corse_gain[i] = 0x00;
//      runTime.dacConfig.fine_gain[i] = 0x00;
//      runTime.dacConfig.offset[i] = 0x00;
//    }
//    runTime.pidConfig.tqbc_boundary.out_max_2set = DMOTC_DFLT_OUT_MAX;
//    runTime.pidConfig.tqbc_boundary.out_min_2set = DMOTC_DFLT_OUT_MIN;
//    runTime.pidConfig.tqbc_boundary.gain_2set = DMOTC_DFLT_GAIN;
//    runTime.pidConfig.tqbc_boundary.zcp_2set = DMOTC_DFLT_ZCP;
//    runTime.pidConfig.tqbc_boundary.out_max_set = DMOTC_DFLT_OUT_MAX;
//    runTime.pidConfig.tqbc_boundary.out_min_set = DMOTC_DFLT_OUT_MIN;
//    runTime.pidConfig.tqbc_boundary.gain_set = DMOTC_DFLT_GAIN;
//    runTime.pidConfig.tqbc_boundary.zcp_set = DMOTC_DFLT_ZCP;
//    
//    runTime.pidConfig.dmotc_default.pos_s_min = DMOTC_DFLT_POS_S_MIN;
//    runTime.pidConfig.dmotc_default.pos_s_max = DMOTC_DFLT_POS_S_MAX;
//    runTime.pidConfig.dmotc_default.pos_kp = DMOTC_DFLT_P_PID_KP;
//    runTime.pidConfig.dmotc_default.pos_ki = DMOTC_DFLT_P_PID_KI;
//    runTime.pidConfig.dmotc_default.pos_kd = DMOTC_DFLT_P_PID_KD;
//    runTime.pidConfig.dmotc_default.spd_kp = DMOTC_DFLT_S_PID_KP;
//    runTime.pidConfig.dmotc_default.spd_ki = DMOTC_DFLT_S_PID_KI;
//    runTime.pidConfig.dmotc_default.spd_kd = DMOTC_DFLT_S_PID_KD;
////    runTime.pidConfig.dmotc_default.speed = ;
////    runTime.pidConfig.dmotc_default.pos2_thold = ;
//  }
//  /* load from storage */
//  else{
    uint16_t u16;
    float f32;
    // corse gain for DAC
    OD_entry_t *od = OD_find(OD,0x2100);
    if(od != NULL){                
      for(uint8_t i=0;i<AD57_CHANNELS;i++){
        OD_get_u16(od,i+1,&u16,true);
        runTime.dacConfig.corse_gain[i] = u16;
      }
    }
    // fine gain 
    od = OD_find(OD,0x2101);
    if(od != NULL){                
      for(uint8_t i=0;i<AD57_CHANNELS;i++){
        OD_get_u16(od,i+1,&u16,true);
        runTime.dacConfig.fine_gain[i] = u16;
      }
    }
    // offsset
    od = OD_find(OD,0x2102);
    if(od != NULL){                
      for(uint8_t i=0;i<AD57_CHANNELS;i++){
        OD_get_u16(od,i+1,&u16,true);
        runTime.dacConfig.offset[i] = u16;
      }
    }
    
//    od = OD_find(OD,0x2103);
//    if(od != NULL){                
//      OD_get_f32(od,1,&f32,true);
//      runTime.pidConfig.tqbc_boundary.out_max_2set = f32;
//      OD_get_f32(od,2,&f32,true);
//      runTime.pidConfig.tqbc_boundary.out_min_2set = f32;
//      OD_get_f32(od,3,&f32,true);
//      runTime.pidConfig.tqbc_boundary.gain_2set = f32;
//      OD_get_f32(od,4,&f32,true);
//      runTime.pidConfig.tqbc_boundary.zcp_2set = f32;
//      OD_get_f32(od,5,&f32,true);
//      runTime.pidConfig.tqbc_boundary.out_max_set = f32;
//      OD_get_f32(od,6,&f32,true);
//      runTime.pidConfig.tqbc_boundary.out_min_set = f32;
//      OD_get_f32(od,7,&f32,true);
//      runTime.pidConfig.tqbc_boundary.gain_set = f32;
//      OD_get_f32(od,8,&f32,true);
//      runTime.pidConfig.tqbc_boundary.zcp_set = f32;
//    }
//    
//    /* PID parameters */
//    od = OD_find(OD,0x2104);
//    if(od != NULL){                
//      OD_get_f32(od,1,&f32,true);
//      runTime.pidConfig.dmotc_default.pos_kp = f32;
//      OD_get_f32(od,2,&f32,true);
//      runTime.pidConfig.dmotc_default.pos_ki = f32;
//      OD_get_f32(od,3,&f32,true);
//      runTime.pidConfig.dmotc_default.pos_kd = f32;
//      OD_get_f32(od,4,&f32,true);
//      runTime.pidConfig.dmotc_default.spd_kp = f32;
//      OD_get_f32(od,5,&f32,true);
//      runTime.pidConfig.dmotc_default.spd_ki = f32;
//      OD_get_f32(od,6,&f32,true);
//      runTime.pidConfig.dmotc_default.spd_kd = f32;
//    }

    // digital i/o polarity
    od = OD_find(OD,0x6122);
    if(od != NULL){
      uint32_t u32;
      OD_get_u32(od,0,&u32,true);
      digital_set_iso_inpw(0, u32);
    }
    od = OD_find(OD,0x6302);
    if(od != NULL){
      OD_get_u16(od,0,&u16,true);
      digital_set_iso_outpw(0, u16);
    }
    
    // I/O Map
//    od = OD_find(OD,0x2005);
//    if(od != NULL){
//      uint8_t u8;
//      OD_get_u8(od,0x1,&u8,true);
//      runTime.pidAZ.ctrl_map.driver_en[0] = u8;
//
//      OD_get_u8(od,0x2,&u8,true);
//      runTime.pidAZ.ctrl_map.driver_on[0] = u8;
//
//      OD_get_u8(od,0x3,&u8,true);
//      runTime.pidAZ.ctrl_map.driver_en[1] = u8;
//
//      OD_get_u8(od,0x4,&u8,true);
//      runTime.pidAZ.ctrl_map.driver_on[1] = u8;
//
//      OD_get_u8(od,0x5,&u8,true);
//      runTime.pidEL.ctrl_map.driver_en[0] = u8;
//
//      OD_get_u8(od,0x6,&u8,true);
//      runTime.pidEL.ctrl_map.driver_on[0] = u8;
//
//      OD_get_u8(od,0x7,&u8,true);
//      runTime.pidEL.ctrl_map.driver_en[1] = u8;
//
//      OD_get_u8(od,0x8,&u8,true);
//      runTime.pidEL.ctrl_map.driver_on[1] = u8;
//      
//      OD_get_u8(od,0x9,&u8,true);
//      runTime.pidAZ.ctrl_map.stow_lock_out = u8;
//
//      OD_get_u8(od,0xa,&u8,true);
//      runTime.pidAZ.ctrl_map.stow_unlock_out = u8;
//
//      OD_get_u8(od,0xb,&u8,true);
//      runTime.pidEL.ctrl_map.stow_lock_out = u8;
//
//      OD_get_u8(od,0xc,&u8,true);
//      runTime.pidEL.ctrl_map.stow_unlock_out = u8;
//    }
    
    // input map
//    od = OD_find(OD,0x2006);
//    if(od != NULL){
//      uint8_t u8;
//      OD_get_u8(od,0x1,&u8,true);
//      runTime.pidAZ.ctrl_map.stow_lock_in = u8;
//      OD_get_u8(od,0x2,&u8,true);
//      runTime.pidAZ.ctrl_map.stow_unlock_in = u8;
//      
//      OD_get_u8(od,0x3,&u8,true);
//      runTime.pidEL.ctrl_map.stow_lock_in = u8;
//      OD_get_u8(od,0x4,&u8,true);
//      runTime.pidEL.ctrl_map.stow_unlock_in = u8;
//
//      OD_get_u8(od,0x5,&u8,true);
//      runTime.pidAZ.ctrl_map.driver_rdy[0] = u8;
//      OD_get_u8(od,0x6,&u8,true);
//      runTime.pidAZ.ctrl_map.driver_rdy[1] = u8;
//
//      OD_get_u8(od,0x7,&u8,true);
//      runTime.pidEL.ctrl_map.driver_rdy[0] = u8;
//      OD_get_u8(od,0x8,&u8,true);
//      runTime.pidEL.ctrl_map.driver_rdy[1] = u8;
//    }
    
//    od = OD_find(OD,0x2007);
//    if(od != NULL){
//      uint8_t u8;
//      OD_get_u8(od,0x1,&u8,true);
//      runTime.pidAZ.ctrl_map.driver_output[0] = u8;
//      OD_get_u8(od,0x2,&u8,true);
//      runTime.pidAZ.ctrl_map.driver_output[1] = u8;
//
//      OD_get_u8(od,0x3,&u8,true);
//      runTime.pidEL.ctrl_map.driver_output[0] = u8;
//      OD_get_u8(od,0x4,&u8,true);
//      runTime.pidEL.ctrl_map.driver_output[1] = u8;
//
//      OD_get_u8(od,0x5,&u8,true);
//      runTime.pidAZ.ctrl_map.resolver = u8;
//      OD_get_u8(od,0x6,&u8,true);
//      runTime.pidEL.ctrl_map.resolver = u8;
//    }
    
    
    load_pid_param_ex();
//    load_pid_param_dummy();
    // set parameter to initial state
    
    
//    runTime.pidAZ.stowValid = false;
//    runTime.pidAZ.stowState = STOW_UNKNOW;
//    runTime.pidEL.stowValid = false;
//    runTime.pidEL.stowState = STOW_UNKNOW;
//    runTime.mcStatus = 0xAA; // raise all stow status indicate not initialized
//    
//    runTime.pidAZ.joystickMode = false;
//    runTime.pidEL.joystickMode = false;
//
    // for test purpose, remove if done
    runTime.pidConfig[0].stowValid = true;
    runTime.pidConfig[0].stowState = STOW_UNLOCKED;
    runTime.pidConfig[1].stowValid = true;
    runTime.pidConfig[1].stowState = STOW_UNLOCKED;
//    runTime.pidAZ.stowValid = true;
//    runTime.pidAZ.stowState = STOW_UNLOCKED;
//    runTime.pidEL.stowValid = true;
//    runTime.pidEL.stowState = STOW_UNLOCKED;
    
    runTime.pidAutoStart = true;

}

static THD_WORKING_AREA(waApp, 512);
static THD_FUNCTION(procApp,p)
{
  chRegSetThreadName("MAIN");

  load_app_param();
  /* start peripheral task */
  analog_input_task_init();
  analog_output_task_init(&runTime.dacConfig);
  resolver_task_init();
  digital_init();
  /* start PID task */
  //tpcmdh_taskInit();
  //tdmotc_algorithm_task_init(&runTime.pidAZ);
  
  // extension read/write
  OD_extension_t extension;
  extension.object = NULL;
  extension.read = OD_readOriginal;
  extension.write = OD_write_digital_6300;
  OD_extension_init(OD_ENTRY_H6300, &extension);
  
  OD_extension_t ext_ao;
  ext_ao.object = NULL;
  ext_ao.read = OD_readOriginal;
  ext_ao.write = OD_write_ao_6411_raw;
  OD_extension_init(OD_ENTRY_H6411, &ext_ao);

  OD_extension_t ext_ao_real;
  ext_ao_real.object = NULL;
  ext_ao_real.read = OD_readOriginal;
  ext_ao_real.write = OD_write_ao_6413_real;
  OD_extension_init(OD_ENTRY_H6413, &ext_ao_real);

  OD_extension_t ext_ctrl;
  ext_ctrl.object = NULL;
  ext_ctrl.read = OD_readOriginal;
  ext_ctrl.write = OD_Control_Input_6040;
  OD_extension_init(OD_ENTRY_H6040, &ext_ctrl);

  OD_extension_t ext_sp;
  ext_sp.object = NULL;
  ext_sp.read = OD_readOriginal;
  ext_sp.write = OD_Remote_SP_6063;
  OD_extension_init(OD_ENTRY_H6063, &ext_sp);

  bool _stop = false;
  msg_t reason = MSG_OK;
  OD_entry_t *od;
  
  //runTime.stowControl.stowValidState = 0x0;
  
  if(runTime.pidAutoStart){
    start_pid();
  }
  
  chVTObjectInit(&runTime.vtLog);
  od_set_status(runTime.mcStatus);
  OD_requestTPDO(runTime.cosFlags.statusFlag,1);
  
  while(!_stop)
  {
    if(chThdShouldTerminateX()){
      _stop = true;
    }
    
    // update OD
    uint8_t sta = stow_state();
    if(sta != 0){
      //uint8_t state = stow_state();
      if(runTime.stowControl.axisId == AXIS_AZ){
        if(sta < 4){
          updateStatus(STA_MC_AZ_STOW_LOCK_IN_PROGRESS,1);
        }
        else{
          updateStatus(STA_MC_AZ_STOW_UNLOCK_IN_PROGRESS,1);
        }
      }
      else if(runTime.stowControl.axisId == AXIS_EL){
        if(sta < 4){
          updateStatus(STA_MC_EL_STOW_LOCK_IN_PROGRESS,1);
        }
        else{
          updateStatus(STA_MC_EL_STOW_UNLOCK_IN_PROGRESS,1);
        }
      }
    }
    
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));
    if(evt & EV_ADC_ACQUIRED){
      // update adc data 
      int32_t buffer[AD76_NOF_CHANNEL];
      analog_input_read(0xff,buffer);
      od = OD_find(OD,0x6401);
      int16_t i16;
      for(uint8_t i=0;i<AD76_NOF_CHANNEL;i++){
        i16 = (int16_t)buffer[i];
        OD_set_i16(od,i+1,i16,true);
      }      
      od = OD_find(OD,0x6403);
      float bufferf[AD76_NOF_CHANNEL];
      analog_input_readf(0xff,bufferf);
      for(uint8_t i=0;i<AD76_NOF_CHANNEL;i++){
        float f32 = buffer[i];
        OD_set_f32(od,i+1,f32,true);
      }      
    }
    if(evt & EV_RESOLVER_ACQUIRED){
      float fv;
      od = OD_find(OD,0x6064);
      fv = (resolver_get_position(0));
      if(fv < 0) fv += PIMUL2;
      fv *= RAD2DEG;
      OD_set_f32(od,1,fv,true);
      fv = resolver_get_speed(0);
      OD_set_f32(od,2,fv,true);
      
      fv = resolver_get_position(1)*RAD2DEG;
      OD_set_f32(od,3,fv,true);
      fv = resolver_get_speed(1);
      OD_set_f32(od,4,fv,true);
      
      OD_requestTPDO(runTime.cosFlags.pvFlag,1);
      OD_requestTPDO(runTime.cosFlags.pvFlag,2);
      OD_requestTPDO(runTime.cosFlags.pvFlag,3);
      OD_requestTPDO(runTime.cosFlags.pvFlag,4);

    }
    if(evt & EV_PID_UPDATEDA){
      
    }
    if(evt & EV_PID_START){
      //start_pid();
      uint16_t mode;
      OD_get_u16(OD_ENTRY_H6040,0x04,&mode,true);
      pid_run_ex(0,mode);
//      pid_run(1,mode);
      
    }
    if(evt & EV_PID_STOP){
      //stop_pid();
      pid_stop_ex(0);
      //pid_stop(1);
    }
    
    if(evt & EV_PID_AZ_STARTED){
      updateStatus(STA_SYS_PIDAZ_READY,1);
    }
    if(evt & EV_PID_AZ_STOPPED){
      updateStatus(STA_SYS_PIDAZ_READY,0);
    }
    if(evt & EV_PID_EL_STARTED){
      updateStatus(STA_SYS_PIDEL_READY,1);
    }
    if(evt & EV_PID_EL_STOPPED){
      updateStatus(STA_SYS_PIDEL_READY,0);
    }

    if(evt & EV_DIN_COS){
      // update digital input periodically
      od = OD_find(OD,0x6100);
      uint16_t u16;
      digital_get_iso_inw(0,&u16);
      OD_set_u16(od,1,u16,true);
      OD_requestTPDO(runTime.cosFlags.diFlag,1);
    }
    
    if(evt & EV_AZ_STOW_LOCK){
      runTime.stowControl.axisId = runTime.pidAZ.axisId;
      validStow(runTime.stowCommand,&runTime.stowControl);
    }
       
    if(evt & EV_EL_STOW_LOCK){
      runTime.stowControl.axisId = runTime.pidEL.axisId;
      validStow(runTime.stowCommand,&runTime.stowControl);
    }
    
    if(evt & EV_STOW_ACTION_DONE){
      //sysValidStow(runTime.stowCommand,&runTime.pidAZ);
      runTime.stowValidState = 0;
      if(runTime.stowControl.axisId == AXIS_AZ){
        if(runTime.stowControl.stowStatus == STA_MC_AZ_STOW_LOCKD){
          runTime.pidAZ.stowState = STOW_LOCKED;
        }
        else if(runTime.stowControl.stowStatus == STA_MC_AZ_STOW_UNLOCKD){
          runTime.pidAZ.stowState = STOW_UNLOCKED;
        }
        else{
          runTime.pidAZ.stowState = STOW_UNKNOW;
        }        
        updateStatus(STA_MC_STOW_AZ_ERROR,0);
      }
      else if(runTime.stowControl.axisId == AXIS_EL){
        if(runTime.stowControl.stowStatus == STA_MC_EL_STOW_LOCKD){
          runTime.pidEL.stowState = STOW_LOCKED;
        }
        else if(runTime.stowControl.stowStatus == STA_MC_EL_STOW_UNLOCKD){
          runTime.pidEL.stowState = STOW_UNLOCKED;
        }
        else{
          runTime.pidEL.stowState = STOW_UNKNOW;
        }        
        updateStatus(STA_MC_STOW_EL_ERROR,0);
      }
    }
       
    if(evt & EV_STOW_ACTION_FAIL){
      //sysValidStow(runTime.stowCommand,&runTime.pidEL);
      runTime.stowValidState = 0;
      if(runTime.stowControl.axisId == AXIS_AZ){
        updateStatus(STA_MC_STOW_AZ_ERROR,1);
      }
      else if(runTime.stowControl.axisId == AXIS_EL){
        updateStatus(STA_MC_STOW_EL_ERROR,1);
      }
    }
    
    if(evt & EV_PID_AZ_RDY){
      updateStatus(STA_SYS_PIDAZ_READY,1);
    }
    if(evt & EV_PID_EL_RDY){
      updateStatus(STA_SYS_PIDEL_READY,1);
    }

    if(evt & EV_LOCAL_LOG_START){
      
      runTime.localLog = true;
      chVTSet(&runTime.vtLog,TIME_MS2I(runTime.logIntervalMs),logTimer,NULL);
    }

    if(evt & EV_LOCAL_LOG_STOP){
      runTime.localLog = false;
    }
    if(evt & EV_LOCAL_LOG_EVT){
      //sys_log();
      sys_logEx(&runTime.pidConfig[0]);
      sys_logEx(&runTime.pidConfig[1]);
    }
    

  }
  
    /* stop pid first */
    stop_pid();
  /* stop peripheral */
  analog_input_task_stop();
  analog_output_task_stop();
  resolver_task_stop();
  chThdExit(reason);
}

static void start_app()
{
  if(runTime.appThread == NULL){
    runTime.appThread = chThdCreateStatic(waApp,sizeof(waApp),NORMALPRIO,procApp,NULL);
    runTime.appStarted = true;
  }
}

static void stop_app()
{
  if(runTime.appThread != NULL){
    chThdTerminate(runTime.appThread);
    msg_t ret = chThdWait(runTime.appThread);
    
    runTime.appThread = NULL;
  }
}

static void co_periodic_poll(void *arg)
{
  chSysLockFromISR();
  if(runTime.nmt_reset_cmd == CO_RESET_NOT){
  chVTSetI(&runTime.vt_periodic,TIME_MS2I(1),co_periodic_poll,NULL);
  uint32_t timeDifference_us = 1000;
    if(!runTime.CO->nodeIdUnconfigured && runTime.CO->CANmodule->CANnormal){
      bool syncWas = false;
#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
            syncWas = CO_process_SYNC(runTime.CO, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
            CO_process_RPDO(runTime.CO, syncWas, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
            CO_process_TPDO(runTime.CO, syncWas, timeDifference_us, NULL);
#endif
      
    }  
  }
  chSysUnlockFromISR();
}

static void registerFlagsPDO()
{
  OD_extension_init(OD_ENTRY_H6100,&OD_extension[0]);
  OD_extension_init(OD_ENTRY_H6300,&OD_extension[1]);
  OD_extension_init(OD_ENTRY_H6064,&OD_extension[2]);
  OD_extension_init(OD_ENTRY_H6041,&OD_extension[3]);
  
  runTime.cosFlags.diFlag = OD_getFlagsPDO(OD_ENTRY_H6100); // 16-bit
  runTime.cosFlags.doFlag = OD_getFlagsPDO(OD_ENTRY_H6300); // 16-bit
  //runTime.cosFlags.spFlag = OD_getFlagsPDO(OD_ENTRY_H6063); // sp variables 
  runTime.cosFlags.pvFlag = OD_getFlagsPDO(OD_ENTRY_H6064); // pv variable
  runTime.cosFlags.statusFlag = OD_getFlagsPDO(OD_ENTRY_H6041);
}

static void load_canopen_params()
{
  uint8_t tmp = 0x0;
  runTime.appAutoStart = true;
  runTime.pidAutoStart = false;
  runTime.appStarted = false;

  OD_PERSIST_COMM.x1018_identity.vendor_ID = VENDER_ID;
  OD_PERSIST_COMM.x1018_identity.productCode = PROEUCT_CODE;
  OD_PERSIST_COMM.x1018_identity.revisionNumber = REVISION_NUMBER;
  OD_PERSIST_COMM.x1018_identity.serialNumber = SERIAL_NUMVER;

  OD_entry_t *od = OD_find(OD,0x2010);
  runTime.nodeId = 0x01;
  if(od != NULL){                
    OD_get_u8(od,0x0,&runTime.nodeId,true);
  }
  if(runTime.nodeId == 0x00)
    runTime.nodeId = 0x01;
  runTime.CO->nodeIdUnconfigured = 0;
 // read initial settings
  od = OD_find(OD,0x6000); 
  if(od != NULL){
    uint32_t u32;
    OD_get_u32(od,0x1,&u32,true);
    if(u32 == 1){
      runTime.pidAutoStart = true;
    }
    else{
      runTime.pidAutoStart = false;
    }
    
    OD_get_u32(od,0x2,&u32,true);
    if((u32 >= 1) && (u32 <=10000)){
      runTime.logIntervalMs = u32;
    }
  }
}

static msg_t reset_canopen(void *p)
{
  uint32_t errInfo = 0;
   load_canopen_params();
  // initialize new instance of CANOpen */
  pCORunTime = &runTime;
  if(runTime.CO != NULL){
    CO_delete(runTime.CO);
  }
  if((runTime.CO = CO_new(NULL, &runTime.co_heap_usage)) == NULL){
    return MSG_RESET;
  }
  runTime.CO->CANmodule->CANptr = p;
  
  uint16_t pendingBitRate = 125;
  
  
  // storage
#if(CO_CONFIGURE_STORAGE_ENABLE) & CO_CONFIG_STORAGE_ENABLE
  uint8_t storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);
  uint32_t storageInitError = 0;
  
  CO_storageEeprom_init(&runTime.storage, runTime.CO->CANmodule, &storageModule,OD_ENTRY_H1010_storeParameters,OD_ENTRY_H1011_restoreDefaultParameters,
                        storageEntries,storageEntriesCount, &storageInitError);
  
#endif
  CO_ReturnError_t err;
  runTime.CO->CANmodule->CANnormal = false;
  CO_CANsetConfigurationMode(runTime.CO->CANmodule->CANptr);
  CO_CANmodule_disable(runTime.CO->CANmodule);
  
  /* initialize CANOpen */
  if((err = CO_CANinit(runTime.CO, runTime.CO->CANmodule->CANptr, pendingBitRate)) != CO_ERROR_NO){
    // handle init error 
  }
  
  /* setup LSS( layer-setting-service) block */
  
  if((err = CO_LSSinit(runTime.CO, &lssAddress,&runTime.nodeId, &pendingBitRate)) != CO_ERROR_NO){
    
  }
  
  /* initialize core stack */
  err = CO_CANopenInit(runTime.CO,
                       NULL,
                       NULL,
                       OD,
                       OD_STATUS_BITS,
                       NMT_CONTROL,
                       FIRST_HB_TIME,
                       SDO_SRV_TIMEOUT_TIME,
                       SDO_CLI_TIMEOUT_TIME,
                       SDO_CLI_BLOCK,
                       runTime.nodeId,
                       &errInfo);
  if(err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS){
    if(err == CO_ERROR_OD_PARAMETERS){
      
    }
    
  }
  
  
  registerFlagsPDO();

  /* initialize PDO */
  err = CO_CANopenInitPDO(runTime.CO, runTime.CO->em, OD, runTime.nodeId, &errInfo);
  if(err != CO_ERROR_NO){
    if( err == CO_ERROR_OD_PARAMETERS){
      
    }
    else{
      
    }
  }
  
    /* configure CANopen callbacks */
  
  //CO_SDOserver_initCallbackPre(runTime.CO->SDOserver,runTime.CO->SDOserver,sdo_callback);
  // custom read/write function
  //CO_NMT_initCallbackChanged(runTime.CO->NMT,nmt_callback);
  
//  OD_extension_t extension;
//  extension.read = OD_read_2100;
//  extension.write = OD_write_2100;
  
//  for(uint16_t odv = 0x2100; odv < 0x2120; odv++){
//    OD_entry_t *odt = OD_find(OD,odv);
//    if(odt != NULL){
//      OD_extension_t *ext = calloc(1, sizeof(OD_extension_t));
//      if(ext != NULL){
//        ext->object = odt;
//        ext->read = OD_read_2100;
//        ext->write = OD_write_2100;
//        OD_extension_init(odt, ext);
//      }
//                                 
//    }
//    
//  }
                               
#if (CO_CONFIG_STORAGE_ENABLE) & CO_CONFIGURE_STORAGE_ENABLE
    if(storageInitError != 0){
      CO_errorReport(runTime.CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE,storageInitError);
      //runTime.appAutoStart = false;
    }
#endif
//  if(!runTime.CO->nodeIdUnconfigured){
//  }
//  else{
//    
//  }
  
  /* start CAN to receive message */
  CO_CANsetNormalMode(runTime.CO->CANmodule);
  
  runTime.nmt_reset_cmd = CO_RESET_NOT;

  
  return MSG_OK;
}

static THD_WORKING_AREA(waCO,1024);
static THD_FUNCTION(procCO,p)
{
  uint32_t max_sleep_time_us;
  uint32_t timeDifference_us;
  uint32_t time_old, time_current;
  bool stop = false;
  

  
  time_old = time_current = chVTGetSystemTime();
  max_sleep_time_us = 0;
  systime_t time = chVTGetSystemTimeX();
  
  
  //startApp();
  runTime.co_opState = 0xff;
  runTime.appStarted = false;
  
  runTime.nmt_reset_cmd = CO_RESET_COMM;
  runTime.localLog = false;
  runTime.logIntervalMs = 100;
  //chVTObjectInit(&runTime.vt_periodic);
  //chVTSet(&runTime.vt_periodic,TIME_MS2I(1),co_periodic_poll,NULL);
  
  bool appParamLoaded = false;
  while(!stop){
    
    if(runTime.nmt_reset_cmd == CO_RESET_COMM){
      reset_canopen(p);
      runTime.nmt_reset_cmd = CO_RESET_NOT;
    }
    
    
    
    time_current = chVTGetSystemTime();
    timeDifference_us = TIME_I2US(chVTTimeElapsedSinceX(time_old));
    time_old = time_current;
    // update error register
    
    if(runTime.nmt_reset_cmd == CO_RESET_NOT){
//      if(!appParamLoaded){
//        load_app_param();
//      }
      runTime.nmt_reset_cmd = CO_process(runTime.CO, false, timeDifference_us, &max_sleep_time_us);
      CO_LOCK_OD(runTime.CO);
      if(!runTime.CO->nodeIdUnconfigured && runTime.CO->CANmodule->CANnormal){
        bool syncWas = false;
#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
        syncWas = CO_process_SYNC(runTime.CO, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
        CO_process_RPDO(runTime.CO, syncWas, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
        CO_process_TPDO(runTime.CO, syncWas, timeDifference_us, NULL);
#endif
      CO_UNLOCK_OD(runTime.CO);
    }  
      
      if(runTime.appAutoStart){
        if(!runTime.appStarted){
          start_app();
        }
      }
    }
    
    if(runTime.nmt_reset_cmd == CO_RESET_COMM){
      runTime.nmt_reset_cmd = CO_RESET_NOT;
    }

    if(runTime.nmt_reset_cmd == CO_RESET_APP){
      stop_app();
      runTime.nmt_reset_cmd = CO_RESET_NOT;
    }
    
//    chThdSleepMicroseconds(max_sleep_time_us);
    chThdSleepMicroseconds(500);
    
  }
  
}


typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
} CO_CANrxMsg_t;

static THD_WORKING_AREA(waCANRX1,512);
//static THD_WORKING_AREA(waCANRX2,512);
static THD_FUNCTION(thCanRx,p)
{
  CANDriver *ip = (CANDriver*)p;
  // register event listener
  event_listener_t el;
  chEvtRegister(&(ip)->rxfull_event,&el, 1);
    
  CANRxFrame rxmsg;
//  CO_CANrxMsg_t rcvMsg;
  CO_CANrx_t *buffer = NULL;
  while(true){
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    
    if(evt & EVENT_MASK(1)){
      uint16_t index;
      uint32_t rcvMsgIdent;
      uint8_t messageFound = 0;
      buffer = NULL;
      //chSysLock();
      while(canReceive(ip,CAN_ANY_MAILBOX,&rxmsg,TIME_IMMEDIATE) == MSG_OK){
        
        rcvMsgIdent = ((rxmsg.RTR == CAN_RTR_DATA)?0:FLAG_RTR);
        rcvMsgIdent |= rxmsg.SID;
        if(runTime.CO->CANmodule->useCANrxFilters){
          
        }
        else{
          buffer = runTime.CO->CANmodule->rxArray;
          for(index = runTime.CO->CANmodule->rxSize; index > 0U; --index, ++buffer){
            if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U){
//              if(buffer!=NULL && buffer->CANrx_callback != NULL){
//                buffer->CANrx_callback(buffer->object, (void*)&rxmsg);
//              }
              messageFound = 1;
              break;
            }
          }
        }
        if(messageFound && buffer!=NULL && buffer->CANrx_callback != NULL){
          buffer->CANrx_callback(buffer->object, (void*)&rxmsg);
          //chThdYield();
        }
      }
      //chSysUnlock();
    }
    
  }
  
}

void task_canopen_init()
{
  sdStart(&CONSOLE,&serialCfg);
  //chprintf(&CONSOLE,"Hello\n");
  chThdCreateStatic(waShell, sizeof(waShell), NORMALPRIO, shellThread, (void *)&shell_cfg);

  pCORunTime = &runTime;
  chThdCreateStatic(waCO,sizeof(waCO),NORMALPRIO,procCO,&CAND1);
  chThdCreateStatic(waCANRX1,sizeof(waCANRX1),NORMALPRIO-1,thCanRx,&CAND1);
}

static void sys_logEx(_nested_pid_t *pid)
{
  if(pid->stream  == NULL) return;

  char msg[128];
  int32_t adv[8];
  char *ptr = msg;
  systime_t now = chVTGetSystemTime();
  ptr += chsnprintf(ptr,128,"%ul,", TIME_I2MS(now));
  ptr += chsnprintf(ptr,128,"AXIS:%s,", pid->controlMap.axis_id == AXIS_AZ?"AZ\0":"EL\0");
  ptr += chsnprintf(ptr,128,"%ul,",TIME_I2MS(chVTGetSystemTime()));
  ptr += chsnprintf(ptr,128,"%5.2f,%5.2f,",pid->pos.cal.pv,pid->pos.cal.sp);
  ptr += chsnprintf(ptr,128,"%5.2f,%5.2f,",pid->spd.cal.pv, pid->spd.cal.sp);
  analog_input_read(0xff,adv);
  ptr += chsnprintf(ptr,128,"%d,%d,%d,%d,",adv[0],adv[1],adv[2],adv[3]);
  uint16_t dav[4];
  analot_output_get_raw(0xff,dav);
  ptr += chsnprintf(ptr,128,"%d,%d,%d,%d,",dav[0],dav[1],dav[2],dav[3]);
  uint16_t iso_in, iso_out;
  digital_get_iso_inw(0,&iso_in);
  digital_get_iso_outw(0,&iso_out);
  ptr += chsnprintf(ptr,128,"%04x,%04x,",iso_in,iso_out);
  ptr += chsnprintf(ptr,128,"\r\n");
  chprintf((BaseSequentialStream*)&CONSOLE,msg);
  
}


static void sys_log()
{
  char msg[128];
  int32_t adv[8];
  char *ptr = msg;
  ptr += chsnprintf(ptr,128,"%ul,",TIME_I2MS(chVTGetSystemTime()));
  ptr += chsnprintf(ptr,128,"%5.2f,%5.2f,",tdmotc_GetPosCmd(AXIS_AZ),tdmotc_GetPosCmd(AXIS_EL));
  ptr += chsnprintf(ptr,128,"%5.2f,%5.2f,",resolver_get_position(AXIS_AZ),resolver_get_position(AXIS_EL));
  ptr += chsnprintf(ptr,128,"%5.2f,%5.2f,",resolver_get_speed(AXIS_AZ),resolver_get_speed(AXIS_EL));
  analog_input_read(0xff,adv);
  ptr += chsnprintf(ptr,128,"%d,%d,%d,%d,",adv[0],adv[1],adv[2],adv[3]);
  uint16_t dav[4];
  analot_output_get_raw(0xff,dav);
  ptr += chsnprintf(ptr,128,"%d,%d,%d,%d,",dav[0],dav[1],dav[2],dav[3]);
  uint16_t iso_in, iso_out;
  digital_get_iso_inw(0,&iso_in);
  digital_get_iso_outw(0,&iso_out);
  ptr += chsnprintf(ptr,128,"%04x,%04x,",iso_in,iso_out);
  ptr += chsnprintf(ptr,128,"\r\n");
  chprintf((BaseSequentialStream*)&CONSOLE,msg);
}

static void cmd_report(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  chprintf(chp,"HYMCW7\r\n");
}

/*
  usage log start/stop [interval_ms]

*/
static void cmd_log(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 1){
    if(strncmp(argv[1],"start",5) == 0){
      chEvtSignal(runTime.appThread,EV_LOCAL_LOG_START);
    }
    else if(strncmp(argv[1],"stop",4) == 0){
      chEvtSignal(runTime.appThread,EV_LOCAL_LOG_STOP);
    }
  }
  else if(argc == 2){
    int ms = strtol(argv[2],NULL,10);
    if((ms >=5) && (ms <=1000)){
      runTime.logIntervalMs = ms;
    }
    if(strncmp(argv[1],"start",5) == 0){
      chEvtSignal(runTime.appThread,EV_LOCAL_LOG_START);
    }
    else if(strncmp(argv[1],"stop",5) == 0){
      chEvtSignal(runTime.appThread,EV_LOCAL_LOG_STOP);
    }
  }
}