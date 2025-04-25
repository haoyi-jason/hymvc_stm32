
#include "ch.h"
#include "hal.h"

#include <stdlib.h>

#include "task_can1.h"
#include "task_analog_input.h"
#include "task_dac.h"
#include "task_resolver.h"
#include "app_config.h"
#include "ad76_drv.h"
#include "ad57_drv.h"
#include "digital_io.h"
#include "pid/nested_pid.h"
#include "pid/stow_control.h"

#include "ylib/utility/st_chipid.h"
#include "database.h"


typedef struct{
  thread_t *appThread;
  thread_reference_t ref;
  systime_t runtime;
  uint32_t co_heap_usage;
  virtual_timer_t vt_periodic;
  bool appStarted;
  uint8_t co_opState;
  _pid_config_t pidAZ;
  _pid_config_t pidEL;
  _dac_config dacConfig;
  uint8_t nodeId;
  bool appAutoStart;
  bool pidAutoStart;
  uint32_t mcStatus,mcStatus_new;
  uint16_t stowCommand;
  uint8_t stowValidState;
  bool localLog;
  uint32_t logIntervalMs;
  virtual_timer_t vtLog;
  virtual_timer_t vtUpdate;
  virtual_timer_t vtRequestTPDO;
  _nested_pid_t pidConfig[NOF_AXES];
  _stow_config_t stowConfig[NOF_AXES];
  bool stow_control_enable;
  uint8_t activeStowAxis;
  systime_t tpdoUpdateInterval;
}_runTime_t;

static _runTime_t runTime;


void load_pid_param_ex()
{
  // pid parameters
  _nested_pid_t *pid;
  _stow_config_t *stow;
  
  pid = &runTime.pidConfig[AXIS_AZ];
  stow = &runTime.stowConfig[AXIS_AZ];
  db_read_param(DATA_SECTION_PID_PARAM,PID_PARAM_AZ_POS_KP,(void*)&pid->pos.param.kp);
  db_read_param(DATA_SECTION_PID_PARAM,PID_PARAM_AZ_POS_KI,(void*)&pid->pos.param.ki);
  db_read_param(DATA_SECTION_PID_PARAM,PID_PARAM_AZ_POS_KD,(void*)&pid->pos.param.kd);
  db_read_param(DATA_SECTION_PID_PARAM,PID_PARAM_AZ_SPD_KP,(void*)&pid->spd.param.kp);
  db_read_param(DATA_SECTION_PID_PARAM,PID_PARAM_AZ_SPD_KI,(void*)&pid->spd.param.ki);
  db_read_param(DATA_SECTION_PID_PARAM,PID_PARAM_AZ_SPD_KD,(void*)&pid->spd.param.kd);
  
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_AZ_POS_INPUT_MIN,(void*)&pid->pos.input_range.min);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_AZ_POS_INPUT_MAX,(void*)&pid->pos.input_range.max);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_AZ_POS_INPUT_OFFSET,(void*)&pid->pos.input_range.offset);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_AZ_SPD_INPUT_MIN,(void*)&pid->spd.input_range.min);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_AZ_SPD_INPUT_MAX,(void*)&pid->spd.input_range.max);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_AZ_SPD_INPUT_OFFSET,(void*)&pid->spd.input_range.offset);
  
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_AZ_POS_OUTPUT_MIN,(void*)&pid->pos.output_clamp.min);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_AZ_POS_OUTPUT_MAX,(void*)&pid->pos.output_clamp.max);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_AZ_POS_OUTPUT_OFFSET,(void*)&pid->pos.output_clamp.offset);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_AZ_SPD_OUTPUT_MIN,(void*)&pid->spd.output_clamp.min);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_AZ_SPD_OUTPUT_MAX,(void*)&pid->spd.output_clamp.max);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_AZ_SPD_OUTPUT_OFFSET,(void*)&pid->spd.output_clamp.offset);
  
  

  pid->homeOffset = 0;
  pid->homeError = 0;
  pid->joystickScale = 1.0;
  if(pid->pos.input_range.min > 0){
    pid->pos.input_range.clipped = true;
  }
  if(pid->pos.input_range.min > 0){
    pid->pos.input_range.clipped = true;
  }
  
  pid->pos.cal.pv = 0.;
  pid->pos.cal.sp = 0.;
  pid->pos.cal.out = 0.;
  pid->pos.cal.err = 0.;
  pid->pos.getPv = resolver_get_position_deg;
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
  
  pid->spd.input_range.clipped = false;
  // control map
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_AZ_M1_EN_ID,(void*)&pid->controlMap.servo_en[0]);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_AZ_M1_SVON_ID,(void*)&pid->controlMap.servo_on[0]);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_AZ_M2_EN_ID,(void*)&pid->controlMap.servo_en[1]);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_AZ_M2_SVON_ID,(void*)&pid->controlMap.servo_on[1]);
  pid->controlMap.controlMode = 0x1;
  db_read_param(DATA_SECTION_STOW_PARAM,STOW_PARAM_POWER_EN_ID,(void*)&stow->pwr_control);
  db_read_param(DATA_SECTION_STOW_PARAM,STOW_PARAM_DIRECTION_ID,(void*)&stow->dir_control);
  db_read_param(DATA_SECTION_STOW_PARAM,STOW_PARAM_AZEL_SELECTION,(void*)&stow->axis_select);
  
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_AZ_LOCK_ID,(void*)&stow->lock_sense);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_AZ_UNLOCK_ID,(void*)&stow->unlock_sense);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_AZ_M2_SVON_ID,(void*)&pid->controlMap.servo_on[1]);
  
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_AZ_M1_RDY_ID,(void*)&pid->controlMap.servo_rdy[0]);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_AZ_M2_RDY_ID,(void*)&pid->controlMap.servo_rdy[1]);
  pid->controlMap.stow_lock_sense = stow->lock_sense;
  pid->controlMap.stow_unlock_sense = stow->unlock_sense;
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_AZ_M1_OUTPUT_CHANNEL,(void*)&pid->controlMap.servo_cmd[0]);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_AZ_M2_OUTPUT_CHANNEL,(void*)&pid->controlMap.servo_cmd[1]);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_AZ_RV_INPUT_CHANNEL,(void*)&pid->controlMap.pv_pos);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_AZ_RV_INPUT_CHANNEL,(void*)&pid->controlMap.pv_spd);
  
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_AZ_RV_INPUT_CHANNEL,(void*)&pid->controlMap.sp_pos);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_AZ_RV_INPUT_CHANNEL,(void*)&pid->controlMap.sp_spd);
  
  //pid->controlMap.sp_pos = 0;
  //pid->controlMap.sp_spd = 0;
  pid->cycleTimeUs = 5000;
  pid->stowValid = false;
  pid->stowState = STOW_UNKNOW;
  pid->mcStatus = 0xAA;
  pid->joystickMode = false;
  //pid->stream = (BaseSequentialStream*)&CONSOLE;
  pid->axisId = AXIS_AZ;
 
  pid->stowConfig = stow;


    // AXIS EL
  pid = &runTime.pidConfig[AXIS_EL];
  stow = &runTime.stowConfig[AXIS_EL];
  db_read_param(DATA_SECTION_PID_PARAM,PID_PARAM_EL_POS_KP,(void*)&pid->pos.param.kp);
  db_read_param(DATA_SECTION_PID_PARAM,PID_PARAM_EL_POS_KI,(void*)&pid->pos.param.ki);
  db_read_param(DATA_SECTION_PID_PARAM,PID_PARAM_EL_POS_KD,(void*)&pid->pos.param.kd);
  db_read_param(DATA_SECTION_PID_PARAM,PID_PARAM_EL_SPD_KP,(void*)&pid->spd.param.kp);
  db_read_param(DATA_SECTION_PID_PARAM,PID_PARAM_EL_SPD_KI,(void*)&pid->spd.param.ki);
  db_read_param(DATA_SECTION_PID_PARAM,PID_PARAM_EL_SPD_KD,(void*)&pid->spd.param.kd);
  
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_EL_POS_INPUT_MIN,(void*)&pid->pos.input_range.min);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_EL_POS_INPUT_MAX,(void*)&pid->pos.input_range.max);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_EL_POS_INPUT_OFFSET,(void*)&pid->pos.input_range.offset);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_EL_SPD_INPUT_MIN,(void*)&pid->spd.input_range.min);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_EL_SPD_INPUT_MAX,(void*)&pid->spd.input_range.max);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_EL_SPD_INPUT_OFFSET,(void*)&pid->spd.input_range.offset);
  
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_EL_POS_OUTPUT_MIN,(void*)&pid->pos.output_clamp.min);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_EL_POS_OUTPUT_MAX,(void*)&pid->pos.output_clamp.max);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_EL_POS_OUTPUT_OFFSET,(void*)&pid->pos.output_clamp.offset);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_EL_SPD_OUTPUT_MIN,(void*)&pid->spd.output_clamp.min);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_EL_SPD_OUTPUT_MAX,(void*)&pid->spd.output_clamp.max);
  db_read_param(DATA_SECTION_BASE_PARAM,BASE_PARAM_EL_SPD_OUTPUT_OFFSET,(void*)&pid->spd.output_clamp.offset);

  pid->homeOffset = 0;
  pid->homeError = 0;
  pid->joystickScale = 1.0;
  if(pid->pos.input_range.min > 0){
    pid->pos.input_range.clipped = true;
  }
  if(pid->pos.input_range.min > 0){
    pid->pos.input_range.clipped = true;
  }
  pid->pos.cal.pv = 0.;
  pid->pos.cal.sp = 0.;
  pid->pos.cal.out = 0.;
  pid->pos.cal.err = 0.;
  pid->pos.getPv = resolver_get_position_deg;
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
  
  pid->spd.input_range.clipped = false;
  
  // controlmap
  
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_EL_M1_EN_ID,(void*)&pid->controlMap.servo_en[0]);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_EL_M1_SVON_ID,(void*)&pid->controlMap.servo_on[0]);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_EL_M2_EN_ID,(void*)&pid->controlMap.servo_en[1]);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_EL_M2_SVON_ID,(void*)&pid->controlMap.servo_on[1]);
  pid->controlMap.controlMode = 0x1;
  db_read_param(DATA_SECTION_STOW_PARAM,STOW_PARAM_POWER_EN_ID,(void*)&stow->pwr_control);
  db_read_param(DATA_SECTION_STOW_PARAM,STOW_PARAM_DIRECTION_ID,(void*)&stow->dir_control);
  db_read_param(DATA_SECTION_STOW_PARAM,STOW_PARAM_AZEL_SELECTION,(void*)&stow->axis_select);
  
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_EL_LOCK_ID,(void*)&stow->lock_sense);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_EL_UNLOCK_ID,(void*)&stow->unlock_sense);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_EL_M2_SVON_ID,(void*)&pid->controlMap.servo_on[1]);
  
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_EL_M1_RDY_ID,(void*)&pid->controlMap.servo_rdy[0]);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_EL_M2_RDY_ID,(void*)&pid->controlMap.servo_rdy[1]);
  pid->controlMap.stow_lock_sense = stow->lock_sense;
  pid->controlMap.stow_unlock_sense = stow->unlock_sense;
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_EL_M1_OUTPUT_CHANNEL,(void*)&pid->controlMap.servo_cmd[0]);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_EL_M2_OUTPUT_CHANNEL,(void*)&pid->controlMap.servo_cmd[1]);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_EL_RV_INPUT_CHANNEL,(void*)&pid->controlMap.pv_pos);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_EL_RV_INPUT_CHANNEL,(void*)&pid->controlMap.pv_spd);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_EL_RV_INPUT_CHANNEL,(void*)&pid->controlMap.sp_pos);
  db_read_param(DATA_SECTION_CONTROL_PARAM,CONTROL_PARAM_EL_RV_INPUT_CHANNEL,(void*)&pid->controlMap.sp_spd);

  //pid->controlMap.sp_pos = 0;
  //pid->controlMap.sp_spd = 0;
  pid->cycleTimeUs = 5000;
  pid->stowValid = false;
  pid->stowState = STOW_UNKNOW;
  pid->mcStatus = 0xAA;
  pid->joystickMode = false;
  //pid->stream = (BaseSequentialStream*)&CONSOLE;
  pid->axisId = AXIS_EL;
 
  pid->stowConfig = stow;
}

static void load_app_param()
{
  uint16_t i;
  uint16_t u16;
  float f32;
  
  load_pid_param_ex();
  
//  for(i=0;i<AD57_CHANNELS;i++){
//    db_read_param(DATA_SECTION_DAC_PARAM
//  }
  // for test purpose, remove if done
  runTime.pidConfig[0].stowValid = true;
  runTime.pidConfig[0].stowState = STOW_UNLOCKED;
  runTime.pidConfig[1].stowValid = true;
  runTime.pidConfig[1].stowState = STOW_UNLOCKED;
  


  // for test purpose, remove if done
  runTime.pidConfig[0].stowValid = true;
  runTime.pidConfig[0].stowState = STOW_UNLOCKED;
  runTime.pidConfig[1].stowValid = true;
  runTime.pidConfig[1].stowState = STOW_UNLOCKED;

  runTime.pidAutoStart = true;
  runTime.activeStowAxis = 0xFF;
  
}

static void pid_run_ex()
{
  //if(axis < NOF_AXES){
    load_pid_param_ex();
    uint16_t mode;
    //OD_get_u16(OD_ENTRY_H6040,0x04,&mode,true);
    
    runTime.pidConfig[AXIS_AZ].speedMode = (mode==0)?true:false;
    runTime.pidConfig[AXIS_EL].speedMode = (mode==0)?true:false;
    nested_pid_init(AXIS_AZ,&runTime.pidConfig[AXIS_AZ]);
    nested_pid_init(AXIS_EL,&runTime.pidConfig[AXIS_EL]);
  //}
}

static void pid_stop_ex(uint8_t axis)
{
  nested_pid_stop(axis);
}

static void start_pid()
{
  uint16_t mode = 0;
  
  if(!runTime.pidAZ.stowValid) return;
  if(!runTime.pidEL.stowValid) return;
  
  //OD_get_u16(OD_ENTRY_H6040,0x04,&mode,true);
  
  runTime.pidConfig[AXIS_AZ].speedMode = (mode==0)?true:false;
  runTime.pidConfig[AXIS_EL].speedMode = (mode==0)?true:false;
  
//  tpcmdh_taskInit(&runTime.pidAZ,&runTime.pidEL);
//  tdmotc_algorithm_task_init(&runTime.pidAZ,&runTime.pidEL,mode);
//  tdmotc_Start(AXIS_AZ, mode);
  nested_pid_init(AXIS_AZ, &runTime.pidConfig[0]);
}



static void updateStatus(uint8_t mask, bool set)
{
    uint32_t sta = runTime.mcStatus;
    if(set){
       sta |= (1 << mask);
    }
    else{
      sta &= ~(1 << mask);
    }
    runTime.mcStatus_new = sta;
    db_write_live_data(LIVEDATA_SECTION_SYSTEM, LIVEDATA_MC_STATUS,(void*)&runTime.mcStatus_new);
}

static void stop_pid()
{
  updateStatus(STA_SYS_PIDAZ_READY,0);
  updateStatus(STA_SYS_PIDEL_READY,0);
  
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

  bool _stop = false;
  msg_t reason = MSG_OK;
  
  //runTime.stowControl.stowValidState = 0x0;
  
  if(runTime.pidAutoStart){
    pid_run_ex();
  }
  
  chVTObjectInit(&runTime.vtLog);
  
  chVTObjectInit(&runTime.vtUpdate);
//  chVTSetI(&runTime.vtUpdate,TIME_MS2I(100),updateTimer,NULL);
  
  if(runTime.tpdoUpdateInterval < 5){
    runTime.tpdoUpdateInterval = 5;
  }

  chThdResume(&runTime.ref,MSG_OK);
  //issueStatusUpdate(true);  
  while(!_stop)
  {
    if(chThdShouldTerminateX()){
      _stop = true;
    }
    
    // update OD
//    uint8_t sta = stow_state();
//    if(sta != 0){
//      //uint8_t state = stow_state();
//      if(runTime.stowControl.axisId == AXIS_AZ){
//        if(sta < 4){
//          updateStatus(STA_MC_AZ_STOW_LOCK_IN_PROGRESS,1);
//        }
//        else{
//          updateStatus(STA_MC_AZ_STOW_UNLOCK_IN_PROGRESS,1);
//        }
//      }
//      else if(runTime.stowControl.axisId == AXIS_EL){
//        if(sta < 4){
//          updateStatus(STA_MC_EL_STOW_LOCK_IN_PROGRESS,1);
//        }
//        else{
//          updateStatus(STA_MC_EL_STOW_UNLOCK_IN_PROGRESS,1);
//        }
//      }
//    }
    
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(2));
    if(evt & EV_UPDATE){
      //issueStatusUpdate(true);
    }
    if(evt & EV_REQUEST_TPDO){
//      OD_requestTPDO(runTime.cosFlags.pvFlag,1);
//      OD_requestTPDO(runTime.cosFlags.pvFlag,2);
//      OD_requestTPDO(runTime.cosFlags.pvFlag,3);
//      OD_requestTPDO(runTime.cosFlags.pvFlag,4);
    }
    if(evt & EV_ADC_ACQUIRED){
      // update adc data 
      int32_t buffer[AD76_NOF_CHANNEL];
      analog_input_read(0xff,buffer);
      for(uint8_t i=0;i<AD76_NOF_CHANNEL;i++){
        db_write_live_data(LIVEDATA_SECTION_ANALOG_IN,LIVEDATA_VIN_RAW_CH0+i,(void*)&buffer[i]);
      }      
      float bufferf[AD76_NOF_CHANNEL];
      analog_input_readf(0xff,bufferf);
      for(uint8_t i=0;i<AD76_NOF_CHANNEL;i++){
        db_write_live_data(LIVEDATA_SECTION_ANALOG_IN,LIVEDATA_VIN_VOLT_CH0+i,(void*)&buffer[i]);
      }      
    }
    if(evt & EV_RESOLVER_ACQUIRED){
      float fv;
      //od = OD_find(OD,0x6064);
      fv = (resolver_get_position_deg(0));
      if(fv < 0) {
        fv += 360;
      }
      db_write_live_data(LIVEDATA_SECTION_RESOLVER,LIVEDATA_AZ_DEG,(void*)&fv);
      fv = resolver_get_speed(0);
      db_write_live_data(LIVEDATA_SECTION_RESOLVER,LIVEDATA_AZ_DPS,(void*)&fv);
      fv = (resolver_get_position_deg(1));
      db_write_live_data(LIVEDATA_SECTION_RESOLVER,LIVEDATA_EL_DEG,(void*)&fv);
      fv = resolver_get_speed(1);
      db_write_live_data(LIVEDATA_SECTION_RESOLVER,LIVEDATA_EL_DPS,(void*)&fv);
    }
    if(evt & EV_PID_STATE_ERROR_AZ){
      updateStatus(STA_SYS_PIDAZ_READY,0);
    }
    if(evt & EV_PID_START){
      nested_pid_set_user_control(false);
    }
    if(evt & EV_PID_STOP){
      nested_pid_set_user_control(true);
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
      uint16_t u16;
      digital_get_iso_inw(0,&u16);
      db_write_live_data(LIVEDATA_SECTION_DIGITAL_IO,LIVEDATA_DIGITAL_INPUT_WORD,(void*)&u16);    
      digital_get_iso_outw(0,&u16);
      db_write_live_data(LIVEDATA_SECTION_DIGITAL_IO,LIVEDATA_DIGITAL_OUTPUT_WORD,(void*)&u16);    

      if((1 << runTime.stowConfig[0].lock_sense) & u16){
        updateStatus(STA_MC_AZ_STOW_LOCKD,1);
      }
      else{
        updateStatus(STA_MC_AZ_STOW_LOCKD,0);
      }
      if((1 << runTime.stowConfig[0].unlock_sense) & u16){
        updateStatus(STA_MC_AZ_STOW_UNLOCKD,1);
      }
      else{
        updateStatus(STA_MC_AZ_STOW_UNLOCKD,0);
      }

      if((1 << runTime.stowConfig[1].lock_sense) & u16){
        updateStatus(STA_MC_EL_STOW_LOCKD,1);
      }
      else{
        updateStatus(STA_MC_EL_STOW_LOCKD,0);
      }
      if((1 << runTime.stowConfig[1].unlock_sense) & u16){
        updateStatus(STA_MC_EL_STOW_UNLOCKD,1);
      }
      else{
        updateStatus(STA_MC_EL_STOW_UNLOCKD,0);
      }
    }
    
    if(evt & EV_STOW_CONTROL){
      //runTime.stowControl.axisId = runTime.pidAZ.axisId;
      //validStow(runTime.stowCommand,&runTime.stowControl);
      runTime.activeStowAxis = AXIS_AZ;
      nested_pid_stow_control(runTime.stow_control_enable,runTime.activeStowAxis);
    }
    
    if(evt & EV_STOW_UPDATE){
        if(runTime.stowConfig[runTime.activeStowAxis].activity == STOW_GO_LOCK){
          if(runTime.activeStowAxis == AXIS_AZ)
            updateStatus(STA_MC_AZ_STOW_LOCK_IN_PROGRESS,1);
          else if(runTime.activeStowAxis == AXIS_EL)
            updateStatus(STA_MC_EL_STOW_LOCK_IN_PROGRESS,1);
        }
        else if(runTime.stowConfig[runTime.activeStowAxis].activity == STOW_GO_UNLOCK){
          if(runTime.activeStowAxis == AXIS_AZ)
            updateStatus(STA_MC_AZ_STOW_UNLOCK_IN_PROGRESS,1);
          else if(runTime.activeStowAxis == AXIS_EL)
            updateStatus(STA_MC_EL_STOW_UNLOCK_IN_PROGRESS,1);
        }
    }
       
    if(evt & EV_EL_STOW_LOCK){
    }
    
    if(evt & EV_STOW_ACTION_DONE){
      //sysValidStow(runTime.stowCommand,&runTime.pidAZ);
      //runTime.stowValidState = 0;
      if(runTime.stowConfig[AXIS_AZ].state != STOW_PROCESSING){
      }
      if(runTime.stowConfig[AXIS_EL].state != STOW_PROCESSING){
      }
      if(runTime.activeStowAxis == AXIS_AZ){
        updateStatus(STA_MC_STOW_AZ_ERROR,0);
        updateStatus(STA_MC_AZ_STOW_LOCK_IN_PROGRESS,0);
        updateStatus(STA_MC_AZ_STOW_UNLOCK_IN_PROGRESS,0);
        // stop AZ and start EL
        nested_pid_stow_control(false, runTime.activeStowAxis);
        runTime.activeStowAxis = AXIS_EL;
        nested_pid_stow_control(runTime.stow_control_enable, runTime.activeStowAxis);
      }
      else if(runTime.activeStowAxis == AXIS_EL){
        updateStatus(STA_MC_STOW_EL_ERROR,0);
        updateStatus(STA_MC_EL_STOW_LOCK_IN_PROGRESS,0);
        updateStatus(STA_MC_EL_STOW_UNLOCK_IN_PROGRESS,0);
        runTime.activeStowAxis = 0xFF;
        nested_pid_stow_control(false, runTime.activeStowAxis);
      }
    }
       
    if(evt & EV_STOW_ACTION_FAIL){
      //sysValidStow(runTime.stowCommand,&runTime.pidEL);
      //runTime.stowValidState = 0;
      if(runTime.activeStowAxis == AXIS_AZ){
        updateStatus(STA_MC_STOW_AZ_ERROR,1);
        updateStatus(STA_MC_AZ_STOW_LOCK_IN_PROGRESS,0);
        updateStatus(STA_MC_AZ_STOW_UNLOCK_IN_PROGRESS,0);
        //updateStatus(STA_MC_AZ_STOW_LOCKD,0);
        //updateStatus(STA_MC_AZ_STOW_UNLOCKD,0);
      }
      else if(runTime.activeStowAxis == AXIS_EL){
        updateStatus(STA_MC_STOW_EL_ERROR,1);
        updateStatus(STA_MC_EL_STOW_LOCK_IN_PROGRESS,0);
        updateStatus(STA_MC_EL_STOW_UNLOCK_IN_PROGRESS,0);
        //updateStatus(STA_MC_EL_STOW_LOCKD,0);
        //updateStatus(STA_MC_EL_STOW_UNLOCKD,0);
      }
      nested_pid_stow_control(false,runTime.activeStowAxis);
    }
    
    if(evt & EV_PID_AZ_RDY){
      updateStatus(STA_SYS_PIDAZ_READY,1);
    }
    if(evt & EV_PID_EL_RDY){
      updateStatus(STA_SYS_PIDEL_READY,1);
    }

   
    if(evt & EV_ANT_STOW){
      if(runTime.stow_control_enable){
        load_pid_param_ex();
        nested_pid_home_init(AXIS_AZ,&runTime.pidConfig[0]);
        nested_pid_home_init(AXIS_EL,&runTime.pidConfig[1]);
      }
      else{
        nested_pid_stop();
      }
    }
  
    if(evt & EV_AZ_HOMESEARCHING){
      if(runTime.pidConfig[AXIS_AZ].homeState == HOME_INIT){
        updateStatus(STA_SYS_ANT_STOW_AZ,1);
      }
      else{
        updateStatus(STA_SYS_ANT_STOW_AZ,0);
      }
    }
    if(evt & EV_EL_HOMESEARCHING){
      if(runTime.pidConfig[AXIS_EL].homeState == HOME_INIT){
        updateStatus(STA_SYS_ANT_STOW_EL,1);
      }
      else{
        updateStatus(STA_SYS_ANT_STOW_AZ,0);
      }
    }
    
    //issueStatusUpdate(false);
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
    runTime.appThread = chThdCreateStatic(waApp,sizeof(waApp),NORMALPRIO-1,procApp,NULL);
    chSysLock();
    chThdSuspendS(&runTime.ref);
    chSysUnlock();
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


void task_baseOperationInit()
{
  database_init();
  start_app();
  task_can1_init();
}
