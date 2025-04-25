#include "ch.h"
#include "hal.h"
#include "task_systemvalid.h"
#include "app_config.h"
#include "digital_io.h"


typedef struct{
  thread_t *self;
  thread_t *main;
  thread_reference_t ref;
  _stow_state_t sm;
  _stow_control_t *pctrl;
  uint32_t executeTime;
  uint8_t lock_state;
  uint8_t unlock_state;
}_svRuntime;

static _svRuntime svRuntime;

static THD_WORKING_AREA(waStowCtrl,512);
static THD_FUNCTION(procStowCtrl,  p)
{
//  _stow_control_t *pctrl = (_stow_control_t*)p;
  
//  if(pctrl == NULL) {
//    chThdExit(MSG_RESET);
//  }
//  
//  if(pctrl->cycletime_ms < pctrl->timeout_ms){
//    chThdExit(MSG_RESET);
//  }
  
  bool done = false;
  uint16_t runMs = 0;
  int8_t state = 0;
  
  svRuntime.pctrl = NULL;
  svRuntime.sm.state = STOW_NONE;
  svRuntime.sm.nextState = STOW_NONE;
  svRuntime.sm.subState = 0;
  
  while(!chThdShouldTerminateX()){
    
    switch(svRuntime.sm.state){
    case STOW_NONE:
      if(svRuntime.sm.nextState == STOW_LOCK_INIT){
        svRuntime.sm.state = STOW_LOCK_INIT;
        svRuntime.sm.nextState = STOW_NONE;
        if(svRuntime.pctrl->axisId == AXIS_AZ)
          svRuntime.pctrl->stowStatus = STA_MC_AZ_STOW_LOCK_IN_PROGRESS;
        else
          svRuntime.pctrl->stowStatus = STA_MC_EL_STOW_LOCK_IN_PROGRESS;
        svRuntime.sm.state = STOW_LOCK_INIT;
      }
      else if(svRuntime.sm.nextState == STOW_UNLOCK_INIT){
        svRuntime.sm.state = STOW_UNLOCK_INIT;
        svRuntime.sm.nextState = STOW_NONE;
        if(svRuntime.pctrl->axisId == AXIS_AZ)
          svRuntime.pctrl->stowStatus = STA_MC_AZ_STOW_UNLOCK_IN_PROGRESS;
        else
          svRuntime.pctrl->stowStatus = STA_MC_EL_STOW_UNLOCK_IN_PROGRESS;
        svRuntime.sm.state = STOW_UNLOCK_INIT;
      }
      break;
    case STOW_LOCK_INIT:
      svRuntime.lock_state = digital_get_iso_in(svRuntime.pctrl->ctrl_map->stow_lock_in);
      svRuntime.unlock_state = digital_get_iso_in(svRuntime.pctrl->ctrl_map->stow_unlock_in);
      if(svRuntime.lock_state == 1){
        if(svRuntime.unlock_state == 1){ //error
          if(svRuntime.pctrl->axisId == AXIS_AZ)
            svRuntime.pctrl->stowStatus = STA_MC_STOW_AZ_ERROR;
          else
            svRuntime.pctrl->stowStatus = STA_MC_STOW_EL_ERROR;
          }
          svRuntime.sm.state = STOW_CHECK_FAIL;
      }
      else{
        // drive output 
        digital_set_iso_out(svRuntime.pctrl->ctrl_map->stow_lock_out, 1);
        svRuntime.executeTime = 0;
        if(svRuntime.pctrl->axisId == AXIS_AZ)
          svRuntime.pctrl->stowStatus = STA_MC_AZ_STOW_LOCK_IN_PROGRESS;
        else
          svRuntime.pctrl->stowStatus = STA_MC_EL_STOW_LOCK_IN_PROGRESS;
        svRuntime.sm.state = STOW_LOCKING;
      }
      break;
    case STOW_LOCKING:
      svRuntime.lock_state = digital_get_iso_in(svRuntime.pctrl->ctrl_map->stow_lock_in);
      svRuntime.executeTime += svRuntime.pctrl->cycletime_ms;
      if(svRuntime.lock_state == 1){
        svRuntime.sm.state = STOW_LOCK_DONE;
        digital_set_iso_out(svRuntime.pctrl->ctrl_map->stow_lock_out, 0);
        if(svRuntime.pctrl->axisId == AXIS_AZ)
          svRuntime.pctrl->stowStatus = STA_MC_AZ_STOW_LOCKD;
        else
          svRuntime.pctrl->stowStatus = STA_MC_EL_STOW_LOCKD;
      }
      else if(svRuntime.executeTime > svRuntime.pctrl->timeout_ms){
        svRuntime.sm.state = STOW_TIMEOUT;
        digital_set_iso_out(svRuntime.pctrl->ctrl_map->stow_lock_out, 0);
        if(svRuntime.pctrl->axisId == AXIS_AZ)
          svRuntime.pctrl->stowStatus = STA_MC_STOW_AZ_ERROR;
        else
          svRuntime.pctrl->stowStatus = STA_MC_STOW_EL_ERROR;
      }
      break;
    case STOW_LOCK_DONE:
      if(svRuntime.main != NULL){
        chEvtSignal(svRuntime.main, EV_STOW_ACTION_DONE);
      }
      svRuntime.sm.state = STOW_NONE;
      break;
    case STOW_UNLOCK_INIT:
      svRuntime.lock_state = digital_get_iso_in(svRuntime.pctrl->ctrl_map->stow_lock_in);
      svRuntime.unlock_state = digital_get_iso_in(svRuntime.pctrl->ctrl_map->stow_unlock_in);
      if(svRuntime.unlock_state == 1){
        if(svRuntime.lock_state == 1){ //error
          if(svRuntime.pctrl->axisId == AXIS_AZ)
            svRuntime.pctrl->stowStatus = STA_MC_STOW_AZ_ERROR;
          else
            svRuntime.pctrl->stowStatus = STA_MC_STOW_EL_ERROR;
          svRuntime.sm.state = STOW_CHECK_FAIL;
        }
      }
      else{
        // drive output 
        digital_set_iso_out(svRuntime.pctrl->ctrl_map->stow_unlock_out, 1);
        svRuntime.executeTime = 0;
        svRuntime.sm.state = STOW_UNLOCKING;
        if(svRuntime.pctrl->axisId == AXIS_AZ)
          svRuntime.pctrl->stowStatus = STA_MC_AZ_STOW_LOCKD;
        else
          svRuntime.pctrl->stowStatus = STA_MC_EL_STOW_LOCKD;
      }
      break;
    case STOW_UNLOCKING:
      svRuntime.unlock_state = digital_get_iso_in(svRuntime.pctrl->ctrl_map->stow_unlock_in);
      svRuntime.executeTime += svRuntime.pctrl->cycletime_ms;
      if(svRuntime.unlock_state == 1){
        svRuntime.sm.state = STOW_LOCK_DONE;
        digital_set_iso_out(svRuntime.pctrl->ctrl_map->stow_unlock_out, 0);
        if(svRuntime.pctrl->axisId == AXIS_AZ)
          svRuntime.pctrl->stowStatus = STA_MC_AZ_STOW_UNLOCK_IN_PROGRESS;
        else
          svRuntime.pctrl->stowStatus = STA_MC_EL_STOW_UNLOCK_IN_PROGRESS;
      }
      else if(svRuntime.executeTime > svRuntime.pctrl->timeout_ms){
        svRuntime.sm.state = STOW_TIMEOUT;
        digital_set_iso_out(svRuntime.pctrl->ctrl_map->stow_unlock_out, 0);
        if(svRuntime.pctrl->axisId == AXIS_AZ)
          svRuntime.pctrl->stowStatus = STA_MC_STOW_AZ_ERROR;
        else
          svRuntime.pctrl->stowStatus = STA_MC_STOW_EL_ERROR;
      }
      break;
    case STOW_UNLOCK_DONE:
      if(svRuntime.main != NULL){
        chEvtSignal(svRuntime.main, EV_STOW_ACTION_DONE);
      }
      svRuntime.sm.state = STOW_NONE;
      if(svRuntime.pctrl->axisId == AXIS_AZ)
        svRuntime.pctrl->stowStatus = STA_MC_AZ_STOW_UNLOCKD;
      else
        svRuntime.pctrl->stowStatus = STA_MC_EL_STOW_UNLOCKD;
      break;
    case STOW_TIMEOUT:
    case STOW_CHECK_FAIL:
      if(svRuntime.main != NULL){
        chEvtSignal(svRuntime.main, EV_STOW_ACTION_FAIL);
      }
      svRuntime.sm.state = STOW_NONE;
      break;
    }
    
//    switch(pctrl->stow_activity){
//    case ACT_LOCK:
//      digital_set_iso_out(pctrl->ctrl_map->stow_lock_out,1);
//      state = digital_get_iso_in(pctrl->ctrl_map->stow_lock_in);
//      break;
//    case ACT_UNLOCK:
//      digital_set_iso_out(pctrl->ctrl_map->stow_unlock_out,1);
//      state = digital_get_iso_in(pctrl->ctrl_map->stow_unlock_in);
//      break;
//    }    
    
    chThdSleepMilliseconds(svRuntime.pctrl->cycletime_ms);
    svRuntime.executeTime += svRuntime.pctrl->cycletime_ms;
  }  
  chThdExit(MSG_OK);
}

uint8_t stow_state()
{
  return svRuntime.sm.state;
}

void validStow(uint8_t activity, _stow_control_t *pctrl)
{
  if(svRuntime.sm.state == STOW_NONE){
    svRuntime.pctrl = pctrl;
    if(activity == ACT_LOCK){
      svRuntime.sm.nextState = STOW_LOCK_INIT;
    }
    else if(activity == ACT_UNLOCK){
      svRuntime.sm.nextState = STOW_UNLOCK_INIT;
    }
  }
}

void stow_valid_init()
{
  if(svRuntime.self == NULL){
    svRuntime.main = chRegFindThreadByName("MAIN");
    svRuntime.self = chThdCreateStatic(waStowCtrl, sizeof(waStowCtrl),NORMALPRIO,procStowCtrl,NULL);
  }
}


void stopValidStow()
{
  if(svRuntime.self != NULL){
    chThdTerminate(svRuntime.self);
    chThdWait(svRuntime.self);
    svRuntime.self = NULL;
  }
}

