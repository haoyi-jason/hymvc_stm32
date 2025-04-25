/**
 * @file       task_dual_motor_ctrl.c
 * @addtogroup TASK_DUAL_MOTOR_CTRL
 * @{
 */
 /*Standard include*/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/*Chibios include*/
#include "ch.h"
#include "hal.h"

/*Self include*/
#include "task_dual_motor_ctrl.h"

/*Module include*/
#include "task_dac.h"
#include "dual_motor_ctrl/dual_motor_ctrl.h"
#include "dual_motor_ctrl/saturation.h"
#include "task_resolver.h"
//#include "task_mbmaster.h"
#include "digital_io.h"
#include "pos_ctrl.h"
#include "task_pos_cmd_handler.h"

/*Other include*/

/*Config file include*/
#include "dual_motor_ctrl_config.h"
   
#include "app_config.h"

/*Define*/
#define PIN_MOT1_AIN 0U                     /**< @brief  DAC channel of motor 1.*/
#define PIN_MOT2_AIN 1U                     /**< @brief  DAC channel of motor 2.*/
/*Constant*/

/*Static variable*/
static DMOTC_HANDLE_T dmotch[NOF_AXES];               /**< @brief  Handle of dual_motor_controller.*/

/*PID related*/
static PID_HANDLE_T pospid;                 /**< @brief  Handle of position PID.*/        

/**
 * @brief  Configuration handle of position PID.
 * @note   Not in used any more. Can be deleted in version afterward.
 */
static PID_CFG_T pidcfg_pos =
{
  .cfg1 = 0,
  .kp = DMOTC_DFLT_P_PID_KP,
  .ki = DMOTC_DFLT_P_PID_KI,
  .kd = DMOTC_DFLT_P_PID_KD
};

/**
 * @brief  Configuration handle of speed PID.
 */
static PID_CFG_T  pidcfg_speed =
{
  .cfg1 = 0,
  .kp = DMOTC_DFLT_S_PID_KP,
  .ki = DMOTC_DFLT_S_PID_KI,
  .kd = DMOTC_DFLT_S_PID_KD
};

/*TQBC related*/
static TQBC_CFG_T tqbccfg[NOF_AXES] = {{0.0f, 0.0f, 0.0f, 0.0f},{0.0f, 0.0f, 0.0f, 0.0f}}; /**< @brief  Configuration handle of torque bias calculator. */

static float out_max_2set[NOF_AXES] = {DMOTC_DFLT_OUT_MAX,DMOTC_DFLT_OUT_MAX};       /**< @brief  Maximum bias torque to be set.*/
static float out_min_2set[NOF_AXES] = {DMOTC_DFLT_OUT_MIN,DMOTC_DFLT_OUT_MIN};       /**< @brief  Minimum bias torque to be set.*/
static float gain_2set[NOF_AXES] = {DMOTC_DFLT_GAIN,DMOTC_DFLT_GAIN};             /**< @brief  Slop of bias torque to be set.*/
static float zcp_2set[NOF_AXES] = {DMOTC_DFLT_ZCP,DMOTC_DFLT_ZCP};               /**< @brief  Zero crossing point to be set.*/
static float out_max_set[NOF_AXES] = {DMOTC_DFLT_OUT_MAX,DMOTC_DFLT_OUT_MAX};        /**< @brief  Maximum bias torque set.*/
static float out_min_set[NOF_AXES] = {DMOTC_DFLT_OUT_MIN,DMOTC_DFLT_OUT_MIN};        /**< @brief  Minimum bias torque set.*/
static float gain_set[NOF_AXES] = {DMOTC_DFLT_GAIN,DMOTC_DFLT_GAIN};              /**< @brief  Slop of bias torque set.*/
static float zcp_set[NOF_AXES] = {DMOTC_DFLT_ZCP,DMOTC_DFLT_ZCP};                /**< @brief  Zero crossing point set.*/

/*Position control related*/
POSC_HANDLE_T posch;

//static POSC_CMD_HANDLE_T pccmdh = DFLT_INIT_POSC_CMD_HANDLE_T(); /**< @brief  Position controller data handle.*/
/**
 * @brief  Configuration handle of position controller.
 */
//static POSC_CFG_HANDLE_T pccfgh = INIT_POSC_CFG_HANDLE_T(POSC_ON_THOLD_U16, DMOTC_DFLT_POS_S_MIN, DMOTC_DFLT_POS_S_MAX, DMOTC_DFLT_POS_KP);

/*Contorl related signal*/
static bool dmotc_is_good[NOF_AXES] = {true,true};                     /**< @brief  Flag task is good.*/
static bool start[NOF_AXES] = {false,false};                            /**< @brief  Flag start.*/
static bool start_last[NOF_AXES] = {false,false};                       /**< @brief  Last value of start.*/
static tdmotc_mode_t mode[NOF_AXES] = {TDMOTC_MODE_S,TDMOTC_MODE_S};            /**< @brief  Mode of run.*/
static bool can_config[NOF_AXES] = {false,false};                       /**< @brief  Flag parameter be config.*/

/*Other parameter and signals*/
static float tq_mot_max_abs[NOF_AXES] = {DMOTC_DFLT_TQ_MOT_MAX_ABS_PC,DMOTC_DFLT_TQ_MOT_MAX_ABS_PC};   /**< @brief  Maximum motor torque in percentage.*/
static float s_axis_max_abs[NOF_AXES] = {DMOTC_DFLT_S_AXIS_MAX_ABS_RPM,DMOTC_DFLT_S_AXIS_MAX_ABS_RPM};  /**< @brief  Maximum allowable axis speed in RPM.*/
static float tq_mot_pc[NOF_AXES][2] = {{0.0f, 0.0f},{0.0f, 0.0f}};     /**< @brief  DA signal in percentage.*/
static float tq_mot_v[NOF_AXES][2] = {{0.0f, 0.0f},{0.0f, 0.0f}};      /**< @brief  DA signal in voltage.*/
static float speed_act_rpm[NOF_AXES] = {0.0f,0.0f};            /**< @brief  Axis actual speed in RPM.*/
//static long speed_act_lsb = 0;                /**< @brief  Axis actual speed in raw format.*/

static float pos_act_deg = 0.0f;              /**< @brief  Axis actual position in degree.*/

static float speed_cmd_rpm[NOF_AXES] = {0.0f,0.0f};            /**< @brief  Axis speed command in RPM.*/
static float pos_cmd_deg[NOF_AXES] = {0.0f,0.0f};              /**< @brief  Axis position command in degree.*/

/*Declare private functions*/
static float _GetPosDEG(uint8_t channel);
//static long _GetSpeedLSB(void);

/*Thread*/
typedef struct{
  thread_t *self;
  thread_t *self_el;
  thread_t *main;
  thread_reference_t ref;
}_runTime;
static _runTime runTime, *pDMOTCRuntime;
//static thread_t *tp_dmotc;

static THD_WORKING_AREA(waDMOTC,1024);
static THD_WORKING_AREA(waDMOTC_EL,1024);
static THD_FUNCTION(procDMOTC ,p)
{
  _pid_config_t *pcfg = p;
  /*Declare local variable*/
  systime_t prev = chVTGetSystemTime(); /* Current system time.*/

  static float _priv_speeed_cmd_rpm = 0.0f;
  float _priv_pos_cmd = 0.0f;
  static pos_u16t _priv_pos_act_u16;


  /*Initialization*/

  if(pcfg != NULL){
    tdmotc_ResetFault(pcfg->axisId);
//    tqbccfg.out_max = pcfg->tqbc_cfg.out_max;
//    tqbccfg.out_min = pcfg->tqbc_cfg.out_min;
//    tqbccfg.gain = pcfg->tqbc_cfg.gain;
//    tqbccfg.bias = pcfg->tqbc_cfg.bias;
    //memcpy((void*)&tqbccfg[pcfg->axisId],(void*)&pcfg->tqbc_cfg,sizeof(tqbccfg));
    out_max_set[pcfg->axisId] = pcfg->tqbc_boundary.out_max_set;
    out_min_set[pcfg->axisId] = pcfg->tqbc_boundary.out_min_set;
    gain_set[pcfg->axisId] = pcfg->tqbc_boundary.gain_set;
    zcp_set[pcfg->axisId] = pcfg->tqbc_boundary.zcp_set;
    tq_mot_max_abs[pcfg->axisId] = pcfg->tqbc_cfg.out_max;
    
    //memcpy((void*)&pidcfg_speed,(void*)&pcfg->pid_cfg_speed, sizeof(pidcfg_speed));
    pidcfg_speed.kp = pcfg->pid_cfg_speed.kp;
    pidcfg_speed.ki = pcfg->pid_cfg_speed.ki;
    pidcfg_speed.kd = pcfg->pid_cfg_speed.kd;
    pidcfg_speed.cfg1 = pcfg->pid_cfg_speed.cfg1;
  }
  else{
    chThdExit(MSG_RESET);
  }
  /*Set default config*/
  if(DMOTC_MSG_OK != DMOTC_SetTQBCConfig(&tqbccfg[pcfg->axisId], out_max_set[pcfg->axisId], out_min_set[pcfg->axisId], gain_set[pcfg->axisId], zcp_set[pcfg->axisId]))
  {
    /*Torque bias calculator setup failed*/
    dmotc_is_good[pcfg->axisId] = false;
  }

  /*Initialize*/
  if(dmotc_is_good[pcfg->axisId])
  {  
    if(DMOTC_MSG_OK != DMOTC_Init(&dmotch[pcfg->axisId], 
                                  &pcfg->pidcfg_speed,
                                  &tqbccfg[pcfg->axisId],
                                  (1.0f), 
                                  (-1.0f * tq_mot_max_abs[pcfg->axisId]),
                                  tq_mot_max_abs[pcfg->axisId]))
    {
      /*Initialization failed*/
      dmotc_is_good[pcfg->axisId] = false;
    }
  }

  if(PID_MSG_OK != PID_Init(&pospid, &pidcfg_pos))
  {
    /*Position PID controller initialize failed*/
    dmotc_is_good[pcfg->axisId] = false;
  }

  /*Init position controller to default value*/
  POSC_Init(&posch, 
             POSC_ON_THOLD_U16, 
             DMOTC_DFLT_POS_S_MIN, 
             DMOTC_DFLT_POS_S_MAX,
             DMOTC_DFLT_P_PID_KP,
             DMOTC_DFLT_P_PID_KI,
             DMOTC_DFLT_P_PID_KD);


  /*Start algorithm*/
  chEvtSignal(runTime.main,(pcfg->axisId == AXIS_AZ)?EV_PID_AZ_RDY:EV_PID_EL_RDY);
  chSysLock();
  chThdResume(&runTime.ref,MSG_OK);
  chSysUnlock();
  while(!chThdShouldTerminateX())
  {
    /*Timing*/
    //palSetPad(GPIOI, GPIOI_PIN6);

    /*Run periodically*/
    /*State contol*/
    if(start && dmotc_is_good[pcfg->axisId] )
    {
      /*Start signal detected and good*/
      if(!start_last[pcfg->axisId])
      {
        /*Starting*/
        /*Restart position PID*/
        //PID_Restart(&pospid);
        POSC_Restart(&posch);

        /*Reset privite variable*/
        _priv_speeed_cmd_rpm  = 0.0f;
        _priv_pos_cmd = 0.0f;

        /*Reset input and output*/
        tdmotc_ResetIO(pcfg->axisId);

        /*Reinitialize and start*/
        if(DMOTC_MSG_OK != DMOTC_Init(&dmotch[pcfg->axisId], 
                                      &pidcfg_speed,
                                      &tqbccfg[pcfg->axisId],
                                      (1.0f), 
                                      (-1.0f * tq_mot_max_abs[pcfg->axisId]), 
                                      tq_mot_max_abs[pcfg->axisId]))
        {
          /*Initialization failed*/
          dmotc_is_good[pcfg->axisId] = false;
        }

        if(dmotc_is_good[pcfg->axisId])
        {
          /*Proceed to start*/
          if(DMOTC_MSG_OK != DMOTC_Start(&dmotch[pcfg->axisId]))
          {
            /*Start failed*/
            dmotc_is_good[pcfg->axisId] = false;
          }
        }
        can_config[pcfg->axisId] = false;
        // enable servo
        digital_set_iso_out(pcfg->ctrl_map.driver_on[0],1);
        digital_set_iso_out(pcfg->ctrl_map.driver_en[0],1);
        digital_set_iso_out(pcfg->ctrl_map.driver_on[1],1);
        digital_set_iso_out(pcfg->ctrl_map.driver_en[1],1);
        if(pcfg->axisId == AXIS_AZ){
          chEvtSignal(runTime.main,EV_PID_AZ_STARTED);
        }
        else if(pcfg->axisId == AXIS_EL){
          chEvtSignal(runTime.main,EV_PID_EL_STARTED);
        }
        start_last[pcfg->axisId] = start[pcfg->axisId];
       // start = false;
      }
    }
    else
    {
      /*Start signal off or not good*/
      if(start_last[pcfg->axisId])
      {
        /*Stopping*/
        /*Force output to 0.0 and stop*/
        DMOTC_Stop(&dmotch[pcfg->axisId]);

        /*Reset input and output*/
        tdmotc_SetSpeedCmd(pcfg->axisId,0.0f);
        tpcmdh_SetPosCmd(pcfg->axisId,0.0f);
        tdmotc_ResetIO(pcfg->axisId);
        analog_output_set_voltage(PIN_MOT1_AIN, &tq_mot_v[pcfg->axisId][0]);
        analog_output_set_voltage(PIN_MOT2_AIN, &tq_mot_v[pcfg->axisId][1]);

        // disable servo
        digital_set_iso_out(pcfg->ctrl_map.driver_on[0],0);
        digital_set_iso_out(pcfg->ctrl_map.driver_en[0],0);
        digital_set_iso_out(pcfg->ctrl_map.driver_on[1],0);
        digital_set_iso_out(pcfg->ctrl_map.driver_en[1],0);
        if(pcfg->axisId == AXIS_AZ){
          chEvtSignal(runTime.main,EV_PID_AZ_STOPPED);
        }
        else if(pcfg->axisId == AXIS_EL){
          chEvtSignal(runTime.main,EV_PID_EL_STOPPED);
        }

        start_last[pcfg->axisId] = false;
      }
      else
      {
        can_config[pcfg->axisId] = true;
      }
    }

    /*Algorithm contorl*/
    if(start[pcfg->axisId] && start_last[pcfg->axisId] && dmotc_is_good[pcfg->axisId])
    {
      /*Control mode set*/
      if(TDMOTC_MODE_P == mode[pcfg->axisId])
      {
        /*Get command values*/
        chSysLock();
        //pccmdh.pos_cmd_u16 = tpcmdh_GetPosCmdU16();
        //pccmdh.direction_cmd = tpcmdh_GetDirection();
        posch.cmd.pos_cmd_u16 = tpcmdh_GetPosCmdU16(pcfg->axisId);
        posch.cmd.direction_cmd = tpcmdh_GetDirection(pcfg->axisId);
        chSysUnlock();

        /*Get actual value and convert*/
        pos_act_deg = _GetPosDEG(pcfg->axisId);
        _priv_pos_act_u16 = POSC_ConvertDeg2U16(pos_act_deg);

        /*Run position control algorithm*/
        //_priv_speeed_cmd_rpm = POSC_Run(&pccmdh, &pccfgh, _priv_pos_act_u16);
        _priv_speeed_cmd_rpm =  POSC_Run(&posch, _priv_pos_act_u16);
      }
      else if(TDMOTC_MODE_P2 == mode[pcfg->axisId])
      {
        /*Position mode 2, run at fix speed to target position and stop*/
        _priv_pos_cmd = tdmotc_GetPosCmd(pcfg->axisId);
        pos_act_deg = _GetPosDEG(pcfg->axisId);

        if((_priv_pos_cmd - pos_act_deg) > DMOTC_DFLT_POS2_THOLD)
        {
          /*Run with positive speed*/
          _priv_speeed_cmd_rpm = DMOTC_DFLT_SPEED;
        }
        else if((_priv_pos_cmd - pos_act_deg) < (-1.0f * DMOTC_DFLT_POS2_THOLD))
        {
          /*Run with negative speed*/
          _priv_speeed_cmd_rpm = -1.0f * DMOTC_DFLT_SPEED;
        }
        else
        {
          /*Stop*/
          _priv_speeed_cmd_rpm = 0.0f;
        }
      }
      else
      {
        /*Run TDMOTC_MODE_S in default*/
        /*Get input*/
        _priv_speeed_cmd_rpm = tdmotc_GetSpeedCmd(pcfg->axisId);
      }


      //speed_act_lsb = _GetSpeedLSB();
      //speed_act_rpm = ((float)speed_act_lsb * -0.1f)/200.0f; // Using modus speed
      //speed_act_rpm = (float)speed_act_lsb * 0.00381f;
      speed_act_rpm[pcfg->axisId] = resolver_get_speed(0)*60.f;
  
      /*Run algorithm*/
      if(DMOTC_MSG_OK != DMOTC_Run(&dmotch[pcfg->axisId],
                                   _priv_speeed_cmd_rpm, 
                                   speed_act_rpm[pcfg->axisId],
                                   &tq_mot_pc[pcfg->axisId][0],
                                   &tq_mot_pc[pcfg->axisId][1]))
      {
        /*Run failed*/
        dmotc_is_good[pcfg->axisId] = false;
      }
  
      /*Set output*/
      tq_mot_v[pcfg->axisId][0] = tq_mot_pc[pcfg->axisId][0] * 0.1f;
      tq_mot_v[pcfg->axisId][1] = tq_mot_pc[pcfg->axisId][1] * 0.1f;

//      analog_output_set_voltage(PIN_MOT1_AIN, &tq_mot_v[pcfg->axisId][0]);
//      analog_output_set_voltage(PIN_MOT2_AIN, &tq_mot_v[pcfg->axisId][1]);
      analog_output_set_voltage(pcfg->ctrl_map.driver_output[0], &tq_mot_v[pcfg->axisId][0]);
      analog_output_set_voltage(pcfg->ctrl_map.driver_output[1], &tq_mot_v[pcfg->axisId][1]);
      chEvtSignal(runTime.main,EV_PID_UPDATEDA);
    }

    /*GPIO toggle*/
    //chThdSleepMilliseconds(5);
    //palClearPad(GPIOI, GPIOI_PIN6);
    
    /*Sleep*/
    prev = chThdSleepUntilWindowed(prev, chTimeAddX(prev, TIME_MS2I(10)));
  }
  
  chThdExit(MSG_OK);
}

/*Static functions*/
static float _GetPosDEG(uint8_t channel)
{
  float output = 0.0f;
  /*Implement get method and conversion here*/
  output = (resolver_get_position(channel) * 0.016667f);
  return output;
}

//static long _GetSpeedLSB(void)
//{
//  long output = 0;
//  /*Implement get method and conversion here*/
//  output = modbus_master_GetSpeed();
//  return output;
//}

/*Functions*/
void tdmotc_algorithm_task_init(void* config, void *config_el,uint8_t m)
{
  if(runTime.self == NULL){
    pDMOTCRuntime = &runTime;
    runTime.main = chRegFindThreadByName("MAIN");
    mode[0] = m;
    mode[1] = m;
    if(config != NULL){
      // todo: valid I/O state first
      
      runTime.self = chThdCreateStatic(waDMOTC, sizeof(waDMOTC), NORMALPRIO, procDMOTC, config);
      msg_t ret = chThdSuspendS(&runTime.ref);
      tdmotc_Start(0,mode[0]);
    }
    if(config_el != NULL){
      //tdmotc_ResetFault(AXIS_EL);
      runTime.self_el = chThdCreateStatic(waDMOTC_EL, sizeof(waDMOTC_EL), NORMALPRIO, procDMOTC, config_el);
      msg_t ret =chThdSuspendS(&runTime.ref);
      tdmotc_Start(1,mode[1]);
    }
  }
}

void tdmotc_algorithm_task_stop(void)
{
  if(runTime.self != NULL){
    chThdTerminate(runTime.self);
    chThdWait(runTime.self);
    runTime.self = NULL;
  }
  if(runTime.self_el != NULL){
    chThdTerminate(runTime.self_el);
    chThdWait(runTime.self_el);
    runTime.self_el = NULL;
  }
}


/**
 * @brief      Start algorithm with specific mode.
 *
 * @param[in]  _mode  Mode to be run with.
 */
void tdmotc_Start(uint8_t axis,tdmotc_mode_t _mode)
{
  if(dmotch[axis].state == DMOTC_STATE_STOP)
  {
    switch (_mode)
    {
      case TDMOTC_MODE_S:
      start[axis] = true;
      mode[axis] = TDMOTC_MODE_S;
      break;

      case TDMOTC_MODE_P:
      start[axis] = true;
      mode[axis] = TDMOTC_MODE_P;
      break;

      case TDMOTC_MODE_P2:
      start[axis] = true;
      mode[axis] = TDMOTC_MODE_P2;
      break;

      default:
      break;
    }
  }
}

/**
 * @brief      Stop algorithm.
 */
void tdmotc_Stop(uint8_t axis)
{
  /*Set start flag to false*/
  start[axis] = false;
}

/**
 * @brief      Reset IO signals.
 */
void tdmotc_ResetIO(uint8_t axis)
{
  for (int i = 0; i < 2; i++)
  {
    tq_mot_pc[axis][i] = 0.0f;
    tq_mot_v[axis][i] = 0.0f;
  }
}

tdmotc_mode_t tdmotc_GetMode(uint8_t axis)
{
  return mode[axis];
}

bool tdmotc_GetFault(uint8_t axis)
{
  return !dmotc_is_good[axis];
}

void tdmotc_ResetFault(uint8_t axis)
{
  dmotc_is_good[axis] = true;
}

void tdmotc_SetSpeedCmd(uint8_t axis, float val)
{
  float _priv_axis_max_s = tdmotc_GetAxisSMaxAbs(axis);
  if(isfinite(val))
  {
    /*Proceed*/
    if(val > _priv_axis_max_s)
    {
      speed_cmd_rpm[axis] = _priv_axis_max_s;
    }
    else if(val < (-1.0f *_priv_axis_max_s))
    {
      speed_cmd_rpm[axis] = (-1.0f *_priv_axis_max_s);
    }
    else
    {
      speed_cmd_rpm[axis] = val;
    }
  }
}

float tdmotc_GetSpeedCmd(uint8_t axis)
{
  return speed_cmd_rpm[axis];
}

//void tdmotc_SetPosCmd(float val)
//{
//  if(isfinite(val))
//  {
//    if(val > 345.0f)
//    {
//      pos_cmd_deg = 345.0f;
//    }
//    else if(val < 15.0f)
//    {
//      pos_cmd_deg = 15.0f;
//    }
//    else
//    {
//      pos_cmd_deg = val;
//    }
//  }
//}

void tdmotc_SetPosCmd2(uint8_t axis, float val)
{
  if(isfinite(val))
  {
    if(val > 358.0f)
    {
      pos_cmd_deg[axis] = 358.0f;
    }
    else if(val < 2.0f)
    {
      pos_cmd_deg[axis] = 2.0f;
    }
    else
    {
      pos_cmd_deg[axis] = val;
    }
  }
}

float tdmotc_GetPosCmd(uint8_t axis)
{
  return pos_cmd_deg[axis];
}

/**
 * @brief      Set PID parameters according to index
 *
 * @param[in]  pid        Index of PID handle
 * @param[in]  pid_index  Index of PID parameter.
 * @param[in]  val        Value to be set.
 */
void tdmotc_SetPID(uint8_t axis, uint8_t pid, uint8_t pid_index, float val)
{
  if(isfinite(val) && can_config[axis])
  {
    switch (pid)
    {
      case TDMOTC_PID_S:
      switch (pid_index)
      {
        case TDMOTC_PID_ID_P:
        if(val <= 0.0f)
        {
          val = 0.0f;
        }
        pidcfg_speed.kp = val;
        break;
  
        case TDMOTC_PID_ID_I:
        if(val <= 0.0f)
        {
          val = 0.0f;
        }
        pidcfg_speed.ki = val;
        break;
  
        case TDMOTC_PID_ID_D:
        if(val <= 0.0f)
        {
          val = 0.0f;
        }
        pidcfg_speed.kd = val;
        break;
  
        default:
        /*Invalid pid_index*/
        break;
      }
      break;    
        
      case TDMOTC_PID_P:
      switch (pid_index)
      {
        case TDMOTC_PID_ID_P:
        if(val <= 0.0f)
        {
          val = 0.0f;
        }
        posch.pid_cfg.kp = val;
        break;
        
        case TDMOTC_PID_ID_I:
        if(val <= 0.0f)
        {
          val = 0.0f;
        }
        posch.pid_cfg.ki = val;
        break;
        
        case TDMOTC_PID_ID_D:
        if(val <= 0.0f)
        {
          val = 0.0f;
        }
        posch.pid_cfg.kd = val;
        break;
        
        default:
        /*Invalid pid_index*/
        break;
      }
      break;
  
      default:
      /*Invalid pid*/
      break;
    }
  }
}

float tdmotc_GetPID(uint8_t axis, uint8_t pid, uint8_t pid_index)
{
  float value = 0.0f;

  switch (pid)
  {
    case TDMOTC_PID_S:
    switch (pid_index)
    {
      case TDMOTC_PID_ID_P:
      value = pidcfg_speed.kp;
      break;

      case TDMOTC_PID_ID_I:
      value = pidcfg_speed.ki;
      break;

      case TDMOTC_PID_ID_D:
      value = pidcfg_speed.kd;
      break;

      default:
      /*Invalid pid_index*/
      break;
    }
    break;

    case TDMOTC_PID_P:
    switch (pid_index)
    {
      case TDMOTC_PID_ID_P:
      value = posch.pid_cfg.kp;
      break;

      case TDMOTC_PID_ID_I:
      value = posch.pid_cfg.ki;
      break;

      case TDMOTC_PID_ID_D:
      value = posch.pid_cfg.kd;
      break;

      default:
      /*Invalid pid_index*/
      break;
    }
    break;
 
    default:
    /*Invalid pid*/
    break;
  }

  return value;
}

/**
 * @brief      Set torque bias calculator parameters according to index
 * 
 * @param[in]  index  Index of parameters.
 * @param[in]  val    Value to be set.
 * 
 * @note       Function "tdmotc_UpdateTQBC()" must be invoke after set in order
 *             to update the equation. 
 */
void tdmotc_SetTQBC(uint8_t axis, uint8_t index, float val)
{
  if(isfinite(val) && can_config[axis])
  {
    switch (index)
    {
      case TDMOTC_TQBC_ID_OUT_MAX:
      if(val >= tq_mot_max_abs[axis])
      {
        out_max_2set[axis] = tq_mot_max_abs[axis];
      }
      else if(val <= 0.0f)
      {
        out_max_2set[axis] = 0.0f;
      }
      else
      {
        out_max_2set[axis] = val;
      }
      break;
  
      case TDMOTC_TQBC_ID_OUT_MIN:
      if(val >= tq_mot_max_abs[axis])
      {
        out_min_2set[axis] = tq_mot_max_abs[axis];
      }
      else if(val <= 0.0f)
      {
        out_min_2set[axis] = 0.0f;
      }
      else
      {
        out_min_2set[axis] = val;
      }
      break;
  
      case TDMOTC_TQBC_ID_GAIN:
      if(val >= -0.01f)
      {
        val = -0.01f;
      }
      gain_2set[axis] = val;
      break;
  
      case TDMOTC_TQBC_ID_ZCP:
      if(val <= 0.1f)
      {
        val = 0.1f;
      }
      zcp_2set[axis] = val;
      break;
  
      default:
      /*Invalid index*/
      break;
    }
  }
}

/**
 * @brief      Get parameter currently being used by torque bias calculator.
 *
 * @param[in]  index  Index of parameters.
 *
 * @return     valus of parameter.
 */
float tdmotc_GetTQBC(uint8_t axis,uint8_t index)
{
  float value = 0.0f;

  switch (index)
  {
    case TDMOTC_TQBC_ID_OUT_MAX:
    value = out_max_set[axis];
    break;

    case TDMOTC_TQBC_ID_OUT_MIN:
    value = out_min_set[axis];
    break;

    case TDMOTC_TQBC_ID_GAIN:
    value = gain_set[axis];
    break;

    case TDMOTC_TQBC_ID_ZCP:
    value = zcp_set[axis];
    break;

    default:
    /*Invalid index*/
    break;
  }

  return value;
}

/**
 * @brief      Update equation of torque bias calculator.
 */
void tdmotc_UpdateTQBC(uint8_t axis)
{
  if(can_config[axis])
  {
    if(DMOTC_MSG_OK == DMOTC_SetTQBCConfig(&tqbccfg, out_max_2set[axis], out_min_2set[axis], gain_2set[axis], zcp_2set[axis]))
    {
      /*Set successfully*/
      out_max_set[axis] = out_max_2set[axis];
      out_min_set[axis] = out_min_2set[axis];
      gain_set[axis] = gain_2set[axis];
      zcp_set[axis] = zcp_2set[axis];
    }
  }
}

void tdmotc_SetMotTqMaxAbs(uint8_t axis, float val)
{
  if(isfinite(val) && can_config[axis])
  {
    /*Proceed*/
    if(val < 0.0f)
    {
      tq_mot_max_abs[axis] = 0.0f;
    }
    else
    {
      tq_mot_max_abs[axis] = val;
    }    
  }
}

float tdmotc_GetMotTqMaxAbs(uint8_t axis)
{
  return tq_mot_max_abs[axis];
}

void tdmotc_SetAxisSMaxAbs(uint8_t axis,float val)
{
  if(isfinite(val) && can_config[axis])
  {
    /*Proceed*/
    if(val < 0.0f)
    {
      s_axis_max_abs[axis] = 0.0f;
    }
    else
    {
      s_axis_max_abs[axis] = val;
    }
  }
}

float tdmotc_GetAxisSMaxAbs(uint8_t axis)
{
  return s_axis_max_abs[axis];
}

float tdmotc_GetAxisSpeedAct(uint8_t axis)
{
  return speed_act_rpm[axis];
}

float tdmotc_GetMotTqActV(uint8_t axis, uint8_t index)
{
  float output = 0.0;
  if(index < 2U)
  {
    output = tq_mot_v[axis][index];
  }

  return output;
}

bool tdmotc_GetIsStart(uint8_t axis)
{
  bool output = false;
  if(start[axis] && start_last[axis])
  {
    output = true;
  }
  return output;
}


void tdmotc_SetPCCFG(uint8_t axis, uint8_t index, float val)
{
  if(isfinite(val) && can_config[axis])
  {
    switch (index)
    {
      case TDMOTC_PCCFG_ID_PERR_THOLD:
      POSC_SetCfgPErrThold(&posch.cfg, val);
      break;

      case TDMOTC_PCCFG_ID_S_CMD_MIN:
      if(val < 0.0f)
      {
        POSC_SetCfgSCmdMin(&posch.cfg, 0.0f);
      }
      else
      {
        POSC_SetCfgSCmdMin(&posch.cfg, val);
      }
      break;
      
      case TDMOTC_PCCFG_ID_S_CMD_MAX:
      if(val < 0.0f)
      {
        POSC_SetCfgSCmdMax(&posch.cfg, 0.0f);
      }
      else
      {
        POSC_SetCfgSCmdMax(&posch.cfg, val);
      }
      break;
      
      case TDMOTC_PCCFG_ID_KP:
      if(val < 0.0f)
      {
        POSC_SetCfgKp(&posch.cfg, 0.0f);
      }
      else if( val > 1.0f)
      {
        POSC_SetCfgKp(&posch.cfg, 1.0f);
      }
      else
      {
        POSC_SetCfgKp(&posch.cfg, val);
      }
      break;

      default:
      /*Invalid index*/
      break;
    }
  }
}

float tdmotc_GetPCCFG(uint8_t axis, uint8_t index)
{
  float retval = 0.0f;

  switch (index)
  {

    case TDMOTC_PCCFG_ID_PERR_THOLD:
    retval = POSC_GetCfgPErrThold(&posch.cfg);
    break;

    case TDMOTC_PCCFG_ID_S_CMD_MIN:
    retval = POSC_GetCfgSCmdMin(&posch.cfg);
    break;
    
    case TDMOTC_PCCFG_ID_S_CMD_MAX:
    retval = POSC_GetCfgSCmdMax(&posch.cfg);
    break;
    
    case TDMOTC_PCCFG_ID_KP:
    retval = POSC_GetCfgKp(&posch.cfg);
    break;

    default:
    /*Invalid index*/
    break;
  }

  return retval;
}

float tdmotc_ConvCAN2Sig(uint8_t axis, int32_t input)
{
  float output = 0.0f;
  output = (float)input * TDMOTC_CAN2SIG_GAIN_F;
  return output;
}

int32_t tdmotc_ConvSig2CAN(uint8_t axis, float input)
{
  int32_t output = 0;
  if(isfinite(input))
  {
    output = (int32_t)(input * 1.0f/TDMOTC_CAN2SIG_GAIN_F);
  }
  return output;
}



/**
 * @}
 */