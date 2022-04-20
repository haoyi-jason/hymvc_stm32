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

/*Define*/
#define PIN_MOT1_AIN 0U                     /**< @brief  DAC channel of motor 1.*/
#define PIN_MOT2_AIN 1U                     /**< @brief  DAC channel of motor 2.*/
/*Constant*/

/*Static variable*/
static DMOTC_HANDLE_T dmotch;               /**< @brief  Handle of dual_motor_controller.*/

/*PID related*/
static PID_HANDLE_T pospid;                 /**< @brief  Handle of position PID.*/        

/**
 * @brief  Configuration handle of position PID.
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
static TQBC_CFG_T tqbccfg = {0.0f, 0.0f, 0.0f, 0.0f}; /**< @brief  Configuration handle of torque bias calculator. */

static float out_max_2set = DMOTC_DFLT_OUT_MAX;       /**< @brief  Maximum bias torque to be set.*/
static float out_min_2set = DMOTC_DFLT_OUT_MIN;       /**< @brief  Minimum bias torque to be set.*/
static float gain_2set = DMOTC_DFLT_GAIN;             /**< @brief  Slop of bias torque to be set.*/
static float zcp_2set = DMOTC_DFLT_ZCP;               /**< @brief  Zero crossing point to be set.*/
static float out_max_set = DMOTC_DFLT_OUT_MAX;        /**< @brief  Maximum bias torque set.*/
static float out_min_set = DMOTC_DFLT_OUT_MIN;        /**< @brief  Minimum bias torque set.*/
static float gain_set = DMOTC_DFLT_GAIN;              /**< @brief  Slop of bias torque set.*/
static float zcp_set = DMOTC_DFLT_ZCP;                /**< @brief  Zero crossing point set.*/

/*Position control related*/
static POSC_CMD_HANDLE_T pccmdh = DFLT_INIT_POSC_CMD_HANDLE_T(); /**< @brief  Position controller data handle.*/
/**
 * @brief  Configuration handle of position PID.
 */
static POSC_CFG_HANDLE_T pccfgh = INIT_POSC_CFG_HANDLE_T(POSC_ON_THOLD_U16, DMOTC_DFLT_POS_S_MIN, DMOTC_DFLT_POS_S_MAX, DMOTC_DFLT_POS_KP);

/*Contorl related signal*/
static bool dmotc_is_good = true;                     /**< @brief  Flag task is good.*/
static bool start = false;                            /**< @brief  Flag start.*/
static bool start_last = false;                       /**< @brief  Last value of start.*/
static tdmotc_mode_t mode = TDMOTC_MODE_S;            /**< @brief  Mode of run.*/
static bool can_config = false;                       /**< @brief  Flag parameter be config.*/

/*Other parameter and signals*/
static float tq_mot_max_abs = DMOTC_DFLT_TQ_MOT_MAX_ABS_PC;   /**< @brief  Maximum motor torque in percentage.*/
static float s_axis_max_abs = DMOTC_DFLT_S_AXIS_MAX_ABS_RPM;  /**< @brief  Maximum allowable axis speed in RPM.*/
static float tq_mot_pc[2] = {0.0f, 0.0f};     /**< @brief  DA signal in percentage.*/
static float tq_mot_v[2] = {0.0f, 0.0f};      /**< @brief  DA signal in voltage.*/
static float speed_act_rpm = 0.0f;            /**< @brief  Axis actual speed in RPM.*/
//static long speed_act_lsb = 0;                /**< @brief  Axis actual speed in raw format.*/

static float pos_act_deg = 0.0f;              /**< @brief  Axis actual position in degree.*/

static float speed_cmd_rpm = 0.0f;            /**< @brief  Axis speed command in RPM.*/
static float pos_cmd_deg = 0.0f;              /**< @brief  Axis position command in degree.*/

/*Declare private functions*/
static float _GetPosDEG(void);
//static long _GetSpeedLSB(void);

/*Thread*/
static thread_t *tp_dmotc;
static THD_WORKING_AREA(waDMOTC,1024);
static THD_FUNCTION(procDMOTC ,p)
{
  /*Declare local variable*/
  systime_t prev = chVTGetSystemTime(); /* Current system time.*/

  float _priv_speeed_cmd_rpm = 0.0f;
  float _priv_pos_cmd = 0.0f;
  static pos_u16t _priv_pos_act_u16;


  /*Initialization*/

  /*Set default config*/
  if(DMOTC_MSG_OK != DMOTC_SetTQBCConfig(&tqbccfg, out_max_set, out_min_set, gain_set, zcp_set))
  {
    /*Torque bias calculator setup failed*/
    dmotc_is_good = false;
  }

  /*Initialize*/
  if(dmotc_is_good)
  {  
    if(DMOTC_MSG_OK != DMOTC_Init(&dmotch, 
                                  &pidcfg_speed,
                                  &tqbccfg,
                                  (1.0f), 
                                  (-1.0f * tq_mot_max_abs),
                                  tq_mot_max_abs))
    {
      /*Initialization failed*/
      dmotc_is_good = false;
    }
  }

  if(PID_MSG_OK != PID_Init(&pospid, &pidcfg_pos))
  {
    /*Position PID controller initialize failed*/
    dmotc_is_good = false;
  }


  /*Start algorithm*/

  while(!chThdShouldTerminateX())
  {
    /*Timing*/
    palSetPad(GPIOI, GPIOI_PIN6);

    /*Run periodically*/
    /*State contol*/
    if(start && dmotc_is_good )
    {
      /*Start signal detected and good*/
      if(!start_last)
      {
        /*Starting*/
        /*Restart position PID*/
        PID_Restart(&pospid);

        /*Reset privite variable*/
        _priv_speeed_cmd_rpm  = 0.0f;
        _priv_pos_cmd = 0.0f;

        /*Reset input and output*/
        tdmotc_ResetIO();

        /*Reinitialize and start*/
        if(DMOTC_MSG_OK != DMOTC_Init(&dmotch, 
                                      &pidcfg_speed,
                                      &tqbccfg,
                                      (1.0f), 
                                      (-1.0f * tq_mot_max_abs), 
                                      tq_mot_max_abs))
        {
          /*Initialization failed*/
          dmotc_is_good = false;
        }

        if(dmotc_is_good)
        {
          /*Proceed to start*/
          if(DMOTC_MSG_OK != DMOTC_Start(&dmotch))
          {
            /*Start failed*/
            dmotc_is_good = false;
          }
        }
        can_config = false;
        // enable servo
        digital_set_iso_out(4,1);
        digital_set_iso_out(5,1);
        digital_set_iso_out(6,1);
        digital_set_iso_out(7,1);

        start_last = start;
       // start = false;
      }
    }
    else
    {
      /*Start signal off or not good*/
      if(start_last)
      {
        /*Stopping*/
        /*Force output to 0.0 and stop*/
        DMOTC_Stop(&dmotch);

        /*Reset input and output*/
        tdmotc_SetSpeedCmd(0.0f);
        tpcmdh_SetPosCmd(0.0f);
        tdmotc_ResetIO();
        analog_output_set_voltage(PIN_MOT1_AIN, &tq_mot_v[0]);
        analog_output_set_voltage(PIN_MOT2_AIN, &tq_mot_v[1]);

        // disable servo
        digital_set_iso_out(4,0);
        digital_set_iso_out(5,0);
        digital_set_iso_out(6,0);
        digital_set_iso_out(7,0);

        start_last = false;
      }
      else
      {
        can_config = true;
      }
    }

    /*Algorithm contorl*/
    if(start && start_last && dmotc_is_good)
    {
      /*Control mode set*/
      if(TDMOTC_MODE_P == mode)
      {
        /*Get input*/
        chSysLock();
        pccmdh.pos_cmd_u16 = tpcmdh_GetPosCmdU16();
        pccmdh.direction_cmd = tpcmdh_GetDirection();
        chSysUnlock();
        //_priv_pos_cmd = tdmotc_GetPosCmd();
        pos_act_deg = _GetPosDEG();
        _priv_pos_act_u16 = POSC_ConvertDeg2U16(pos_act_deg);

        _priv_speeed_cmd_rpm = POSC_Run(&pccmdh, &pccfgh, _priv_pos_act_u16);
      }
      else if(TDMOTC_MODE_P2 == mode)
      {
        /*Position mode 2, run at fix speed to target position and stop*/
        _priv_pos_cmd = tdmotc_GetPosCmd();
        pos_act_deg = _GetPosDEG();

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
        _priv_speeed_cmd_rpm = tdmotc_GetSpeedCmd();
      }


      //speed_act_lsb = _GetSpeedLSB();
      //speed_act_rpm = ((float)speed_act_lsb * -0.1f)/200.0f; // Using modus speed
      //speed_act_rpm = (float)speed_act_lsb * 0.00381f;
      speed_act_rpm = resolver_get_speed(0)*60.f;
  
      /*Run algorithm*/
      if(DMOTC_MSG_OK != DMOTC_Run(&dmotch,
                                   _priv_speeed_cmd_rpm, 
                                   speed_act_rpm,
                                   &tq_mot_pc[0],
                                   &tq_mot_pc[1]))
      {
        /*Run failed*/
        dmotc_is_good = false;
      }
  
      /*Set output*/
      tq_mot_v[0] = tq_mot_pc[0] * 0.1f;
      tq_mot_v[1] = tq_mot_pc[1] * 0.1f;

      analog_output_set_voltage(PIN_MOT1_AIN, &tq_mot_v[0]);
      analog_output_set_voltage(PIN_MOT2_AIN, &tq_mot_v[1]);
    }

    /*GPIO toggle*/
    chThdSleepMilliseconds(1);
    palClearPad(GPIOI, GPIOI_PIN6);
    
    /*Sleep*/
    prev = chThdSleepUntilWindowed(prev, chTimeAddX(prev, TIME_MS2I(5)));
  }
}

/*Static functions*/
static float _GetPosDEG(void)
{
  float output = 0.0f;
  /*Implement get method and conversion here*/
  output = (resolver_get_position(0) * 0.016667f);
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
void tdmotc_algorithm_task_init(void)
{
  tp_dmotc = chThdCreateStatic(waDMOTC, sizeof(waDMOTC), NORMALPRIO, procDMOTC, NULL);
}

/**
 * @brief      Start algorithm with specific mode.
 *
 * @param[in]  _mode  Mode to be run with.
 */
void tdmotc_Start(tdmotc_mode_t _mode)
{
  if(dmotch.state == DMOTC_STATE_STOP)
  {
    switch (_mode)
    {
      case TDMOTC_MODE_S:
      start = true;
      mode = TDMOTC_MODE_S;
      break;

      case TDMOTC_MODE_P:
      start = true;
      mode = TDMOTC_MODE_P;
      break;

      case TDMOTC_MODE_P2:
      start = true;
      mode = TDMOTC_MODE_P2;
      break;

      default:
      break;
    }
  }
}

/**
 * @brief      Stop algorithm.
 */
void tdmotc_Stop(void)
{
  /*Set start flag to false*/
  start = false;
}

/**
 * @brief      Reset IO signals.
 */
void tdmotc_ResetIO(void)
{
  for (int i = 0; i < 2; i++)
  {
    tq_mot_pc[i] = 0.0f;
    tq_mot_v[i] = 0.0f;
  }
}

tdmotc_mode_t tdmotc_GetMode(void)
{
  return mode;
}

bool tdmotc_GetFault(void)
{
  return !dmotc_is_good;
}

void tdmotc_ResetFault(void)
{
  dmotc_is_good = true;
}

void tdmotc_SetSpeedCmd(float val)
{
  float _priv_axis_max_s = tdmotc_GetAxisSMaxAbs();
  if(isfinite(val))
  {
    /*Proceed*/
    if(val > _priv_axis_max_s)
    {
      speed_cmd_rpm = _priv_axis_max_s;
    }
    else if(val < (-1.0f *_priv_axis_max_s))
    {
      speed_cmd_rpm = (-1.0f *_priv_axis_max_s);
    }
    else
    {
      speed_cmd_rpm = val;
    }
  }
}

float tdmotc_GetSpeedCmd(void)
{
  return speed_cmd_rpm;
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

void tdmotc_SetPosCmd2(float val)
{
  if(isfinite(val))
  {
    if(val > 358.0f)
    {
      pos_cmd_deg = 358.0f;
    }
    else if(val < 2.0f)
    {
      pos_cmd_deg = 2.0f;
    }
    else
    {
      pos_cmd_deg = val;
    }
  }
}

float tdmotc_GetPosCmd(void)
{
  return pos_cmd_deg;
}

/**
 * @brief      Set PID parameters according to index
 *
 * @param[in]  pid        Index of PID handle
 * @param[in]  pid_index  Index of PID parameter.
 * @param[in]  val        Value to be set.
 */
void tdmotc_SetPID(uint8_t pid, uint8_t pid_index, float val)
{
  if(isfinite(val) && can_config)
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
        pidcfg_pos.kp = val;
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

float tdmotc_GetPID(uint8_t pid, uint8_t pid_index)
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
      value = pidcfg_pos.kp;
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
void tdmotc_SetTQBC(uint8_t index, float val)
{
  if(isfinite(val) && can_config)
  {
    switch (index)
    {
      case TDMOTC_TQBC_ID_OUT_MAX:
      if(val >= tq_mot_max_abs)
      {
        out_max_2set = tq_mot_max_abs;
      }
      else if(val <= 0.0f)
      {
        out_max_2set = 0.0f;
      }
      else
      {
        out_max_2set = val;
      }
      break;
  
      case TDMOTC_TQBC_ID_OUT_MIN:
      if(val >= tq_mot_max_abs)
      {
        out_min_2set = tq_mot_max_abs;
      }
      else if(val <= 0.0f)
      {
        out_min_2set = 0.0f;
      }
      else
      {
        out_min_2set = val;
      }
      break;
  
      case TDMOTC_TQBC_ID_GAIN:
      if(val >= -0.01f)
      {
        val = -0.01f;
      }
      gain_2set = val;
      break;
  
      case TDMOTC_TQBC_ID_ZCP:
      if(val <= 0.1f)
      {
        val = 0.1f;
      }
      zcp_2set = val;
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
float tdmotc_GetTQBC(uint8_t index)
{
  float value = 0.0f;

  switch (index)
  {
    case TDMOTC_TQBC_ID_OUT_MAX:
    value = out_max_set;
    break;

    case TDMOTC_TQBC_ID_OUT_MIN:
    value = out_min_set;
    break;

    case TDMOTC_TQBC_ID_GAIN:
    value = gain_set;
    break;

    case TDMOTC_TQBC_ID_ZCP:
    value = zcp_set;
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
void tdmotc_UpdateTQBC(void)
{
  if(can_config)
  {
    if(DMOTC_MSG_OK == DMOTC_SetTQBCConfig(&tqbccfg, out_max_2set, out_min_2set, gain_2set, zcp_2set))
    {
      /*Set successfully*/
      out_max_set = out_max_2set;
      out_min_set = out_min_2set;
      gain_set = gain_2set;
      zcp_set = zcp_2set;
    }
  }
}

void tdmotc_SetMotTqMaxAbs(float val)
{
  if(isfinite(val) && can_config)
  {
    /*Proceed*/
    if(val < 0.0f)
    {
      tq_mot_max_abs = 0.0f;
    }
    else
    {
      tq_mot_max_abs = val;
    }    
  }
}

float tdmotc_GetMotTqMaxAbs(void)
{
  return tq_mot_max_abs;
}

void tdmotc_SetAxisSMaxAbs(float val)
{
  if(isfinite(val) && can_config)
  {
    /*Proceed*/
    if(val < 0.0f)
    {
      s_axis_max_abs = 0.0f;
    }
    else
    {
      s_axis_max_abs = val;
    }
  }
}

float tdmotc_GetAxisSMaxAbs(void)
{
  return s_axis_max_abs;
}

float tdmotc_GetAxisSpeedAct(void)
{
  return speed_act_rpm;
}

float tdmotc_GetMotTqActV(uint8_t index)
{
  float output = 0.0;
  if(index < 2U)
  {
    output = tq_mot_v[index];
  }

  return output;
}

bool tdmotc_GetIsStart(void)
{
  bool output = false;
  if(start && start_last)
  {
    output = true;
  }
  return output;
}


void tdmotc_SetPCCFG(uint8_t index, float val)
{
  if(isfinite(val) && can_config)
  {
    switch (index)
    {
      case TDMOTC_PCCFG_ID_PERR_THOLD:
      POSC_SetCfgPErrThold(&pccfgh, val);
      break;

      case TDMOTC_PCCFG_ID_S_CMD_MIN:
      if(val < 0.0f)
      {
        POSC_SetCfgSCmdMin(&pccfgh, 0.0f);
      }
      else
      {
        POSC_SetCfgSCmdMin(&pccfgh, val);
      }
      break;
      
      case TDMOTC_PCCFG_ID_S_CMD_MAX:
      if(val < 0.0f)
      {
        POSC_SetCfgSCmdMax(&pccfgh, 0.0f);
      }
      else
      {
        POSC_SetCfgSCmdMax(&pccfgh, val);
      }
      break;
      
      case TDMOTC_PCCFG_ID_KP:
      if(val < 0.0f)
      {
        POSC_SetCfgKp(&pccfgh, 0.0f);
      }
      else if( val > 1.0f)
      {
        POSC_SetCfgKp(&pccfgh, 1.0f);
      }
      else
      {
        POSC_SetCfgKp(&pccfgh, val);
      }
      break;

      default:
      /*Invalid index*/
      break;
    }
  }
}

float tdmotc_GetPCCFG(uint8_t index)
{
  float retval = 0.0f;

  switch (index)
  {

    case TDMOTC_PCCFG_ID_PERR_THOLD:
    retval = POSC_GetCfgPErrThold(&pccfgh);
    break;

    case TDMOTC_PCCFG_ID_S_CMD_MIN:
    retval = POSC_GetCfgSCmdMin(&pccfgh);
    break;
    
    case TDMOTC_PCCFG_ID_S_CMD_MAX:
    retval = POSC_GetCfgSCmdMax(&pccfgh);
    break;
    
    case TDMOTC_PCCFG_ID_KP:
    retval = POSC_GetCfgKp(&pccfgh);
    break;

    default:
    /*Invalid index*/
    break;
  }

  return retval;
}

float tdmotc_ConvCAN2Sig(int32_t input)
{
  float output = 0.0f;
  output = (float)input * TDMOTC_CAN2SIG_GAIN_F;
  return output;
}

int32_t tdmotc_ConvSig2CAN(float input)
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