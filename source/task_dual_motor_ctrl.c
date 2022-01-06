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
#include "dual_motor_ctrl.h"
#include "saturation.h"
#include "task_resolver.h"
#include "task_mbmaster.h"
/*Other include*/

/*Config file include*/
#include "dual_motor_ctrl_config.h"

/*Define*/
#define PIN_MOT1_AIN 0U
#define PIN_MOT2_AIN 1U
/*Constant*/

/*Static variable*/
static DMOTC_HANDLE_T dmotch;

/*PID related*/
static PID_HANDLE_T pospid;
static PID_CFG_T pidcfg_pos = 
{
  .cfg1 = 0,
  .kp = DMOTC_DFLT_P_PID_KP,
  .ki = DMOTC_DFLT_P_PID_KI,
  .kd = DMOTC_DFLT_P_PID_KD
};

static PID_CFG_T  pidcfg_speed =
{
  .cfg1 = 0,
  .kp = DMOTC_DFLT_S_PID_KP,
  .ki = DMOTC_DFLT_S_PID_KI,
  .kd = DMOTC_DFLT_S_PID_KD
};

/*TQBC related*/
static TQBC_CFG_T tqbccfg = {0.0f, 0.0f, 0.0f, 0.0f};

static float out_max_2set = DMOTC_DFLT_OUT_MAX;
static float out_min_2set = DMOTC_DFLT_OUT_MIN;
static float gain_2set = DMOTC_DFLT_GAIN;
static float zcp_2set = DMOTC_DFLT_ZCP;
static float out_max_set = DMOTC_DFLT_OUT_MAX;
static float out_min_set = DMOTC_DFLT_OUT_MIN;
static float gain_set = DMOTC_DFLT_GAIN;
static float zcp_set = DMOTC_DFLT_ZCP;

/*Contorl related signal*/
static bool dmotc_is_good = true;
static bool start = false;
static bool start_last = false;
static tdmotc_mode_t mode = TDMOTC_MODE_S;
static bool can_config = false;
/*Other parameter and signals*/
static float tq_mot_max_abs = DMOTC_DFLT_TQ_MOT_MAX_ABS_PC;
static float s_axis_max_abs = DMOTC_DFLT_S_AXIS_MAX_ABS_RPM;
/*Messages*/
//static uint8_t err_msg_tqbc_setconfig[] = "Failed on TQBC_SetConfig() \r\n";
//static uint8_t err_msg_dmotc_init[] = "Failed on DMOTC_Init() \r\n";
//static uint8_t err_msg_dmotc_start[] = "Failed on DMOTC_Start \r\n";
//static uint8_t err_msg_dmotc_run[] = "Failed on DMOTC_Run() \r\n";

//static const SerialConfig baud_115200_8N1 =
//{
//  115200,
//  0,
//  USART_CR2_STOP1_BITS,
//  0
//};
static float tq_mot_pc[2] = {0.0f, 0.0f};
static float tq_mot_v[2] = {0.0f, 0.0f};
static float speed_act_rpm = 0.0f;
static long speed_act_lsb = 0;

static float pos_act_deg = 0.0f;
/*Global variable*/

//float tq_mot_pc = 0.0f;

//int16_t tq_mot1_lsb = 0;
//int16_t tq_mot2_lsb = 0;



static float speed_cmd_rpm = 0.0f;
static float pos_cmd_deg = 0.0f;

/*Declare private functions*/
static float _GetPosDEG(void);
static long _GetSpeedLSB(void);

/*Thread*/
static thread_t *tp_dmotc;
static THD_WORKING_AREA(waDMOTC,1024);
static THD_FUNCTION(procDMOTC ,p)
{
  /*Declare local variable*/
  float _priv_speeed_cmd_rpm = 0.0f;
  float _priv_pos_cmd = 0.0f;
  float _priv_speed_cmd_raw_rpm = 0.0f;
  /*Initialization*/

  /*Start SD6*/
  //sdStart(&SD6, &baud_115200_8N1);

  /*Set default config*/
  if(DMOTC_MSG_OK != DMOTC_SetTQBCConfig(&tqbccfg, out_max_set, out_min_set, gain_set, zcp_set))
  {
    /*Torque bias calculator setup failed*/
    dmotc_is_good = false;
    //sdWrite(&SD6, err_msg_tqbc_setconfig, sizeof(err_msg_tqbc_setconfig));
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
      //sdWrite(&SD6, err_msg_dmotc_init, sizeof(err_msg_dmotc_init));
    }
  }

  if(PID_MSG_OK != PID_Init(&pospid, &pidcfg_pos))
  {
    /*Position PID controller initialize failed*/
    dmotc_is_good = false;
  }


  /*Start algorithm*/
  //if(dmotc_is_good)
  //{
  //  if(DMOTC_MSG_OK != DMOTC_Start(&dmotch))
  //  {
  //    /*Start failed*/
  //    dmotc_is_good = false;
  //    //sdWrite(&SD6, err_msg_dmotc_start, sizeof(err_msg_dmotc_start));
  //  }
  //}

  while(!chThdShouldTerminateX())
  {
    /*Run periodically*/
    /*State contol*/
    if(start && dmotc_is_good)
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
        _priv_speed_cmd_raw_rpm = 0.0f;

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

        modbus_master_WriteCtrl(1, 3);
        chThdSleepMilliseconds(500);
        modbus_master_WriteCtrl(2, 3);
        chThdSleepMilliseconds(500);
        start_last = start;
      }
    }
    else
    {
      if(start_last)
      {
        /*Stopping*/
        /*Force output to 0.0 and stop*/
        DMOTC_Stop(&dmotch);

        /*Reset input and output*/
        tdmotc_ResetIO();
        analog_output_set_voltage(PIN_MOT1_AIN, &tq_mot_v[0]);
        analog_output_set_voltage(PIN_MOT2_AIN, &tq_mot_v[1]);

        modbus_master_WriteCtrl(1, 0);
        chThdSleepMilliseconds(500);
        modbus_master_WriteCtrl(2, 0);
        chThdSleepMilliseconds(500);

        start_last = start;
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
        _priv_pos_cmd = tdmotc_GetPosCmd();
        pos_act_deg = _GetPosDEG();

        /*Run position PID*/
        if(PID_MSG_OK != PID_RunPID(&pospid, _priv_pos_cmd, pos_act_deg, _priv_speeed_cmd_rpm, &_priv_speed_cmd_raw_rpm))
        {
          /*Pos PID run failed*/
          dmotc_is_good = false;
        }

        /*Saturation*/
        _priv_speeed_cmd_rpm = SAT_fSat(_priv_speed_cmd_raw_rpm, (-1.0f * s_axis_max_abs), s_axis_max_abs);
      }
      else
      {
        /*Run TDMOTC_MODE_S in default*/
        /*Get input*/
        _priv_speeed_cmd_rpm = tdmotc_GetSpeedCmd();
      }


      speed_act_lsb = _GetSpeedLSB();
      speed_act_rpm = ((float)speed_act_lsb * -0.1f)/200.0f; // Using modus speed
      //speed_act_rpm = (float)speed_act_lsb * 0.00381f;
  
      /*Run algorithm*/
      if(DMOTC_MSG_OK != DMOTC_Run(&dmotch,
                                   _priv_speeed_cmd_rpm, 
                                   speed_act_rpm,
                                   &tq_mot_pc[0],
                                   &tq_mot_pc[1]))
      {
        /*Run failed*/
        dmotc_is_good = false;
        //sdWrite(&SD6, err_msg_dmotc_run, sizeof(err_msg_dmotc_run));
      }
  
      /*Set output*/
      //tq_mot1_lsb = (int16_t)(tq_mot1_pc / 3180.0f);
      //tq_mot2_lsb = (int16_t)(tq_mot2_pc / 3180.0f);
      
      //analog_output_set_data(PIN_MOT1_AIN, &tq_mot1_lsb);
      //analog_output_set_data(PIN_MOT2_AIN, &tq_mot2_lsb);

      tq_mot_v[0] = tq_mot_pc[0] * 0.1f;
      tq_mot_v[1] = tq_mot_pc[1] * 0.1f;

      analog_output_set_voltage(PIN_MOT1_AIN, &tq_mot_v[0]);
      analog_output_set_voltage(PIN_MOT2_AIN, &tq_mot_v[1]);
    }

    /*Sleep*/
    chThdSleepMilliseconds(10);
  }
}

/*Static functions*/
static float _GetPosDEG(void)
{
  float output = 0;
  /*Implement get method and conversion here*/
  output = (resolver_get_position(0) / 60.0f);
  return output;
}

static long _GetSpeedLSB(void)
{
  long output = 0;
  /*Implement get method and conversion here*/
  output = modbus_master_GetSpeed();
  return output;
}

/*Functions*/
void tdmotc_algorithm_task_init(void)
{
  tp_dmotc = chThdCreateStatic(waDMOTC, sizeof(waDMOTC), NORMALPRIO, procDMOTC, NULL);
}

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

      default:
      break;
    }
  }
}

void tdmotc_Stop(void)
{
  /*Set start flag to false*/
  start = false;
}

void tdmotc_ResetIO(void)
{
  tdmotc_SetSpeedCmd(0.0f);
  tdmotc_SetPosCmd(0.0f);

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
  return dmotc_is_good;
}

void tdmotc_ResetFault(void)
{
  dmotc_is_good = true;
}


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

void tdmotc_SetSpeedCmd(float val)
{
  float _priv_axis_max_s = tdmotc_GetAxisSMaxAbs();
  if(isfinite(val) && can_config)
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

void tdmotc_SetPosCmd(float val)
{
  if(isfinite(val) && can_config)
  {
    if(val > 358.0f)
    {
      pos_cmd_deg = 358.0f;
    }
    else if(val > 2.0f)
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