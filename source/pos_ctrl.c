/**
 * @file       pos_ctrl.c
 * @addtogroup POS_CTRL
 * @{
 */
/*Standard include*/
#include <stdbool.h>
#include <string.h>
#include <math.h>

/*Self include*/
#include "pos_ctrl.h"

/*Module include*/
#include "dual_motor_ctrl/saturation.h"
#include "dual_motor_ctrl/pid_controller.h"


void POSC_Init( POSC_HANDLE_T *ppch, 
                float _pos_err_thold_u16, 
                float _s_cmd_min, 
                float _s_cmd_max,
                float _kp,
                float _ki,
                float _kd)
{
  if(NULL != ppch && isfinite(_pos_err_thold_u16) && isfinite(_s_cmd_min) && isfinite(_s_cmd_max) && isfinite(_kp) && isfinite(_ki) && isfinite(_kd))
  {
    /*Init POSC_CFG_HANDLE_T*/
    ppch->cfg.pos_err_thold_u16 = _pos_err_thold_u16;
    ppch->cfg.s_cmd_min = _s_cmd_min;
    ppch->cfg.s_cmd_max = _s_cmd_max;

    /*Init PID_CFG_T*/
    ppch->pid_cfg.cfg1 = 0;
    ppch->pid_cfg.kp = _kp;
    ppch->pid_cfg.ki = _ki;
    ppch->pid_cfg.kd = _kd;
    
    /*Init PID_HANDLE_T*/
    (void)PID_Init(&(ppch->pid), &(ppch->pid_cfg));

    /*Init POSC_CMD_HANDLE_T*/
    ppch->cmd.pos_cmd_u16 = 0U;
    ppch->cmd.direction_cmd = true;

    /*Reset IO value*/
    ppch->perr = 0U;
    ppch->speed_cmd_rpm = 0.0f;
    
    ppch->last_cmd = 0;
    ppch->last_pos = 0;
    ppch->dir_changed = 0;
    ppch->posIndex = 0;
  }
}

void POSC_Restart(POSC_HANDLE_T *ppch)
{
  if(NULL != ppch)
  {
    (void)PID_Restart(&ppch->pid);
  }
}

/**
 * @brief      This function will calculate position error.
 *
 * @param[in]  dir   Direction
 * @param[in]  cmd   Command position value
 * @param[in]  act   Actual position value
 *
 * @return     Position error
 */
pos_u16t POSC_CalcPerr(bool dir, pos_u16t cmd, pos_u16t act)
{
  pos_u16t retval;
  static int32_t err;
  static pos_u16t t1,t2;
  t1 = cmd;
  t2 = act;

  if(dir)
  {
    //err = (cmd > act)?(cmd - act):((0xffff - act) + cmd); 
    retval = cmd - act;
  }
  else
  {
//    err = (act > cmd)?(act - cmd):((0xffff - cmd) + act);
    retval = act - cmd;
  }

//  retval = (pos_u16t)err;
//  if(retval > 32768){
//    uint16_t tmp = retval;
//  }
  
  
  
  return retval;
}

/**
 * @brief      This function will calculate target direction.
 *
 * @param[in]  dir_curr  Current target direction.
 * @param[in]  cmd       Command position value
 * @param[in]  act       Actual position value
 * @param[in]  thold     Threshold for target direction changed.
 *
 * @return     New terget direction
 */
bool POSC_CalcDirection(bool dir_curr, pos_u16t cmd, pos_u16t act, pos_u16t thold)
{
  if(POSC_CalcPerr(dir_curr, cmd, act) > thold)
  {
    dir_curr = !dir_curr;
  }

  return dir_curr;
}

/**
 * @brief      Run position controller once.
 *
 * @param      ppccmdh  Pointer to POSC_CMD_HANDLE_T
 * @param      ppccfgh  Pointer to POSC_CFG_HANDLE_T
 * @param[in]  act      Actual position value
 *
 * @return     Platform target speed in rpm.
 */
  static uint16_t ttt;
//float POSC_Run(POSC_CMD_HANDLE_T *ppccmdh, POSC_CFG_HANDLE_T *ppccfgh, pos_u16t act)
float POSC_Run(POSC_HANDLE_T *ppch, pos_u16t act)
{
  ttt = act;
  /*Declare private variables*/
  float retval = 0.0f;
  //float _speeed_cmd_rpm = 0.0f;
  //bool  _output_is_saturate = false;

  //POSC_CMD_HANDLE_T _priv_pccmdh = DFLT_INIT_POSC_CMD_HANDLE_T();
  //pos_u16t _perr;


  if(NULL != ppch)
  {
    /*Proceed*/
    ppch->perr = POSC_CalcPerr(ppch->cmd.direction_cmd, ppch->cmd.pos_cmd_u16, act);

    /*Run direction calculator to handle possible overshoot if ppch->perr is within range.*/
//    if((ppch->perr < POSC_POSU16_45DEG) || (ppch->perr > POSC_POSU16_315DEG))
//    {
//      ppch->cmd.direction_cmd = POSC_CalcDirection(ppch->cmd.direction_cmd, ppch->cmd.pos_cmd_u16, act, POSC_POSU16_180DEG);
//      ppch->perr = POSC_CalcPerr(ppch->cmd.direction_cmd, ppch->cmd.pos_cmd_u16, act);
//    }
    
//    ppch->pos_hist[ppch->posIndex & 0x1F] = act;
//    ppch->dir_hist[ppch->posIndex++ & 0x1F] = ppch->cmd.direction_cmd?1:0;

    if(ppch->perr > POSC_POSU16_180DEG){
      ppch->cmd.direction_cmd = !ppch->cmd.direction_cmd;
      ppch->perr = 0xFFFF - ppch->perr;
      ppch->last_pos = act;
      ppch->last_cmd = ppch->cmd.pos_cmd_u16;
      ppch->dir_changed++;
      ppch->last_err = ppch->perr;
      ppch->last_pos_hist[ppch->posIndex & 0x1F] = act;
      ppch->last_dir_hist[ppch->posIndex++ & 0x1F] = ppch->cmd.direction_cmd?1:0;
      //memcpy(ppch->last_pos_hist, ppch->pos_hist, 64);
      //memcpy(ppch->last_dir_hist, ppch->dir_hist,32);
    }

    if(ppch->perr >= ppch->cfg.pos_err_thold_u16)
    {
      (void)PID_RunPIDExtErr(&(ppch->pid), ((float)ppch->perr * POSC_PREGAIN_1), ppch->speed_cmd_rpm, &(ppch->speed_cmd_rpm));

      //retval = ((float)_perr * POSC_PREGAIN_1 * (ppccfgh->kp)) + ppccfgh->s_cmd_min;
      ppch->speed_cmd_rpm = SAT_fSat(ppch->speed_cmd_rpm, ppch->cfg.s_cmd_min, ppch->cfg.s_cmd_max);
      retval = ppch->speed_cmd_rpm;

      if(!ppch->cmd.direction_cmd)
      {+
        
        /*Negative direction*/
        retval *= -1.0f;
      }
    }
    else
    {
      /*Reset PID if error within threshold*/
      //(void)PID_Restart(&(ppch->pid));
    }
  }

  return retval;
}

/*Helper functions*/
/**
 * @brief      This function convert angle form degree to pos_u16t.
 *
 * @param[in]  degree  Angle in degree
 *
 * @return     Angle in pos_u16t format
 */
pos_u16t POSC_ConvertDeg2U16(float degree)
{
  pos_u16t retval = 0;

  if(isfinite(degree))
  {
    retval = (pos_u16t)((float)POSC_POSU16_MAX * degree/POSC_DEG_MAX);
  }

  return retval;
}

/**
 * @brief      This function convert angle form pos_u16t to degree.
 *
 * @param[in]  degree_u16  Angle in pos_u16t format
 *
 * @return     Angle in degree
 */
float POSC_ConvertU162Deg(pos_u16t degree_u16)
{
  float retval = 0.0f;

  retval = POSC_DEG_MAX * (float)degree_u16/(float)POSC_POSU16_MAX;

  return retval;
}

/*Set and get functions*/
/**
 * @brief      This function set @p pos_err_thold_u16 within @p POSC_CFG_HANDLE_T
 *
 * @param      ppccfgh  Pointer to @p POSC_CFG_HANDLE_T
 * @param[in]  val      Value to be set
 */
void POSC_SetCfgPErrThold(POSC_CFG_HANDLE_T *ppccfgh, float val)
{
  if((NULL != ppccfgh) && isfinite(val))
  {
    val = SAT_fSat(val, POSC_DEG_MIN, POSC_DEG_MAX);
    ppccfgh->pos_err_thold_u16 = POSC_ConvertDeg2U16(val);
  }  
}

/**
 * @brief      This function get @p pos_err_thold_u16 within @p POSC_CFG_HANDLE_T
 *
 * @param      ppccfgh  Pointer to @p POSC_CFG_HANDLE_T
 *
 * @return     Current setup value
 */
float POSC_GetCfgPErrThold(POSC_CFG_HANDLE_T *ppccfgh)
{
  float retval = 0.0f;

  if(NULL != ppccfgh)
  {
    retval = POSC_ConvertU162Deg(ppccfgh->pos_err_thold_u16);
  }
  return retval;
}

/**
 * @brief      This function set @p s_cmd_min within @p POSC_CFG_HANDLE_T
 *
 * @param      ppccfgh  Pointer to @p POSC_CFG_HANDLE_T
 * @param[in]  val      Value to be set
 */
void POSC_SetCfgSCmdMin(POSC_CFG_HANDLE_T *ppccfgh, float val)
{
  if(NULL != ppccfgh && isfinite(val))
  {
    ppccfgh->s_cmd_min = val;
  }
  
}

/**
 * @brief      This function get @p s_cmd_min within @p POSC_CFG_HANDLE_T
 *
 * @param      ppccfgh  Pointer to @p POSC_CFG_HANDLE_T
 *
 * @return     Current setup value
 */
float POSC_GetCfgSCmdMin(POSC_CFG_HANDLE_T *ppccfgh)
{
  float retval = 0.0f;

  if(NULL != ppccfgh)
  {
    retval = ppccfgh->s_cmd_min;
  }
  return retval;
}

/**
 * @brief      This function set @p s_cmd_max within @p POSC_CFG_HANDLE_T
 *
 * @param      ppccfgh  Pointer to @p POSC_CFG_HANDLE_T
 * @param[in]  val      Value to be set
 */
void POSC_SetCfgSCmdMax(POSC_CFG_HANDLE_T *ppccfgh, float val)
{
  if(NULL != ppccfgh && isfinite(val))
  {
    ppccfgh->s_cmd_max = val;
  }
  
}

/**
 * @brief      This function get @p s_cmd_max within @p POSC_CFG_HANDLE_T
 *
 * @param      ppccfgh  Pointer to @p POSC_CFG_HANDLE_T
 *
 * @return     Current setup value
 */
float POSC_GetCfgSCmdMax(POSC_CFG_HANDLE_T *ppccfgh)
{
  float retval = 0.0f;

  if(NULL != ppccfgh)
  {
    retval = ppccfgh->s_cmd_max;
  }
  return retval;
}

/**
 * @brief      This function set @p kp within @p POSC_CFG_HANDLE_T
 *
 * @param      ppccfgh  Pointer to @p POSC_CFG_HANDLE_T
 * @param[in]  val      Value to be set
 */
void POSC_SetCfgKp(POSC_CFG_HANDLE_T *ppccfgh, float val)
{
  if(NULL != ppccfgh && isfinite(val))
  {
    ppccfgh->kp = val;
  }
  
}

/**
 * @brief      This function get @p kp within @p POSC_CFG_HANDLE_T
 *
 * @param      ppccfgh  Pointer to @p POSC_CFG_HANDLE_T
 *
 * @return     Current setup value
 */
float POSC_GetCfgKp(POSC_CFG_HANDLE_T *ppccfgh)
{
  float retval = 0.0f;

  if(NULL != ppccfgh)
  {
    retval = ppccfgh->kp;
  }
  return retval;
}

/**
 * @}
 */
