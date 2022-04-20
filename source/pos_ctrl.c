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

  if(dir)
  {
    retval = cmd - act;
  }
  else
  {
    retval = act - cmd;
  }

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
float POSC_Run(POSC_CMD_HANDLE_T *ppccmdh, POSC_CFG_HANDLE_T *ppccfgh, pos_u16t act)
{
  /*Declare private variables*/
  float retval = 0.0f;

  //POSC_CMD_HANDLE_T _priv_pccmdh = DFLT_INIT_POSC_CMD_HANDLE_T();
  pos_u16t _priv_perr;


  if((NULL != ppccmdh) && (NULL != ppccfgh))
  {
    /*Proceed*/
    _priv_perr = POSC_CalcPerr(ppccmdh->direction_cmd, ppccmdh->pos_cmd_u16, act);

    /*Run direction calculator to handle possible overshoot if _priv_perr is within range.*/
    if((_priv_perr < POSC_POSU16_45DEG) || (_priv_perr > POSC_POSU16_315DEG))
    {
      ppccmdh->direction_cmd = POSC_CalcDirection(ppccmdh->direction_cmd, ppccmdh->pos_cmd_u16, act, POSC_POSU16_180DEG);
      _priv_perr = POSC_CalcPerr(ppccmdh->direction_cmd, ppccmdh->pos_cmd_u16, act);
    }

    if(_priv_perr >= ppccfgh->pos_err_thold_u16)
    {
      retval = ((float)_priv_perr * POSC_PREGAIN_1 * (ppccfgh->kp)) + ppccfgh->s_cmd_min;
      retval = SAT_fSat(retval, ppccfgh->s_cmd_min, ppccfgh->s_cmd_max);

      if(!ppccmdh->direction_cmd)
      {
        /*Negative direction*/
        retval *= -1.0f;
      }
    }

    return retval;
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
