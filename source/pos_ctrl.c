/*Standard include*/
#include <stdbool.h>
#include <string.h>
#include <math.h>

/*Self include*/
#include "pos_ctrl.h"

/*Module include*/
#include "saturation.h"

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

bool POSC_CalcDirection(bool dir_curr, pos_u16t cmd, pos_u16t act, pos_u16t thold)
{
  if(POSC_CalcPerr(dir_curr, cmd, act) > thold)
  {
    dir_curr = !dir_curr;
  }

  return dir_curr;
}

float POSC_Run(POSC_CMD_HANDLE_T *ppccmdh, POSC_CFG_HANDLE_T *ppccfgh, pos_u16t act)
{
  /*Declare private variables*/
  float retval = 0.0f;

  //POSC_CMD_HANDLE_T _priv_pccmdh = DFLT_INIT_POSC_CMD_HANDLE_T();
  pos_u16t _priv_perr;


  if((NULL != ppccmdh) && (NULL != ppccfgh))
  {
    /*Proceed*/
    /*Copy data*/
    //chSysLock();
    //memcpy(&_priv_pccmdh, ppccmdh, sizeof(POSC_CMD_HANDLE_T));
    //chSysUnlock();

    ppccmdh->direction_cmd = POSC_CalcDirection(ppccmdh->direction_cmd, ppccmdh->pos_cmd_u16, act, POSC_POSU16_180DEG);

    _priv_perr = POSC_CalcPerr(ppccmdh->direction_cmd, ppccmdh->pos_cmd_u16, act);


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
pos_u16t POSC_ConvertDeg2U16(float degree)
{
  pos_u16t retval = 0;

  if(isfinite(degree))
  {
    retval = (pos_u16t)((float)POSC_POSU16_MAX * degree/POSC_DEG_MAX);
  }

  return retval;
}

float POSC_ConvertU162Deg(pos_u16t degree_u16)
{
  float retval = 0.0f;

  retval = POSC_DEG_MAX * (float)degree_u16/(float)POSC_POSU16_MAX;

  return retval;
}

/*Set and get functions*/
void POSC_SetCfgPErrThold(POSC_CFG_HANDLE_T *ppccfgh, float val)
{
  if((NULL != ppccfgh) && isfinite(val))
  {
    val = SAT_fSat(val, POSC_DEG_MIN, POSC_DEG_MAX);
    ppccfgh->pos_err_thold_u16 = POSC_ConvertDeg2U16(val);
  }  
}

float POSC_GetCfgPErrThold(POSC_CFG_HANDLE_T *ppccfgh)
{
  float retval = 0.0f;

  if(NULL != ppccfgh)
  {
    retval = POSC_ConvertU162Deg(ppccfgh->pos_err_thold_u16);
  }
  return retval;
}


void POSC_SetCfgSCmdMin(POSC_CFG_HANDLE_T *ppccfgh, float val)
{
  if(NULL != ppccfgh && isfinite(val))
  {
    ppccfgh->s_cmd_min = val;
  }
  
}

float POSC_GetCfgSCmdMin(POSC_CFG_HANDLE_T *ppccfgh)
{
  float retval = 0.0f;

  if(NULL != ppccfgh)
  {
    retval = ppccfgh->s_cmd_min;
  }
  return retval;
}

void POSC_SetCfgSCmdMax(POSC_CFG_HANDLE_T *ppccfgh, float val)
{
  if(NULL != ppccfgh && isfinite(val))
  {
    ppccfgh->s_cmd_max = val;
  }
  
}

float POSC_GetCfgSCmdMax(POSC_CFG_HANDLE_T *ppccfgh)
{
  float retval = 0.0f;

  if(NULL != ppccfgh)
  {
    retval = ppccfgh->s_cmd_max;
  }
  return retval;
}

void POSC_SetCfgKp(POSC_CFG_HANDLE_T *ppccfgh, float val)
{
  if(NULL != ppccfgh && isfinite(val))
  {
    ppccfgh->kp = val;
  }
  
}

float POSC_GetCfgKp(POSC_CFG_HANDLE_T *ppccfgh)
{
  float retval = 0.0f;

  if(NULL != ppccfgh)
  {
    retval = ppccfgh->kp;
  }
  return retval;
}