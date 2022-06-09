/**
 * @file       pid_controller.c
 * @addtogroup PID_CONTROLLER
 * @{
 */
/*Standard include*/
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

/*Self include*/
#include "pid_controller.h"

/*Other include*/
#include "saturation.h"
#include "bit_mask.h"

/*Debug include*/
#ifdef PID_DEBUG_WITH_STDIO
#include <stdio.h>
#endif

/**
 * @brief      This function help initialize @p PID_HANDLE_T.
 *
 * @param      ph     Pointer to @p PID_HANDLE_T.
 * @param      pcfgh  Pointer to @p PID_CFG_T.
 *
 * @return     Result of initialization.  
 * 
 * @retval     PID_MSG_ERR_PTR   NULL pointer detected at input.
 * @retval     PID_MSG_ERR_1     Ivalid configuration detected.
 * @retval     PID_MSG_OK        Initailze sucessfully.
 */
pid_msg_t PID_Init(PID_HANDLE_T *ph, PID_CFG_T *pcfgh)
{
  pid_msg_t result = PID_MSG_INIT;

  if((NULL == ph) || (NULL == pcfgh))
  {
    /*Invalid pointer*/
    result = PID_MSG_ERR_PTR;
  }

  if(PID_MSG_INIT == result)
  {
    /*Proceed*/
    /*Test configuration*/
    if(PID_MSG_OK != PID_CheckConfig(pcfgh))
    {
      /*Incorrect configuration*/
      result = PID_MSG_ERR_1;
    }
    else
    {
      /*Reset handle and set config*/
      /*Return value check is not needed since input is already checked*/
      PID_ResetHandle(ph); 

      ph->pcfg = pcfgh;
      result = PID_MSG_OK;
    }
  }

  return result;
}

/**
 * @brief      This function run PID controller once.
 *
 * @note       This function does not use sample period duration hence
 *             the function should be invoke with accurate period to gain good result.
 *              
 * @param      ph           Pointer to @p PID_HANDLE_T.
 * @param[in]  cmd          Command value.
 * @param[in]  act          Actual value.
 * @param[in]  out_aft_sat  Output value after saturation function.
 * @param[out] p_out        Pointer to output value.
 *
 * @return     Result of run.
 * 
 * @retval     PID_MSG_ERR_PTR   NULL pointer detected at input.
 * @retval     PID_MSG_ERR_IN    NAN or INF detected on float type arguments.
 * @retval     PID_MSG_OK        PID run sucessfully.
 * 
 */
pid_msg_t PID_RunPID(PID_HANDLE_T *ph, float cmd, float act, float out_aft_sat,float *p_out)
{
  /*Private variable*/
  pid_msg_t result = PID_MSG_INIT;
  float _err_now = cmd - act;
  bool _integral_is_clamped = false;

  if((NULL == ph) || (NULL == p_out))
  {
    /*Pointer error*/
    result = PID_MSG_ERR_PTR;
  }

  if(PID_MSG_INIT == result)
  {
    if(!isfinite(cmd) || !isfinite(act) || !isfinite(out_aft_sat))
    {
      /*Input value is NaN*/
      result = PID_MSG_ERR_IN;
    }
  }

  if(PID_MSG_INIT == result)
  {
    /*Process integral clamping*/
    if(out_aft_sat != ph->out)
    {
      /*Saturation detected*/
      /*Check for sign of output and sign of error*/
      /*Clamp if sign are identical*/
      if((ph->out > 0) && (_err_now > 0))
      {
        /*Set _integral_is_clamped*/
        _integral_is_clamped = true;
      }
      else if((ph->out <= 0) && (_err_now <= 0))
      {
        /*Set _integral_is_clamped*/
        _integral_is_clamped = true;
      }
      else
      {
        /*Do nothing*/
      }
    }

    /*Process PID*/
    ph->data.val_aft_kp = _err_now * ph->pcfg->kp;
    ph->data.val_aft_kd = ((_err_now - ph->data.err_last) * (ph->pcfg->kd));
    
    /*Run integral if clamp flag is low*/
    if(!_integral_is_clamped)
    {
      /*Continue to integral if clamped flag is low*/
      ph->data.err_sum += _err_now;
    }
    
    ph->data.val_aft_ki = ph->data.err_sum * ph->pcfg->ki;

    ph->out = (ph->data.val_aft_kp) + (ph->data.val_aft_ki) + (ph->data.val_aft_kd);
    *p_out = ph->out;

    /*Updeate err_last*/
    ph->data.err_last = _err_now;

    result = PID_MSG_OK;

/*Print debug message if macro is defined*/
#ifdef PID_DEBUG_WITH_STDIO
        /*Print out process*/
        PID_PrintData(&(ph->data));
        PID_PrintInterface(ph);
#endif
  }



  return result;
}


pid_msg_t PID_RunPIDCore(PID_HANDLE_T *ph, float err, bool output_is_saturate, float *p_out)
{
  /*Private variable*/
  pid_msg_t result = PID_MSG_INIT;

  if((NULL == ph) || (NULL == p_out))
  {
    /*Pointer error*/
    result = PID_MSG_ERR_PTR;
  }

  if(PID_MSG_INIT == result)
  {
    if(!isfinite(err))
    {
      /*Input value is NaN*/
      result = PID_MSG_ERR_IN;
    }
  }

  if(PID_MSG_INIT == result)
  {

    /*Process PID*/
    ph->data.val_aft_kp = err * ph->pcfg->kp;
    ph->data.val_aft_kd = ((err - ph->data.err_last) * (ph->pcfg->kd));
    
    /*Run integral if clamp flag is low*/
    if(!output_is_saturate)
    {
      /*Continue to integral if clamped flag is low*/
      ph->data.err_sum += err;
    }
    
    ph->data.val_aft_ki = ph->data.err_sum * ph->pcfg->ki;

    ph->out = (ph->data.val_aft_kp) + (ph->data.val_aft_ki) + (ph->data.val_aft_kd);
    *p_out = ph->out;

    /*Updeate err_last*/
    ph->data.err_last = err;

    result = PID_MSG_OK;
  }

  return result;
}

pid_msg_t PID_RunPIDExtErr(PID_HANDLE_T *ph, float err, float out_aft_sat,float *p_out)
{
  /*Private variable*/
  pid_msg_t result = PID_MSG_INIT;
  bool _integral_is_clamped = false;

  if((NULL == ph) || (NULL == p_out))
  {
    /*Pointer error*/
    result = PID_MSG_ERR_PTR;
  }

  if(PID_MSG_INIT == result)
  {
    if(!isfinite(err) || !isfinite(out_aft_sat))
    {
      /*Input value is NaN*/
      result = PID_MSG_ERR_IN;
    }
  }

  if(PID_MSG_INIT == result)
  {
    /*Process integral clamping*/
    if(out_aft_sat != ph->out)
    {
      /*Saturation detected*/
      /*Check for sign of output and sign of error*/
      /*Clamp if sign are identical*/
      if((ph->out > 0) && (err > 0))
      {
        /*Set _integral_is_clamped*/
        _integral_is_clamped = true;
      }
      else if((ph->out <= 0) && (err <= 0))
      {
        /*Set _integral_is_clamped*/
        _integral_is_clamped = true;
      }
      else
      {
        /*Do nothing*/
      }
    }

    /*Process PID*/
    ph->data.val_aft_kp = err * ph->pcfg->kp;
    ph->data.val_aft_kd = ((err - ph->data.err_last) * (ph->pcfg->kd));
    
    /*Run integral if clamp flag is low*/
    if(!_integral_is_clamped)
    {
      /*Continue to integral if clamped flag is low*/
      ph->data.err_sum += err;
    }
    
    ph->data.val_aft_ki = ph->data.err_sum * ph->pcfg->ki;

    ph->out = (ph->data.val_aft_kp) + (ph->data.val_aft_ki) + (ph->data.val_aft_kd);
    *p_out = ph->out;

    /*Updeate err_last*/
    ph->data.err_last = err;

    result = PID_MSG_OK;

/*Print debug message if macro is defined*/
#ifdef PID_DEBUG_WITH_STDIO
        /*Print out process*/
        PID_PrintData(&(ph->data));
        PID_PrintInterface(ph);
#endif
  }

  return result;
}

/**
 * @brief      Get output value of PID controller.
 * 
 * @note       Thie function will return @p 0.0 if input pointer is a NULL pointer.
 *          
 * @details    This function will return the output from @p PID_HANDLE_T.
 *             This function will check if pointer is invalid.
 *             
 * @param      ph    pointer to @p PID_HANDLE_T.
 *
 * @return     out in @p PID_HANDLE_T.
 */
float PID_GetOutput(PID_HANDLE_T *ph)
{
  if(NULL != ph)
  {
    return ph->out;
  }
  else
  {
    return 0.0f;    
  }  
}

/**
 * @brief      Check if configuration of @p PID_CFG_T is valid.
 *
 * @param      pcfgh  Pointer to @p PID_CFG_T.
 *
 * @return     Result of test.
 * 
 * @retval     PID_MSG_ERR_PTR   NULL pointer at input.
 * @retval     PID_MSG_ERR_NG    NAN or INF detected in @p PID_CFG_T.
 * @retval     PID_MSG_OK        Pass.
 */
pid_msg_t PID_CheckConfig(PID_CFG_T *pcfgh)
{
  pid_msg_t result = PID_MSG_INIT;

  if(NULL == pcfgh)
  {
    result = PID_MSG_ERR_PTR;
  }

  if(PID_MSG_INIT == result)
  {
    /*Proceed to check config content*/
    if(!isfinite(pcfgh->kp) || !isfinite(pcfgh->ki) || !isfinite(pcfgh->kd))
    {
      /*Invalid config*/
      result = PID_MSG_ERR_NG; 
    }
    else
    {
      result = PID_MSG_OK;
    }
  }

  return result;
}

/**
 * @brief      Reset intermediate data @p PID_DATA_T.
 *
 * @param      pdatah  Pointer to @p PID_DATA_T.
 *
 * @return     Result of reset.
 * 
 * @retval     PID_MSG_ERR_PTR   NULL pointer at input.
 * @retval     PID_MSG_OK        Data reset sucessfully.
 */
pid_msg_t PID_ResetData(PID_DATA_T *pdatah)
{
  pid_msg_t result = PID_MSG_INIT;

  if(NULL == pdatah)
  {
    /*Invalid pointer*/
    result = PID_MSG_ERR_PTR;
  }

  if(PID_MSG_INIT == result)
  {
    /*proceed*/
    pdatah->err_now = 0.0f;
    pdatah->err_last = 0.0f;
    pdatah->err_sum = 0.0f;
    pdatah->val_aft_kp = 0.0f;
    pdatah->val_aft_kd = 0.0f;
    pdatah->val_aft_ki = 0.0f;

    result = PID_MSG_OK;
  }

  return result;
}

/**
 * @brief      Reset pid handle @p PID_HANDLE_T.
 *
 * @param      ph    Pointer to @p PID_HANDLE_T.
 *
 * @return     Result of reset.
 * 
 * @retval     PID_MSG_ERR_PTR   NULL pointer at input.
 * @retval     PID_MSG_OK        Handle reset sucessfully.
 */
pid_msg_t PID_ResetHandle(PID_HANDLE_T *ph)
{
  pid_msg_t result = PID_MSG_INIT;

  if(NULL == ph)
  {
    /*Invalid pointer*/
    result = PID_MSG_ERR_PTR;
  }

  if(PID_MSG_INIT == result)
  {
    /*proceed*/
    ph->pcfg = NULL;
    PID_ResetData(&(ph->data));
    ph->cmd = 0.0f;
    ph->act = 0.0f;
    ph->out = 0.0f;

    result = PID_MSG_OK;
  }

  return result;
}

/**
 * @brief      This function reset signal and data in @p PID_HANDLE_T but won't reset configuration @p PID_CFG_T.
 *
 * @param      ph    Pointer to @p PID_HANDLE_T.
 *
 * @return     Result of restart.
 * 
 * @retval     PID_MSG_ERR_PTR   NULL pointer at input.
 * @retval     PID_MSG_OK        Restart sucessfully.
 */
pid_msg_t PID_Restart(PID_HANDLE_T *ph)
{
  pid_msg_t result = PID_MSG_INIT;

  if(NULL == ph)
  {
    /*Invalid pointer*/
    result = PID_MSG_ERR_PTR;
  }

  if(PID_MSG_INIT == result)
  {
    /*proceed*/
    PID_ResetData(&(ph->data));
    ph->cmd = 0.0f;
    ph->act = 0.0f;
    ph->out = 0.0f;

    result = PID_MSG_OK;
  }

  return result;
}

/*Definition of debug related funcitons*/
#ifdef PID_DEBUG_WITH_STDIO
void PID_PrintHandle(PID_HANDLE_T *ph)
{
  if(NULL == ph)
  {
    printf("!! Invalid pointer address of pointer to PID_HANDLE_T !!\n");
  }
  else
  {
    PID_PrintCfg(ph->pcfg);
    PID_PrintData(&(ph->data));
    PID_PrintInterface(ph);
  }
}

void PID_PrintCfg(PID_CFG_T *ph)
{
  if(NULL == ph)
  {
    printf("!! Invalid pointer address of pointer to PD_CFG_T !!\n");
  }
  else
  {
    printf("cfg1 is %d\n", ph->cfg1);
    printf("kp is %5.5f\n", ph->kp);
    printf("kd is %5.5f\n", ph->kd);
  }
}

void PID_PrintData(PID_DATA_T *ph)
{
  if(NULL == ph)
  {
    printf("!! Invalid pointer address of pointer to PD_DATA_T !!\n");
  }
  else
  {
    printf("err_now is: %5.5f\n",ph->err_now);
    printf("err_last is: %5.5f\n",ph->err_last);
    printf("err_sum is: %5.5f\n",ph->err_sum);
    printf("val_aft_kp is: %5.5f\n",ph->val_aft_kp);
    printf("val_aft_ki is: %5.5f\n",ph->val_aft_ki);
    printf("val_aft_kd is: %5.5f\n",ph->val_aft_kd);
  }
}

void PID_PrintInterface(PID_HANDLE_T *ph)
{
  if(NULL == ph)
  {
    printf("!! Invalid pointer address of pointer to PID_HANDLE_T !!\n");
  }
  else
  {
    printf("cmd is:%5.5f\n",ph->cmd);
    printf("act is:%5.5f\n",ph->act);
    printf("out is:%5.5f\n\n",ph->out);
  }
}
#endif

/**
 * @}
 */
