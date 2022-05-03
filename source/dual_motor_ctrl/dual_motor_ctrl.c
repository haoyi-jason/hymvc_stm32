/**
 * @file       dual_motor_ctrl.c
 * @addtogroup DUAL_MOTOR_CTRL
 * @{
 */
/*Standard include*/
#include <stdint.h>
#include <string.h>
#include <math.h>

/*Self include*/
#include "dual_motor_ctrl.h"

/*Module include*/
#include "pid_controller.h"
#include "tq_bias_calc.h"
#include "saturation.h"

/*Other include*/

/*Function*/


/**
 * @brief      This is a wrapper function of TQBC_SetConfig()
 * 
 * @note       @p dmotc_msg_t and @p tqbc_msg_t have same data definition hence they can 
 *             be cast to each other witout additional mapping.
 *
 * @param      pcfgh             Pointer to @p TQBC_CFG_T.
 * @param[in]  out_max           Upper boundry of output value to be set.
 * @param[in]  out_min           Lower boundry of output value to be set.
 * @param[in]  gain              Gain of linear function to be set.
 * @param[in]  zero_cross_point  Zero cross point f(x) = 0 to be set.
 *
 * @return     Result of setup.
 * 
 * @retval     DMOTC_MSG_ERR_PTR   NULL pointer detected at input.
 * @retval     DMOTC_MSG_ERR_IN    Ivalid configuration detected.
 * @retval     DMOTC_MSG_ERR_1     Incorrect boundry relationship (@p out_max and @p out_min).
 * @retval     DMOTC_MSG_ERR_2     Incorrect gain value.
 * @retval     DMOTC_MSG_ERR_3     Incorrect bias value.
 * @retval     DMOTC_MSG_OK        Check pass.
 * 
 */
dmotc_msg_t DMOTC_SetTQBCConfig(TQBC_CFG_T *pcfgh,
                                float out_max, 
                                float out_min, 
                                float gain, 
                                float zero_cross_point)
{
  tqbc_msg_t result = TQBC_SetConfig(pcfgh, out_max, out_min, gain, zero_cross_point);
  return (dmotc_msg_t)result;
}

/**
 * @brief      This function tend to setup all necessary componment to run the algorithm.
 * 
 * @details    This function will set state to @b STOP. 
 *
 * @param      ph            Pointer to handle of dual motor control.
 * @param      pspidcfgh     Pointer to speed control pid configuration.
 * @param      ptqbccfgh     Pointer to torque bias calculator configuration.
 * @param[in]  gain_atq2mtq  Gain "axis torque to motor torque" to be set.
 * @param[in]  mot_tq_min    Motor output torque lower limit boundry.
 * @param[in]  mot_tq_max    Motor output torque upper limit boundry.
 *
 * @return     Result of initialization.  
 * 
 * @retval     DMOTC_MSG_OK         Initialize successfully.
 * @retval     DMOTC_MSG_ERR_PTR    NULL pointer detected at input.
 * @retval     DMOTC_MSG_ERR_IN     NAN or INF detected on float type arguments.
 * @retval     DMOTC_MSG_ERR_STATE  Incorrect state, state should be @p DMOTC_STATE_INIT or @p DMOTC_STATE_STOP.
 * @retval     DMOTC_MSG_ERR_1      Speed PID controller initiation failed.
 * @retval     DMOTC_MSG_ERR_2      Torque bias calculator initiation failed.
 * @retval     DMOTC_MSG_ERR_3      Gain "axis torque to motor torque" setup failed.
 * @retval     DMOTC_MSG_ERR_4      Output limiter setup failed.
 */
dmotc_msg_t DMOTC_Init(DMOTC_HANDLE_T *ph, 
                       PID_CFG_T *pspidcfgh, 
                       TQBC_CFG_T *ptqbccfgh, 
                       float gain_atq2mtq, 
                       float mot_tq_min, 
                       float mot_tq_max)
{
  dmotc_msg_t result = DMOTC_MSG_INIT;

  if((NULL == ph) || (NULL == pspidcfgh) || (NULL == ptqbccfgh))
  {
    /*Invalid pointer*/
    result = DMOTC_MSG_ERR_PTR;
  }

  if(!isfinite(gain_atq2mtq) || !isfinite(mot_tq_min) || !isfinite(mot_tq_max))
  {
    /*Invalid input*/
    result = DMOTC_MSG_ERR_IN;
  }

  if(DMOTC_MSG_INIT == result)
  {
    if(DMOTC_STATE_RUN == ph->state)
    {
      /*Incorrect state*/
      result = DMOTC_MSG_ERR_STATE;
    }
    else
    {
      if(PID_MSG_OK != PID_Init(&(ph->axis_s_pid), pspidcfgh))
      {
        /*Speed PID controller initiation failed*/
        result = DMOTC_MSG_ERR_1;
      }
      else if(TQBC_MSG_OK != TQBC_Init(&(ph->tqbc), ptqbccfgh))
      {
        /*Torque bias calculator initiation failed*/
        result = DMOTC_MSG_ERR_2;
      }
      else if(DMOTC_MSG_OK != DMOTC_SetGain(ph, DMOTC_GAIN_ID_ATQ2MTQ, gain_atq2mtq))
      {
        /*Gain setup failed*/
        result = DMOTC_MSG_ERR_3;
      }
      else if(DMOTC_MSG_OK != DMOTC_SetTqBoud(ph, mot_tq_min, mot_tq_max))
      {
        /*Output limiter setup failed*/
        result = DMOTC_MSG_ERR_4;
      }
      else
      {
        /*Setup complete without error*/
        /*Set state to DMOTC_STATE_STOP*/
        ph->state = DMOTC_STATE_STOP;
        result = DMOTC_MSG_OK;
      }
    }
  }

  return result;
}


/**
 * @brief      This function will try to start the algorithm.
 * 
 * @details    This function will try to start the algorithm by reset intermidate signals and I/O signals
 *             and set state to @b RUN.  
 *
 * @param      ph  Pointer to handle of dual motor control.
 *
 * @return     Result of start.
 * 
 * @retval     DMOTC_MSG_OK         Initialize successfully.
 * @retval     DMOTC_MSG_ERR_PTR    NULL pointer detected at input.
 * @retval     DMOTC_MSG_ERR_STATE  Incorrect state, state should be @p DMOTC_STATE_STOP.
 * 
 */
dmotc_msg_t DMOTC_Start(DMOTC_HANDLE_T *ph)
{
  dmotc_msg_t result = DMOTC_MSG_INIT;

  if(NULL == ph)
  {
    /*Invalid pointer*/
    result = DMOTC_MSG_ERR_PTR;
  }

  if(DMOTC_MSG_INIT == result)
  {
    /*Check if state is DMOTC_STATE_STOP*/
    if(DMOTC_STATE_STOP != ph->state)
    {
      /*Incorrect state*/
      result = DMOTC_MSG_ERR_STATE;
    }
    else
    {
      /*Reset necessary data*/
      /*No need to check return value since pionter has been checked previously*/
      DMOTC_ResetData(&(ph->data));
      PID_Restart(&(ph->axis_s_pid));
      TQBC_Restart(&(ph->tqbc));

      /*Set state to DMOTC_STATE_RUN*/
      ph->state = DMOTC_STATE_RUN;
      
      result = DMOTC_MSG_OK;
    }
  }

  return result;
}

/**
 * @brief      This function will run algorithm once.
 *
 * @param      ph            Pointer to handle of dual motor control.
 * @param[in]  axis_s_set    Axis speed command.
 * @param[in]  axis_s_act    Axis speed actual.
 * @param[out] pmot1_tq_set  Motor 1 torque output.
 * @param[out] pmot2_tq_set  Motor 2 torque output.
 *
 * @return     Result of run.
 * 
 * @retval     DMOTC_MSG_OK         Run successfully.
 * @retval     DMOTC_MSG_ERR_PTR    NULL pointer detected at input.
 * @retval     DMOTC_MSG_ERR_IN     NAN or INF detected on float type arguments.
 * @retval     DMOTC_MSG_ERR_STATE  Incorrect state, state should be @p DMOTC_STATE_RUN.
 * 
 */
dmotc_msg_t DMOTC_Run(DMOTC_HANDLE_T *ph,
                      float axis_s_set, 
                      float axis_s_act,
                      float *pmot1_tq_set,
                      float *pmot2_tq_set)
{
  dmotc_msg_t result = DMOTC_MSG_INIT;
  float _mot_tq_set_bef_bias = 0.0f;

  if((NULL == ph) || (NULL == pmot1_tq_set) || (NULL == pmot2_tq_set))
  {
    /*Invalid pointer*/
    result = DMOTC_MSG_ERR_PTR;
  }

  if(!isfinite(axis_s_set) || !isfinite(axis_s_act))
  {
    /*Invalid input*/
    result = DMOTC_MSG_ERR_IN;
  }

  if(DMOTC_MSG_INIT == result)
  {
    if(DMOTC_STATE_RUN != ph->state)
    {
      /*Incorrect state*/
      result = DMOTC_MSG_ERR_STATE;
    }
    else
    {
      /*Run algorithm once*/
      /*Set input*/
      ph->data.axis_s_set = axis_s_set;
      ph->data.axis_s_act = axis_s_act;

      PID_RunPID(&(ph->axis_s_pid), ph->data.axis_s_set, ph->data.axis_s_act, ph->data.axis_tq_cmd, &(ph->data.axis_tq_cmd));      
      TQBC_Run(&(ph->tqbc), ph->data.axis_tq_cmd, &(ph->data.mot_tq_bias));
      ph->data.axis_tq_cmd = SAT_fSat(ph->data.axis_tq_cmd, (ph->mot_tq_min)/(ph->gain_atq2mtq), (ph->mot_tq_max)/(ph->gain_atq2mtq));
      _mot_tq_set_bef_bias = ph->data.axis_tq_cmd * ph->gain_atq2mtq;
      ph->data.mot1_tq_set = SAT_fSat(_mot_tq_set_bef_bias - ph->data.mot_tq_bias, ph->mot_tq_min, ph->mot_tq_max);
//      ph->data.mot1_tq_set = SAT_fSat(_mot_tq_set_bef_bias + ph->data.mot_tq_bias, ph->mot_tq_min, ph->mot_tq_max);
      ph->data.mot2_tq_set = SAT_fSat(_mot_tq_set_bef_bias + ph->data.mot_tq_bias, ph->mot_tq_min, ph->mot_tq_max);
//      ph->data.mot1_tq_set = SAT_fSat(_mot_tq_set_bef_bias, ph->mot_tq_min, ph->mot_tq_max);
//      ph->data.mot2_tq_set = SAT_fSat(_mot_tq_set_bef_bias, ph->mot_tq_min, ph->mot_tq_max);


      /*Assign output*/
      *pmot1_tq_set = ph->data.mot1_tq_set;
      *pmot2_tq_set = ph->data.mot2_tq_set;

      result = DMOTC_MSG_OK;
    }
  }

  return result;
}


/**
 * @brief      This function will try to set state to @b STOP.
 * 
 * @note       This function will @b not reset intermidate signals nor I/O signals.
 *
 * @param      ph            Pointer to handle of dual motor control.
 *
 * @return     Result of stop.
 * 
 * @retval     DMOTC_MSG_OK         Stop successfully.
 * @retval     DMOTC_MSG_ERR_PTR    NULL pointer detected at input.
 * @retval     DMOTC_MSG_ERR_STATE  Incorrect state, state should be @p DMOTC_STATE_RUN or @p DMOTC_STATE_STOP.
 */
dmotc_msg_t DMOTC_Stop(DMOTC_HANDLE_T *ph)
{
  dmotc_msg_t result = DMOTC_MSG_INIT;

  if(NULL == ph)
  {
    /*Invalid pointer*/
    result = DMOTC_MSG_ERR_PTR;
  }

  if(DMOTC_MSG_INIT == result)
  {
    if(DMOTC_STATE_INIT== ph->state)
    {
      /*Incorrect state*/
      result = DMOTC_MSG_ERR_STATE;
    }
    else
    {
      /*Set state to DMOTC_STATE_STOP*/
      ph->state = DMOTC_STATE_STOP;
      /*Handle will be reset when invoking DMOTC_Start()*/
      result = DMOTC_MSG_OK;
    }
  }

  return result;
}


/**
 * @brief      This function will try to set torque limit boundry.
 *
 * @param      ph            Pointer to handle of dual motor control.
 * @param[in]  mot_tq_min    Motor output torque lower limit boundry.
 * @param[in]  mot_tq_max    Motor output torque upper limit boundry.
 *
 * @return     Result of set.
 * 
 * @retval     DMOTC_MSG_OK         Set successfully.
 * @retval     DMOTC_MSG_ERR_PTR    NULL pointer detected at input.
 * @retval     DMOTC_MSG_ERR_IN     NAN or INF detected on float type arguments.
 * 
 */
dmotc_msg_t DMOTC_SetTqBoud(DMOTC_HANDLE_T *ph, float mot_tq_min, float mot_tq_max)
{
  dmotc_msg_t result = DMOTC_MSG_INIT;

  if(NULL == ph)
  {
    /*Invalid pointer*/
    result = DMOTC_MSG_ERR_PTR;
  }

  if(DMOTC_MSG_INIT == result)
  {
    /*Proceed*/
    if(!isfinite(mot_tq_min) || !isfinite(mot_tq_max))
    {
      /*Invalid input*/
      result = DMOTC_MSG_ERR_IN;
    }
    else if(mot_tq_min >= mot_tq_max)
    {
      /*Invalid boundry*/
      result = DMOTC_MSG_ERR_1;
    }
    else
    {
      /*Proceed*/
      ph->mot_tq_min = mot_tq_min;
      ph->mot_tq_max = mot_tq_max;

      result = DMOTC_MSG_OK;
    }
  }
  return result;
}

/**
 * @brief      This function will try to set gain within algorithm according to the index value.
 *
 * @param      ph       Pointer to handle of dual motor control.
 * @param[in]  gain_no  Index number of gain.
 * @param[in]  val      Value to be set.
 *
 * @return     Result of set.
 * 
 * @retval     DMOTC_MSG_OK         Set successfully.
 * @retval     DMOTC_MSG_ERR_PTR    NULL pointer detected at input.
 * @retval     DMOTC_MSG_ERR_IN     NAN or INF detected on float type arguments.
 * @retval     DMOTC_MSG_ERR_OTHER  Incorrect gain index.
 * @retval     DMOTC_MSG_ERR_1      Gain "axis torque to motor torque" set value out of range.
 */
dmotc_msg_t DMOTC_SetGain(DMOTC_HANDLE_T *ph, dmotc_gain_no gain_no, float val)
{
  dmotc_msg_t result = DMOTC_MSG_INIT;

  if(NULL == ph)
  {
    /*Invalid pointer*/
    result = DMOTC_MSG_ERR_PTR;
  }

  if(DMOTC_MSG_INIT == result)
  {
    /*Proceed*/
    if(!isfinite(val))
    {
      result = DMOTC_MSG_ERR_IN;
    }
    else
    {
      switch (gain_no)
      {
        case DMOTC_GAIN_ID_ATQ2MTQ:
        /*Gain should be a positive non zero number*/
        if(val > 0.0f)
        {
          result = DMOTC_MSG_OK;
          ph->gain_atq2mtq = val;
        }
        else
        {
          result = DMOTC_MSG_ERR_1;
        }
        break;

        default:
        result = DMOTC_MSG_ERR_OTHER;
        break;
      }
    }
  }

  return result;
}

/**
 * @brief      This function will try to reset intermidate data and I/O signals used by algorithm.
 *
 * @param      pdatah  Pointer to data used by dual motor controller.
 *
 * @return     Result of reset.
 * 
 * @retval     DMOTC_MSG_OK         Set successfully.
 * @retval     DMOTC_MSG_ERR_PTR    NULL pointer detected at input.
 */
dmotc_msg_t DMOTC_ResetData(DMOTC_DATA_T *pdatah)
{
  dmotc_msg_t result = DMOTC_MSG_INIT;

  if(NULL == pdatah)
  {
    /*Invalid pointer*/
    result = DMOTC_MSG_ERR_PTR;
  }

  if(DMOTC_MSG_INIT == result)
  {
    pdatah->axis_s_set = 0.0f;
    pdatah->axis_s_act = 0.0f;
    pdatah->axis_tq_cmd = 0.0f;
    pdatah->mot_tq_bias = 0.0f;
    pdatah->mot1_tq_set = 0.0f;
    pdatah->mot2_tq_set = 0.0f;

    result = DMOTC_MSG_OK;
  }

  return result;
}

/**
 * @}
 */
