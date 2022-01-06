/**
 * @file       tq_bias_calc.c
 * @addtogroup TQ_BIAS_CALC
 * @{
 */
/*Standard include*/
#include <stdint.h>
#include <math.h>
#include <string.h>

/*Self include*/
#include "tq_bias_calc.h"

/*Module include*/
#include "saturation.h"

/**
 * @brief      This function help initialize @p TQBC_HANDLE_T.
 *
 * @param      ph     Pointer to @p TQBC_HANDLE_T.
 * @param      pcfgh  Pointer to @p TQBC_CFG_T.
 *
 * @return     Result of initialization. 
 * 
 * @retval     TQBC_MSG_ERR_PTR   NULL pointer detected at input.
 * @retval     TQBC_MSG_ERR_IN    Ivalid configuration detected.
 * @retval     TQBC_MSG_OK        Initailze sucessfully.
 */
tqbc_msg_t TQBC_Init(TQBC_HANDLE_T *ph, TQBC_CFG_T *pcfgh)
{
  tqbc_msg_t result = TQBC_MSG_INIT;

  if((NULL == ph) || (NULL == pcfgh))
  {
    /*Invalid pointer*/
    result = TQBC_MSG_ERR_PTR;
  }
  else if(TQBC_MSG_OK != TQBC_CheckConfig(pcfgh))
  {
    /*Incorrect configuration*/
    result = TQBC_MSG_ERR_IN;
  }
  else
  {
    /*Proceed*/
    /*Reset the handle*/
    TQBC_Restart(ph);
    /*Set pcfgh*/
    ph->pcfg = pcfgh;

    result = TQBC_MSG_OK;
  }

  return result;
}


/**
 * @brief      Run torque bias calculator
 *
 * @param      ph       Pointer to @p TQBC_HANDLE_T.
 * @param[in]  input    Input value.
 * @param[out] poutput  Pointer to output value.
 *
 * @return     Result of run.
 * 
 * @retval     TQBC_MSG_ERR_PTR   NULL pointer detected at input.
 * @retval     TQBC_MSG_ERR_IN    Ivalid configuration detected.
 * @retval     TQBC_MSG_OK        Run sucessfully.
 */
tqbc_msg_t TQBC_Run(TQBC_HANDLE_T *ph, float input, float *poutput)
{
  tqbc_msg_t result = TQBC_MSG_INIT;

  float _out = 0.0f;

  if((NULL == ph) || (NULL == poutput))
  {
    result = TQBC_MSG_ERR_PTR;
  }
  else if(!isfinite(input))
  {
    result = TQBC_MSG_ERR_IN;  
  }
  else
  {
    /*Proceed*/
    ph->input = fabsf(input);
    _out = ((ph->input)*(ph->pcfg->gain)) + (ph->pcfg->bias);
    _out = SAT_fSat(_out, ph->pcfg->out_min, ph->pcfg->out_max);

    ph->output = _out;
    *poutput = _out;

    result = TQBC_MSG_OK;
  }

  return result;
}

/**
 * @brief      This function set configuration @p TQBC_CFG_T.
 * 
 * @note       This function use zero cross point rather than bias as input cause
 *             zero cross point is more intuitive during design. 
 *
 * @param      pcfgh             Pointer to @p TQBC_CFG_T.
 * @param[in]  out_max           Upper boundry of output value to be set.
 * @param[in]  out_min           Lower boundry of output value to be set.
 * @param[in]  gain              Gain of linear function to be set.
 * @param[in]  zero_cross_point  Zero cross point f(x) = 0 to be set.
 *
 * @return     Result of setup.
 * 
 * @retval     TQBC_MSG_ERR_PTR   NULL pointer detected at input.
 * @retval     TQBC_MSG_ERR_IN    Ivalid configuration detected.
 * @retval     Other_value        Check retun value of @p TQBC_CheckConfig().
 */
tqbc_msg_t TQBC_SetConfig(TQBC_CFG_T *pcfgh, float out_max, float out_min, float gain, float zero_cross_point)
{
  tqbc_msg_t result = TQBC_MSG_INIT;
  TQBC_CFG_T _cfgh_tmp = {0.0f, 0.0f, 0.0f, 0.0f};

  if(NULL == pcfgh)
  {
    /*Invalid pointer*/
    result = TQBC_MSG_ERR_PTR;
  }

  if(TQBC_MSG_INIT == result)
  {
    /*Set config to _cfgh_tmp and test*/
    _cfgh_tmp.out_max = out_max;
    _cfgh_tmp.out_min = out_min;
    _cfgh_tmp.gain = gain;
    _cfgh_tmp.bias = -1.0f * gain * zero_cross_point;


    /*Check config failed*/
    result = TQBC_CheckConfig(&_cfgh_tmp);

    if(TQBC_MSG_OK == result)
    {
      /*Test passed, copy _cfgh_tmp to pcfgh*/
      memcpy(pcfgh, &_cfgh_tmp, sizeof(TQBC_CFG_T)); /*!!! Need further valuation*/

      result = TQBC_MSG_OK;
    }
  }
  return result;
}


/**
 * @brief      This function check if configuration is correct.
 *
 * @param      pcfgh  Pointer to @p TQBC_CFG_T.
 *
 * @return     Result of check.
 * 
 * @retval     TQBC_MSG_ERR_PTR   NULL pointer detected at input.
 * @retval     TQBC_MSG_ERR_IN    NAN or INF detected in float type contents.
 * @retval     TQBC_MSG_ERR_1     Incorrect boundry relationship (@p out_max and @p out_min).
 * @retval     TQBC_MSG_ERR_2     Incorrect gain value.
 * @retval     TQBC_MSG_ERR_3     Incorrect bias value.
 * @retval     TQBC_MSG_OK        Check pass.
 */
tqbc_msg_t TQBC_CheckConfig(TQBC_CFG_T *pcfgh)
{
  tqbc_msg_t result = TQBC_MSG_INIT;

  if(NULL == pcfgh)
  {
    /*Invalid pointer*/
    result = TQBC_MSG_ERR_PTR;
  }
  else if(!isfinite(pcfgh->out_max) || 
          !isfinite(pcfgh->out_min) || 
          !isfinite(pcfgh->gain) || 
          !isfinite(pcfgh->bias))
  {
    /*Invalid configuration value*/
    result = TQBC_MSG_ERR_IN;
  }
  else if(pcfgh->out_max <= pcfgh->out_min)
  {
    /*Incorrect boundary*/
    result = TQBC_MSG_ERR_1;
  }
  else if(pcfgh->gain >= 0.0f)
  {
    /*Incorrect gain*/
    result = TQBC_MSG_ERR_2;
  }
  else if(pcfgh->bias <= 0.0f)
  {
    /*Incorrect bias*/
    result = TQBC_MSG_ERR_3;
  }
  else
  {
    /*Pass*/
    result = TQBC_MSG_OK;
  }

  return result;
}

/**
 * @brief      This function restart @TQBC_HANDLE_T
 *
 * @details    This function will reset @p input and @p output within @TQBC_HANDLE_T.
 *             This function will not reset value within @p *pcfg.
 * 
 * @param      ph     Pointer to @p TQBC_HANDLE_T.
 *
 * @return     Result of restart.
 * 
 * @retval     TQBC_MSG_ERR_PTR   NULL pointer detected at input.
 * @retval     TQBC_MSG_OK        Reset successfully. 
 */
tqbc_msg_t TQBC_Restart(TQBC_HANDLE_T *ph)
{
  tqbc_msg_t result = TQBC_MSG_INIT;

  if(NULL == ph)
  {
    /*Invalid pointer*/
    result = TQBC_MSG_ERR_PTR;
  }

  if(TQBC_MSG_INIT == result)
  {
    /*Proceed*/
    ph->input = 0.0f;
    ph->output = 0.0f;

    result = TQBC_MSG_OK;
  }

  return result;
}

/**
 * @}
 */
