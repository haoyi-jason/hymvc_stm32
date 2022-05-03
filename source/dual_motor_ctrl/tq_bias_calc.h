/**
 * @file       tq_bias_calc.h
 * @author     Maxie
 * @brief      This file implements torque bias calculator.
 * 
 * @addtogroup TQ_BIAS_CALC Torque Bias Calculator
 * @{
 *  
 * @details    This module include a torque bias calculator that calculate bias torque
 *             according to the input and linear function set.
 *             
 * @version    1.0.0
 */
#ifndef TQ_BIAS_CALC_H
#define TQ_BIAS_CALC_H
#ifdef __cplusplus
extern "C" {
#endif

/*Standard include*/
#include <stdint.h>

/*Other include*/

/*Typedef*/
typedef uint8_t   tqbc_msg_t; /**< @brief Return message type.*/ 

/*Configuration deifne*/

/*Debug Macro*/


/*Define msg_t*/
/**
 * @name Return message type
 * @{
 */
#define TQBC_MSG_INIT               ((tqbc_msg_t)0xFF)  /**<  @brief  Initialization form.*/
#define TQBC_MSG_OK                 ((tqbc_msg_t)0x00)  /**<  @brief  OK or pass.*/
#define TQBC_MSG_ERR_PTR            ((tqbc_msg_t)0x01)  /**<  @brief  Input pointer error.*/
#define TQBC_MSG_ERR_IN             ((tqbc_msg_t)0x02)  /**<  @brief  Input value or content error.*/
#define TQBC_MSG_ERR_NG             ((tqbc_msg_t)0x03)  /**<  @brief  Check failed.*/
#define TQBC_MSG_ERR_OTHER          ((tqbc_msg_t)0xFE)  /**<  @brief  Other error.*/
#define TQBC_MSG_ERR_1              ((tqbc_msg_t)0xE1)  /**<  @brief  Function dependent error NO.1.*/
#define TQBC_MSG_ERR_2              ((tqbc_msg_t)0xE2)  /**<  @brief  Function dependent error NO.2.*/
#define TQBC_MSG_ERR_3              ((tqbc_msg_t)0xE3)  /**<  @brief  Function dependent error NO.3.*/
#define TQBC_MSG_ERR_4              ((tqbc_msg_t)0xE4)  /**<  @brief  Function dependent error NO.4.*/
#define TQBC_MSG_ERR_5              ((tqbc_msg_t)0xE5)  /**<  @brief  Function dependent error NO.5.*/
/** @} */


/*Structs*/
/**
 * @brief      Structure representing configuration of Torque bias calculator.
 */
typedef struct
{
  float out_max;    /**<  @brief  Upper boundry of output value.*/
  float out_min;    /**<  @brief  Lower boundry of output value.*/
  float gain;       /**<  @brief  Gain of linear function.*/
  float bias;       /**<  @brief  Bias of linear function.*/
} TQBC_CFG_T;

/**
 * @brief      Handle of torque bias calculator.
 */
typedef struct
{
  TQBC_CFG_T *pcfg;   /**< @brief  Pointer to @p TQBC_CFG_T.*/
  float input;        /**< @brief  Input.*/
  float output;       /**< @brief  Output.*/
} TQBC_HANDLE_T;

/*Function*/
tqbc_msg_t TQBC_Init(TQBC_HANDLE_T *ph, TQBC_CFG_T *pcfgh);
tqbc_msg_t TQBC_Run(TQBC_HANDLE_T *ph, float input, float *poutput);
tqbc_msg_t TQBC_SetConfig(TQBC_CFG_T *pcfgh, float out_max, float out_min, float gain, float zero_cross_point);
tqbc_msg_t TQBC_CheckConfig(TQBC_CFG_T *pcfgh);
tqbc_msg_t TQBC_Restart(TQBC_HANDLE_T *ph);

#ifdef __cplusplus
}
#endif
#endif

/**
 * @}
 */
