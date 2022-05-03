/**
 * @file       dual_motor_ctrl.h
 * @author     Maxie
 * @brief      This file implements dual motor control.
 * 
 * @addtogroup DUAL_MOTOR_CTRL Dual Motor Controller
 * @{
 * 
 * @details    This module act as an interface of control algorithm.
 *             Program using "dual motor control" algorithm only need to access functions in this module.
 *             
 * @version    1.0.0
 */

#ifndef DUAL_MOTOR_CTRL_H
#define DUAL_MOTOR_CTRL_H
#ifdef __cplusplus
extern "C" {
#endif

/*Standard include*/
#include <stdint.h>


/*Module include*/
#include "pid_controller.h"
#include "tq_bias_calc.h"

/*Other include*/

/*Typedef*/
typedef uint8_t   dmotc_msg_t;    /**<  @brief  Return message type.*/ 
typedef uint8_t   dmotc_state_t;  /**<  @brief  State type.*/
typedef uint8_t   dmotc_gain_no;  /**<  @brief  Index of gain.*/

/*Configuration deifne*/

/**
 * @name Return message type
 * @{
 */
/*Define msg_t*/
#define DMOTC_MSG_INIT              ((dmotc_msg_t)0xFF)   /**<  @brief  Initialization form.*/
#define DMOTC_MSG_OK                ((dmotc_msg_t)0x00)   /**<  @brief  OK or pass.*/
#define DMOTC_MSG_ERR_PTR           ((dmotc_msg_t)0x01)   /**<  @brief  Input pointer error.*/
#define DMOTC_MSG_ERR_IN            ((dmotc_msg_t)0x02)   /**<  @brief  Input value or content error.*/
#define DMOTC_MSG_ERR_NG            ((dmotc_msg_t)0x03)   /**<  @brief  Check failed.*/
#define DMOTC_MSG_ERR_STATE         ((dmotc_msg_t)0x04)   /**<  @brief  Incorrect state.*/
#define DMOTC_MSG_ERR_OTHER         ((dmotc_msg_t)0xFE)   /**<  @brief  Other error.*/

#define DMOTC_MSG_ERR_1              ((dmotc_msg_t)0xE1)  /**<  @brief  Function dependent error NO.1.*/
#define DMOTC_MSG_ERR_2              ((dmotc_msg_t)0xE2)  /**<  @brief  Function dependent error NO.2.*/
#define DMOTC_MSG_ERR_3              ((dmotc_msg_t)0xE3)  /**<  @brief  Function dependent error NO.3.*/
#define DMOTC_MSG_ERR_4              ((dmotc_msg_t)0xE4)  /**<  @brief  Function dependent error NO.4.*/
#define DMOTC_MSG_ERR_5              ((dmotc_msg_t)0xE5)  /**<  @brief  Function dependent error NO.5.*/
/** @} */

/**
 * @name Module state definition
 * @{
 */
/*Define state_t*/
#define DMOTC_STATE_INIT            ((dmotc_msg_t)0x00)   /**<  @brief  State Initialization.*/
#define DMOTC_STATE_STOP            ((dmotc_msg_t)0x01)   /**<  @brief  State Stopped.*/
#define DMOTC_STATE_RUN             ((dmotc_msg_t)0x02)   /**<  @brief  State Running.*/
/** @} */
    
/*Define gain number*/
/**
 * @brief      Gain index for "Axis torque to motor torque".
 */
#define DMOTC_GAIN_ID_ATQ2MTQ          ((dmotc_gain_no)0x01) 

/*Structs*/
/**
 * @brief      Structure representing data or signals used by dual motor controller.
 * 
 * @note       Data or signals within included modules are declared in their handle.
 *             Unit of data or signals is irrelevent.
 */
typedef struct
{
  float axis_s_set;   /**<  @brief  Input signal. Target axis speed.*/
  float axis_s_act;   /**<  @brief  Input signal. Actual axis speed.*/
  float axis_tq_cmd;  /**<  @brief  Metadata. Axis torque commend.*/
  float mot_tq_bias;  /**<  @brief  Metadata. Motor torque bias.*/
  float mot1_tq_set;  /**<  @brief  Output signal. Motor 1 torque command.*/
  float mot2_tq_set;  /**<  @brief  Output signal. Motor 2 torque command.*/
} DMOTC_DATA_T;

/**
 * @brief      Handle of dual motor controller
 */
typedef struct
{
  dmotc_state_t   state;        /**<  @brief  State of module.*/
  PID_HANDLE_T    axis_s_pid;   /**<  @brief  Handle of "Axis speed PID controller".*/
  TQBC_HANDLE_T   tqbc;         /**<  @brief  Handle of "Torque bias calculator".*/
  DMOTC_DATA_T    data;         /**<  @brief  Data or signals.*/
  float           gain_atq2mtq; /**<  @brief  Parameter. Gain "Axis torque to motor torque".*/
  float           mot_tq_min;   /**<  @brief  Parameter. Minimum motor torque.*/
  float           mot_tq_max;   /**<  @brief  Parameter. Maximum motor torque.*/
} DMOTC_HANDLE_T;

/*Function*/
dmotc_msg_t DMOTC_SetTQBCConfig(TQBC_CFG_T *pcfgh,
                                float out_max, 
                                float out_min, 
                                float gain, 
                                float zero_cross_point);
dmotc_msg_t DMOTC_Init(DMOTC_HANDLE_T *ph, 
                       PID_CFG_T *pspidcfgh, 
                       TQBC_CFG_T *ptqbccfgh, 
                       float gain_atq2mtq, 
                       float mot_tq_min, 
                       float mot_tq_max);
dmotc_msg_t DMOTC_Start(DMOTC_HANDLE_T *ph);
dmotc_msg_t DMOTC_Run(DMOTC_HANDLE_T *ph,
                      float axis_s_set, 
                      float axis_s_act,
                      float *pmot1_tq_set,
                      float *pmot2_tq_set);
dmotc_msg_t DMOTC_Stop(DMOTC_HANDLE_T *ph);
dmotc_msg_t DMOTC_SetTqBoud(DMOTC_HANDLE_T *ph, float mot_tq_min, float mot_tq_max);
dmotc_msg_t DMOTC_SetGain(DMOTC_HANDLE_T *ph, dmotc_gain_no gain_no, float val);
dmotc_msg_t DMOTC_ResetData(DMOTC_DATA_T *pdatah);
#ifdef __cplusplus
}
#endif
#endif


/**
 * @}
 */
