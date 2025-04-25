/**
 * @file       pid_controller.h
 * @author     Maxie
 * @brief      This file implements PID controller.
 *
 * @addtogroup PID_CONTROLLER PID controller
 * @{
 * 
 * @details    This module include a PID controller with output "Conditional Integration" feature
 *             to prevent integral windup.
 *             
 * @version    1.0.0
 */
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#ifdef __cplusplus
extern "C" {
#endif

/*Standard include*/
#include <stdint.h>
//#include "unit_test_pid_controller.h"

/*Other include*/

/*Typedef*/
typedef uint8_t   pid_msg_t;        /**< @brief Return message type.*/ 
typedef uint32_t  pid_cfg_word_t;   /**< @brief PID Configuration word type.*/ 

/*Configuration deifne*/

/*Debug Macro*/
//#define PID_DEBUG_WITH_STDIO

/**
 * @name Return message type
 * @{
 */
/*Define msg_t*/
#define PID_MSG_INIT              ((pid_msg_t)0xFF) /**<  @brief  Initialization form.*/
#define PID_MSG_OK                ((pid_msg_t)0x00) /**<  @brief  OK or pass.*/
#define PID_MSG_ERR_PTR           ((pid_msg_t)0x01) /**<  @brief  Input pointer error.*/
#define PID_MSG_ERR_IN            ((pid_msg_t)0x02) /**<  @brief  Input value or content error.*/
#define PID_MSG_ERR_NG            ((pid_msg_t)0x03) /**<  @brief  Check failed.*/

#define PID_MSG_ERR_1             ((pid_msg_t)0xE1) /**<  @brief  Function dependent error NO.1.*/
#define PID_MSG_ERR_2             ((pid_msg_t)0xE2) /**<  @brief  Function dependent error NO.2.*/
#define PID_MSG_ERR_3             ((pid_msg_t)0xE3) /**<  @brief  Function dependent error NO.3.*/
#define PID_MSG_ERR_4             ((pid_msg_t)0xE4) /**<  @brief  Function dependent error NO.4.*/
#define PID_MSG_ERR_5             ((pid_msg_t)0xE5) /**<  @brief  Function dependent error NO.5.*/

#define PID_MSG_ERR_OTHER         ((pid_msg_t)0xFE) /**<  @brief  Other error.*/
/** @} */


/*Define CFG1*/



/*Structs*/
/**
 * @brief      Structure representing configuration of PID controller.
 */
typedef struct
{
  pid_cfg_word_t  cfg1; /**< @brief Configuration word 1.*/
  float      kp;        /**< @brief Kp.*/
  float      ki;        /**< @brief Ki.*/
  float      kd;        /**< @brief Kd.*/
} PID_CFG_T;


/**
 * @brief      Structure representing intermediate data of PID controller.
 */
typedef struct
{
  float err_now;      
  float err_last;     
  float err_sum;      
  float val_aft_kp;   
  float val_aft_kd;   
  float val_aft_ki;   
} PID_DATA_T;

/**
 * @brief      Handle of PID controller.
 */
typedef struct
{
  PID_CFG_T *pcfg;  /**< @brief      Pointer to @p PID_CFG_T.*/
  PID_DATA_T data;  /**< @brief      Intermidate data.*/
  float cmd;        /**< @brief      Command value.*/
  float act;        /**< @brief      Actual value.*/
  float out;        /**< @brief      Output value.*/
} PID_HANDLE_T;


/*Functions*/
pid_msg_t PID_Init(PID_HANDLE_T *ph, PID_CFG_T *pcfgh);
pid_msg_t PID_RunPID(PID_HANDLE_T *ph, float cmd, float act, float out_aft_sat,float *p_out);
pid_msg_t PID_RunPIDCore(PID_HANDLE_T *ph, float err, bool output_is_saturate, float *p_out);
pid_msg_t PID_RunPIDExtErr(PID_HANDLE_T *ph, float err, float out_aft_sat,float *p_out);

float PID_GetOutput(PID_HANDLE_T *ph);
pid_msg_t PID_Restart(PID_HANDLE_T *ph);

pid_msg_t PID_CheckConfig(PID_CFG_T *pcfgh);
pid_msg_t PID_ResetData(PID_DATA_T *pdatah);
pid_msg_t PID_ResetHandle(PID_HANDLE_T *ph);

/*Fuctions for debug purpose*/
#ifdef PID_DEBUG_WITH_STDIO
void PID_PrintHandle(PID_HANDLE_T *ph);
void PID_PrintCfg(PID_CFG_T *ph);
void PID_PrintData(PID_DATA_T *ph);
void PID_PrintInterface(PID_HANDLE_T *ph);
#endif

#ifdef __cplusplus
}
#endif
#endif

/**
 * @}
 */

