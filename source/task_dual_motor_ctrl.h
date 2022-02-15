/**
 * @file       task_dual_motor_ctrl.h
 * @author     Maxie
 * @brief      This module include a task that gerenate a step response.
 *
 * @addtogroup TASK_DUAL_MOTOR_CTRL dual motor contoller task
 * @{
 *
 * @details    This module include a task that run dual motor controller with
 *             fix command input. This module is used to understand the steady state characteristic
 *             of the controller.
 *             
 * @version    1.0.0
 */

#ifndef TASK_DMOTC_H
#define TASK_DMOTC_H
#ifdef __cplusplus
extern "C" {
#endif

/*Typedef*/
typedef uint8_t   tdmotc_mode_t;    /**<  @brief  Mode type.*/ 
typedef uint8_t   tdmotc_msg_t;

/*Define mode_t*/
#define TDMOTC_MODE_S            ((tdmotc_mode_t)0x00)   /**<  @brief  Speed control mode.*/
#define TDMOTC_MODE_P            ((tdmotc_mode_t)0x01)   /**<  @brief  Position control mode.*/
#define TDMOTC_MODE_P2           ((tdmotc_mode_t)0x02)   /**<  @brief  Fix speed position control mode.*/

/*Define PID*/
#define TDMOTC_PID_S             0U                      /**< @brief  Index of speed PID.*/
#define TDMOTC_PID_P             1U                      /**< @brief  Index of position PID.*/

#define TDMOTC_PID_ID_P       0U                         /**< @brief  Index of KP.*/
#define TDMOTC_PID_ID_I       1U                         /**< @brief  Index of KI.*/
#define TDMOTC_PID_ID_D       2U                         /**< @brief  Index of KD.*/

/*Define TQBC*/
#define TDMOTC_TQBC_ID_OUT_MAX   0U                      /**< @brief  Index of maximum bias torque.*/
#define TDMOTC_TQBC_ID_OUT_MIN   1U                      /**< @brief  Index of minimum bias torque.*/
#define TDMOTC_TQBC_ID_GAIN      2U                      /**< @brief  Index of slop of bias torque.*/
#define TDMOTC_TQBC_ID_ZCP       3U                      /**< @brief  Index of zero crossing point.*/

/*Define signal gain*/
#define TDMOTC_CAN2SIG_GAIN_F    (1.0f/10000.0f)         /**< @brief  Conversion gain from can format (int32_t) to float.*/

/*Functions*/
/*Task control*/
void tdmotc_algorithm_task_init(void);
void tdmotc_Start(tdmotc_mode_t _mode);
void tdmotc_Stop(void);

void tdmotc_ResetIO(void);

tdmotc_mode_t tdmotc_GetMode(void);

bool tdmotc_GetFault(void);
void tdmotc_ResetFault(void);

/*Set and Get command signal*/
void tdmotc_SetSpeedCmd(float val);
float tdmotc_GetSpeedCmd(void);

void tdmotc_SetPosCmd(float val);
float tdmotc_GetPosCmd(void);

/*Set and get parameters or signals*/
void tdmotc_SetPID(uint8_t pid, uint8_t pid_index, float val);
float tdmotc_GetPID(uint8_t pid, uint8_t pid_index);
void tdmotc_SetTQBC(uint8_t index, float val);
float tdmotc_GetTQBC(uint8_t index);
void tdmotc_UpdateTQBC(void);
void tdmotc_SetMotTqMaxAbs(float val);
float tdmotc_GetMotTqMaxAbs(void);
void tdmotc_SetAxisSMaxAbs(float val);
float tdmotc_GetAxisSMaxAbs(void);
float tdmotc_GetAxisSpeedAct(void);
float tdmotc_GetMotTqActV(uint8_t index);
bool tdmotc_GetIsStart(void);
float tdmotc_ConvCAN2Sig(int32_t input);
int32_t tdmotc_ConvSig2CAN(float input);


#ifdef __cplusplus
}
#endif
#endif

/**
 * @}
 */
