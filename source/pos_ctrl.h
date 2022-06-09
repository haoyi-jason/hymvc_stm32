/**
 * @file       pos_ctrl.h
 * @author     Maxie
 * @brief      This module includes position control related parameter and functions.
 *
 * @addtogroup POS_CTRL position control algorithm.
 * @{
 *
 * @details    This module contains position control algorithm alone with useful functions and APIs.
 * @version    1.0.2
 */

#ifndef POS_CTRL_H
#define POS_CTRL_H
#ifdef __cplusplus
extern "C" {
#endif

/*Standard include*/
#include <stdint.h>
#include <stdbool.h>
  
/*Module include*/
#include "dual_motor_ctrl/pid_controller.h"

/*Typedef*/
typedef uint16_t pos_u16t;    /**<  @brief  Position type in u16. Range from 0 - 65535*/

/**
 * @name Parameters
 * @{
 */
#define   POSC_DEG_MAX        359.999f
#define   POSC_DEG_MIN        0.0f
#define   POSC_POSU16_MAX     UINT16_MAX
#define   POSC_ON_THOLD_DEG   2.0f
#define   POSC_ON_THOLD_U16   364U
#define   POSC_PREGAIN_1      0.001f
#define   POSC_ALT_DIR_THOLD_U16  ((pos_u16t)36408) /*200.0 deg*/
/** @} */

/**
 * @name Preset values
 * @{
 */
#define   POSC_POSU16_180DEG      ((pos_u16t)32767) 
#define   POSC_POSU16_45DEG       ((pos_u16t)8191) 
#define   POSC_POSU16_315DEG      ((pos_u16t)57343) 
/** @} */

/*Structs*/
/**
 * @brief  Structure representing position command.
 */
typedef struct
{
  pos_u16t pos_cmd_u16;  /**< @brief Target angle in pos_u16t format.*/
  bool direction_cmd;    /**< @brief Target direction of rotation.*/
} POSC_CMD_HANDLE_T;

#define DFLT_INIT_POSC_CMD_HANDLE_T() {0U, true} /**< @brief Macro function that initialize POSC_CMD_HANDLE_T.*/

/**
 * @brief  Structure representing position controller related parameters.
 */
typedef struct
{
  pos_u16t pos_err_thold_u16;  /**< @brief  Threshold of position controller.*/
  float s_cmd_min;             /**< @brief  Minimum output speed.*/
  float s_cmd_max;             /**< @brief  Maximum output speed.*/
  float kp;                    /**< @brief  Kp of position controller.*/
} POSC_CFG_HANDLE_T;

/**
 * @brief  Macro function that initializ POSC_CFG_HANDLE_T, initial values required.
 */
#define INIT_POSC_CFG_HANDLE_T(thold, s_min, s_max, val_kp)\
{\
  .pos_err_thold_u16 = thold,\
  .s_cmd_min = s_min,\
  .s_cmd_max = s_max,\
  .kp = val_kp\
}

typedef struct
{
  POSC_CFG_HANDLE_T cfg;
  PID_CFG_T pid_cfg;
  PID_HANDLE_T pid;
  POSC_CMD_HANDLE_T cmd;
  pos_u16t perr;
  float speed_cmd_rpm;
  uint16_t last_cmd;
  uint16_t last_pos;
  uint8_t dir_changed;
  uint16_t last_err;
  uint16_t pos_hist[32];
  uint16_t last_pos_hist[32];
  uint8_t dir_hist[32];
  uint8_t last_dir_hist[32];
  uint8_t posIndex;
} POSC_HANDLE_T;

/*Functions*/
void POSC_Init( POSC_HANDLE_T *ppch, 
                float _pos_err_thold_u16, 
                float _s_cmd_min, 
                float _s_cmd_max,
                float _kp,
                float _ki,
                float _kd);
void POSC_ResetData();
void POSC_Restart(POSC_HANDLE_T *ppch);

pos_u16t POSC_CalcPerr(bool dir, pos_u16t cmd, pos_u16t act);
bool POSC_CalcDirection(bool dir_curr, pos_u16t cmd, pos_u16t act, pos_u16t thold);
//float POSC_Run(POSC_CMD_HANDLE_T *ppcmdh, POSC_CFG_HANDLE_T *ppccfgh, pos_u16t act);
float POSC_Run(POSC_HANDLE_T *ppch, pos_u16t act);

/*Helper functions*/
pos_u16t POSC_ConvertDeg2U16(float degree);
float    POSC_ConvertU162Deg(pos_u16t degree_u16);

/*Set and get functions*/
void POSC_SetCfgPErrThold(POSC_CFG_HANDLE_T *ppccfgh, float val);
float POSC_GetCfgPErrThold(POSC_CFG_HANDLE_T *ppccfgh);
void POSC_SetCfgSCmdMin(POSC_CFG_HANDLE_T *ppccfgh, float val);
float POSC_GetCfgSCmdMin(POSC_CFG_HANDLE_T *ppccfgh);
void POSC_SetCfgSCmdMax(POSC_CFG_HANDLE_T *ppccfgh, float val);
float POSC_GetCfgSCmdMax(POSC_CFG_HANDLE_T *ppccfgh);
void POSC_SetCfgKp(POSC_CFG_HANDLE_T *ppccfgh, float val);
float POSC_GetCfgKp(POSC_CFG_HANDLE_T *ppccfgh);

#ifdef __cplusplus
}
#endif
#endif

/**
 * @}
 */