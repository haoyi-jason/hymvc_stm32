/**
 * @file       pos_ctrl.h
 * @author     Maxie
 * @brief      This module includes position control related parameter and functions.
 *
 * @addtogroup POS_CTRL position control
 * @{
 *
 * @details    { detailed_item_description }
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

/*Typedef*/
typedef uint16_t pos_u16t;    /**<  @brief  Position type in u16. Range from 0 - 65535*/

/*Parameter*/
#define   POSC_DEG_MAX        359.999f
#define   POSC_DEG_MIN        0.0f
#define   POSC_POSU16_MAX     UINT16_MAX
#define   POSC_ON_THOLD_DEG   2.0f
#define   POSC_ON_THOLD_U16   364U
#define   POSC_PREGAIN_1      0.001f

#define   POSC_ALT_DIR_THOLD_U16  ((pos_u16t)36408) /*200.0 deg*/
#define   POSC_POSU16_180DEG      ((pos_u16t)32767) 
#define   POSC_POSU16_45DEG       ((pos_u16t)8191) 
#define   POSC_POSU16_315DEG      ((pos_u16t)57343) 


typedef struct
{
  pos_u16t pos_cmd_u16;
  bool direction_cmd;
} POSC_CMD_HANDLE_T;

#define DFLT_INIT_POSC_CMD_HANDLE_T() {0U, true}

typedef struct
{
  pos_u16t pos_err_thold_u16;
  float s_cmd_min;
  float s_cmd_max;
  float kp;
} POSC_CFG_HANDLE_T;

#define INIT_POSC_CFG_HANDLE_T(thold, s_min, s_max, val_kp)\
{\
  .pos_err_thold_u16 = thold,\
  .s_cmd_min = s_min,\
  .s_cmd_max = s_max,\
  .kp = val_kp\
}


/*Functions*/
pos_u16t POSC_CalcPerr(bool dir, pos_u16t cmd, pos_u16t act);
bool POSC_CalcDirection(bool dir_curr, pos_u16t cmd, pos_u16t act, pos_u16t thold);
float POSC_Run(POSC_CMD_HANDLE_T *ppcmdh, POSC_CFG_HANDLE_T *ppccfgh, pos_u16t act);

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