/**
 * @file       task_pos_cmd_handler.h
 * @author     Maxie
 * @brief      This module contains thread of postion command processer and its APIs.
 *
 * @addtogroup TASK_POS_CMD_HANDLER Position command processer.
 * @{
 *
 * @version    1.0.0
 */
#ifndef TASK_PCMDH_H
#define TASK_PCMDH_H
#ifdef __cplusplus
extern "C" {
#endif

/*Standard include*/
#include <stdint.h>
#include <stdbool.h>

/*Module include*/
#include "pos_ctrl.h"

/*Other include*/

/*Functions*/
void tpcmdh_bsemInit(void);
void tpcmdh_taskInit(void *, void *);
void tpcmdh_taskStop();

/*Set and get functions*/
void tpcmdh_SetPosCmd(uint8_t axis, float val);
float tpcmdh_GetPosCmd(uint8_t axis);
pos_u16t tpcmdh_GetPosCmdU16(uint8_t axis);
bool tpcmdh_GetDirection(uint8_t axis);
void tpcmdh_GetCommand(POSC_CMD_HANDLE_T *p_handle_out);
bool tpcmdh_CmdIsAvailable();

#ifdef __cplusplus
}
#endif
#endif

/**
 * @}
 */
