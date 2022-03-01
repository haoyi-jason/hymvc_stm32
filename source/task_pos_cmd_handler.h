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
void tpcmdh_taskInit(void);
//bool tpcmdh_CalcDirection(bool dir_curr, pos_u16t cmd, pos_u16t act, pos_u16t thold);

/*Set and get functions*/
void tpcmdh_SetPosCmd(float val);
float tpcmdh_GetPosCmd(void);
pos_u16t tpcmdh_GetPosCmdU16(void);
bool tpcmdh_GetDirection(void);

#ifdef __cplusplus
}
#endif
#endif