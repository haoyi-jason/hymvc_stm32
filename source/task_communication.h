#ifndef _TASK_COMMUNICATION_
#define _TASK_COMMUNICATION_

void task_communication_init();

#define CAN_GSID_KP_P            1U
#define CAN_GSID_KP_S            2U
#define CAN_GSID_KI_S            3U
#define CAN_GSID_KD_S            4U
#define CAN_GSID_MOT_T_MAX_ABS   5U
#define CAN_GSID_MOT_S_MAX_ABS   6U
#define CAN_GSID_G_A2M           7U
#define CAN_GSID_TQBC_MIN        8U
#define CAN_GSID_TQBC_MAX        9U
#define CAN_GSID_G               10U
#define CAN_GSID_ZCP             11U
#define CAN_GSID_UPDATE          12U
#define CAN_GSID_PC_PERR_THOLD   13U
#define CAN_GSID_PC_S_CMD_MIN    14U
#define CAN_GSID_PC_S_CMD_MAX    15U

#endif