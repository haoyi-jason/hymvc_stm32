#ifndef _TASK_MBMASTER_
#define _TASK_MBMASTER_

void modbus_master_task_init();
int8_t modbus_master_WriteCtrl(uint16_t id, uint16_t value);
uint16_t modbus_master_GetStatus(uint16_t id);
int32_t modbus_master_GetSpeed(void);
int32_t modbus_master_GetPosition(void);
#endif