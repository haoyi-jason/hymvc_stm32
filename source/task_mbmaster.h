#ifndef _TASK_MBMASTER_
#define _TASK_MBMASTER_

void modbus_master_task_init();
int8_t modbus_master_WriteCtrl(uint16_t id, uint16_t value);
uint16_t modbus_master_ReadStatus(uint16_t id);
long modbus_master_ReadSpeed(void);
long modbus_master_ReadPosition(void);
#endif