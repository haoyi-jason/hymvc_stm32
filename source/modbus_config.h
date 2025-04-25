#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H
#ifdef __cplusplus
extern "C" {
#endif

#include "mbport.h"

#define MODBUS_BAUDRATE           ((ULONG)10000)
#define MODBUS_ADD_INV1           ((UCHAR)1)
#define MODBUS_ADD_INV2           ((UCHAR)2)

#define MODBUS_REG_ADD_CTRL       ((USHORT)0x300)
#define MODBUS_REG_ADD_STATUS     ((USHORT)0x301)

#define MODBUS_REG_ADD_POS_ACT    ((USHORT)0x305)
#define MODBUS_REG_ADD_SPEED_ACT  ((USHORT)0x307)

#define MODBUS_REG_ADD_TQ_MAX     ((USHORT)0x801)


#ifdef __cplusplus
}
#endif
#endif