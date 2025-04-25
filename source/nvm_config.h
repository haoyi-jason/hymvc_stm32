#ifndef _NVM_CONFIG_
#define _NVM_CONFIG_



#define SZ_NVM_BOARD    32
#define SZ_NVM_CONFIG   1024 // adc channel config
#define SZ_NVM_USER     64
#define SZ_NVM_DIGCFG   128

#define OFFSET_NVM_BOARD        0
#define OFFSET_NVM_CONFIG       OFFSET_NVM_BOARD + SZ_NVM_BOARD
#define OFFSET_NVM_USER         OFFSET_NVM_CONFIG + SZ_NVM_CONFIG
#define OFFSET_NVM_DIGCFG       OFFSET_NVM_USER + SZ_NVM_USER


#endif