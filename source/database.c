#include "ch.h"
#include "hal.h"
#include "database.h"
#include "app_config.h"

#include "task_analog_input.h"
#include "task_dac.h"
#include "task_resolver.h"

#include "ylib/eeprom/at24_eep.h"
#include "ylib/encryption/crc16/crc16-ccitt.h"

#include "task_can1.h"
#include "pid/nested_pid.h"

typedef union {
  uint8_t data8[8];
  uint16_t data16[4];
  uint32_t data32[2];
  float fv[2];
}CANBUS_DATA_u;

struct crc_16_param_s
{
  uint16_t crc16[16];
};

struct pid_param_s{
  float pos_kp;
  float pos_ki;
  float pos_kd;
  float spd_kp;
  float spd_ki;
  float spd_kd;
};
struct stow_param_s{
  uint8_t pwr_output_id;
  uint8_t dire_output_id;
  uint8_t az_el_select_id;
};

typedef struct{
  float min;
  float max;
  float offset;
}_limit_param_t;

struct base_param_s{
  _limit_param_t pos_input;
  _limit_param_t pos_output;
  _limit_param_t spd_input;
  _limit_param_t spd_output;
};

typedef struct{
  uint8_t en_id;
  uint8_t svon_id;
  uint8_t rdy_id;
  uint8_t control_channel;
}_control_pin_t;

struct control_param_s{
  _control_pin_t m1;
  _control_pin_t m2;
  uint8_t lock_sense_id;
  uint8_t unlock_sense_id;
  uint8_t resolver_id;
};

struct config_param_s{
  uint8_t pid_auto_start;
  uint16_t stow_cycle_time_ms;
  uint16_t stow_timeout_ms;
  uint16_t az_valid_mask;
  uint16_t el_valid_mask;
  uint16_t stow_activity_idle_time_ms;
  uint8_t can_rate_1;
  
};

struct canbus_id_def_s{
  uint32_t id;
  uint16_t interval;
};

struct board_param_s
{
  uint32_t hw_version;
  uint32_t sw_version;
  uint32_t user_id;
  uint32_t function_code;
  uint8_t can1_rate;
  uint8_t can2_rate;
};

typedef struct{
  struct crc_16_param_s crc16;
  struct config_param_s config_param;
  struct pid_param_s pid_param[NOF_AXES];
  struct stow_param_s stow_param;
  struct control_param_s control_param[NOF_AXES];  
  struct base_param_s base_param[NOF_AXES];
  struct canbus_id_def_s can_msg[CANBUS_MESSAGE_COUNT];
//  _dac_channel_config_t dac[AD57_CHANNELS];
  struct board_param_s board_param;
}_pid_nvm_t;

struct livedata_s{
  uint32_t enableReport;
  uint32_t mcStatus;
  uint16_t digital_input;
  uint16_t digital_output;
  struct{
    float pos;
    float speed;
    float pos_err;
    float speed_err;
  }sp[NOF_AXES];
  struct{
    float pos;
    float speed;
  }pv[NOF_AXES];
  struct{
    int32_t rawData;
    float scaledData;
  }analog_inputs[8];
  struct{
    int32_t rawData;
    float scaledData;
  }analog_outputs[4];
};

static struct livedata_s liveData;


static _pid_nvm_t nvmParams;

//static _storageModule_t storageModule = {
//  &I2CD2,
//  0x50,
//  32,
//  128,
//  2
//};
//
bool eeprom_init()
{
  bool ret = false;
//  _storageModule_t *module = (_storageModule_t*)storageModule;
  
  at24eep_init(&I2CD2, 32, 128, 0x50, 2);
  //at24eep_init(module->dev, module->pageSize, module->pageSize,module->i2cAddress, module->addressBytes);

//  module->totalBytes = module->pages * module->pageSize;
//  module->addrNextAuto = 0;
//  module->addrNextProt = module->totalBytes>>1;
  ret = true;
  
  //eepTest();
  return ret;
}

static void eeprom_readBlock(void *storageModule, uint8_t *data,size_t eepromAddr, size_t len)
{
  eepromRead(eepromAddr, len, data);
}

static bool eeprom_writeBlock(void *storageModule, uint8_t *data,size_t eepromAddr, size_t len)
{
  bool ret = false;
  
  eepromWrite(eepromAddr, len, data);
  
  return ret;
}

static void save_crc_param()
{
  uint16_t crc = 0x0;
  uint16_t len = DATA_FLASH_CRC_SIZE;
  eepromWrite(DATA_FLASH_CRC_PARAM_OFFSET, len, (uint8_t*)&nvmParams.crc16);
}
static void load_crc_param()
{
  uint16_t crc = 0x0;
  uint16_t len = DATA_FLASH_CRC_SIZE;
  eepromRead(DATA_FLASH_CRC_PARAM_OFFSET, len, (uint8_t*)&nvmParams.crc16);
}

static void load_config_param_default()
{
  nvmParams.config_param.az_valid_mask = 0xFFFF;
  nvmParams.config_param.el_valid_mask = 0xFFFF;
  nvmParams.config_param.pid_auto_start = 1;
  nvmParams.config_param.stow_activity_idle_time_ms = 5000;
  nvmParams.config_param.stow_cycle_time_ms = 100;
  nvmParams.config_param.stow_timeout_ms = 15000;
  nvmParams.config_param.can_rate_1 = 0;
}
static void save_config_param()
{
  uint16_t crc = 0x0;
  uint16_t len = sizeof(struct config_param_s);
  crc = crc16_ccitt((uint8_t*)&nvmParams.config_param,len,crc);
  nvmParams.crc16.crc16[DATA_SECTION_CONFIG_PARAM] = crc;
  eepromWrite(DATA_FLASH_CONFIG_PARAM_OFFSET, len, (uint8_t*)&nvmParams.config_param);
  save_crc_param();
}
static void load_config_param(uint16_t crc_stored)
{
  uint16_t crc = 0x0;
  uint16_t len = sizeof(struct config_param_s);
  eepromRead(DATA_FLASH_CONFIG_PARAM_OFFSET, len, (uint8_t*)&nvmParams.config_param);
  crc = crc16_ccitt((uint8_t*)&nvmParams.config_param,len,crc);
  
  if(crc != crc_stored){
    load_config_param_default();
    save_config_param();
  }
}


static void load_pid_param_default()
{
  nvmParams.pid_param[0].pos_kp = 1.0;
  nvmParams.pid_param[0].pos_ki = 0.0;
  nvmParams.pid_param[0].pos_kd = 0.0;
  nvmParams.pid_param[0].spd_kp = 1.0;
  nvmParams.pid_param[0].spd_ki = 0.0;
  nvmParams.pid_param[0].spd_kd = 0.0;

  nvmParams.pid_param[1].pos_kp = 1.0;
  nvmParams.pid_param[1].pos_ki = 0.0;
  nvmParams.pid_param[1].pos_kd = 0.0;
  nvmParams.pid_param[1].spd_kp = 1.0;
  nvmParams.pid_param[1].spd_ki = 0.0;
  nvmParams.pid_param[1].spd_kd = 0.0;
  
  
}

static void save_pid_param()
{
  uint16_t crc = 0x0;
  uint16_t len = sizeof(struct pid_param_s)*NOF_AXES;
  crc = crc16_ccitt((uint8_t*)&nvmParams.pid_param,len,crc);
  nvmParams.crc16.crc16[DATA_SECTION_PID_PARAM] = crc;
  eepromWrite(DATA_FLASH_PID_PARAM_OFFSET, len, (uint8_t*)&nvmParams.pid_param);
  save_crc_param();
}

static void load_pid_param(uint16_t crc_stored)
{
  uint16_t crc = 0x0;
  uint16_t len = sizeof(struct pid_param_s) * NOF_AXES;
  eepromRead(DATA_FLASH_PID_PARAM_OFFSET, len, (uint8_t*)&nvmParams.pid_param);
  crc = crc16_ccitt((uint8_t*)&nvmParams.pid_param,len,crc);
  if(crc_stored != crc){
    load_pid_param_default();
    save_pid_param();
  }
}


static void load_stow_param_default()
{
  nvmParams.stow_param.pwr_output_id = 8;
  nvmParams.stow_param.dire_output_id = 9;
  nvmParams.stow_param.az_el_select_id = 10;
}
static void save_stow_param()
{
  uint16_t crc = 0x0;
  uint16_t len = sizeof(struct stow_param_s);
  crc = crc16_ccitt((uint8_t*)&nvmParams.stow_param,len,crc);
  nvmParams.crc16.crc16[DATA_SECTION_STOW_PARAM] = crc;
  eepromWrite(DATA_FLASH_STOW_PARAM_OFFSET, len, (uint8_t*)&nvmParams.stow_param);
  save_crc_param();
}
static void load_stow_param(uint16_t crc_stored)
{
  uint16_t crc = 0x0;
  uint16_t len = sizeof(struct stow_param_s);
  eepromRead(DATA_FLASH_STOW_PARAM_OFFSET, len, (uint8_t*)&nvmParams.stow_param);
  crc = crc16_ccitt((uint8_t*)&nvmParams.stow_param,len,crc);
  if(crc_stored != crc){
    load_stow_param_default();
    save_stow_param();
  }

}
static void load_control_param_default()
{
  struct control_param_s *cfg = &nvmParams.control_param[AXIS_AZ];
  cfg->m1.en_id = 0;
  cfg->m1.svon_id = 1;
  cfg->m1.rdy_id = 4;
  cfg->m1.control_channel = 0;

  cfg->m2.en_id = 2;
  cfg->m2.svon_id = 3;
  cfg->m2.rdy_id = 5;
  cfg->m2.control_channel = 1;
  
  cfg->resolver_id = 0;
  cfg->lock_sense_id = 0;
  cfg->unlock_sense_id = 1;
  
  cfg = &nvmParams.control_param[AXIS_EL];
  cfg->m1.en_id = 4;
  cfg->m1.svon_id =5 ;
  cfg->m1.rdy_id = 6;
  cfg->m1.control_channel = 2;

  cfg->m2.en_id = 6;
  cfg->m2.svon_id = 7;
  cfg->m2.rdy_id = 7;
  cfg->m2.control_channel = 3;
  
  cfg->resolver_id = 1;
  cfg->lock_sense_id = 2;
  cfg->unlock_sense_id = 3;
  
  
  
}
static void save_control_param()
{
  uint16_t crc = 0x0;
  uint16_t len = sizeof(struct control_param_s)*NOF_AXES;
  crc = crc16_ccitt((uint8_t*)&nvmParams.control_param,len,crc);
  nvmParams.crc16.crc16[DATA_SECTION_CONTROL_PARAM] = crc;
  eepromWrite(DATA_FLASH_CONTROL_PARAM_OFFSET, len, (uint8_t*)&nvmParams.control_param);
  save_crc_param();
}
static void load_control_param(uint16_t crc_stored)
{
  uint16_t crc = 0x0;
  uint16_t len = sizeof(struct control_param_s);
  eepromRead(DATA_SECTION_CONTROL_PARAM, len, (uint8_t*)&nvmParams.control_param);
  crc = crc16_ccitt((uint8_t*)&nvmParams.control_param,len,crc);
  if(crc_stored != crc){
    load_control_param_default();
    save_control_param();
  }
}
static void load_base_param_default()
{
  struct base_param_s *base = &nvmParams.base_param[AXIS_AZ];
  base->pos_input.min = 0;
  base->pos_input.max = 360;
  base->pos_input.offset = 0;
  base->pos_output.min = -10;
  base->pos_output.max = 10;
  base->pos_output.offset = 0;
  base->spd_input.min = -10;
  base->spd_input.max = 10;
  base->spd_input.offset = 0;
  base->spd_output.min = -10;
  base->spd_output.max = 10;
  base->spd_output.offset = 0;


  base = &nvmParams.base_param[AXIS_EL];
  base->pos_input.min = -15;
  base->pos_input.max = 80;
  base->pos_input.offset = 0;
  base->pos_output.min = -10;
  base->pos_output.max = 10;
  base->pos_output.offset = 0;
  base->spd_input.min = -10;
  base->spd_input.max = 10;
  base->spd_input.offset = 0;
  base->spd_output.min = -10;
  base->spd_output.max = 10;
  base->spd_output.offset = 0;
}
static void save_base_param()
{
  uint16_t crc = 0x0;
  uint16_t len = sizeof(struct base_param_s)*NOF_AXES;
  crc = crc16_ccitt((uint8_t*)&nvmParams.base_param,len,crc);
  nvmParams.crc16.crc16[DATA_SECTION_BASE_PARAM] = crc;
  eepromWrite(DATA_FLASH_BASE_PARAM_OFFSET, len, (uint8_t*)&nvmParams.base_param);
  save_crc_param();
}
static void load_base_param(uint16_t crc_stored)
{
  uint16_t crc = 0x0;
  uint16_t len = sizeof(struct base_param_s);
  eepromRead(DATA_SECTION_BASE_PARAM, len, (uint8_t*)&nvmParams.base_param);
  crc = crc16_ccitt((uint8_t*)&nvmParams.base_param,len,crc);
  if(crc_stored != crc){
    load_base_param_default();
    save_base_param();
  }
}

static void load_canmsg_param_default()
{
  for(uint8_t i=0;i<CANBUS_MESSAGE_COUNT;i++){
    nvmParams.can_msg[i].id = 0x12905300+i*0x80;
    nvmParams.can_msg[i].interval = 10;
  }
}
static void save_canmsg_param()
{
  uint16_t crc = 0x0;
  uint16_t len = sizeof(struct canbus_id_def_s)*CANBUS_MESSAGE_COUNT;
  crc = crc16_ccitt((uint8_t*)&nvmParams.can_msg,len,crc);
  nvmParams.crc16.crc16[DATA_SECTION_CANBUS_MSG] = crc;
  eepromWrite(DATA_FLASH_CANMSG_PARAM_OFFSET, len, (uint8_t*)&nvmParams.can_msg);
  save_crc_param();
}
static void load_canmsg_param(uint16_t crc_stored)
{
  uint16_t crc = 0x0;
  uint16_t len = sizeof(struct canbus_id_def_s)*CANBUS_MESSAGE_COUNT;
  eepromRead(DATA_FLASH_CANMSG_PARAM_OFFSET, len, (uint8_t*)&nvmParams.can_msg);
  crc = crc16_ccitt((uint8_t*)&nvmParams.can_msg,len,crc);
  if(crc_stored != crc){
    load_canmsg_param_default();
    save_canmsg_param();
  }

}


static void load_dac_param_default()
{
  
}
static void save_dac_param()
{
  
}
static void load_dac_param(uint16_t crc_stored)
{
  
}

static void load_board_param_default()
{
  nvmParams.board_param.hw_version = 0x21080000;
  nvmParams.board_param.sw_version = 0x00000001;
  nvmParams.board_param.user_id = 0x01;
  nvmParams.board_param.function_code = 0x00;
  nvmParams.board_param.can1_rate = 0x01;
  nvmParams.board_param.can2_rate = 0x01;
}
static void save_board_param()
{
  uint16_t crc = 0x0;
  uint16_t len = sizeof(struct board_param_s);
  crc = crc16_ccitt((uint8_t*)&nvmParams.board_param,len,crc);
  nvmParams.crc16.crc16[DATA_SECTION_BOARD_PARAM] = crc;
  eepromWrite(DATA_FLASH_BOARD_PARAM_OFFSET, len, (uint8_t*)&nvmParams.board_param);
  save_crc_param();
}
static void load_board_param(uint16_t crc_stored)
{
  uint16_t crc = 0x0;
  uint16_t len = sizeof(struct board_param_s);
  eepromRead(DATA_FLASH_BOARD_PARAM_OFFSET, len, (uint8_t*)&nvmParams.board_param);
  crc = crc16_ccitt((uint8_t*)&nvmParams.board_param,len,crc);
  if(crc_stored != crc){
    load_board_param_default();
    save_board_param();
  }
}


void database_init()
{
  eeprom_init();
  load_crc_param();
  load_config_param(nvmParams.crc16.crc16[DATA_SECTION_CONFIG_PARAM]);
  load_pid_param(nvmParams.crc16.crc16[DATA_SECTION_PID_PARAM]);
  load_stow_param(nvmParams.crc16.crc16[DATA_SECTION_STOW_PARAM]);
  load_control_param(nvmParams.crc16.crc16[DATA_SECTION_CONTROL_PARAM]);
  load_base_param(nvmParams.crc16.crc16[DATA_SECTION_BASE_PARAM]);
  load_canmsg_param(nvmParams.crc16.crc16[DATA_SECTION_CANBUS_MSG]);
  load_board_param(nvmParams.crc16.crc16[DATA_SECTION_BOARD_PARAM]);
  liveData.enableReport = 1;
  
  for(uint8_t i=5;i<CANBUS_MESSAGE_COUNT;i++){
    nvmParams.can_msg[i].interval = 0xFFFF;
  }
  for(uint8_t i=0;i<6;i++){
    nvmParams.can_msg[i].interval = 5;
  }
  
}

void db_save_section(uint8_t section)
{
  switch(section){
  case DATA_SECTION_CONFIG_PARAM:save_config_param();break;
  case DATA_SECTION_PID_PARAM:save_pid_param();break;
  case DATA_SECTION_STOW_PARAM:save_stow_param();break;
  case DATA_SECTION_CONTROL_PARAM:save_control_param();break;
  case DATA_SECTION_BASE_PARAM:save_base_param();break;
  case DATA_SECTION_DAC_PARAM:save_dac_param();break;
  case DATA_SECTION_CANBUS_MSG:save_canmsg_param();break;
  case DATA_SECTION_BOARD_PARAM:save_board_param();break;
  default:break;
  }
}

void db_default_section(uint8_t section)
{
  switch(section){
  case DATA_SECTION_CONFIG_PARAM:load_config_param_default();break;
  case DATA_SECTION_PID_PARAM:load_pid_param_default();break;
  case DATA_SECTION_STOW_PARAM:load_stow_param_default();break;
  case DATA_SECTION_CONTROL_PARAM:load_control_param_default();break;
  case DATA_SECTION_BASE_PARAM:load_base_param_default();break;
  case DATA_SECTION_DAC_PARAM:load_dac_param_default();break;
  case DATA_SECTION_CANBUS_MSG:load_canmsg_param_default();break;
  case DATA_SECTION_BOARD_PARAM:load_board_param_default();break;
  default:break;
  }
}

int8_t db_read_param(uint8_t section, uint8_t index, uint8_t *value)
{
  int sz = 0;
  switch(section){
  case DATA_SECTION_CONFIG_PARAM:
    switch(index){
    case CONFIG_PARAM_PID_AUTO_START:
      sz = 1;
      value = (uint8_t*)&nvmParams.config_param.pid_auto_start;
      break;
    case CONFIG_PARAM_STOW_CYCLE_TIME_MS:
      sz = 2;
      memcpy(value,(uint8_t*)&nvmParams.config_param.stow_cycle_time_ms,sz);
      break;
    case CONFIG_PARAM_STOW_TIMEOUT_MS:
      sz = 2;
      memcpy(value,(uint8_t*)&nvmParams.config_param.stow_timeout_ms,sz);
      break;
    case CONFIG_PARAM_AZ_VALID_MASK:
      sz = 2;
      memcpy(value,(uint8_t*)&nvmParams.config_param.az_valid_mask,sz);
      break;
    case CONFIG_PARAM_EL_VALID_MASK:
      sz = 2;
      memcpy(value,(uint8_t*)&nvmParams.config_param.el_valid_mask,sz);
      break;
    case CONFIG_PARAM_STOW_ACTIVITY_IDLE_TIME_MS:
      sz = 2;
      memcpy(value,(uint8_t*)&nvmParams.config_param.stow_activity_idle_time_ms,sz);
      break;
    case CONFIG_PARAM_CAN_RATE_1:
      sz = 1;
      memcpy(value,(uint8_t*)&nvmParams.config_param.can_rate_1,sz);
      break;
    default:break;
    }
    break;
  case DATA_SECTION_PID_PARAM:
    switch(index){
    case PID_PARAM_AZ_POS_KP: sz = 4;memcpy(value,(uint8_t*)&nvmParams.pid_param[0].pos_kp,sz); break;
    case PID_PARAM_AZ_POS_KI: sz = 4;memcpy(value,(uint8_t*)&nvmParams.pid_param[0].pos_ki,sz); break;
    case PID_PARAM_AZ_POS_KD: sz = 4;memcpy(value,(uint8_t*)&nvmParams.pid_param[0].pos_kd,sz); break;
    case PID_PARAM_AZ_SPD_KP: sz = 4;memcpy(value,(uint8_t*)&nvmParams.pid_param[0].spd_kp,sz); break;
    case PID_PARAM_AZ_SPD_KI: sz = 4;memcpy(value,(uint8_t*)&nvmParams.pid_param[0].spd_ki,sz); break;
    case PID_PARAM_AZ_SPD_KD: sz = 4;memcpy(value,(uint8_t*)&nvmParams.pid_param[0].spd_kd,sz); break;
    case PID_PARAM_EL_POS_KP: sz = 4;memcpy(value,(uint8_t*)&nvmParams.pid_param[1].pos_kp,sz); break;
    case PID_PARAM_EL_POS_KI: sz = 4;memcpy(value,(uint8_t*)&nvmParams.pid_param[1].pos_ki,sz); break;
    case PID_PARAM_EL_POS_KD: sz = 4;memcpy(value,(uint8_t*)&nvmParams.pid_param[1].pos_kd,sz); break;
    case PID_PARAM_EL_SPD_KP: sz = 4;memcpy(value,(uint8_t*)&nvmParams.pid_param[1].spd_kp,sz); break;
    case PID_PARAM_EL_SPD_KI: sz = 4;memcpy(value,(uint8_t*)&nvmParams.pid_param[1].spd_ki,sz); break;
    case PID_PARAM_EL_SPD_KD: sz = 4;memcpy(value,(uint8_t*)&nvmParams.pid_param[1].spd_kd,sz); break;
    default:break;
    }
    break;
  case DATA_SECTION_STOW_PARAM:
    switch(index){
    case STOW_PARAM_POWER_EN_ID:    sz = 1;memcpy(value ,(uint8_t*)&nvmParams.stow_param.pwr_output_id,sz);break;
    case STOW_PARAM_DIRECTION_ID:   sz = 1;memcpy(value ,(uint8_t*)&nvmParams.stow_param.dire_output_id,sz);break;
    case STOW_PARAM_AZEL_SELECTION: sz = 1;memcpy(value ,(uint8_t*)&nvmParams.stow_param.az_el_select_id,sz);break;
    default:break;
    }
    break;
  case DATA_SECTION_CONTROL_PARAM:
    switch(index){
    case CONTROL_PARAM_AZ_M1_EN_ID:*value = nvmParams.control_param[0].m1.en_id;sz = 1;break;
    case CONTROL_PARAM_AZ_M1_SVON_ID:*value = nvmParams.control_param[0].m1.svon_id;sz = 1;break;
    case CONTROL_PARAM_AZ_M1_RDY_ID:*value = nvmParams.control_param[0].m1.rdy_id;sz = 1;break;
    case CONTROL_PARAM_AZ_M1_OUTPUT_CHANNEL:*value = nvmParams.control_param[0].m1.control_channel;sz = 1;break;
    case CONTROL_PARAM_AZ_M2_EN_ID:*value = nvmParams.control_param[0].m2.en_id;sz = 1;break;
    case CONTROL_PARAM_AZ_M2_SVON_ID:*value = nvmParams.control_param[0].m2.svon_id;sz = 1;break;
    case CONTROL_PARAM_AZ_M2_RDY_ID:*value = nvmParams.control_param[0].m2.rdy_id;sz = 1;break;
    case CONTROL_PARAM_AZ_M2_OUTPUT_CHANNEL:*value = nvmParams.control_param[0].m2.control_channel;sz = 1;break;
    case CONTROL_PARAM_AZ_LOCK_ID:*value = nvmParams.control_param[0].lock_sense_id;sz = 1;break;
    case CONTROL_PARAM_AZ_UNLOCK_ID:*value = nvmParams.control_param[0].unlock_sense_id;sz = 1;break;
    case CONTROL_PARAM_AZ_RV_INPUT_CHANNEL:*value = nvmParams.control_param[0].resolver_id;sz = 1;break;
    case CONTROL_PARAM_EL_M1_EN_ID:*value = nvmParams.control_param[1].m1.en_id;sz = 1;break;
    case CONTROL_PARAM_EL_M1_SVON_ID:*value = nvmParams.control_param[1].m1.svon_id;sz = 1;break;
    case CONTROL_PARAM_EL_M1_RDY_ID:*value = nvmParams.control_param[1].m1.rdy_id;sz = 1;break;
    case CONTROL_PARAM_EL_M1_OUTPUT_CHANNEL:*value = nvmParams.control_param[1].m1.control_channel;sz = 1;break;
    case CONTROL_PARAM_EL_M2_EN_ID:*value = nvmParams.control_param[1].m2.en_id;sz = 1;break;
    case CONTROL_PARAM_EL_M2_SVON_ID:*value = nvmParams.control_param[1].m2.svon_id;sz = 1;break;
    case CONTROL_PARAM_EL_M2_RDY_ID:*value = nvmParams.control_param[1].m2.rdy_id;sz = 1;break;
    case CONTROL_PARAM_EL_M2_OUTPUT_CHANNEL:*value = nvmParams.control_param[1].m2.control_channel;sz = 1;break;
    case CONTROL_PARAM_EL_LOCK_ID:*value = nvmParams.control_param[1].lock_sense_id;sz = 1;break;
    case CONTROL_PARAM_EL_UNLOCK_ID:*value = nvmParams.control_param[1].unlock_sense_id;sz = 1;break;
    case CONTROL_PARAM_EL_RV_INPUT_CHANNEL:*value = nvmParams.control_param[1].resolver_id;sz = 1;break;
    }
    break;
  case DATA_SECTION_BASE_PARAM:
    sz = 4;
    switch(index){
    case BASE_PARAM_AZ_POS_INPUT_MIN:    memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_AZ].pos_input.min,sz);break;
    case BASE_PARAM_AZ_POS_INPUT_MAX:    memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_AZ].pos_input.max,sz);break;
    case BASE_PARAM_AZ_POS_INPUT_OFFSET: memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_AZ].pos_input.offset,sz);break;
    case BASE_PARAM_AZ_POS_OUTPUT_MIN:   memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_AZ].pos_output.min,sz);break;
    case BASE_PARAM_AZ_POS_OUTPUT_MAX:   memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_AZ].pos_output.max,sz);break;
    case BASE_PARAM_AZ_POS_OUTPUT_OFFSET:memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_AZ].pos_output.offset,sz);break;
    case BASE_PARAM_AZ_SPD_INPUT_MIN:    memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_AZ].spd_input.min,sz);break;
    case BASE_PARAM_AZ_SPD_INPUT_MAX:    memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_AZ].spd_input.max,sz);break;
    case BASE_PARAM_AZ_SPD_INPUT_OFFSET: memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_AZ].spd_input.offset,sz);break;
    case BASE_PARAM_AZ_SPD_OUTPUT_MIN:   memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_AZ].spd_output.min,sz);break;
    case BASE_PARAM_AZ_SPD_OUTPUT_MAX:   memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_AZ].spd_output.max,sz);break;
    case BASE_PARAM_AZ_SPD_OUTPUT_OFFSET:memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_AZ].spd_output.offset,sz);break;
    case BASE_PARAM_EL_POS_INPUT_MIN:    memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_EL].pos_input.min,sz);break;
    case BASE_PARAM_EL_POS_INPUT_MAX:    memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_EL].pos_input.max,sz);break;
    case BASE_PARAM_EL_POS_INPUT_OFFSET: memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_EL].pos_input.offset,sz);break;
    case BASE_PARAM_EL_POS_OUTPUT_MIN:   memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_EL].pos_output.min,sz);break;
    case BASE_PARAM_EL_POS_OUTPUT_MAX:   memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_EL].pos_output.max,sz);break;
    case BASE_PARAM_EL_POS_OUTPUT_OFFSET:memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_EL].pos_output.offset,sz);break;
    case BASE_PARAM_EL_SPD_INPUT_MIN:    memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_EL].spd_input.min,sz);break;
    case BASE_PARAM_EL_SPD_INPUT_MAX:    memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_EL].spd_input.max,sz);break;
    case BASE_PARAM_EL_SPD_INPUT_OFFSET: memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_EL].spd_input.offset,sz);break;
    case BASE_PARAM_EL_SPD_OUTPUT_MIN:   memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_EL].spd_output.min,sz);break;
    case BASE_PARAM_EL_SPD_OUTPUT_MAX:   memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_EL].spd_output.max,sz);break;
    case BASE_PARAM_EL_SPD_OUTPUT_OFFSET:memcpy(value,(uint8_t *)&nvmParams.base_param[AXIS_EL].spd_output.offset,sz);break;
    }
    break;
  case DATA_SECTION_DAC_PARAM:
    break;
  case DATA_SECTION_CANBUS_MSG:
    if(index < CANBUS_MSG_INTERVAL){
      sz= 4;
      memcpy(value, (void*)&nvmParams.can_msg[index].id,4);
    }
    else{
      sz = 2;
      memcpy(value, (void*)&nvmParams.can_msg[index-CANBUS_MSG_INTERVAL].interval,2);
    }
    break;
  case DATA_SECTION_BOARD_PARAM:
    switch(index){
    case BOARD_PARAM_HW_VERSION: sz=4; memcpy(value,(uint8_t *)&nvmParams.board_param.hw_version,sz);break;
    case BOARD_PARAM_SW_VERSION: sz=4; memcpy(value,(uint8_t *)&nvmParams.board_param.sw_version,sz);break;
    case BOARD_PARAM_USER_ID:    sz=4; memcpy(value,(uint8_t *)&nvmParams.board_param.user_id,sz)   ;break;
    case BOARD_PARAM_FUNCTION_CODE: sz=4; memcpy(value,(uint8_t *)&nvmParams.board_param.function_code,sz);break;
    case BOARD_PARAM_CAN1_BITRATE: sz=1; memcpy(value,(uint8_t *)&nvmParams.board_param.can1_rate,sz);break;
    case BOARD_PARAM_CAN2_BITRATE: sz=1; memcpy(value,(uint8_t *)&nvmParams.board_param.can2_rate,sz);break;
    default:break;
    }
    break;
  default:break;
  }
  return sz;
}

int8_t db_write_param(uint8_t section, uint8_t index, uint8_t *value)
{
  int8_t sz = -1;
  float fv;
  switch(section){
  case DATA_SECTION_CONFIG_PARAM:
    switch(index){
    case CONFIG_PARAM_PID_AUTO_START: memcpy((uint8_t*)&nvmParams.config_param.pid_auto_start,value,1);break;
    case CONFIG_PARAM_STOW_CYCLE_TIME_MS: memcpy((uint8_t*)&nvmParams.config_param.stow_cycle_time_ms,value,2);break;
    case CONFIG_PARAM_STOW_TIMEOUT_MS: memcpy((uint8_t*)&nvmParams.config_param.stow_timeout_ms,value,2);break;
    case CONFIG_PARAM_AZ_VALID_MASK: memcpy((uint8_t*)&nvmParams.config_param.az_valid_mask,value,2);break;
    case CONFIG_PARAM_EL_VALID_MASK: memcpy((uint8_t*)&nvmParams.config_param.el_valid_mask,value,2);break;
    case CONFIG_PARAM_STOW_ACTIVITY_IDLE_TIME_MS: memcpy((uint8_t*)&nvmParams.config_param.stow_activity_idle_time_ms,value,2);break;
    case CONFIG_PARAM_CAN_RATE_1: memcpy((uint8_t*)&nvmParams.config_param.can_rate_1,value,1);break;
    }
    break;
  case DATA_SECTION_PID_PARAM:
    switch(index){
    case PID_PARAM_AZ_POS_KP:memcpy((void*)&nvmParams.pid_param[0].pos_kp,value,4);break;
    case PID_PARAM_AZ_POS_KI:memcpy((void*)&nvmParams.pid_param[0].pos_ki,value,4);break;
    case PID_PARAM_AZ_POS_KD:memcpy((void*)&nvmParams.pid_param[0].pos_kd,value,4);break;
    case PID_PARAM_AZ_SPD_KP:memcpy((void*)&nvmParams.pid_param[0].spd_kp,value,4);break;
    case PID_PARAM_AZ_SPD_KI:memcpy((void*)&nvmParams.pid_param[0].spd_ki,value,4);break;
    case PID_PARAM_AZ_SPD_KD:memcpy((void*)&nvmParams.pid_param[0].spd_kd,value,4);break;
    case PID_PARAM_EL_POS_KP:memcpy((void*)&nvmParams.pid_param[1].pos_kp,value,4);break;
    case PID_PARAM_EL_POS_KI:memcpy((void*)&nvmParams.pid_param[1].pos_ki,value,4);break;
    case PID_PARAM_EL_POS_KD:memcpy((void*)&nvmParams.pid_param[1].pos_kd,value,4);break;
    case PID_PARAM_EL_SPD_KP:memcpy((void*)&nvmParams.pid_param[1].spd_kp,value,4);break;
    case PID_PARAM_EL_SPD_KI:memcpy((void*)&nvmParams.pid_param[1].spd_ki,value,4);break;
    case PID_PARAM_EL_SPD_KD:memcpy((void*)&nvmParams.pid_param[1].spd_kd,value,4);break;
    }
    break;
  case DATA_SECTION_STOW_PARAM:
    switch(index){
    case STOW_PARAM_POWER_EN_ID:memcpy((void*)&nvmParams.stow_param.pwr_output_id,value,1);break;
    case STOW_PARAM_DIRECTION_ID:memcpy((void*)&nvmParams.stow_param.dire_output_id,value,1);break;
    case STOW_PARAM_AZEL_SELECTION:memcpy((void*)&nvmParams.stow_param.az_el_select_id,value,1);break;
    }
    break;
  case DATA_SECTION_CONTROL_PARAM:
    switch(index){
    case CONTROL_PARAM_AZ_M1_EN_ID:memcpy((void*)&nvmParams.control_param[0].m1.en_id,value,1);break;
    case CONTROL_PARAM_AZ_M1_SVON_ID:memcpy((void*)&nvmParams.control_param[0].m1.svon_id,value,1);break;
    case CONTROL_PARAM_AZ_M1_RDY_ID:memcpy((void*)&nvmParams.control_param[0].m1.rdy_id,value,1);break;
    case CONTROL_PARAM_AZ_M2_EN_ID:memcpy((void*)&nvmParams.control_param[0].m2.en_id,value,1);break;
    case CONTROL_PARAM_AZ_M2_SVON_ID:memcpy((void*)&nvmParams.control_param[0].m2.svon_id,value,1);break;
    case CONTROL_PARAM_AZ_M2_RDY_ID:memcpy((void*)&nvmParams.control_param[0].m2.rdy_id,value,1);break;
    case CONTROL_PARAM_AZ_LOCK_ID:memcpy((void*)&nvmParams.control_param[0].lock_sense_id,value,1);break;
    case CONTROL_PARAM_AZ_UNLOCK_ID:memcpy((void*)&nvmParams.control_param[0].unlock_sense_id,value,1);break;
    case CONTROL_PARAM_AZ_M1_OUTPUT_CHANNEL:memcpy((void*)&nvmParams.control_param[0].m1.control_channel,value,1);break;
    case CONTROL_PARAM_AZ_M2_OUTPUT_CHANNEL:memcpy((void*)&nvmParams.control_param[0].m2.control_channel,value,1);break;
    case CONTROL_PARAM_EL_M1_EN_ID:memcpy((void*)&nvmParams.control_param[1].m1.en_id,value,1);break;
    case CONTROL_PARAM_EL_M1_SVON_ID:memcpy((void*)&nvmParams.control_param[1].m1.svon_id,value,1);break;
    case CONTROL_PARAM_EL_M1_RDY_ID:memcpy((void*)&nvmParams.control_param[1].m1.rdy_id,value,1);break;
    case CONTROL_PARAM_EL_M2_EN_ID:memcpy((void*)&nvmParams.control_param[1].m2.en_id,value,1);break;
    case CONTROL_PARAM_EL_M2_SVON_ID:memcpy((void*)&nvmParams.control_param[1].m2.svon_id,value,1);break;
    case CONTROL_PARAM_EL_M2_RDY_ID:memcpy((void*)&nvmParams.control_param[1].m2.rdy_id,value,1);break;
    case CONTROL_PARAM_EL_LOCK_ID:memcpy((void*)&nvmParams.control_param[1].lock_sense_id,value,1);break;
    case CONTROL_PARAM_EL_UNLOCK_ID:memcpy((void*)&nvmParams.control_param[1].unlock_sense_id,value,1);break;
    case CONTROL_PARAM_EL_M1_OUTPUT_CHANNEL:memcpy((void*)&nvmParams.control_param[1].m1.control_channel,value,1);break;
    case CONTROL_PARAM_EL_M2_OUTPUT_CHANNEL:memcpy((void*)&nvmParams.control_param[1].m2.control_channel,value,1);break;
    }
    break;
  case DATA_SECTION_BASE_PARAM:
    switch(index){
    case BASE_PARAM_AZ_POS_INPUT_MIN:memcpy((void*)&nvmParams.base_param[AXIS_AZ].pos_input.min,value,4);break;
    case BASE_PARAM_AZ_POS_INPUT_MAX:memcpy((void*)&nvmParams.base_param[AXIS_AZ].pos_input.max,value,4);break;
    case BASE_PARAM_AZ_POS_INPUT_OFFSET:memcpy((void*)&nvmParams.base_param[AXIS_AZ].pos_input.offset,value,4);break;
    case BASE_PARAM_AZ_POS_OUTPUT_MIN:memcpy((void*)&nvmParams.base_param[AXIS_AZ].pos_output.min,value,4);break;
    case BASE_PARAM_AZ_POS_OUTPUT_MAX:memcpy((void*)&nvmParams.base_param[AXIS_AZ].pos_output.max,value,4);break;
    case BASE_PARAM_AZ_POS_OUTPUT_OFFSET:memcpy((void*)&nvmParams.base_param[AXIS_AZ].pos_output.offset,value,4);break;
    case BASE_PARAM_AZ_SPD_INPUT_MIN:memcpy((void*)&nvmParams.base_param[AXIS_AZ].spd_input.min,value,4);break;
    case BASE_PARAM_AZ_SPD_INPUT_MAX:memcpy((void*)&nvmParams.base_param[AXIS_AZ].spd_input.max,value,4);break;
    case BASE_PARAM_AZ_SPD_INPUT_OFFSET:memcpy((void*)&nvmParams.base_param[AXIS_AZ].spd_input.offset,value,4);break;
    case BASE_PARAM_AZ_SPD_OUTPUT_MIN:memcpy((void*)&nvmParams.base_param[AXIS_AZ].spd_output.min,value,4);break;
    case BASE_PARAM_AZ_SPD_OUTPUT_MAX:memcpy((void*)&nvmParams.base_param[AXIS_AZ].spd_output.max,value,4);break;
    case BASE_PARAM_AZ_SPD_OUTPUT_OFFSET:memcpy((void*)&nvmParams.base_param[AXIS_AZ].spd_output.offset,value,4);break;
    case BASE_PARAM_EL_POS_INPUT_MIN:memcpy((void*)&nvmParams.base_param[AXIS_EL].pos_input.min,value,4);break;
    case BASE_PARAM_EL_POS_INPUT_MAX:memcpy((void*)&nvmParams.base_param[AXIS_EL].pos_input.max,value,4);break;
    case BASE_PARAM_EL_POS_INPUT_OFFSET:memcpy((void*)&nvmParams.base_param[AXIS_EL].pos_input.offset,value,4);break;
    case BASE_PARAM_EL_POS_OUTPUT_MIN:memcpy((void*)&nvmParams.base_param[AXIS_EL].pos_output.min,value,4);break;
    case BASE_PARAM_EL_POS_OUTPUT_MAX:memcpy((void*)&nvmParams.base_param[AXIS_EL].pos_output.max,value,4);break;
    case BASE_PARAM_EL_POS_OUTPUT_OFFSET:memcpy((void*)&nvmParams.base_param[AXIS_EL].pos_output.offset,value,4);break;
    case BASE_PARAM_EL_SPD_INPUT_MIN:memcpy((void*)&nvmParams.base_param[AXIS_EL].spd_input.min,value,4);break;
    case BASE_PARAM_EL_SPD_INPUT_MAX:memcpy((void*)&nvmParams.base_param[AXIS_EL].spd_input.max,value,4);break;
    case BASE_PARAM_EL_SPD_INPUT_OFFSET:memcpy((void*)&nvmParams.base_param[AXIS_EL].spd_input.offset,value,4);break;
    case BASE_PARAM_EL_SPD_OUTPUT_MIN:memcpy((void*)&nvmParams.base_param[AXIS_EL].spd_output.min,value,4);break;
    case BASE_PARAM_EL_SPD_OUTPUT_MAX:memcpy((void*)&nvmParams.base_param[AXIS_EL].spd_output.max,value,4);break;
    case BASE_PARAM_EL_SPD_OUTPUT_OFFSET:memcpy((void*)&nvmParams.base_param[AXIS_EL].spd_output.offset,value,4);break;
    }
    break;
  case DATA_SECTION_DAC_PARAM:
    break;
  case DATA_SECTION_BOARD_PARAM:
    switch(index)
    {
    case BOARD_PARAM_HW_VERSION:sz = 4; memcpy((void*)&nvmParams.board_param.hw_version,value,sz);break;
    case BOARD_PARAM_SW_VERSION:sz = 4; memcpy((void*)&nvmParams.board_param.sw_version,value,sz);break;
    case BOARD_PARAM_USER_ID:sz = 4; memcpy((void*)&nvmParams.board_param.user_id,value,sz);break;
    case BOARD_PARAM_FUNCTION_CODE:sz = 4; memcpy((void*)&nvmParams.board_param.function_code,value,sz);break;
    case BOARD_PARAM_CAN1_BITRATE:sz = 1; memcpy((void*)&nvmParams.board_param.can1_rate,value,sz);break;
    case BOARD_PARAM_CAN2_BITRATE:sz = 1; memcpy((void*)&nvmParams.board_param.can2_rate,value,sz);break;
    default:break;
    }
    break;
  case DATA_SECTION_COMMAND:
    switch(index){
    case NVM_CMD_SAVE_SECTION:db_save_section(*value);break;
    case NVM_CMD_DEFAULT_SECTION:db_default_section(*value);break;
    case NVM_CMD_REBOOT:break;
    case NVM_CMD_REPORT:can0_send_section(*value);break;
    case NVM_CMD_UPDATE_PID:nested_pid_update_param(*value);break;
    case NVM_CMD_USER_CONTROL:
      
      break;
    case NVM_CMD_AZ_POS_SP:
//      memcpy((void*)&fv,value,4);
      db_write_live_data(LIVEDATA_SECTION_COMMAND,LIVEDATA_COMMAND_AZ_SP_POS,value);
      //nested_pid_set_pos_command(AXIS_AZ,fv);
      break;
    case NVM_CMD_AZ_SPD_SP:
      db_write_live_data(LIVEDATA_SECTION_COMMAND,LIVEDATA_COMMAND_AZ_SP_SPD,value);
//      memcpy((void*)&fv,value,4);
//      nested_pid_set_spd_command(AXIS_AZ,fv);
      break;
    case NVM_CMD_EL_POS_SP:
      db_write_live_data(LIVEDATA_SECTION_COMMAND,LIVEDATA_COMMAND_EL_SP_POS,value);
//      memcpy((void*)&fv,value,4);
//      nested_pid_set_pos_command(AXIS_EL,fv);
      break;
    case NVM_CMD_EL_SPD_SP:
      db_write_live_data(LIVEDATA_SECTION_COMMAND,LIVEDATA_COMMAND_EL_SP_SPD,value);
//      memcpy((void*)&fv,value,4);
//      nested_pid_set_spd_command(AXIS_EL,fv);
      break;
    case NVM_CMD_PID_MODE:
      nested_pid_set_mode(value[0],value[1]);
      break;
    case NVM_CMD_REPORT_CONFIG:
      memcpy((void*)&liveData.enableReport,value,4);
      
      if(liveData.enableReport & 0x10000000){
        nvmParams.can_msg[CANBUS_MSG_ID_AZ_PV].interval = 1;
        nvmParams.can_msg[CANBUS_MSG_ID_AZ_MC_ERR].interval = 1;
        nvmParams.can_msg[CANBUS_MSG_ID_EL_PV].interval = 4;
        nvmParams.can_msg[CANBUS_MSG_ID_EL_MC_ERR].interval = 4;
        can0_reload_message_struct();
      }
      else if(liveData.enableReport & 0x20000000){
        nvmParams.can_msg[CANBUS_MSG_ID_AZ_PV].interval = 4;
        nvmParams.can_msg[CANBUS_MSG_ID_AZ_MC_ERR].interval = 4;
        nvmParams.can_msg[CANBUS_MSG_ID_EL_PV].interval = 1;
        nvmParams.can_msg[CANBUS_MSG_ID_EL_MC_ERR].interval = 1;
        can0_reload_message_struct();
      }
      else{
        nvmParams.can_msg[CANBUS_MSG_ID_AZ_PV].interval = 4;
        nvmParams.can_msg[CANBUS_MSG_ID_AZ_MC_ERR].interval = 4;
        nvmParams.can_msg[CANBUS_MSG_ID_EL_PV].interval = 4;
        nvmParams.can_msg[CANBUS_MSG_ID_EL_MC_ERR].interval = 4;
        can0_reload_message_struct();
      }
      break;
    default:break;
    }
    break;
  default:break;
  }
  
  return sz;
}

int8_t db_read_live_data(uint8_t section, uint8_t index, uint8_t *value)
{
  int sz = -1;
  switch(section){
  case LIVEDATA_SECTION_COMMAND:
    switch(index){
    case LIVEDATA_COMMAND_AZ_SP_POS: memcpy(value, (void*)&liveData.sp[AXIS_AZ].pos,4);sz=4;break;
    case LIVEDATA_COMMAND_AZ_SP_SPD: memcpy(value, (void*)&liveData.sp[AXIS_AZ].speed,4);sz=4;break;
    case LIVEDATA_COMMAND_AZ_POS_ERR: memcpy(value, (void*)&liveData.sp[AXIS_AZ].pos_err,4);sz=4;break;
    case LIVEDATA_COMMAND_AZ_SPD_ERR: memcpy(value, (void*)&liveData.sp[AXIS_AZ].speed_err,4);sz=4;break;
    case LIVEDATA_COMMAND_EL_SP_POS: memcpy(value, (void*)&liveData.sp[AXIS_EL].pos,4);sz=4;break;
    case LIVEDATA_COMMAND_EL_SP_SPD: memcpy(value, (void*)&liveData.sp[AXIS_EL].speed,4);sz=4;break;
    case LIVEDATA_COMMAND_EL_POS_ERR: memcpy(value, (void*)&liveData.sp[AXIS_EL].pos_err,4);sz=4;break;
    case LIVEDATA_COMMAND_EL_SPD_ERR: memcpy(value, (void*)&liveData.sp[AXIS_EL].speed_err,4);sz=4;break;
    default:break;
    }
    break;
  case LIVEDATA_SECTION_ANALOG_IN:
    switch(index){
    case LIVEDATA_VIN_RAW_CH0: memcpy(value, (void*)&liveData.analog_inputs[0].rawData,4);sz=4;break;
    case LIVEDATA_VIN_RAW_CH1: memcpy(value, (void*)&liveData.analog_inputs[1].rawData,4);sz=4;break;
    case LIVEDATA_VIN_RAW_CH2: memcpy(value, (void*)&liveData.analog_inputs[2].rawData,4);sz=4;break;
    case LIVEDATA_VIN_RAW_CH3: memcpy(value, (void*)&liveData.analog_inputs[3].rawData,4);sz=4;break;
    case LIVEDATA_VIN_RAW_CH4: memcpy(value, (void*)&liveData.analog_inputs[4].rawData,4);sz=4;break;
    case LIVEDATA_VIN_RAW_CH5: memcpy(value, (void*)&liveData.analog_inputs[5].rawData,4);sz=4;break;
    case LIVEDATA_VIN_RAW_CH6: memcpy(value, (void*)&liveData.analog_inputs[6].rawData,4);sz=4;break;
    case LIVEDATA_VIN_RAW_CH7: memcpy(value, (void*)&liveData.analog_inputs[7].rawData,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH0: memcpy(value, (void*)&liveData.analog_inputs[0].scaledData,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH1: memcpy(value, (void*)&liveData.analog_inputs[1].scaledData,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH2: memcpy(value, (void*)&liveData.analog_inputs[2].scaledData,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH3: memcpy(value, (void*)&liveData.analog_inputs[3].scaledData,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH4: memcpy(value, (void*)&liveData.analog_inputs[4].scaledData,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH5: memcpy(value, (void*)&liveData.analog_inputs[5].scaledData,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH6: memcpy(value, (void*)&liveData.analog_inputs[6].scaledData,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH7: memcpy(value, (void*)&liveData.analog_inputs[7].scaledData,4);sz=4;break;
    default:break;
    }
    break;
  case LIVDDATA_SECTION_ANALOG_OUT:
    switch(index){
    case LIVEDATA_VOUT_RAW_CH0: memcpy(value, (void*)&liveData.analog_outputs[0].rawData,2);sz=2;break;
    case LIVEDATA_VOUT_RAW_CH1: memcpy(value, (void*)&liveData.analog_outputs[1].rawData,2);sz=2;break;
    case LIVEDATA_VOUT_RAW_CH2: memcpy(value, (void*)&liveData.analog_outputs[2].rawData,2);sz=2;break;
    case LIVEDATA_VOUT_RAW_CH3: memcpy(value, (void*)&liveData.analog_outputs[3].rawData,2);sz=2;break;
//    case LIVEDATA_VOUT_RAW_CH0: memcpy(value, (void*)&liveData.analog_outputs[0].rawData,4);sz=4;break;
//    case LIVEDATA_VOUT_RAW_CH1: memcpy(value, (void*)&liveData.analog_outputs[1].rawData,4);sz=4;break;
//    case LIVEDATA_VOUT_RAW_CH2: memcpy(value, (void*)&liveData.analog_outputs[2].rawData,4);sz=4;break;
//    case LIVEDATA_VOUT_RAW_CH3: memcpy(value, (void*)&liveData.analog_outputs[3].rawData,4);sz=4;break;
    case LIVEDATA_VOUT_VOLT_CH0: memcpy(value, (void*)&liveData.analog_outputs[0].scaledData,4);sz=4;break;
    case LIVEDATA_VOUT_VOLT_CH1: memcpy(value, (void*)&liveData.analog_outputs[1].scaledData,4);sz=4;break;
    case LIVEDATA_VOUT_VOLT_CH2: memcpy(value, (void*)&liveData.analog_outputs[2].scaledData,4);sz=4;break;
    case LIVEDATA_VOUT_VOLT_CH3: memcpy(value, (void*)&liveData.analog_outputs[3].scaledData,4);sz=4;break;
    default:break;
    }
    break;
  case LIVEDATA_SECTION_RESOLVER:
    switch(index){
    case LIVEDATA_AZ_RAD: memcpy(value, (void*)&liveData.pv[AXIS_AZ].pos,4);sz=4;break;
    case LIVEDATA_AZ_DPS: memcpy(value, (void*)&liveData.pv[AXIS_AZ].speed,4);sz=4;break;
    case LIVEDATA_EL_RAD: memcpy(value, (void*)&liveData.pv[AXIS_EL].pos,4);sz=4;break;
    case LIVEDATA_EL_DPS: memcpy(value, (void*)&liveData.pv[AXIS_EL].speed,4);sz=4;break;
    default:break;
    }
    break;
  case LIVEDATA_SECTION_SYSTEM:
    switch(index){
    case LIVEDATA_MC_STATUS:memcpy(value, (void*)&liveData.mcStatus,4);sz=4;break;
    default:break;
    }
    break;
  case LIVEDATA_SECTION_DIGITAL_IO:
    switch(index){
    case LIVEDATA_DIGITAL_INPUT_WORD:memcpy(value, (void*)&liveData.digital_input,2);sz=2;break;
    case LIVEDATA_DIGITAL_OUTPUT_WORD:memcpy(value, (void*)&liveData.digital_output,2);sz=2;break;
    default:break;
    }
    break;
  }
  
  return sz;
}

int8_t db_write_live_data(uint8_t section, uint8_t index, uint8_t *value)
{
  int sz = -1;
  switch(section){
  case LIVEDATA_SECTION_COMMAND:
    switch(index){
    case LIVEDATA_COMMAND_AZ_SP_POS: memcpy((void*)&liveData.sp[AXIS_AZ].pos,value,4);sz=4;break;
    case LIVEDATA_COMMAND_AZ_SP_SPD: memcpy((void*)&liveData.sp[AXIS_AZ].speed,value,4);sz=4;break;
    case LIVEDATA_COMMAND_EL_SP_POS: memcpy((void*)&liveData.sp[AXIS_EL].pos,value,4);sz=4;break;
    case LIVEDATA_COMMAND_EL_SP_SPD: memcpy((void*)&liveData.sp[AXIS_EL].speed,value,4);sz=4;break;
    case LIVEDATA_COMMAND_AZ_POS_ERR: memcpy((void*)&liveData.sp[AXIS_AZ].pos_err,value,4);sz=4;break;
    case LIVEDATA_COMMAND_AZ_SPD_ERR: memcpy((void*)&liveData.sp[AXIS_AZ].speed_err,value,4);sz=4;break;
    case LIVEDATA_COMMAND_EL_POS_ERR: memcpy((void*)&liveData.sp[AXIS_EL].pos_err,value,4);sz=4;break;
    case LIVEDATA_COMMAND_EL_SPD_ERR: memcpy((void*)&liveData.sp[AXIS_EL].speed_err,value,4);sz=4;break;
    default:break;
    }
    break;
  case LIVEDATA_SECTION_ANALOG_IN:
    switch(index){
    case LIVEDATA_VIN_RAW_CH0: memcpy((void*)&liveData.analog_inputs[0].rawData,value,4);sz=4;break;
    case LIVEDATA_VIN_RAW_CH1: memcpy((void*)&liveData.analog_inputs[1].rawData,value,4);sz=4;break;
    case LIVEDATA_VIN_RAW_CH2: memcpy((void*)&liveData.analog_inputs[2].rawData,value,4);sz=4;break;
    case LIVEDATA_VIN_RAW_CH3: memcpy((void*)&liveData.analog_inputs[3].rawData,value,4);sz=4;break;
    case LIVEDATA_VIN_RAW_CH4: memcpy((void*)&liveData.analog_inputs[4].rawData,value,4);sz=4;break;
    case LIVEDATA_VIN_RAW_CH5: memcpy((void*)&liveData.analog_inputs[5].rawData,value,4);sz=4;break;
    case LIVEDATA_VIN_RAW_CH6: memcpy((void*)&liveData.analog_inputs[6].rawData,value,4);sz=4;break;
    case LIVEDATA_VIN_RAW_CH7: memcpy((void*)&liveData.analog_inputs[7].rawData,value,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH0: memcpy((void*)&liveData.analog_inputs[0].scaledData,value,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH1: memcpy((void*)&liveData.analog_inputs[1].scaledData,value,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH2: memcpy((void*)&liveData.analog_inputs[2].scaledData,value,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH3: memcpy((void*)&liveData.analog_inputs[3].scaledData,value,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH4: memcpy((void*)&liveData.analog_inputs[4].scaledData,value,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH5: memcpy((void*)&liveData.analog_inputs[5].scaledData,value,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH6: memcpy((void*)&liveData.analog_inputs[6].scaledData,value,4);sz=4;break;
    case LIVEDATA_VIN_VOLT_CH7: memcpy((void*)&liveData.analog_inputs[7].scaledData,value,4);sz=4;break;
    default:break;
    }
    break;
  case LIVDDATA_SECTION_ANALOG_OUT:
    switch(index){
    case LIVEDATA_VOUT_RAW_CH0: memcpy((void*)&liveData.analog_outputs[0].rawData,value,2);sz=2;break;
    case LIVEDATA_VOUT_RAW_CH1: memcpy((void*)&liveData.analog_outputs[1].rawData,value,2);sz=2;break;
    case LIVEDATA_VOUT_RAW_CH2: memcpy((void*)&liveData.analog_outputs[2].rawData,value,2);sz=2;break;
    case LIVEDATA_VOUT_RAW_CH3: memcpy((void*)&liveData.analog_outputs[3].rawData,value,2);sz=2;break;
    case LIVEDATA_VOUT_VOLT_CH0: memcpy((void*)&liveData.analog_outputs[0].scaledData,value,4);sz=4;break;
    case LIVEDATA_VOUT_VOLT_CH1: memcpy((void*)&liveData.analog_outputs[1].scaledData,value,4);sz=4;break;
    case LIVEDATA_VOUT_VOLT_CH2: memcpy((void*)&liveData.analog_outputs[2].scaledData,value,4);sz=4;break;
    case LIVEDATA_VOUT_VOLT_CH3: memcpy((void*)&liveData.analog_outputs[3].scaledData,value,4);sz=4;break;
    default:break;
    }
    break;
  case LIVEDATA_SECTION_RESOLVER:
    switch(index){
    case LIVEDATA_AZ_DEG: memcpy((void*)&liveData.pv[AXIS_AZ].pos,value,4);sz=4;break;
    case LIVEDATA_AZ_DPS: memcpy((void*)&liveData.pv[AXIS_AZ].speed,value,4);sz=4;break;
    case LIVEDATA_EL_DEG: memcpy((void*)&liveData.pv[AXIS_EL].pos,value,4);sz=4;break;
    case LIVEDATA_EL_DPS: memcpy((void*)&liveData.pv[AXIS_EL].speed,value,4);sz=4;break;
    default:break;
    }
    break;
  case LIVEDATA_SECTION_SYSTEM:
    switch(index){
    case LIVEDATA_MC_STATUS:memcpy((void*)&liveData.mcStatus,value,4);sz=4;break;
    default:break;
    }
    break;
  case LIVEDATA_SECTION_DIGITAL_IO:
    switch(index){
    case LIVEDATA_DIGITAL_INPUT_WORD:memcpy((void*)&liveData.digital_input,value,2);sz=2;break;
    case LIVEDATA_DIGITAL_OUTPUT_WORD:memcpy((void*)&liveData.digital_output,value,2);sz=2;break;
    default:break;
    }
    break;
  }
  return sz;
}

int8_t db_exec_cmd(uint8_t section, uint8_t index, uint8_t *value)
{
  
  return 0;
}

int8_t db_fill_canbus_message(uint8_t section, uint32_t *id, uint8_t *packet)
{
  int8_t sz = -1;
  CANBUS_DATA_u *dptr = (CANBUS_DATA_u*)packet;
  //if(liveData.enableReport == 0) return -1;
  
  if(section >= NOF_CANBUS_MSG) return -1;
  *id = nvmParams.can_msg[section].id;
  switch(section){
  case CANBUS_MSG_ID_AZ_PV:
    //if(liveData.enableReport & LIVE_DATA_REPORT_AZ){
      dptr->data16[0] = (uint16_t)(liveData.pv[AXIS_AZ].pos*100);
      dptr->data16[1] = (int16_t)(liveData.sp[AXIS_AZ].pos*100);
      dptr->data16[2] = (int16_t)(liveData.pv[AXIS_AZ].speed*100);
      dptr->data16[3] = (int16_t)(liveData.sp[AXIS_AZ].speed*100);
      sz = 0;
    //}
    break;
  case CANBUS_MSG_ID_AZ_MC_ERR:
    //if(liveData.enableReport & LIVE_DATA_REPORT_AZ){
      dptr->data16[0] = (int16_t)(liveData.sp[AXIS_AZ].pos_err*100);
      dptr->data16[1] = (int16_t)(liveData.sp[AXIS_AZ].speed_err*100);
      dptr->data16[2] = (int16_t)(liveData.analog_outputs[nvmParams.control_param[AXIS_AZ].m1.control_channel].scaledData*100);
      dptr->data16[3] = (int16_t)(liveData.analog_outputs[nvmParams.control_param[AXIS_AZ].m2.control_channel].scaledData*100);
      sz = 0;
    //}
    break;
  case CANBUS_MSG_ID_EL_PV:
    //if(liveData.enableReport & LIVE_DATA_REPORT_EL){
      dptr->data16[0] = (uint16_t)(liveData.pv[AXIS_EL].pos*100);
      dptr->data16[1] = (int16_t)(liveData.sp[AXIS_EL].pos*100);
      dptr->data16[2] = (int16_t)(liveData.pv[AXIS_EL].speed*100);
      dptr->data16[3] = (int16_t)(liveData.sp[AXIS_EL].speed*100);
      sz = 0;
    //}
    break;
  case CANBUS_MSG_ID_EL_MC_ERR:
    //if(liveData.enableReport & LIVE_DATA_REPORT_EL){
      dptr->data16[0] = (int16_t)(liveData.sp[AXIS_EL].pos_err*100);
      dptr->data16[1] = (int16_t)(liveData.sp[AXIS_EL].speed_err*100);
      dptr->data16[2] = (int16_t)(liveData.analog_outputs[nvmParams.control_param[AXIS_EL].m1.control_channel].scaledData*100);
      dptr->data16[3] = (int16_t)(liveData.analog_outputs[nvmParams.control_param[AXIS_EL].m2.control_channel].scaledData*100);
      sz = 0;
    //}
    break;
  case CANBUS_MSG_ID_DIO:
    dptr->data16[0] = liveData.digital_input;
    dptr->data16[1] = liveData.digital_output;
    dptr->data32[1] = liveData.mcStatus;
    break;
  case CANBUS_MSG_ID_MC_STA:
    dptr->fv[0] = liveData.pv[AXIS_AZ].pos;
    dptr->fv[1] = liveData.sp[AXIS_AZ].pos;
    break;
  case CANBUS_MSG_ID_AZ_PARAM3:
    dptr->fv[0] = liveData.pv[AXIS_AZ].pos;
    dptr->fv[1] = liveData.sp[AXIS_AZ].pos;
    break;
  case CANBUS_MSG_ID_EL_POS_PV_SP:
    dptr->fv[0] = liveData.pv[AXIS_EL].pos;
    dptr->fv[1] = liveData.sp[AXIS_EL].pos;
    break;
  case CANBUS_MSG_ID_EL_POS_ERR:
    dptr->fv[0] = liveData.sp[AXIS_EL].pos_err;
    dptr->fv[1] = liveData.sp[AXIS_EL].speed_err;
    break;
  case CANBUS_MSG_ID_EL_SPD_PV_SP:
    dptr->fv[0] = liveData.pv[AXIS_EL].speed;
    dptr->fv[1] = liveData.sp[AXIS_EL].speed;
    break;
  case CANBUS_MSG_ID_EL_MOTOR_OUT:
    dptr->fv[0] = liveData.analog_outputs[nvmParams.control_param[AXIS_EL].m1.control_channel].scaledData;
    dptr->fv[1] = liveData.analog_outputs[nvmParams.control_param[AXIS_EL].m2.control_channel].scaledData;
    break;
  case CANBUS_MSG_ID_EL_PARAM1:
    dptr->fv[0] = liveData.pv[AXIS_EL].pos;
    dptr->fv[1] = liveData.sp[AXIS_EL].pos;
    break;
  case CANBUS_MSG_ID_EL_PARAM2:
    dptr->fv[0] = liveData.pv[AXIS_EL].pos;
    dptr->fv[1] = liveData.sp[AXIS_EL].pos;
    break;
  case CANBUS_MSG_ID_EL_PARAM3:
    dptr->fv[0] = liveData.pv[AXIS_EL].pos;
    dptr->fv[1] = liveData.sp[AXIS_EL].pos;
    break;
  }
  
  
  return sz;
}


