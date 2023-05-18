#ifndef _APP_CONFIG_
#define _APP_CONFIG_

#include "ch.h"
#include "pid_controller.h"
#include "dual_motor_ctrl\tq_bias_calc.h"
#include "ad57_drv.h"

#define CONSOLE          SD4

#define DEVICE_NAME     "HYMCWV7"
#define HW_VERSION      "2.0.0"
#define VENDER_ID       0x53290921
#define PROEUCT_CODE    0x00021008
#define REVISION_NUMBER 0x00000001
#define SERIAL_NUMVER   0x11111111

enum e_axis
{
  AXIS_AZ,
  AXIS_EL,
  NOF_AXES,
};

/* gloabl message */
#define EV_ADC_ACQUIRED         EVENT_MASK(0)
#define EV_RESOLVER_ACQUIRED    EVENT_MASK(1)
#define EV_DAC_UPDATED          EVENT_MASK(2)
#define EV_DIN_COS              EVENT_MASK(3)
#define EV_PID_START            EVENT_MASK(4)
#define EV_PID_STOP             EVENT_MASK(5)
#define EV_PID_AZ_RDY           EVENT_MASK(6)
#define EV_PID_EL_RDY           EVENT_MASK(7)
#define EV_PID_UPDATEDA         EVENT_MASK(8)
#define EV_PID_UPDATEDE         EVENT_MASK(9)
#define EV_STOW_ACTION_DONE     EVENT_MASK(10)
#define EV_STOW_ACTION_FAIL     EVENT_MASK(11)
#define EV_AZ_STOW_LOCK         EVENT_MASK(12)
#define EV_EL_STOW_LOCK         EVENT_MASK(13)
#define EV_LOCAL_LOG_START      EVENT_MASK(14)
#define EV_LOCAL_LOG_STOP       EVENT_MASK(15)
#define EV_LOCAL_LOG_EVT        EVENT_MASK(16)
#define EV_PID_AZ_STARTED          EVENT_MASK(17)
#define EV_PID_AZ_STOPPED          EVENT_MASK(18)
#define EV_PID_EL_STARTED          EVENT_MASK(17)
#define EV_PID_EL_STOPPED          EVENT_MASK(18)

enum _SYSTEM_STATUS_MASK
{
  STA_MC_AZ_STOW_LOCK_IN_PROGRESS,
  STA_MC_AZ_STOW_LOCKD,
  STA_MC_AZ_STOW_UNLOCK_IN_PROGRESS,
  STA_MC_AZ_STOW_UNLOCKD,
  STA_MC_EL_STOW_LOCK_IN_PROGRESS,
  STA_MC_EL_STOW_LOCKD,
  STA_MC_EL_STOW_UNLOCK_IN_PROGRESS,
  STA_MC_EL_STOW_UNLOCKD,
  STA_MC_STOW_AZ_ERROR,
  STA_MC_STOW_EL_ERROR,
  STA_SYS_ADC,
  STA_SYS_DAC,
  STA_SYS_RESOLVER,
  STA_SYS_PIDAZ_READY,
  STA_SYS_PIDEL_READY,
};

enum _STOW_ACTIVITY
{
  ACT_NONE,
  ACT_UNLOCK,
  ACT_LOCK
};

enum _STOW_STATE
{
  STOW_UNKNOW,
  STOW_LOCKED,
  STOW_UNLOCKED
};

typedef struct{
  uint8_t driver_on[2];  // DO channel to on
  uint8_t driver_en[2];  // do channel to enable
  uint8_t driver_rdy[2]; // ready signal from amplifier/driver
  uint8_t driver_output[2];  // DA channel to output control
  uint8_t resolver;
  uint8_t stow_lock_out;
  uint8_t stow_unlock_out;
  uint8_t stow_lock_in;
  uint8_t stow_unlock_in;
}_control_id_map;



typedef struct{
  PID_CFG_T     pid_cfg_pos;
  PID_CFG_T     pid_cfg_speed;
  TQBC_CFG_T    tqbc_cfg;
  struct{
    float       out_max_2set;   // OD2103.1
    float       out_min_2set;   // OD2103.2
    float       gain_2set;      // OD2103.3
    float       zcp_2set;       // OD2103.4
    float       out_max_set;    // OD2103.5
    float       out_min_set;    // OD2103.6
    float       gain_set;       // OD2103.7
    float       zcp_set;        // OD2103.8
  } tqbc_boundary;
  struct{
    float       pos_s_min;
    float       pos_s_max;
    float       pos2_thold;
    float       speed;
    float       pos_kp;
    float       pos_ki;
    float       pos_kd;
    float       spd_kp;
    float       spd_ki;
    float       spd_kd;
  }dmotc_default;
  float torque_max;
  float output_max;
  float rpm_max;
  float pos_min;
  float pos_max;
  float home_offset;
  _control_id_map ctrl_map;
  uint8_t axisId;
  bool stowValid;
  uint8_t stowState;
  bool joystickMode;
}_pid_config_t;

typedef struct{
  _control_id_map *ctrl_map;
  uint8_t stow_activity;
  uint16_t timeout_ms;
  uint16_t cycletime_ms;
  uint8_t stowStatus;
  uint8_t axisId;
}_stow_control_t;

typedef struct{
  uint16_t corse_gain[AD57_CHANNELS];   // OD 2100
  uint16_t fine_gain[AD57_CHANNELS];    // OD 2101
  uint16_t offset[AD57_CHANNELS];       // OD 2102        
}_dac_config;
#endif