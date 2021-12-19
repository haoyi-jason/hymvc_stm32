#include "ch.h"
#include "hal.h"
#include "task_communication.h"
#include "task_analog_input.h"
#include "task_dac.h"
#include "task_resolver.h"
#include "can_frame_handler.h"
#include "at24_eep.h"
#include "nvm_config.h"
#include "task_mbmaster.h"

int8_t config_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t id_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t digital_input_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t digital_output_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t analog_input_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t analog_output_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t power_output_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t report_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t heartBeatHandler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t analog_output_config(CANRxFrame *prx,CANTxFrame *ptx);

can_frame_handler PacketHandler[] = {
  {0x01,config_handler},
  {0x20,heartBeatHandler},
  {0x81,report_handler},
  {0x99,id_handler},
  {0x140,digital_output_handler},
  {0x141,digital_input_handler},
  {0x150,analog_output_handler},
  {0x160,analog_input_handler},  
  {0x180,power_output_handler},  
  {0x201,analog_output_config},
  {-1,NULL},
};

struct runTime{
  thread_t *self;
  virtual_timer_t vtComm;
  event_listener_t elCAN;
  uint16_t state;
  uint8_t txFrameId;
};

struct _nvmParam{
  uint8_t flag;
  uint8_t canID;
  
};

static CANConfig canCfg500K = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_BRP(3) | CAN_BTR_TS1(14) | CAN_BTR_TS2(1) | CAN_BTR_SJW(0)
};

// can config for 250K baud
//static CANConfig canCfg250K = {
//  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
//  CAN_BTR_BRP(7) | CAN_BTR_TS1(14) | CAN_BTR_TS2(1) | CAN_BTR_SJW(0)
//};

static CANConfig canCfg250K = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_BRP(11) | CAN_BTR_TS1(10) | CAN_BTR_TS2(1) | CAN_BTR_SJW(1)
};

static struct runTime runTime, *commRuntime;
static struct _nvmParam nvmParam, *commNvmParam;


static void save_nvmParam()
{
    nvm_flash_write(OFFSET_NVM_CONFIG,sizeof(nvmParam),(uint8_t*)&nvmParam);
}

static void load_settings()
{
  uint16_t nvmSz = sizeof(nvmParam);
  eepromRead(OFFSET_NVM_CONFIG,nvmSz,(uint8_t*)&nvmParam);
  if(nvmParam.flag != 0xAB){
    nvmParam.flag = 0xAB;
    save_nvmParam();
  }
}

static void report_cb(void *arg)
{
//  chSysLockFromISR();
//  if(commState.active){
//    chEvtSignalI(thisThd, EVENT_MASK(1));
//  }
//  chVTSetI(&vtReport,TIME_MS2I(commState.report_interval_ms),report_cb,NULL);
//  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waCANRX,4096);
static THD_FUNCTION(procCANRx,p){
 // thread_t *parent = (thread_t)p;
  CANDriver *ip = (CANDriver*)p;
  canStart(ip,&canCfg250K);
  CANRxFrame rxMsg;
  int32_t analogInputs[8];
  event_listener_t el;
  CANTxFrame txFrame;
  CANTxFrame txFrames[6];
//  txFrame.DLC = 8;
//  txFrame.RTR = CAN_RTR_DATA;
//  txFrame.EID = 0x234;
//  txFrame.data32[0] = 0x11223344;
//  txFrame.data32[1] = 0x11223344;
//  txFrame.IDE = CAN_IDE_EXT;
  
  chEvtRegister(&ip->rxfull_event,&runTime.elCAN,(0));
  
  //canTransmit(ip,CAN_ANY_MAILBOX,&txFrame,TIME_MS2I(100));
  runTime.state = 0;
  for(uint8_t i=0;i<6;i++){
    txFrames[i].RTR = CAN_RTR_DATA;
    txFrames[i].DLC = 8;
    txFrames[i].IDE = CAN_IDE_EXT;
  }
  
  txFrames[0].EID = 0x160;
  txFrames[1].EID = 0x161;
  txFrames[2].EID = 0x162;
  txFrames[3].EID = 0x163;
  
  txFrames[4].EID = 0x170;
  txFrames[5].EID = 0x171;
  
  while(!chThdShouldTerminateX()){
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,TIME_IMMEDIATE);
    if(evt & EVENT_MASK(0)){
      while(canReceive(ip,CAN_ANY_MAILBOX,&rxMsg,TIME_IMMEDIATE) == MSG_OK){
        uint16_t eidActive = rxMsg.EID & 0xFFF;
        if(findHandler(eidActive, &rxMsg,&txFrame,PacketHandler)>0){
          txFrame.EID = eidActive;
          canTransmit(ip,CAN_ANY_MAILBOX,&txFrame,TIME_MS2I(100));
        }
      }
    }
    
    switch(runTime.state){
    case 0:
      // load data from resolver & ADC
      analog_input_read(0xff,analogInputs);
      txFrames[0].data32[0] = analogInputs[0];
      txFrames[0].data32[1] = analogInputs[1];
      txFrames[1].data32[0] = analogInputs[2];
      txFrames[1].data32[1] = analogInputs[3];
      txFrames[2].data32[0] = analogInputs[4];
      txFrames[2].data32[1] = analogInputs[5];
      txFrames[3].data32[0] = analogInputs[6];
      txFrames[3].data32[1] = analogInputs[7];
      
      float fv;
      fv = resolver_get_position(0);
      memcpy((void*)&txFrames[4].data8[0], (void*)&fv,4);
      fv = resolver_get_speed(0);
      memcpy((void*)&txFrames[4].data8[4], (void*)&fv,4);

      fv = resolver_get_position(1);
      memcpy((void*)&txFrames[5].data8[0], (void*)&fv,4);
      fv = resolver_get_speed(1);
      memcpy((void*)&txFrames[5].data8[4], (void*)&fv,4);
      
      runTime.txFrameId = 0;
      break;
    default:
      if(runTime.txFrameId < 6){
        if(canTransmit(ip,CAN_ANY_MAILBOX,&txFrames[runTime.txFrameId],TIME_MS2I(10)) == MSG_OK){
          runTime.txFrameId++;
        }
      }
      break;
    }
    runTime.state++;
    if(runTime.state == 100){
      runTime.state = 0;
    }
    chThdSleepMilliseconds(10);
  }
}       

void task_communication_init(void)
{
  analog_input_task_init();
  analog_output_task_init();
  resolver_task_init();
  modbus_master_task_init();
//  canStart(&CAND1,&canCfg250K);
  at24eep_init(&I2CD2,32,1024,0x50,2);
  commRuntime = &runTime;
  runTime.self = chThdCreateStatic(waCANRX, sizeof(waCANRX), NORMALPRIO, procCANRx, &CAND1);
}

int8_t config_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  return 0;
}


int8_t id_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  return 0;
}

// set report speed
int8_t report_config_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  return 0;
}

int8_t digital_input_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  return 0;
}
int8_t digital_output_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  return 0;
}
int8_t analog_input_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  return 0;
}
int8_t analog_output_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  if(prx->RTR == CAN_RTR_DATA){
    uint8_t channel = prx->data8[0];
    uint8_t option = prx->data8[1] & 0x7F;
    uint8_t immediateUpdate = (prx->data8[1] & 0x80);
    float v;
    switch(option){
    case 0:
      analog_output_set_data(channel,(int16_t*)&prx->data16[1]);
      break;
    case 1:
      analog_output_set_corse_gain(channel,(uint16_t*)&prx->data16[1]);
      break;
    case 2:
      analog_output_set_fine_gain(channel,(uint16_t*)&prx->data16[1]);
      break;
    case 3:
      analog_output_set_offset(channel,(uint16_t*)&prx->data16[1]);
      break;
    case 4: // updata mode
      analog_output_set_update_on_write(prx->data8[2]);
      break;
    case 5: // update
      analog_output_update();
      break;
    case 6: // save data
      break;
    case 7: // voltage
      memcpy((uint8_t*)&v,(uint8_t*)&prx->data8[2],4);
      analog_output_set_voltage(channel,&v);
      break;
    default:
      break;
    }
  }
  
  return 0; 
}
int8_t analog_output_config(CANRxFrame *prx,CANTxFrame *ptx)
{
  
  return 0; // not supported
}

int8_t power_output_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  return 0; 
}

// set/get report state
int8_t report_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  return 0;
}

int8_t heartBeatHandler(CANRxFrame *prx,CANTxFrame *ptx)
{
  return 0; 
}


