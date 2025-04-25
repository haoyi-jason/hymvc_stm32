#include "ch.h"
#include "hal.h"
#include "task_can1.h"
#include "database.h"
#include <string.h>

#define EV_1MS         EVENT_MASK(0)

typedef struct{
    uint32_t messageID;
    void (*handler)(CANRxFrame *rxmsg);
}_can_frame_handler_t;

typedef struct{
  thread_t *self;
  thread_t *txthd;
  thread_t *mainThread;
  CANDriver *pcan;
  systime_t now;
  uint8_t id;
  uint8_t pendSectionToSend;
  virtual_timer_t vt_1ms;
}_runtime_t;

typedef struct{
	uint16_t section;
	uint16_t reload;
	uint16_t counter;
}MessageReport_t;


/* clock = 42MHz for STM32F4 */
static CANConfig cfg500K = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_BRP(5) | CAN_BTR_TS1(10) | CAN_BTR_TS2(1) | CAN_BTR_SJW(1)
};

// can config for 250K baud, clock = 42MHz
static CANConfig cfg250K = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_BRP(11) | CAN_BTR_TS1(10) | CAN_BTR_TS2(1) | CAN_BTR_SJW(1)
};

static void HY_BOOT_LOAD(CANRxFrame *rxmsg);
static void HY_CONTROL(CANRxFrame *rxmsg);


static _runtime_t local_runtime;


/* messages received from CMU handler */
static _can_frame_handler_t handlers[] = {
    {MSG_ID_CONTROL, HY_CONTROL},
    {MSG_ID_BOOT, HY_BOOT_LOAD},
//    {ID_REPORT_STA, cmu_sta_report_handler},
	{0x0,NULL}
};


static MessageReport_t msg_report_seq[CANBUS_MESSAGE_COUNT] = {
	    //{CANBUS_MSG_ID_SV1,200,200},
	    //{CANBUS_MSG_ID_AUX_A,1000,1000},
		{0xFFFF,0x0,0x0},
};

static void periodic_1ms(void *arg)
{
  (void)arg;
  chSysLockFromISR();
  if(local_runtime.txthd != NULL){
    chEvtSignalI(local_runtime.txthd,EV_1MS);
  }
  chVTSetI(&local_runtime.vt_1ms,TIME_US2I(5000),periodic_1ms,NULL);
  chSysUnlockFromISR();
}

static void canbus_send_boot(uint8_t id)
{
  CANTxFrame txFrame;
  txFrame.DLC = 8;
  txFrame.EID = 0x700 + id;
  txFrame.IDE = CAN_IDE_EXT;
  uint32_t fw = 0x0;//FW_VERSION;
  uint32_t hw = 0x0;//HW_VERSION;
  db_read_param(DATA_SECTION_BOARD_PARAM,BOARD_PARAM_HW_VERSION,&txFrame.data8[0]);
  db_read_param(DATA_SECTION_BOARD_PARAM,BOARD_PARAM_SW_VERSION,&txFrame.data8[4]);
//  memcpy(&txFrame.data8[0],(uint8_t*)&hw,4);
//  memcpy(&txFrame.data8[4],(uint8_t*)&fw,4);

  canTransmit(local_runtime.pcan,CAN_ANY_MAILBOX ,&txFrame,TIME_MS2I(10));
}

static void sendMessage(uint16_t section)
{
  CANTxFrame txFrame;
  txFrame.DLC = 8;
  txFrame.RTR = CAN_RTR_DATA;
  txFrame.IDE = CAN_IDE_EXT;
  uint32_t id;
  int8_t ret = db_fill_canbus_message(section,&id,txFrame.data8);
  if(ret == 0){
    id += local_runtime.id;
    txFrame.EID = id; 
    canTransmit(local_runtime.pcan,CAN_ANY_MAILBOX ,&txFrame,TIME_MS2I(10));
  }
}

static THD_WORKING_AREA(waCANTX,512);
static THD_FUNCTION(thCanTx,p)
{
  chVTObjectInit(&local_runtime.vt_1ms);
  chVTSet(&local_runtime.vt_1ms,TIME_US2I(5000),periodic_1ms,NULL);
  //CANTxFrame txFrame;
  //txFrame.RTR = CAN_RTR_DATA;
  //txFrame.IDE = CAN_IDE_EXT;
    
  can0_reload_message_struct();
  
  local_runtime.now = chVTGetSystemTimeX();
  local_runtime.pendSectionToSend = 0xff;
  // read id
  
  while(!chThdShouldTerminateX()){
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    MessageReport_t *msg = msg_report_seq;
    uint32_t now = chVTGetSystemTimeX();
    while(msg->section != 0xFFFF){
      if(msg->reload != 0xFFFF){
        if(msg->counter > 0){
          msg->counter--;
        }
        else{
          msg->counter = msg->reload-1;
          sendMessage(msg->section);
        }
      }
      msg++;
    }
    if(local_runtime.pendSectionToSend != 0xFF){
      sendMessage(local_runtime.pendSectionToSend);
      local_runtime.pendSectionToSend = 0xff;
    }
    
    // send boot
    if(TIME_I2MS(chVTTimeElapsedSinceX(local_runtime.now)) > 5000){
      local_runtime.now = now;
      canbus_send_boot(local_runtime.id);
    }
   // chThdSleepMilliseconds(5);
  }
  
  chThdExit(MSG_OK);
}

static THD_WORKING_AREA(waCANRX,512);
static THD_FUNCTION(thCanRx,p)
{
  chRegSetThreadName("BCU_THD");
  local_runtime.pcan = (CANDriver*)p;
  if(local_runtime.pcan == NULL){
    chThdExit(MSG_RESET);
  }
  event_listener_t el;
  chEvtRegister(&(local_runtime.pcan)->rxfull_event,&el,1);
  
  uint8_t rate = 1;
  uint32_t id = 0x01;
  db_read_param(DATA_SECTION_BOARD_PARAM,BOARD_PARAM_CAN1_BITRATE,&rate);
  db_read_param(DATA_SECTION_BOARD_PARAM,BOARD_PARAM_USER_ID,&local_runtime.id);
  
  canStop(local_runtime.pcan);
  
  rate = 0; // uncomment this to use setting from nvm, otherwise, 500K
  switch(rate){
  case 0:  canStart(local_runtime.pcan, &cfg500K);break;
  case 1:  canStart(local_runtime.pcan, &cfg250K);break;
  default:  canStart(local_runtime.pcan, &cfg250K);break;
  }
  

  CANRxFrame rxmsg;
  bool stop = false;
  _can_frame_handler_t *h;
  while(!stop){
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    if(evt & EVENT_MASK(1)){
      while(canReceive(local_runtime.pcan,CAN_ANY_MAILBOX,&rxmsg,TIME_IMMEDIATE) == MSG_OK){
        h = handlers;
        uint32_t msg = rxmsg.EID & 0x1FFFFF80;
        while(h->messageID != 0x0){
          if(h->messageID == msg){
            h->handler(&rxmsg);
            break;
          }
          h++;
        }        
      }
    }
    
  }
}


void task_can1_init()
{
  local_runtime.mainThread = chRegFindThreadByName("MAIN");
  local_runtime.txthd = chThdCreateStatic(waCANTX,sizeof(waCANTX),NORMALPRIO+1,thCanTx,&CAND1);
  local_runtime.self = chThdCreateStatic(waCANRX,sizeof(waCANRX),NORMALPRIO,thCanRx,&CAND1);
}

void can0_send_section(uint8_t section)
{
  if(local_runtime.pendSectionToSend == 0xFF){
    local_runtime.pendSectionToSend = section;
  }
}

void can0_reload_message_struct()
{
  MessageReport_t *msg = msg_report_seq;
  uint16_t u16;
  // load message config from database
  for(uint8_t i=0;i<CANBUS_MESSAGE_COUNT;i++){
    db_read_param(DATA_SECTION_CANBUS_MSG,CANBUS_MSG_INTERVAL + i,(void*)&u16);
    msg->section = i;
    msg->reload = u16;
    msg->counter = (u16 - i)%u16;
    msg++;
  }
  msg->section = 0xFFFF;
}

static void HY_BOOT_LOAD(CANRxFrame *rxmsg)
{
  CANTxFrame txFrame;
  txFrame.EID = rxmsg->EID;
  txFrame.DLC = 8;
  txFrame.RTR = 0;
  //hy_flash_handle_canbus_packet(rxmsg->EID,rxmsg->data8,txFrame.data8,rxmsg->DLC);
  canTransmit(local_runtime.pcan,CAN_ANY_MAILBOX ,&txFrame,TIME_MS2I(10));
}
static void HY_CONTROL(CANRxFrame *rxmsg)
{
  uint8_t id;
  CANTxFrame txFrame;
  if(rxmsg->IDE == 0){
    id = rxmsg->SID & 0x7F;
    txFrame.SID = rxmsg->SID;
    txFrame.IDE = 0;
  }
  else{
    id = rxmsg->EID & 0x7F;
    txFrame.EID = rxmsg->EID;
    txFrame.IDE = 1;
  }
  txFrame.DLC = 8;
  if((id == 0) || (id == local_runtime.id)){
    if((rxmsg->data8[0] == 0xAB) && (rxmsg->data8[1] == 0xBA)){
      if(rxmsg->DLC == 8){
        if(rxmsg->data8[2] == 0xAA && rxmsg->data8[3] == 0xBB){
          //todo: reset
        }
        else{
          db_write_param(rxmsg->data8[2],rxmsg->data8[3],&rxmsg->data8[4]);
        }
      }
      else if(rxmsg->DLC == 4){
        memcpy(txFrame.data8,rxmsg->data8,4);
        db_read_param(rxmsg->data8[2],rxmsg->data8[3],&txFrame.data8[4]);
        canTransmit(local_runtime.pcan,CAN_ANY_MAILBOX ,&txFrame,TIME_MS2I(10));
      }
    }
  }
  
  
}
