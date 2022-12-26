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
#include "digital_io.h"
#include "task_dual_motor_ctrl.h"
#include "task_pos_cmd_handler.h"

int8_t config_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t id_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t digital_input_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t digital_output_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t digital_config_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t analog_input_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t analog_output_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t power_output_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t report_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t heartBeatHandler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t analog_output_config(CANRxFrame *prx,CANTxFrame *ptx);

int8_t pid_command(CANRxFrame *prx,CANTxFrame *ptx);
int8_t pid_parameter(CANRxFrame *prx,CANTxFrame *ptx);
int8_t pid_request(CANRxFrame *prx,CANTxFrame *ptx);

can_frame_handler PacketHandler[] = {
  {0x01,config_handler},
  {0x20,heartBeatHandler},
  {0x81,report_handler},
  {0x99,id_handler},
  {0x140,digital_output_handler},
  {0x141,digital_input_handler},
  {0x142,digital_config_handler},
  {0x150,analog_output_handler},
  {0x160,analog_input_handler},  
  {0x180,power_output_handler},  
  {0x201,analog_output_config},
  {0x100,pid_command},
  {0x101,pid_parameter},
  {0x102,pid_request},
  {-1,NULL},
};

struct runTime{
  thread_t *self;
  virtual_timer_t vtComm;
  event_listener_t elCAN;
  uint16_t state;
  uint8_t txFrameId;
  systime_t start,stop;
  systime_t elapsed;
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

static int16_t pid_get_cmd_value(uint8_t mode, uint8_t cmd_index)
{
  float output_f = 0.0;
  int16_t output = 0;

  switch (mode)
  {
    case TDMOTC_MODE_S:
    output_f = tdmotc_GetSpeedCmd();
    break;

    case TDMOTC_MODE_P:
    output_f = tpcmdh_GetPosCmd();
    break;

    case TDMOTC_MODE_P2:
    output_f = tdmotc_GetPosCmd();
    break;


    default:
    break;
  }

  output = (int16_t)(output_f * 0.01f);
  return output;
}

//static int32_t pid_get_para_value(uint8_t cmd_index)
static float pid_get_para_value(uint8_t cmd_index)
{
  //int32_t output = 0.0f;
  float output_f = 0.0f;
  switch (cmd_index)
  {
    case CAN_GSID_KP_P:
    output_f = tdmotc_GetPID(TDMOTC_PID_P, TDMOTC_PID_ID_P);
    //output_f = tdmotc_GetPCCFG(TDMOTC_PCCFG_ID_KP);
    break;

    case CAN_GSID_KI_P:
    output_f = tdmotc_GetPID(TDMOTC_PID_P, TDMOTC_PID_ID_I);
    break;

    case CAN_GSID_KD_P:
    output_f = tdmotc_GetPID(TDMOTC_PID_P, TDMOTC_PID_ID_D);
    break;

    case CAN_GSID_KP_S:
    output_f = tdmotc_GetPID(TDMOTC_PID_S, TDMOTC_PID_ID_P);
    break;

    case CAN_GSID_KI_S:
    output_f = tdmotc_GetPID(TDMOTC_PID_S, TDMOTC_PID_ID_I);
    break;

    case CAN_GSID_KD_S:
    output_f = tdmotc_GetPID(TDMOTC_PID_S, TDMOTC_PID_ID_D);
    break;

    case CAN_GSID_MOT_T_MAX_ABS:
    output_f = tdmotc_GetMotTqMaxAbs();
    break;

    case CAN_GSID_MOT_S_MAX_ABS:
    output_f = tdmotc_GetAxisSMaxAbs();
    break;

    case CAN_GSID_G_A2M:
    /*Currently not avaliable*/
    break;

    case CAN_GSID_TQBC_MAX:
    output_f = tdmotc_GetTQBC(TDMOTC_TQBC_ID_OUT_MAX);
    break;

    case CAN_GSID_TQBC_MIN:
    output_f = tdmotc_GetTQBC(TDMOTC_TQBC_ID_OUT_MIN);
    break;

    case CAN_GSID_G:
    output_f = tdmotc_GetTQBC(TDMOTC_TQBC_ID_GAIN);
    break;

    case CAN_GSID_ZCP:
    output_f = tdmotc_GetTQBC(TDMOTC_TQBC_ID_ZCP);
    break;

    case CAN_GSID_PC_PERR_THOLD:
    output_f = tdmotc_GetPCCFG(TDMOTC_PCCFG_ID_PERR_THOLD);
    break;

    case CAN_GSID_PC_S_CMD_MIN:
    output_f = tdmotc_GetPCCFG(TDMOTC_PCCFG_ID_S_CMD_MIN);
    break;

    case CAN_GSID_PC_S_CMD_MAX:
    output_f = tdmotc_GetPCCFG(TDMOTC_PCCFG_ID_S_CMD_MAX);
    break;

    case CAN_GSID_UPDATE:
    /*This index has no get method.*/
    break;

    default:
    /*Invalid index, do noting*/
    break;
  }

//  output = tdmotc_ConvSig2CAN(output_f);
  return output_f;
}

#define NOF_CAN_PACKET  11
static THD_WORKING_AREA(waCANRX,4096);
static THD_FUNCTION(procCANRx,p){
 // thread_t *parent = (thread_t)p;
  CANDriver *ip = (CANDriver*)p;
  canStart(ip,&canCfg250K);
  CANRxFrame rxMsg;
  int32_t analogInputs[8];
  event_listener_t el;
  CANTxFrame txFrame;
  CANTxFrame txFrames[NOF_CAN_PACKET];
//  txFrame.DLC = 8;
//  txFrame.RTR = CAN_RTR_DATA;
//  txFrame.EID = 0x234;
//  txFrame.data32[0] = 0x11223344;
//  txFrame.data32[1] = 0x11223344;
//  txFrame.IDE = CAN_IDE_EXT;
  
  chEvtRegister(&ip->rxfull_event,&runTime.elCAN,(0));
  
  //canTransmit(ip,CAN_ANY_MAILBOX,&txFrame,TIME_MS2I(100));
  runTime.state = 0;
  for(uint8_t i=0;i<NOF_CAN_PACKET;i++){
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
  
  txFrames[6].EID = 0x140;
  txFrames[7].EID = 0x141;

  txFrames[8].EID = 0x110;
  txFrames[9].EID = 0x111;
  txFrames[10].EID = 0x112;  
  systime_t now;
  while(!chThdShouldTerminateX()){
      runTime.start = chVTGetSystemTime();
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
      
      // read digital io, byte 0~3 for ttl, 4~7 for ISO
      uint8_t dio = 0x0;
      for(uint8_t i=0;i<4;i++){
        if(digital_get_iso_out(i)) dio |= (1 << i);
      }
      txFrames[6].data8[0] = dio;

      txFrames[7].data32[0] = digital_get_input();
      dio = 0;
      for(uint8_t i=0;i<8;i++){
        if(digital_get_iso_in(i)) dio |= (1 << i);
      }
      txFrames[7].data8[4] = dio;

      // Build status 1
      if(tdmotc_GetIsStart())
      {
        txFrames[8].data8[0] = (tdmotc_GetMode() + 2U);
      }
      else
      {
        txFrames[8].data8[0] = 1U;
      }
      txFrames[8].data16[2] = modbus_master_GetStatus(1);
      txFrames[8].data16[3] = modbus_master_GetStatus(2);
      
      float _signal_f = 0.0f;
      // Build status 2
      _signal_f = tdmotc_GetAxisSpeedAct();
//      _signal_f = 5;
//      txFrames[9].data32[0] = (int32_t)(_signal_f * 0.001f);
      memcpy((uint8_t*)&txFrames[9].data32[0],(uint8_t*)&_signal_f,4);

      // Build status 3
      _signal_f = tdmotc_GetMotTqActV(0);
      //txFrames[10].data32[0] = (int32_t)(_signal_f * 0.001f);
      memcpy((uint8_t*)&txFrames[10].data32[0],(uint8_t*)&_signal_f,4);
      _signal_f = tdmotc_GetMotTqActV(1);
      memcpy((uint8_t*)&txFrames[10].data32[1],(uint8_t*)&_signal_f,4);
//      txFrames[10].data32[1] = (int32_t)(_signal_f * 0.001f);
            
      runTime.txFrameId = 0;
      break;

    default:
      if(runTime.txFrameId < NOF_CAN_PACKET){
        if(canTransmit(ip,CAN_ANY_MAILBOX,&txFrames[runTime.txFrameId],TIME_IMMEDIATE) == MSG_OK){
          runTime.txFrameId++;
        }
      }
      break;
    }

    runTime.state++;
    if(runTime.state >= 20){
      runTime.state = 0;
    }

    chThdSleepMilliseconds(5);
    runTime.stop = chVTGetSystemTime();
    runTime.elapsed = TIME_I2MS(chVTTimeElapsedSinceX(runTime.start));
  }
}       

void task_communication_init(void)
{
  analog_input_task_init();
  analog_output_task_init();
  resolver_task_init();
  //modbus_master_task_init();
  //digital_init();
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
  if(prx->RTR == CAN_RTR_DATA){
    uint8_t channel = prx->data8[0];
    uint8_t value = prx->data8[1];
    if(channel & 0x80){ // isolate output
      digital_set_iso_out(channel & 0x7F,value);
    }
    else if(channel & 0x40){
      digital_set_ttl_port(channel&0xf,value);
    }
    else{ // ttl channel
      digital_set_ttl_bit(channel,value);
    }
  }
  return 0;
}
int8_t digital_config_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  if(prx->RTR == CAN_RTR_DATA){
    uint8_t port = prx->data8[0];
    uint8_t dir = prx->data8[1];
    digital_set_ttl_port_dir(port,dir);
  }
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


/* -- PID Handler -- */
/*
  return value = 0: no packet response required
               = 1: ptx packet will be send

*/
int8_t pid_command(CANRxFrame *prx,CANTxFrame *ptx)
{
  int8_t ret = 0;
  if(prx->RTR == CAN_RTR_DATA)
  {
    uint8_t flag = prx->data8[0] & 0x03;
    uint8_t clr_fault = (prx->data8[0] >>2) & 0x01;
    uint8_t mode = (prx->data8[0] >> 4);
    //uint8_t cmd_index = prx->data8[1];
    uint16_t cmd_value = prx->data16[1];
    int16_t cmd_value_i16 = prx->data16[1];

    
    if(clr_fault)
    {
      tdmotc_ResetFault();
    }

    if(2U == flag && (false == tdmotc_GetFault()))
    {
      tdmotc_Start(mode);
    }
    else if(1U == flag){ // stop
      tdmotc_Stop();
    }

    uint8_t mode_readback = tdmotc_GetMode();
    float fv = 0.01f * ((float)cmd_value);
    float fvs = 0.01f * ((float)cmd_value_i16);

    switch (mode_readback)
    {
      case TDMOTC_MODE_S:
      tdmotc_SetSpeedCmd(fvs);
      break;

      case TDMOTC_MODE_P:
      //tdmotc_SetPosCmd(fv);
      tpcmdh_SetPosCmd(fv);
      break;

      case TDMOTC_MODE_P2:
      tdmotc_SetPosCmd2(fv);
      break;

      default:
      break;
    }
  }
  else{ // REMOTE Request, GET
    uint8_t flag = tdmotc_GetIsStart();
    uint8_t mode = tdmotc_GetMode();
    uint8_t cmd_index = prx->data8[1];
    int16_t cmd_value = pid_get_cmd_value(mode, cmd_index);
    
    ptx->RTR = CAN_RTR_DATA;
    ptx->EID = prx->EID;
    ptx->IDE = prx->IDE;
    ptx->DLC = 8;
    
    ptx->data8[0] = (flag & 0x03) | (mode << 4);
    ptx->data8[1] = cmd_index;
    ptx->data16[1] = cmd_value;
    ret = 1;
  }
  return ret;
}


int8_t pid_parameter(CANRxFrame *prx,CANTxFrame *ptx)
{
  int8_t ret = 0;
  if(prx->RTR == CAN_RTR_DATA){
    
    if(prx->DLC == 1){
      //- todo: fulfill variables below from pid command
      uint8_t cmd_index = prx->data8[0];
      float cmd_value = pid_get_para_value(cmd_index);
      
      ptx->RTR = CAN_RTR_DATA;
      ptx->EID = prx->EID;
      ptx->IDE = prx->IDE;
      ptx->DLC = 8;
      
      ptx->data8[0] = cmd_index;
      memcpy(&ptx->data8[1], (void*)&cmd_value,4);
      ret = 1;
    }
    else{
      uint8_t cmd_index = prx->data8[0];
      //int32_t cmd_value;
      // take care of byte order
  //    memcpy((void*)&cmd_value,&prx->data8[1],4);
      float cmd_value_f;// = tdmotc_ConvCAN2Sig(cmd_value);
      memcpy((void*)&cmd_value_f,&prx->data8[1],4);

      switch (cmd_index)
      {
        case CAN_GSID_KP_P:
        tdmotc_SetPID(TDMOTC_PID_P, TDMOTC_PID_ID_P, cmd_value_f);
        //tdmotc_SetPCCFG(TDMOTC_PCCFG_ID_KP, cmd_value_f);
        break;

        case CAN_GSID_KI_P:
        tdmotc_SetPID(TDMOTC_PID_P, TDMOTC_PID_ID_I, cmd_value_f);
        break;

        case CAN_GSID_KD_P:
        tdmotc_SetPID(TDMOTC_PID_P, TDMOTC_PID_ID_D, cmd_value_f);
        break;

        case CAN_GSID_KP_S:
        tdmotc_SetPID(TDMOTC_PID_S, TDMOTC_PID_ID_P, cmd_value_f);
        break;

        case CAN_GSID_KI_S:
        tdmotc_SetPID(TDMOTC_PID_S, TDMOTC_PID_ID_I, cmd_value_f);
        break;

        case CAN_GSID_KD_S:
        tdmotc_SetPID(TDMOTC_PID_S, TDMOTC_PID_ID_D, cmd_value_f);
        break;

        case CAN_GSID_MOT_T_MAX_ABS:
        tdmotc_SetMotTqMaxAbs(cmd_value_f);
        break;

        case CAN_GSID_MOT_S_MAX_ABS:
        tdmotc_SetAxisSMaxAbs(cmd_value_f);
        break;

        case CAN_GSID_G_A2M:
        /*Currently not avaliable*/
        break;

        case CAN_GSID_TQBC_MAX:
        tdmotc_SetTQBC(TDMOTC_TQBC_ID_OUT_MAX, cmd_value_f);
        break;

        case CAN_GSID_TQBC_MIN:
        tdmotc_SetTQBC(TDMOTC_TQBC_ID_OUT_MIN, cmd_value_f);
        break;

        case CAN_GSID_G:
        tdmotc_SetTQBC(TDMOTC_TQBC_ID_GAIN, cmd_value_f);
        break;

        case CAN_GSID_ZCP:
        tdmotc_SetTQBC(TDMOTC_TQBC_ID_ZCP, cmd_value_f);
        break;

        case CAN_GSID_UPDATE:
        tdmotc_UpdateTQBC();
        break;

        case CAN_GSID_PC_PERR_THOLD:
        tdmotc_SetPCCFG(TDMOTC_PCCFG_ID_PERR_THOLD, cmd_value_f);
        break;

        case CAN_GSID_PC_S_CMD_MIN:
        tdmotc_SetPCCFG(TDMOTC_PCCFG_ID_S_CMD_MIN, cmd_value_f);
        break;

        case CAN_GSID_PC_S_CMD_MAX:
        tdmotc_SetPCCFG(TDMOTC_PCCFG_ID_S_CMD_MAX, cmd_value_f);
        break;

      default:
        /*Invalid index, do noting*/
        break;
      }
      
    }
//  else{ // REMOTE Request, GET
  }
  return ret;
}

int8_t pid_request(CANRxFrame *prx,CANTxFrame *ptx)
{
  
}

