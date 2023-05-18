#include "ch.h"
#include "hal.h"
#include "task_dac.h"
#include "ad57_drv.h"
#include "app_config.h"

#define EV_UPDATE_DAC   EVENT_MASK(0)
#define EV_PERIODIC     EVENT_MASK(1)
#define EV_UPDATE_CG     EVENT_MASK(2)
#define EV_UPDATE_FG     EVENT_MASK(3)
#define EV_UPDATE_OFFSET     EVENT_MASK(4)

struct runTime{
  thread_t *self;
  thread_t *main;
  AD57x4Driver *ad57;
  virtual_timer_t vtDAC;
  uint8_t updateChannelMask;
};

static struct runTime runTime, *dacRuntime;

static const SPIConfig spicfg_ad57 = {
  FALSE,
  NULL,
  NULL,
  NULL,
  SPI_CR1_BR_2 |  SPI_CR1_CPHA
};

static AD57Config ad57config = {
  &SPID4,
  &spicfg_ad57,
  GPIOE, // sync
  4,
  GPIOI, // latch
  7,
  GPIOC, // rst
  13,
  GPIOI, // bin
  6,
};

static AD57x4Driver ad57;

static void ad57_to(void *arg)
{
  chSysLockFromISR();
  chEvtSignalI(runTime.self,EV_PERIODIC);
  chVTSetI(&runTime.vtDAC, TIME_MS2I(10),ad57_to,NULL);
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waAD57,1024);
static THD_FUNCTION(procAD57 ,p)
{
  _dac_config *config = (_dac_config*)p;
  
  ad57_init(runTime.ad57,&ad57config);
  ad57_start(runTime.ad57);
  
  ad57_get_corse_gain(runTime.ad57,0xff);
  ad57_get_fine_gain(runTime.ad57);
  ad57_get_offset(runTime.ad57);
  
//  for(uint8_t i=0;i<AD57_CHANNELS;i++){
//    runTime.ad57->corse_gain[i] = config->corse_gain[i];
    
  for(uint8_t i=0;i<4;i++){
    runTime.ad57->data[i] = 0x0;
  }

  memcpy(runTime.ad57->corse_gain, config->corse_gain, AD57_CHANNELS*sizeof(uint16_t));
  memcpy(runTime.ad57->fine_gain, config->fine_gain, AD57_CHANNELS*sizeof(uint16_t));
  memcpy(runTime.ad57->offset, config->offset, AD57_CHANNELS*sizeof(uint16_t));
  
  ad57_set_corse_gain(runTime.ad57,0xff);
  ad57_set_fine_gain(runTime.ad57,0xff);
  ad57_set_offset(runTime.ad57,0xff);
  
  ad57_set_dac(runTime.ad57,0xff);

  
  chVTObjectInit(&runTime.vtDAC);
  // for test purpose, uncomment below line
  //chVTSet(&runTime.vtDAC,TIME_MS2I(100),ad57_to,NULL);
  ad57_set_simulataneous_update(runTime.ad57,1);
  while(!chThdShouldTerminateX()){
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    if(evt & EV_UPDATE_DAC){
      if(runTime.updateChannelMask == 0x0F){
        ad57_set_dac(runTime.ad57,0xff);
      }
      else{
        for(uint8_t i=0;i<4;i++){
          if(runTime.updateChannelMask & (1<<i)){
            ad57_set_dac(runTime.ad57,i);
            runTime.updateChannelMask &= ~(1<<i); // clear
          }
        }
      }
      chEvtSignal(runTime.main,EV_DAC_UPDATED);
    }
    
    if(evt & EV_PERIODIC){
      for(uint8_t i=0;i<4;i++){
        runTime.ad57->data[i] += 0x10;
      }
      ad57_set_dac(runTime.ad57,0xff);
    }
  }
  chThdExit(MSG_OK);
}

void analog_output_task_init(void *config)
{
  runTime.ad57 = &ad57;
  dacRuntime = &runTime;
  runTime.main = chRegFindThreadByName("MAIN");
  runTime.self = chThdCreateStatic(waAD57,sizeof(waAD57),NORMALPRIO,procAD57,config);
}

void analog_output_task_stop()
{
  if(runTime.self != NULL){
    chThdTerminate(runTime.self);
    chThdWait(runTime.self);
  }
}

int8_t analog_output_set_update_on_write(uint8_t flag)
{
  ad57_set_simulataneous_update(runTime.ad57,flag);
  return 0;
}

int8_t analog_output_update()
{
  ad57_trigger_update(runTime.ad57);
  return 0;
}

int8_t analog_output_set_data(uint8_t channel, int16_t *dacv)
{
  int8_t ret = 0;
  if(channel == 0xff){
    //runTime.ad57->data = dacv;
    memcpy(runTime.ad57->data,dacv,8);
    runTime.updateChannelMask = 0xF;
  }
  else if(channel < 4){
    runTime.ad57->data[channel] = *dacv;
    runTime.updateChannelMask = (1 << channel);
  }
  else{
    ret = -1;
    runTime.updateChannelMask = 0x0;
  }
  if(ret == 0){
    chEvtSignal(runTime.self,EV_UPDATE_DAC);
  }
  return ret;
}

static int16_t get_raw_value(float value)
{
  int16_t ret = 0;
  // map raw data from +/- 10v
  float r = (value)/10.;
  ret = r * 32768;
  return ret;
}

int8_t analot_output_get_raw(uint8_t channel, uint16_t *data)
{
  int8_t ret = 0;
  if(channel == 0xff){
    memcpy(data,runTime.ad57->data,8);
  }
  else if(channel < 4){
    *data = runTime.ad57->data[channel];
  }
  return ret;
}

int8_t analog_output_set_voltage(uint8_t channel, float *dacv)
{
  int8_t ret = 0;
  if(channel == 0xff){
    for(uint8_t i=0;i<4;i++){
      runTime.ad57->data[i] = get_raw_value(dacv[i]);
    }
    runTime.updateChannelMask = 0xF;
  }
  else if(channel < 4){
    runTime.ad57->data[channel] = get_raw_value(*dacv);
    runTime.updateChannelMask |= (1 << channel);
  }
  else{
    ret = -1;
    runTime.updateChannelMask = 0x0;
  }
  if(ret == 0){
    chEvtSignal(runTime.self,EV_UPDATE_DAC);
  }
  return ret;
}

int8_t analog_output_set_corse_gain(uint8_t channel, uint16_t *cg)
{
  int8_t ret = 0;
  if(channel == 0xff){
    //runTime.ad57->corse_gain = cg;
    memcpy(runTime.ad57->corse_gain,cg,8);
  }
  else if(channel < 4){
    runTime.ad57->corse_gain[channel] = *cg;
  }
  else{
    ret = -1;
  }
  if(ret == 0){
    chEvtSignal(runTime.self,EV_UPDATE_CG);
  }
  return ret;
}

int8_t analog_output_set_fine_gain(uint8_t channel, uint16_t *fg)
{
  int8_t ret = 0;
  if(channel == 0xff){
    //runTime.ad57->fine_gain = fg;
    memcpy(runTime.ad57->fine_gain,fg,8);
  }
  else if(channel < 4){
    runTime.ad57->fine_gain[channel] = *fg;
  }
  else{
    ret = -1;
  }
  if(ret == 0){
    chEvtSignal(runTime.self,EV_UPDATE_FG);
  }
  return ret;
}

int8_t analog_output_set_offset(uint8_t channel, uint16_t *offset)
{
  int8_t ret = 0;
  if(channel == 0xff){
    //runTime.ad57->offset = offset;
    memcpy(runTime.ad57->offset,offset,8);
  }
  else if(channel < 4){
    runTime.ad57->offset[channel] = *offset;
  }
  else{
    ret = -1;
  }
  if(ret == 0){
    chEvtSignal(runTime.self,EV_UPDATE_OFFSET);
  }
  return ret;
}
