#include "ch.h"
#include "hal.h"
#include "task_analog_input.h"
#include "ad76_drv.h"
#include "app_config.h"

#define EV_AD7606_RDY   EVENT_MASK(10)
#define EV_AD7606_ACQ   EVENT_MASK(11)

#define LSB     20/65536

struct runTime{
  thread_t *thd_ad7606;
  thread_t *main;
  AD7606Driver *ad7606;
  event_listener_t el_ad7606;
  virtual_timer_t vt7606;
  uint16_t sampleIntervalms;
  float convertedData[AD76_NOF_CHANNEL];
};

static struct runTime runTime, *analogRuntime;

static const SPIConfig spicfg_ad76 = {
  FALSE,
  NULL,
  NULL,//GPIOB,
  NULL,//GPIOB_SPI1_CS,
  SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
};

static AD7606Config ad7606config = {
  &SPID5,
  &spicfg_ad76,
  GPIOF, // cs
  6,
  GPIOF, //convsta
  10,
  GPIOI, // convstb
  11,
  GPIOI, // busy
  8,
  GPIOI, // rst
  10,
  GPIOI, // fst data
  9,
  GPIOH, // os0
  2,
  GPIOC, // os1
  3,
  GPIOC, // os2
  2,
  GPIOC, // stby
  1
};

static void ad7606_to(void *arg)
{
  chSysLockFromISR();
  chEvtSignalI(runTime.thd_ad7606,EV_AD7606_ACQ);
  chVTSetI(&runTime.vt7606, TIME_MS2I(runTime.sampleIntervalms),ad7606_to,NULL);
  chSysUnlockFromISR();
}

static AD7606Driver ad7606;
//= {
//  &ad7606config,
//  AD7608
//};

static THD_WORKING_AREA(waAD7606,1024);
static THD_FUNCTION(procAD7606 ,p)
{
  // start ad7606
  ad7606_init(runTime.ad7606,&ad7606config);
  ad7606_start(runTime.ad7606);
  chEvtRegisterMask(&runTime.ad7606->ev_drdy,&runTime.el_ad7606,EV_AD7606_RDY);
  
  chVTObjectInit(&runTime.vt7606);
  chVTSet(&runTime.vt7606,TIME_MS2I(10),ad7606_to,NULL);
  ad7606_reset(runTime.ad7606);
  eventmask_t evt;
//  ad7606_start_conv(runTime.ad7606,1);
  while(!chThdShouldTerminateX()){
    evt = chEvtWaitAny(ALL_EVENTS);
    if(evt & EV_AD7606_RDY){
      ad7606_read_conversion(runTime.ad7606,2);
      for(uint8_t i=0;i<AD76_NOF_CHANNEL;i++){
        runTime.convertedData[i] = runTime.ad7606->data[i] * LSB;
      }
      if(runTime.main != NULL){
        chEvtSignal(runTime.main, EV_ADC_ACQUIRED);
      }
    }
    if(evt & EV_AD7606_ACQ){
      ad7606_start_conv(runTime.ad7606,3);
    }
  }
  
  chVTReset(&runTime.vt7606);
  chThdExit(MSG_OK);
}

void analog_input_task_init()
{
  runTime.ad7606 = &ad7606;
  runTime.sampleIntervalms = 10;
  analogRuntime = &runTime;
  runTime.main = chRegFindThreadByName("MAIN");
  runTime.thd_ad7606 = chThdCreateStatic(waAD7606,sizeof(waAD7606),NORMALPRIO,procAD7606,NULL);
}

void analog_input_task_stop()
{
  if(runTime.thd_ad7606 != NULL){
    chThdTerminate(runTime.thd_ad7606);
    chThdWait(runTime.thd_ad7606);
  }  
}

int8_t analog_input_read(uint8_t channel, int32_t *data)
{
  int8_t ret = 0;
  if(channel == 0xff){
    memcpy(data,ad7606.data,32);
  }
  else if(channel < 8){
    *data = ad7606.data[channel];
  }
  else{
    ret = -1;
  }
  return ret;
}

int8_t analog_input_readf(uint8_t channel, float *data)
{
  int8_t ret = 0;
  if(channel == 0xff){
    memcpy(data,runTime.convertedData,32);
  }
  else if(channel < 8){
    *data = runTime.convertedData[channel];
  }
  else{
    ret = -1;
  }
  return ret;
}

void analog_input_set_sample_interval(uint16_t ms)
{
  runTime.sampleIntervalms = ms;
}