#include "ch.h"
#include "hal.h"
#include "task_resolver.h"
#include "ad2s_drv.h"

#define EV_UPDATE   EVENT_MASK(0)
#define EV_PERIODIC     EVENT_MASK(1)

struct runTime{
  thread_t *self;
  //AD2S1210Driver **ad2s_dev;
  AD2S1210Driver *ad2s_dev;
  AD2S1210Driver *ad2s_dev2;
  virtual_timer_t vtResolver;
};

static struct runTime runTime, *resolverRuntime;

static const SPIConfig spicfg_ad2s_dev0 = {
  FALSE,
  NULL,
  NULL,
  NULL,
  SPI_CR1_BR_1 | SPI_CR1_CPOL
};

static AD2S1210Config ad2s_config_dev0 = {
  &SPID5,
  &spicfg_ad2s_dev0,
  RES_16b,
  GPIOF,6,
  GPIOB,8,
  NULL,0,
  GPIOG,14,
  GPIOG,13,
  GPIOB,4,
  GPIOB,3,
  GPIOC,13,
  GPIOB,7,
  GPIOD,4,
  GPIOD,5,
};

static AD2S1210Config ad2s_config_dev1 = {
  &SPID5,
  &spicfg_ad2s_dev0,
  RES_16b,
  GPIOI,11,
  GPIOB,8,
  NULL,0,
  GPIOG,14,
  GPIOG,13,
  GPIOB,4,
  GPIOB,3,
  GPIOC,13,
  GPIOB,7,
  GPIOD,4,
  GPIOD,5,
};

static AD2S1210Driver ad2s1210[2];

static void ad2s_to(void *arg)
{
  chSysLockFromISR();
  chEvtSignalI(runTime.self,EV_PERIODIC);
  chVTSetI(&runTime.vtResolver, TIME_MS2I(10),ad2s_to,NULL);
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waAD2S,1024);
static THD_FUNCTION(procAD2S ,p)
{
  ad2S_init(runTime.ad2s_dev,&ad2s_config_dev0);
  ad2S_start(runTime.ad2s_dev);
  
  ad2S_init(runTime.ad2s_dev2,&ad2s_config_dev1);
  ad2S_start(runTime.ad2s_dev2);
  chVTObjectInit(&runTime.vtResolver);
  chVTSet(&runTime.vtResolver,TIME_MS2I(10),ad2s_to,NULL);

  ad2s_SetResolution(runTime.ad2s_dev,RES_16b);
  ad2s_reset(runTime.ad2s_dev);
  ad2s_read_registers(runTime.ad2s_dev);
  
  while(!chThdShouldTerminateX()){
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    if(evt & EV_UPDATE){
      
    }
    
    if(evt & EV_PERIODIC){
      ad2S_Refresh(runTime.ad2s_dev);
      //ad2s_read_registers(runTime.ad2s_dev);
      ad2S_Refresh(runTime.ad2s_dev2);
    }
    
  }
}

void resolver_task_init()
{
//  runTime.ad2s_dev = (AD2S1210Driver**)(ad2s1210);
  runTime.ad2s_dev = (ad2s1210);
  runTime.ad2s_dev2 = &ad2s1210[1];
  resolverRuntime = &runTime;
  runTime.self = chThdCreateStatic(waAD2S,sizeof(waAD2S),NORMALPRIO,procAD2S,NULL);
}

float resolver_get_speed(uint8_t id)
{
  if(id < 2){
    return ad2s1210[id].currentSpeed;
  }
  return 0;
}

float resolver_get_position(uint8_t id)
{
  if(id < 2){
    return ad2s1210[id].currentAngle; 
  }
  return 0;
}

