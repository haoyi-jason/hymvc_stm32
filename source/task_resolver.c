#include "ch.h"
#include "hal.h"
#include "task_resolver.h"
#include "ad2s_drv.h"
#include "ylib/numeric/filters/iir.h"

#define EV_UPDATE   EVENT_MASK(0)
#define EV_PERIODIC     EVENT_MASK(1)

struct runTime{
  thread_t *self;
  //AD2S1210Driver **ad2s_dev;
  AD2S1210Driver *ad2s_dev;
  AD2S1210Driver *ad2s_dev2;
  virtual_timer_t vtResolver;
  _float_filter_t speed[2];
  _float_filter_circular_t position[2];
  _int_filter_t position_u[2];
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
  &SPID2,
  &spicfg_ad2s_dev0,
  0,            // reverse
  RES_16b,      // resolution
  GPIOB,12,      // cs
  NULL,8,      // soe
  NULL,0,       // fsync
  GPIOC,5,     // a0
  GPIOB,0,     // a1
  NULL,4,      // res0
  NULL,3,      // res1
  GPIOH,8,     // rst
  GPIOB,10,      // sample
  NULL,4,      // rd
  GPIOB,12,      // wr
};

static AD2S1210Config ad2s_config_dev1 = {
  &SPID2,
  &spicfg_ad2s_dev0,
  0,            // reverse
  RES_16b,      // resolution
  GPIOC,4,      // cs
  NULL,8,      // soe
  NULL,0,       // fsync
  GPIOC,5,      // a0
  GPIOB,0,      // a1
  NULL,4,      // res0
  NULL,3,      // res1
  GPIOH,8,      // rst
  GPIOB,10,     // sample
  NULL,4,      // rd
  GPIOC,4,      // wr
};

static AD2S1210Config ad2s_config_dev2 = {
  &SPID1,
  &spicfg_ad2s_dev0,
  0,
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
  
  for(uint8_t i=0;i<2;i++){
    runTime.speed[i].stages = 4;
    runTime.speed[i].reset = 1;
    runTime.position[i].stages = 4;
    runTime.position[i].reset = 1;
    runTime.position_u[i].stages = 4;
    runTime.position_u[i].reset = 1;
  }
  
  while(!chThdShouldTerminateX()){
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    if(evt & EV_UPDATE){
      
    }
    
    if(evt & EV_PERIODIC){
      ad2S_Refresh(runTime.ad2s_dev);
      //ad2s_read_registers(runTime.ad2s_dev);
      ad2S_Refresh(runTime.ad2s_dev2);
      iir_insert_f(&runTime.speed[0],runTime.ad2s_dev->currentSpeed);
      iir_insert_f(&runTime.speed[1],runTime.ad2s_dev2->currentSpeed);
      iir_insert_circular_f(&runTime.position[0],runTime.ad2s_dev->currentAngle/60.);
      iir_insert_circular_f(&runTime.position[1],runTime.ad2s_dev2->currentAngle/60.);
      
      iir_insert(&runTime.position_u[0],(int32_t)runTime.ad2s_dev->position);
      iir_insert(&runTime.position_u[1],(int32_t)runTime.ad2s_dev2->position);
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
    //return ad2s1210[id].currentSpeed;
    return runTime.speed[id].last;
  }
  return 0;
}

float resolver_get_position(uint8_t id)
{
  if(id < 2){
    //return ad2s1210[id].currentAngle; 
    return runTime.position[id].last*60;
  }
  return 0;
}

float resolver_get_position_deg(uint8_t id)
{
  if(id < 2){
    return (ad2s1210[id].currentAngle * 0.016667f); 
  }
  return 0.0f;
}

uint16_t resolver_get_position_raw(uint8_t id)
{
  if(id < 2){
    return (uint16_t)runTime.position_u[id].last;
  }
  return 0;
}
