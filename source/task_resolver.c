#include "ch.h"
#include "hal.h"
#include "task_resolver.h"
#include "ad2s_drv.h"
#include "ylib/numeric/filters/iir.h"
#include "app_config.h"
#include <arm_math.h>
#include "ylib/numeric/filters/cmsis_fir.h"

//#define EV_UPDATE   EVENT_MASK(0)
#define EV_PERIODIC     EVENT_MASK(1)

#define EXCI_FREQ_1K     4
#define EXCI_FREQ_10K   40
#define EXCI_FERQ       EXCI_FREQ_10K

/* 
  data calculate from http://t-filter.engineerjs.com/
  low pass [0 20] [30 100], FS=200
*/

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 200 Hz

* 0 Hz - 20 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 2.3133909166191504 dB

* 40 Hz - 100 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -45.01618674525617 dB

*/

//#define FILTER_TAP_NUM  27
#define FILTER_BLOCK_SZ 93

//static float32_t filter_taps[FILTER_TAP_NUM] = {
//  -0.01259277478717816,
//  -0.02704833486706803,
//  -0.031157016036431583,
//  -0.003351666747179282,
//  0.06651710329324828,
//  0.1635643048779222,
//  0.249729473226146,
//  0.2842779082622769,
//  0.249729473226146,
//  0.1635643048779222,
//  0.06651710329324828,
//  -0.003351666747179282,
//  -0.031157016036431583,
//  -0.02704833486706803,
//  -0.01259277478717816
//};

//static float32_t filter_taps[FILTER_TAP_NUM] = {
//  -0.007775715121256268,
//  -0.007938974136595613,
//  -0.009534265788246128,
//  -0.008779578259641298,
//  -0.004381884421750879,
//  0.004666131585205163,
//  0.0188044731228937,
//  0.03764144706001848,
//  0.05992101812383003,
//  0.08357444021744635,
//  0.10601855702701225,
//  0.12454015906119098,
//  0.13674393462068657,
//  0.14100385434561774,
//  0.13674393462068657,
//  0.12454015906119098,
//  0.10601855702701225,
//  0.08357444021744635,
//  0.05992101812383003,
//  0.03764144706001848,
//  0.0188044731228937,
//  0.004666131585205163,
//  -0.004381884421750879,
//  -0.008779578259641298,
//  -0.009534265788246128,
//  -0.007938974136595613,
//  -0.007775715121256268
//};
#define FILTER_TAP_NUM 93

static float32_t filter_taps[FILTER_TAP_NUM] = {
  -0.0045644380812448395,
  -0.0004239214773665623,
  -0.0003992160094902055,
  -0.0003410784893278679,
  -0.000246559170922285,
  -0.00011254431170462079,
  0.00006433750801042219,
  0.0002870933492719705,
  0.0005587009072068039,
  0.0008818359747837215,
  0.0012587683624311436,
  0.0016910582068537408,
  0.0021856908690502577,
  0.002726710900242062,
  0.0033400842043236102,
  0.0040111517348650795,
  0.004739077342511737,
  0.005524253099514465,
  0.0063660475475944564,
  0.007263122811066336,
  0.008212361291143213,
  0.009210018111155885,
  0.010251426837018239,
  0.011331469979433535,
  0.01244362364387908,
  0.013584873647048157,
  0.01474370531018102,
  0.015917964027590856,
  0.01709913746351596,
  0.01827867257835206,
  0.01944880098525925,
  0.020601194890077695,
  0.02172846305781673,
  0.022822065011098805,
  0.0238738289199096,
  0.02487547673817073,
  0.025819341620558257,
  0.026697029918866366,
  0.02750356254696736,
  0.028230666791627817,
  0.028873750993741547,
  0.029427571925871056,
  0.02988701980332222,
  0.030248790890860813,
  0.030508789150617766,
  0.030665772386255268,
  0.03071819701905401,
  0.030665772386255268,
  0.030508789150617766,
  0.030248790890860813,
  0.02988701980332222,
  0.029427571925871056,
  0.028873750993741547,
  0.028230666791627817,
  0.02750356254696736,
  0.026697029918866366,
  0.025819341620558257,
  0.02487547673817073,
  0.0238738289199096,
  0.022822065011098805,
  0.02172846305781673,
  0.020601194890077695,
  0.01944880098525925,
  0.01827867257835206,
  0.01709913746351596,
  0.015917964027590856,
  0.01474370531018102,
  0.013584873647048157,
  0.01244362364387908,
  0.011331469979433535,
  0.010251426837018239,
  0.009210018111155885,
  0.008212361291143213,
  0.007263122811066336,
  0.0063660475475944564,
  0.005524253099514465,
  0.004739077342511737,
  0.0040111517348650795,
  0.0033400842043236102,
  0.002726710900242062,
  0.0021856908690502577,
  0.0016910582068537408,
  0.0012587683624311436,
  0.0008818359747837215,
  0.0005587009072068039,
  0.0002870933492719705,
  0.00006433750801042219,
  -0.00011254431170462079,
  -0.000246559170922285,
  -0.0003410784893278679,
  -0.0003992160094902055,
  -0.0004239214773665623,
  -0.0045644380812448395
};


// fir dataset
float32_t az_pos_sin_input[FILTER_TAP_NUM];
float32_t az_pos_cos_input[FILTER_TAP_NUM];
float32_t el_pos_sin_input[FILTER_TAP_NUM];
float32_t el_pos_cos_input[FILTER_TAP_NUM];
float32_t az_pos_sin_output[FILTER_TAP_NUM + 2*FILTER_BLOCK_SZ-1];
float32_t az_pos_cos_output[FILTER_TAP_NUM + 2*FILTER_BLOCK_SZ-1];
float32_t el_pos_sin_output[FILTER_TAP_NUM + 2*FILTER_BLOCK_SZ-1];
float32_t el_pos_cos_output[FILTER_TAP_NUM + 2*FILTER_BLOCK_SZ-1];


float32_t az_spd_input[FILTER_TAP_NUM];
float32_t el_spd_input[FILTER_TAP_NUM];
float32_t az_spd_output[FILTER_TAP_NUM + 2*FILTER_BLOCK_SZ-1];
float32_t el_spd_output[FILTER_TAP_NUM + 2*FILTER_BLOCK_SZ-1];

struct runTime{
  thread_t *self;
  thread_t *main;
  //AD2S1210Driver **ad2s_dev;
  AD2S1210Driver *ad2s_dev;
  AD2S1210Driver *ad2s_dev2;
  virtual_timer_t vtResolver;
  //_float_filter_t speed[2];
  //_float_filter_circular_t position[2];
  //_int_filter_t position_u[2];
  // FIR filter
  _fir_angular_instance_t fir_pos[2];
  _fir_instance_t fir_spd[2];
};

static struct runTime runTime, *resolverRuntime;

static const SPIConfig spicfg_ad2s_dev0 = {
  FALSE,
  NULL,
  NULL,
  NULL,
  SPI_CR1_BR_2  |  SPI_CR1_CPHA
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
  chVTSetI(&runTime.vtResolver, TIME_MS2I(1),ad2s_to,NULL);
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
  chVTSet(&runTime.vtResolver,TIME_MS2I(1),ad2s_to,NULL);

  ad2s_SetResolution(runTime.ad2s_dev,RES_16b);
  runTime.ad2s_dev->exci_freq = EXCI_FERQ; // freq * 2^15/8192000, 4=1K, 40 = 10K
  ad2s_reset(runTime.ad2s_dev);
  ad2s_InitConfig(runTime.ad2s_dev);
  //ad2s_read_registers(runTime.ad2s_dev);
  
  ad2s_SetResolution(runTime.ad2s_dev2,RES_16b);
  runTime.ad2s_dev2->exci_freq = EXCI_FERQ;
  ad2s_reset(runTime.ad2s_dev2);
  ad2s_InitConfig(runTime.ad2s_dev2);

//  for(uint8_t i=0;i<2;i++){
//    runTime.speed[i].stages = 4;
//    runTime.speed[i].reset = 1;
//    runTime.position[i].stages = 4;
//    runTime.position[i].reset = 1;
//    runTime.position_u[i].stages = 4;
//    runTime.position_u[i].reset = 1;
//  }
  
  while(!chThdShouldTerminateX()){
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    if(evt & EV_UPDATE){
      
    }
    
    if(evt & EV_PERIODIC){
      ad2S_Refresh(runTime.ad2s_dev);
      //ad2s_read_registers(runTime.ad2s_dev);
      ad2S_Refresh(runTime.ad2s_dev2);
//      iir_insert_f(&runTime.speed[0],runTime.ad2s_dev->currentSpeed);
//      iir_insert_f(&runTime.speed[1],runTime.ad2s_dev2->currentSpeed);
//      iir_insert_circular_f(&runTime.position[0],runTime.ad2s_dev->currentAngleRad);
//      iir_insert_circular_f(&runTime.position[1],runTime.ad2s_dev2->currentAngleRad);
//      
//      iir_insert(&runTime.position_u[0],(int32_t)runTime.ad2s_dev->position);
//      iir_insert(&runTime.position_u[1],(int32_t)runTime.ad2s_dev2->position);
      
      // FIR
      
      cmssi_fir_update(&runTime.fir_spd[0],runTime.ad2s_dev->currentSpeed);
      cmssi_fir_update(&runTime.fir_spd[1],runTime.ad2s_dev2->currentSpeed);
      cmssi_fir_angular_update(&runTime.fir_pos[0],runTime.ad2s_dev->currentAngleRad);
      cmssi_fir_angular_update(&runTime.fir_pos[1],runTime.ad2s_dev2->currentAngleRad);
      if(runTime.main != NULL){
        chEvtSignal(runTime.main, EV_RESOLVER_ACQUIRED);
      }
    }
    
  }
  
  chVTReset(&runTime.vtResolver);
  chThdExit(MSG_OK);
}

void resolver_task_init()
{
//  runTime.ad2s_dev = (AD2S1210Driver**)(ad2s1210);
  runTime.ad2s_dev = (ad2s1210);
  runTime.ad2s_dev2 = &ad2s1210[1];
  resolverRuntime = &runTime;
  
  runTime.fir_pos[0].sin_input = &az_pos_sin_input[0];
  runTime.fir_pos[0].cos_input = &az_pos_cos_input[0];
  runTime.fir_pos[0].sin_output = &az_pos_sin_output[0];
  runTime.fir_pos[0].cos_output = &az_pos_cos_output[0];
  runTime.fir_pos[0].coe = filter_taps;
  runTime.fir_pos[0].taps = FILTER_TAP_NUM;
  runTime.fir_pos[0].size = FILTER_BLOCK_SZ;

  runTime.fir_pos[1].sin_input = &el_pos_sin_input[0];
  runTime.fir_pos[1].cos_input = &el_pos_cos_input[0];
  runTime.fir_pos[1].sin_output = &el_pos_sin_output[0];
  runTime.fir_pos[1].cos_output = &el_pos_cos_output[0];
  runTime.fir_pos[1].coe = filter_taps;
  runTime.fir_pos[1].taps = FILTER_TAP_NUM;
  runTime.fir_pos[1].size = FILTER_BLOCK_SZ;
  

  runTime.fir_spd[0].input = az_spd_input;
  runTime.fir_spd[0].output = &az_spd_output[0];
  runTime.fir_spd[0].coe = filter_taps;
  runTime.fir_spd[0].taps = FILTER_TAP_NUM;
  runTime.fir_spd[0].size = FILTER_BLOCK_SZ;

  runTime.fir_spd[1].input = el_spd_input;
  runTime.fir_spd[1].output = &el_spd_output[0];
  runTime.fir_spd[1].coe = filter_taps;
  runTime.fir_spd[1].taps = FILTER_TAP_NUM;
  runTime.fir_spd[1].size = FILTER_BLOCK_SZ;
  
  cmsis_fir_angular_init(&runTime.fir_pos[0]);
  cmsis_fir_angular_init(&runTime.fir_pos[1]);
  cmsis_fir_init(&runTime.fir_spd[0]);
  cmsis_fir_init(&runTime.fir_spd[1]);
  runTime.main = chRegFindThreadByName("MAIN");
  runTime.self = chThdCreateStatic(waAD2S,sizeof(waAD2S),NORMALPRIO,procAD2S,NULL);
}

void resolver_task_stop()
{
  if(runTime.self != NULL){
    chThdTerminate(runTime.self);
    chThdWait(runTime.self);
    runTime.self = NULL;
  }
}
float resolver_get_speed(uint8_t id)
{
  if(id < 2){
    //return ad2s1210[id].currentSpeed;
    //if((runTime.speed[id].last < -0.5F) || (runTime.speed[id].last > 0.5F)){
      //return runTime.speed[id].last;
//      return round(runTime.fir_spd[id].output[0]*1000)/1000.;
    if((runTime.fir_spd[id].output[0] < 0.5) && (runTime.fir_spd[id].output[0] > -0.5)){
      return 0.0f;
    }
    else{
      return round(runTime.fir_spd[id].output[0]*100)/100.;
    }
    //}
  }
  return 0;
}

float resolver_get_position(uint8_t id)
{
  if(id < 2){
    //return ad2s1210[id].currentAngle; 
    //return runTime.position[id].last;
    return round(runTime.fir_pos[id].output*100)/100.;
  }
  return 0;
}

/* 240308, reverse direction */
float resolver_get_position_deg(uint8_t id)
{
  if(id < 2){
//    return runTime.position[id].last*RAD2DEGG;
    return runTime.fir_pos[id].output*RAD2DEGG;
  }
  return 0.0f;
}

uint16_t resolver_get_position_raw(uint8_t id)
{
//  if(id < 2){
//    return (uint16_t)runTime.position_u[id].last;
//  }
  return 0;
}

uint8_t lastError(uint8_t id)
{
  if(id < 2){
    return ad2s1210[id].fault;
  }
}
