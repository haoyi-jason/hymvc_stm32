#include "ch.h"
#include "hal.h"
#include "digital_io.h"
#include "app_config.h"

typedef struct{
  thread_t *self;
  thread_t *main;
  thread_reference_t ref;
  uint16_t last_iso_in;
  uint16_t last_iso_out;
}_runTime;

static _runTime runTime,*pRunTime;

dio_def iso_io_out[] = {
  {OUTPUT_GPIO,GPIOC,11},
  {OUTPUT_GPIO,GPIOC,10},
  {OUTPUT_GPIO,GPIOA,15},
  {OUTPUT_GPIO,GPIOI,3},
  {OUTPUT_GPIO,GPIOI,2},
  {OUTPUT_GPIO,GPIOC,8},
  {OUTPUT_GPIO,GPIOC,9},
  {OUTPUT_GPIO,GPIOA,8},
  {OUTPUT_GPIO,GPIOI,1},
  {OUTPUT_GPIO,GPIOI,0},
  {OUTPUT_GPIO,GPIOH,15},
  {OUTPUT_GPIO,GPIOH,14},
  {OUTPUT_GPIO,GPIOH,13},
  {OUTPUT_GPIO,GPIOH,12},
  {OUTPUT_GPIO,GPIOG,7},
  {OUTPUT_GPIO,GPIOG,6},
};

dio_def iso_io_in[] = {
  {INPUT_GPIO,GPIOI,4},
  {INPUT_GPIO,GPIOB,9},
  {INPUT_GPIO,GPIOB,8},
  {INPUT_GPIO,GPIOB,4},
  {INPUT_GPIO,GPIOG,14},
  {INPUT_GPIO,GPIOG,11},
  {INPUT_GPIO,GPIOD,2},
  {INPUT_GPIO,GPIOC,12},
};

dio_def *ttl_io = NULL;

//dio_def ttl_io[] = {
//  {INPUT_GPIO,GPIOB,1},
//  {INPUT_GPIO,GPIOB,0},
//  {INPUT_GPIO,GPIOC,5},
//  {INPUT_GPIO,GPIOC,4},
//  {INPUT_GPIO,GPIOA,7},
//  {INPUT_GPIO,GPIOA,6},
//  {INPUT_GPIO,GPIOA,5},
//  {INPUT_GPIO,GPIOA,4},
//  {INPUT_GPIO,GPIOA,3},
//  {INPUT_GPIO,GPIOH,3},
//  {INPUT_GPIO,GPIOH,2},
//  {INPUT_GPIO,GPIOA,2},
//  {INPUT_GPIO,GPIOA,1},
//  {INPUT_GPIO,GPIOA,0},
//  {INPUT_GPIO,GPIOC,3},
//  {INPUT_GPIO,GPIOC,2},
//};

struct _iio_def{
  dio_def *pad;
  uint8_t nofPads;
};

struct digital_io{
  struct _iio_def ttl_io;
  struct _iio_def iso_di;
  struct _iio_def iso_do;
};

static struct digital_io digital_io = {
  {NULL,0},
  {iso_io_in,8},
  {iso_io_out,16}
};


void digital_set_ttl_output(uint32_t value)
{
  if(digital_io.ttl_io.pad == NULL) return;
  
  dio_def *dio = digital_io.ttl_io.pad;
  
  for(uint8_t i=0;i<digital_io.ttl_io.nofPads;i++){
    uint8_t set = (value >> i) & 0x01;
    dio->value = set;
    if(dio->type == OUTPUT_GPIO){
      if(set)
        palSetPad(dio->port,dio->pad);
      else
        palClearPad(dio->port,dio->pad);
    }
    dio++;
  }
}

void digital_set_ttl_bit(uint8_t bit, uint8_t value)
{
  if(digital_io.ttl_io.pad == NULL) return;
  
  dio_def *dio = digital_io.ttl_io.pad;
  
  dio[bit].value = value;
  if(dio[bit].type == OUTPUT_GPIO){
    if(value)
      palSetPad(dio[bit].port,dio[bit].pad);
    else
      palClearPad(dio[bit].port,dio[bit].pad);
  }
}


void digital_set_ttl_port(uint8_t port, uint8_t value)
{
  if(digital_io.ttl_io.pad == NULL) return;
  
  dio_def *dio = digital_io.ttl_io.pad;
  
  for(uint8_t bit=0;bit<8;bit++){
    dio[bit+port*8].value = value;
    if(dio[bit+port*8].type == OUTPUT_GPIO){
      if(value & (1 << bit))
        palSetPad(dio[bit+port*8].port,dio[bit+port*8].pad);
      else
        palClearPad(dio[bit+port*8].port,dio[bit+port*8].pad);
    }
  }
}

void digital_set_ttl_port_dir(uint8_t port, uint8_t dir)
{
  if(digital_io.ttl_io.pad == NULL) return;
  
  dio_def *dio = ttl_io;
  uint8_t base = port *8;
  //ttl_io[0].type = 4;
  for(uint8_t bit=0;bit<8;bit++){
    switch(dir){
    case 0: // Input;
      dio[bit+port*8].type = INPUT_GPIO;
      palSetPadMode(dio[bit+port*8].port,dio[bit+port*8].pad,PAL_MODE_INPUT);
      break;
    case 1: // input pullup
      dio[bit+port*8].type = INPUT_GPIO;
      palSetPadMode(dio[bit+port*8].port,dio[bit+port*8].pad,PAL_MODE_INPUT_PULLUP);
      break;
    case 0x10: // output push pull
      dio[bit+base].type = OUTPUT_GPIO;
      palSetPadMode(dio[bit+port*8].port,dio[bit+port*8].pad,PAL_MODE_OUTPUT_PUSHPULL);
      break;
    case 0x11: // output open drain
      dio[bit+port*8].type = OUTPUT_GPIO;
      palSetPadMode(dio[bit+port*8].port,dio[bit+port*8].pad,PAL_MODE_OUTPUT_OPENDRAIN);
      break;
    default:break;
    }
  }
}

uint32_t digital_get_input()
{
  if(digital_io.ttl_io.pad == NULL) return 0x0;
  uint32_t ret = 0x0;
  dio_def *dio = digital_io.ttl_io.pad;
  for(uint8_t i=0;i<digital_io.ttl_io.nofPads;i++){
    if(palReadPad(dio->port,dio->pad) == PAL_HIGH){
      ret |= (1 << i);
      dio->value = 0;
    }
    else{
      dio->value = 0;
    }
    dio++;
  }
  return ret;
}

void digital_set_iso_out(uint8_t channel, uint8_t set)
{
  if(channel < digital_io.iso_do.nofPads){
    dio_def *dio = digital_io.iso_do.pad;
    if(set){
      palSetPad(dio[channel].port,dio[channel].pad);
      dio[channel].value = 1;
    }
    else{
      palClearPad(dio[channel].port,dio[channel].pad);
      dio[channel].value = 0;
    }
  }
}
/* set polarity */
void digital_set_iso_outp(uint8_t channel, uint8_t pol)
{
  if(channel < digital_io.iso_do.nofPads){
    dio_def *dio = digital_io.iso_do.pad;
    dio[channel].polarity = pol;
  }
}

void digital_set_iso_outpw(uint8_t port, uint16_t pol)
{
  dio_def *dio = digital_io.iso_do.pad;
  for(uint8_t channel = 0; channel < digital_io.iso_do.nofPads; channel++){
    uint8_t bit = pol & (1 << channel);
    dio[channel].polarity = bit;
  }
}

void digital_set_iso_inp(uint8_t channel, uint8_t pol)
{
  if(channel < digital_io.iso_di.nofPads){
    dio_def *dio = digital_io.iso_di.pad;
    dio[channel].polarity = pol;
  }
}

void digital_set_iso_inpw(uint8_t port, uint32_t pol)
{
  dio_def *dio = digital_io.iso_di.pad;
  for(uint8_t channel = 0; channel < digital_io.iso_di.nofPads; channel++){
    uint8_t bit = pol & ( 1 << channel);
    dio[channel].polarity = bit;
  }
}

void digital_set_iso_outw(uint8_t port, uint16_t value)
{
  dio_def *dio = digital_io.iso_do.pad;
  for(uint8_t i=0;i<16;i++){
    if((1 << i) & value){
      if(dio[i].polarity == POL_NORMAL){
        palSetPad(dio[i].port,dio[i].pad);
        dio[i].value = 1;
      }
      else{
        palClearPad(dio[i].port,dio[i].pad);
        dio[i].value = 0;
      }
    }
    else{
      if(dio[i].polarity == POL_NORMAL){
        palClearPad(dio[i].port,dio[i].pad);
        dio[i].value = 0;
      }
      else{
        palSetPad(dio[i].port,dio[i].pad);
        dio[i].value = 1;
      }
    }
  }
}

int8_t digital_get_iso_out(uint8_t channel)
{
  int8_t ret = -1;
  if(channel < digital_io.iso_do.nofPads){
    if(palReadPad(digital_io.iso_do.pad[channel].port, digital_io.iso_do.pad[channel].pad) == PAL_HIGH){
      ret = (digital_io.iso_do.pad[channel].polarity == POL_NORMAL)?1:0;
    }
    else{
      ret = (digital_io.iso_do.pad[channel].polarity == POL_NORMAL)?0:1;
    }
  }
  return ret;
}

int8_t digital_get_iso_in(uint8_t channel)
{
  int8_t ret = -1;
  if(channel < digital_io.iso_di.nofPads){
    if(palReadPad(digital_io.iso_di.pad[channel].port, digital_io.iso_di.pad[channel].pad) == PAL_LOW){
      ret = (digital_io.iso_di.pad[channel].polarity == POL_NORMAL)?1:0;
    }
    else{
      ret = (digital_io.iso_di.pad[channel].polarity == POL_NORMAL)?0:1;
    }
  }
  return ret;
}
int8_t digital_get_iso_outw(uint8_t port, uint16_t *val)
{
  int8_t ret = 0;
  uint16_t value = 0x0;
  for(uint8_t i=0;i<digital_io.iso_do.nofPads;i++){
    value |= (digital_get_iso_out(i) << i);
  }
  *val = value;
  return ret;
}
int8_t digital_get_iso_inw(uint8_t port, uint16_t *val)
{
  int8_t ret = 0;
  uint16_t value = 0x0;
  for(uint8_t i=0;i<digital_io.iso_di.nofPads;i++){
    value |= (digital_get_iso_in(i) << i);
//    if(palReadPad(digital_io.iso_di.pad[i].port, digital_io.iso_di.pad[i].pad) == PAL_LOW){
//      if(digital_io.iso_do.pad[i].polarity == POL_NORMAL){
//        value |= (1 << i);
//      }
//      else{
//        value &= ~(1 << i);
//      }        
//    }
//    else{
//      if(digital_io.iso_do.pad[i].polarity == POL_NORMAL){
//        value &= ~(1 << i);
//      }
//      else{
//        value |= (1 << i);
//      }        
//    }
  }
  *val = value;
  return ret;
}

static void io_init()
{
    if(digital_io.ttl_io.pad != NULL){
    dio_def *dio = digital_io.ttl_io.pad;
    for(uint8_t i=0;i<digital_io.ttl_io.nofPads;i++){
      if(dio != NULL){
        if(dio->type == INPUT_GPIO){
          palSetPadMode(dio->port, dio->pad, PAL_MODE_INPUT_PULLUP);
        }
        else if(dio->type == OUTPUT_GPIO){
          palSetPadMode(dio->port,dio->pad,PAL_MODE_OUTPUT_PUSHPULL);
        }
      }
      dio++;
    }
  }
  
  if(digital_io.iso_di.pad != NULL){
    dio_def *dio = digital_io.iso_di.pad;
    for(uint8_t i=-0;i<digital_io.iso_di.nofPads;i++){
      if(dio != NULL){
        palSetPadMode(dio->port, dio->pad, PAL_MODE_INPUT_PULLUP);
      }
      dio++;
    }
  }
  
  if(digital_io.iso_do.pad != NULL){
    dio_def *dio = digital_io.iso_do.pad;
    for(uint8_t i=0;i<digital_io.iso_do.nofPads;i++){
      if(dio != NULL){
        palSetPadMode(dio->port,dio->pad, PAL_MODE_OUTPUT_PUSHPULL);
        // set initial state to low
        palClearPad(dio->port, dio->pad);
      }
      dio++;
    }
  }
}

static THD_WORKING_AREA(waDIO,512);
static THD_FUNCTION(procDIO,p)
{
  io_init();
  digital_get_iso_inw(0,&runTime.last_iso_in);
  chThdResume(&runTime.ref,MSG_OK);
  
  bool bStop = false;
  uint16_t isoIn = 0x0;
  digital_set_iso_outw(0,0x00);
  while(!bStop){
    digital_get_iso_inw(0,&isoIn);
    if(isoIn != runTime.last_iso_in){
      if(runTime.main != NULL){
        chEvtSignal(runTime.main,EV_DIN_COS);
      }
      runTime.last_iso_in = isoIn;
    }
    
    if(chThdShouldTerminateX()){
      bStop = true;
    }
    chThdSleepMilliseconds(50);
  }
  
  // cleanup
  
  chThdExit(MSG_OK);
  
}


void digital_init()
{
  pRunTime = &runTime;  
  runTime.main = chRegFindThreadByName("MAIN");
  runTime.self = chThdCreateStatic(waDIO,sizeof(waDIO),NORMALPRIO,procDIO,NULL);
  chSysLock();
  chThdSuspendS(&runTime.ref);
  chSysUnlock();

  
}

