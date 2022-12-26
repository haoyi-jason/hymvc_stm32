#include "ch.h"
#include "hal.h"
#include "ad76_drv.h"
#include "string.h"

#define DBG_CHECK(x)    (x == NULL)

static void chipSel(AD7606Driver *dev, uint8_t set)
{
  if(DBG_CHECK(dev->config->ssport)) return;
  if(DBG_CHECK(dev->config->sspad)) return;
  
  if(set)
    palClearPad(dev->config->ssport, dev->config->sspad);
  else
    palSetPad(dev->config->ssport, dev->config->sspad);
}

static void registerRead(AD7606Driver *dev, uint8_t reg_adr, uint8_t *b, uint16_t n)
{
  if(DBG_CHECK(dev->config->devp)) return;
  
  uint8_t reg = reg_adr ;//| AD57_REG_READ;
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);
  chipSel(dev,1);
  spiSend(dev->config->devp,1,&reg);
  spiReceive(dev->config->devp,n,b);
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);
  
}

static void registerWrite(AD7606Driver *dev, uint8_t reg_adr, uint8_t *b, uint16_t n)
{
  if(DBG_CHECK(dev->config->ssport)) return;
  if(DBG_CHECK(dev->config->sspad)) return;
  if(DBG_CHECK(dev->config->devp)) return;
  
  uint8_t reg = reg_adr ;//| AD57_REG_READ;
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);
  chipSel(dev,1);
  spiSend(dev->config->devp,1,&reg);
  spiSend(dev->config->devp,n,b);
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);
  
}

static void delay(uint32_t n)
{
  uint32_t cntr = n;
  while(--n){
    __NOP();
  }
}


static void drdy_handler(void *arg);
static void ad7606_enable_interrupt(AD7606Driver *dev)
{
  palSetLineCallback(PAL_LINE(dev->config->port_busy, dev->config->pad_busy), drdy_handler,dev);
  palEnableLineEvent(PAL_LINE(dev->config->port_busy, dev->config->pad_busy),PAL_EVENT_MODE_FALLING_EDGE);
}

static void ad7606_disable_interrupt(AD7606Driver *dev)
{
  palDisableLineEvent(PAL_LINE(dev->config->port_busy, dev->config->pad_busy));
}


static void drdy_handler(void *arg)
{
  AD7606Driver *dev = (AD7606Driver*)arg;
  chSysLockFromISR();
  //palClearPad(dev->config->port_stby,dev->config->pad_stby);
  ad7606_disable_interrupt(dev);
  chEvtBroadcastFlagsI(&dev->ev_drdy,EVENT_MASK(0));
  chSysUnlockFromISR();
}


msg_t ad7606_init(AD7606Driver *dev, AD7606Config *config)
{
  if(dev == NULL) return MSG_RESET;
  
  dev->config = config;
  dev->chipID = AD7606;
  
  return MSG_OK;
}

msg_t ad7606_start(AD7606Driver *dev)
{
  if(dev == NULL) return MSG_RESET;
  
  // set pad i/o
  if(!DBG_CHECK(dev->config->ssport)){
    palSetPadMode(dev->config->ssport, dev->config->sspad,PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->ssport, dev->config->sspad);
  }

  if(!DBG_CHECK(dev->config->port_convsta)){
    palSetPadMode(dev->config->port_convsta, dev->config->pad_convsta,PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->port_convsta,dev->config->pad_convsta);
  }
  
  if(!DBG_CHECK(dev->config->port_convstb)){
    palSetPadMode(dev->config->port_convstb, dev->config->pad_convstb,PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->port_convstb,dev->config->pad_convstb);
  }

  if(!DBG_CHECK(dev->config->port_busy)){
    palSetPadMode(dev->config->port_busy, dev->config->pad_busy,PAL_MODE_INPUT);
  }

  if(!DBG_CHECK(dev->config->port_rst)){
    palSetPadMode(dev->config->port_rst, dev->config->pad_rst,PAL_MODE_OUTPUT_PUSHPULL);
    palClearPad(dev->config->port_rst,dev->config->pad_rst);
  }
  
  if(!DBG_CHECK(dev->config->port_fstdata)){
    palSetPadMode(dev->config->port_fstdata, dev->config->pad_fstdata,PAL_MODE_INPUT);
  }

  /*
  Over Sample Control OS[2..0],
  ODR = 200KSPS / 2^ODR, valid from 0 to 6, 7 is invalid

  */
  if(!DBG_CHECK(dev->config->port_os0)){
    palSetPadMode(dev->config->port_os0, dev->config->pad_os0,PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->port_os0, dev->config->pad_os0);
  }
  if(!DBG_CHECK(dev->config->port_os1)){
    palSetPadMode(dev->config->port_os1, dev->config->pad_os1,PAL_MODE_OUTPUT_PUSHPULL);
    palClearPad(dev->config->port_os1,dev->config->pad_os1);
  }
  if(!DBG_CHECK(dev->config->port_os2)){
    palSetPadMode(dev->config->port_os2, dev->config->pad_os2,PAL_MODE_OUTPUT_PUSHPULL);
    palClearPad(dev->config->port_os2,dev->config->pad_os2);
  }
  if(!DBG_CHECK(dev->config->port_stby)){
    palSetPadMode(dev->config->port_stby, dev->config->pad_stby,PAL_MODE_OUTPUT_PUSHPULL);
    palClearPad(dev->config->port_stby,dev->config->pad_stby);
//    palSetPad(dev->config->port_stby,dev->config->pad_stby);
  }
  
  // register busy line for falling edge trigger
  chEvtObjectInit(&dev->ev_drdy);
  
  // reset chip
  ad7606_reset(dev);
  return MSG_OK;
}

void ad7606_reset(AD7606Driver *dev)
{
  palSetPad(dev->config->port_rst,dev->config->pad_rst);
  delay(500);
  palClearPad(dev->config->port_rst,dev->config->pad_rst);
}

void ad7606_start_conv(AD7606Driver *dev, uint8_t option)
{
  //palSetPad(dev->config->port_stby,dev->config->pad_stby);
  ad7606_enable_interrupt(dev);
  switch(option){
  case 1: // a only
    palClearPad(dev->config->port_convsta,dev->config->pad_convsta);
    break;
  case 2: // b only
    palClearPad(dev->config->port_convstb,dev->config->pad_convstb);
    break;
  case 3: // both a & b
    palClearPad(dev->config->port_convsta,dev->config->pad_convsta);
    palClearPad(dev->config->port_convstb,dev->config->pad_convstb);
    break;
  default:break;
  }
  // short delay
  delay(50);
  palSetPad(dev->config->port_convsta,dev->config->pad_convsta);
  palSetPad(dev->config->port_convstb,dev->config->pad_convstb);
  
}

void byteMap(int32_t *dst, uint8_t *arr, uint8_t bitStart)
{
  uint8_t start, shift;
  start = (bitStart)/8;
  shift = (bitStart)%8;  // bits to shift right
  
  uint8_t *ptr = (uint8_t*)dst;
  uint8_t *src = arr;
  
  src += start;
  //memcpy(ptr,src,3);
  *(ptr+3) = *src;
  *(ptr+2) = *(src+1);
  *(ptr+1) = *(src + 2);
  *dst <<= shift;
}

void ad7606_read_conversion(AD7606Driver *dev, uint8_t nofGroup)
{
  uint8_t b[18];
  uint8_t nofTransfer = 0;
  uint8_t bitsPerRecord ;
  uint8_t nofCh = 0;
  if(dev->chipID == AD7606){
    nofCh = 8;
    nofTransfer = nofGroup * 8;
    bitsPerRecord = 16;
  }
  else if(dev->chipID == AD7608){
    nofCh = 8;
    nofTransfer = nofGroup * 9;
    bitsPerRecord = 18;
  }
  
  if(nofTransfer == 0) return;
  
  // todo : read data
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);
  chipSel(dev,1);
  spiReceive(dev->config->devp,nofTransfer,b);
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);
  
  // todo : arrange data
  //uint8_t ch = 0;
  //int8_t bitToShift = bits;
  
//  uint8_t start, shift;
//  uint8_t bits = nofTransfer*8;
  for(uint8_t i=0;i<nofCh;i++){
    byteMap(&dev->data[i],b,bitsPerRecord*i);
    switch(dev->chipID){
    case AD7606:
      dev->data[i] >>= 16;
      break;
    case AD7608:
      dev->data[i] >>= 14;
      break;
    }
  }
}