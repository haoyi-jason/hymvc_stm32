#include "hal.h"
#include "ad57_drv.h"

#define DBG_CHECK(x)    (x == NULL)

static void delay(uint32_t n)
{
  uint32_t cntr=n;
  while(--cntr);
}

static void chipSel(AD57x4Driver *dev, uint8_t set)
{
  if(DBG_CHECK(dev->config->syncport)) return;
  if(DBG_CHECK(dev->config->syncline)) return;
  
  if(set)
    palClearPad(dev->config->syncport, dev->config->syncline);
  else
    palSetPad(dev->config->syncport, dev->config->syncline);
}

static void registerRead(AD57x4Driver *dev, uint8_t reg_adr, uint8_t *b, uint16_t n)
{
  if(DBG_CHECK(dev->config->devp)) return;
  
  uint8_t reg = reg_adr | AD57_REG_READ;
  uint8_t tx[3],rx[3];
  tx[0] = reg_adr | AD57_REG_READ;
  tx[1] = tx[2] = 0x0;
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);
  chipSel(dev,1);
  spiExchange(dev->config->devp,3,tx,rx);
//  spiSend(dev->config->devp,3,tx);
//  chipSel(dev,0);
//  tx[0] = reg_adr;
//  delay(20);
//  chipSel(dev,1);
//  spiReceive(dev->config->devp,3,rx);
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);
  b[0] = rx[2];
  b[1] = rx[1];
//  memcpy(b,&rx[1],2);
}

static void registerWrite(AD57x4Driver *dev, uint8_t reg_adr, uint8_t *b, uint16_t n)
{
  if(DBG_CHECK(dev->config->syncport)) return;
  if(DBG_CHECK(dev->config->syncline)) return;
  if(DBG_CHECK(dev->config->devp)) return;
  
  uint8_t reg = reg_adr;
  uint8_t tx[3];
  tx[0] = reg_adr;
  tx[1] = b[1];
  tx[2] = *b;
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);
  chipSel(dev,1);
//  spiSend(dev->config->devp,1,&reg);
//  spiSend(dev->config->devp,n,b);
  spiSend(dev->config->devp,3,tx);
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);
  
}

msg_t ad57_init(AD57x4Driver *dev, AD57Config *config)
{
  if(dev == NULL) return MSG_RESET;
  if(config == NULL) return MSG_RESET;
  
  dev->config = config;
  
  return MSG_OK;
}

msg_t ad57_start(AD57x4Driver *dev)
{
  if(dev == NULL) return MSG_RESET;
  
  if((!DBG_CHECK(dev->config->syncport))){
    palSetPadMode(dev->config->syncport, dev->config->syncline,PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->syncport, dev->config->syncline);
  }
  
  if(!DBG_CHECK(dev->config->latchport)){
    palSetPadMode(dev->config->latchport, dev->config->latchline, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->latchport, dev->config->latchline);
  }
  
  if(!DBG_CHECK(dev->config->rstport)){
    palSetPadMode(dev->config->rstport, dev->config->rstline, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->rstport, dev->config->rstline);
  }
  
  if(!DBG_CHECK(dev->config->binport)){
    palSetPadMode(dev->config->binport, dev->config->binline, PAL_MODE_OUTPUT_PUSHPULL);
    palClearPad(dev->config->binport, dev->config->binline); // 2'cmp
    //palSetPad(dev->config->binport, dev->config->binline); // offset bin
  }
      
      
}

int8_t ad57_stop(AD57x4Driver *dev)
{
  
  
}


int8_t ad57_set_simulataneous_update(AD57x4Driver *dev, uint8_t flag)
{
  if(flag == 1){
    palClearPad(dev->config->latchport, dev->config->latchline);
  }
  else{
    palSetPad(dev->config->latchport, dev->config->latchline);
  }
  return 0;
}

int8_t ad57_trigger_update(AD57x4Driver *dev)
{
  palClearPad(dev->config->latchport, dev->config->latchline);
  uint16_t n = 100;
  while(--n);
  palSetPad(dev->config->latchport, dev->config->latchline);
  return 0;
}


int8_t ad57_set_dac(AD57x4Driver *dev, uint8_t ch)
{
  uint16_t rb;
  if(ch == 0xff){ // all channel
    for(uint8_t i=0;i<4;i++){
      registerWrite(dev,AD57_REG(REG_DATA) | AD57_DAC(i),(uint8_t*)&dev->data[i],2);
      registerRead(dev, AD57_REG(REG_DATA) | AD57_DAC(i),(uint8_t*)&rb,2);
    }
  }
  else{
      registerWrite(dev,AD57_REG(REG_DATA) | AD57_DAC(ch),(uint8_t*)&dev->data[ch],2);
      registerRead(dev, AD57_REG(REG_DATA) | AD57_DAC(ch),(uint8_t*)&rb,2);
  }
}

int8_t ad57_get_corse_gain(AD57x4Driver *dev, uint8_t ch)
{
  if(ch == 0xff){ // all channel
    for(uint8_t i=0;i<4;i++){
      registerRead(dev,AD57_REG(REG_CORSE_GAIN) | AD57_DAC(i),(uint8_t*)&dev->corse_gain[i],2);
    }
  }
  else{
      registerRead(dev,AD57_REG(REG_CORSE_GAIN) | AD57_DAC(ch),(uint8_t*)&dev->corse_gain[ch],2);
  }
}

int8_t ad57_set_corse_gain(AD57x4Driver *dev, uint8_t ch)
{
  if(ch == 0xff){ // all channel
    for(uint8_t i=0;i<4;i++){
      registerWrite(dev,AD57_REG(REG_CORSE_GAIN) | AD57_DAC(i),(uint8_t*)&dev->corse_gain[i],2);
    }
  }
  else{
      registerWrite(dev,AD57_REG(REG_CORSE_GAIN) | AD57_DAC(ch),(uint8_t*)&dev->corse_gain[ch],2);
  }
}

int8_t ad57_get_fine_gain(AD57x4Driver *dev)
{
    for(uint8_t i=0;i<4;i++){
      registerRead(dev,AD57_REG(REG_FINE_GAIN) | AD57_DAC(i),(uint8_t*)&dev->fine_gain[i],2);
    }
//  if(ch == 0xff){ // all channel
//    for(uint8_t i=0;i<4;i++){
//      registerRead(dev,AD57_REG(REG_FINE_GAIN) | AD57_DAC(i),&dev->fine_gain[i]);
//    }
//  }
//  else{
//      registerRead(dev,AD57_REG(REG_FINE_GAIN) | AD57_DAC(ch),&dev->fine_gain[i]);
//  }
}

int8_t ad57_set_fine_gain(AD57x4Driver *dev, uint8_t ch)
{
    for(uint8_t i=0;i<4;i++){
      registerWrite(dev,AD57_REG(REG_FINE_GAIN) | AD57_DAC(i),(uint8_t*)&dev->fine_gain[i],2);
    }
//  if(ch == 0xff){ // all channel
//    for(uint8_t i=0;i<4;i++){
//      registerWrite(dev,AD57_REG(REG_FINE_GAIN) | AD57_DAC(i),&dev->fine_gain[i]);
//    }
//  }
//  else{
//      registerWrite(dev,AD57_REG(REG_FINE_GAIN) | AD57_DAC(ch),&dev->fine_gain[i]);
//  }
}

int8_t ad57_get_offset(AD57x4Driver *dev)
{
    for(uint8_t i=0;i<4;i++){
      registerRead(dev,AD57_REG(REG_OFFSET) | AD57_DAC(i),(uint8_t*)&dev->offset[i],2);
    }
//  if(ch == 0xff){ // all channel
//    for(uint8_t i=0;i<4;i++){
//      registerRead(dev,AD57_REG(REG_OFFSET) | AD57_DAC(i),&dev->offset[i]);
//    }
//  }
//  else{
//      registerRead(dev,AD57_REG(REG_OFFSET) | AD57_DAC(ch),&dev->offset[i]);
//  }
}

int8_t ad57_set_offset(AD57x4Driver *dev, uint8_t ch)
{
    for(uint8_t i=0;i<4;i++){
      registerWrite(dev,AD57_REG(REG_OFFSET) | AD57_DAC(i),(uint8_t*)&dev->offset[i],2);
    }
//  if(ch == 0xff){ // all channel
//    for(uint8_t i=0;i<4;i++){
//      registerWrite(dev,AD57_REG(REG_OFFSET) | AD57_DAC(i),&dev->offset[i]);
//    }
//  }
//  else{
//      registerWrite(dev,AD57_REG(REG_OFFSET) | AD57_DAC(ch),&dev->offset[i]);
//  }
}
