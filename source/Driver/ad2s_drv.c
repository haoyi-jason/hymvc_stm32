
#include "hal.h"
#include "ch.h"
#include "ad2s_drv.h"

#define DBG_CHECK(x)    (x == NULL)

static const float POS_LSB[] = {21.1,5.3,1.3,21600./65536.}; // arc min
static const float VEL_LSB[] = {4.88,0.488,0.06,0.004}; // rps

static void fsync_trigger(AD2S1210Driver *dev);
static void delay(uint32_t n);

static void chipSel(AD2S1210Driver *dev, uint8_t set)
{
  if(DBG_CHECK(dev->config->ssport)) return;
  if(DBG_CHECK(dev->config->ssline)) return;
  
  if(set){
    palClearPad(dev->config->ssport, dev->config->ssline);
    //palClearPad(dev->config->port_wr, dev->config->line_wr);
  }
  else
  {
    palSetPad(dev->config->ssport, dev->config->ssline);
    //palSetPad(dev->config->port_wr, dev->config->line_wr);
  }
}

static void registerRead(AD2S1210Driver *dev, uint8_t reg_adr, uint8_t *b, uint16_t n)
{
  if(DBG_CHECK(dev->config->devp)) return;
  
  uint8_t reg ;//= reg_adr | AD57_REG_READ;
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);
  chipSel(dev,1);
  spiSend(dev->config->devp,1,&reg);
  spiReceive(dev->config->devp,n,b);
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);
  
}

static void registerWrite(AD2S1210Driver *dev, uint8_t reg_adr, uint8_t *b, uint16_t n)
{
  if(DBG_CHECK(dev->config->ssport)) return;
  if(DBG_CHECK(dev->config->ssline)) return;
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

static void configWrite(AD2S1210Driver *dev, uint8_t reg_adr, uint8_t b)
{
  ad2s_SetMode(dev,NM_CONFIG);

  chipSel(dev,1);
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);
  spiSend(dev->config->devp,1,&reg_adr);
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);

  //delay(20);
  chipSel(dev,1);
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);
  spiSend(dev->config->devp,1,&b);
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);
  
}

static void configRead(AD2S1210Driver *dev, uint8_t reg_adr, uint8_t *b)
{
  uint8_t tx = 0xFF;
  ad2s_SetMode(dev,NM_CONFIG);

  chipSel(dev,1);
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);

  spiSend(dev->config->devp,1,&reg_adr);
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);
  //delay(10);
  
  chipSel(dev,1);
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);
  spiExchange(dev->config->devp,1,&tx,b);

  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);
  
}

static void normalRead(AD2S1210Driver *dev)
{ 
  uint8_t rx[5];
  sample_trigger(dev);
  ad2s_SetMode(dev,NM_POSITION);
  chipSel(dev,1);
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);
  // read position
  spiReceive(dev->config->devp,3,rx);
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);


  ad2s_SetMode(dev,NM_VELOCITY);

  chipSel(dev,1);
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);
  // read velocity & fault register
  spiReceive(dev->config->devp,3,&rx[2]);
  
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);

  uint16_t v = (uint16_t)((rx[2]<<8) | rx[3]);
  dev->position = (rx[0]<<8) | rx[1];
  dev->velocity = (int16_t)v;
  dev->fault = rx[4];
                      
}

static void delay(uint32_t n)
{
  uint32_t cntr = n;
  while(--n){
    __NOP();
  }
}

static void fsync_trigger(AD2S1210Driver *dev)
{
  palClearPad(dev->config->fsyncport, dev->config->fsyncline);
  delay(10);
  palSetPad(dev->config->fsyncport, dev->config->fsyncline);
}

static void sample_trigger(AD2S1210Driver *dev)
{
  palClearPad(dev->config->port_sample, dev->config->line_sample);
  //delay(10);
  palSetPad(dev->config->port_sample, dev->config->line_sample);
}

void ad2s_read_registers(AD2S1210Driver *dev)
{
  
  ad2s_SetMode(dev,NM_CONFIG);
  uint8_t *ptr;
  ptr = (uint8_t*)&dev->position;
  configRead(dev,0x80,ptr);
  ptr++;
  configRead(dev,0x81,ptr);
  
  ptr = (uint8_t*)&dev->velocity;
  configRead(dev,0x82,ptr);
  ptr++;
  configRead(dev,0x83,ptr);
  
  configRead(dev,0x88,&dev->losThres);
  configRead(dev,0x89,&dev->dosOver);
  configRead(dev,0x8a,&dev->dosMismatch);
  configRead(dev,0x8b,&dev->dosResetMax);
  configRead(dev,0x8c,&dev->dosResetMin);
  ptr = (uint8_t*)&dev->lot;
  configRead(dev,0x8d,ptr);
  ptr++;
  configRead(dev,0x8e,ptr);

  configRead(dev,0x91,&dev->exci_freq);
  configWrite(dev,0x92,0x7f);
  configRead(dev,0x92,&dev->ctrl);
  configRead(dev,0xff,&dev->fault);
  
}

msg_t ad2S_init(AD2S1210Driver *dev, AD2S1210Config *config)
{
  if(dev == NULL) return MSG_RESET;
  if(config == NULL) return MSG_RESET;
  
  dev->config = config;
  
  return MSG_OK;  
}


msg_t ad2S_start(AD2S1210Driver *dev)
{
  if(dev == NULL) return MSG_RESET;
  
  if(!DBG_CHECK(dev->config->ssport)){
    palSetPadMode(dev->config->ssport,dev->config->ssline, PAL_MODE_OUTPUT_PUSHPULL);
    palClearPad(dev->config->ssport, dev->config->ssline);
  }
  
  if(!DBG_CHECK(dev->config->soeport)){
    palSetPadMode(dev->config->soeport,dev->config->soeline, PAL_MODE_OUTPUT_PUSHPULL);
    // set to serial mode
    palClearPad(dev->config->soeport, dev->config->soeline);
  }

  if(!DBG_CHECK(dev->config->fsyncport)){
    palSetPadMode(dev->config->fsyncport,dev->config->fsyncline, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->fsyncport, dev->config->fsyncline);
  }
  
  if(!DBG_CHECK(dev->config->a0port)){
    palSetPadMode(dev->config->a0port,dev->config->a0line, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->a0port, dev->config->a0line);
  }

  if(!DBG_CHECK(dev->config->a1port)){
    palSetPadMode(dev->config->a1port,dev->config->a1line, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->a1port, dev->config->a1line);
  }

  if(!DBG_CHECK(dev->config->port_res0)){
    palSetPadMode(dev->config->port_res0,dev->config->line_res0, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->port_res0, dev->config->line_res0);
  }
  if(!DBG_CHECK(dev->config->port_res1)){
    palSetPadMode(dev->config->port_res1,dev->config->line_res1, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->port_res1, dev->config->line_res1);
  }
  if(!DBG_CHECK(dev->config->port_rst)){
    palSetPadMode(dev->config->port_rst,dev->config->line_rst, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->port_rst, dev->config->line_rst);
  }
  if(!DBG_CHECK(dev->config->port_sample)){
    palSetPadMode(dev->config->port_sample,dev->config->line_sample, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->port_sample, dev->config->line_sample);
  }
  
  if(!DBG_CHECK(dev->config->port_rd)){
    palSetPadMode(dev->config->port_rd,dev->config->line_rd, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->port_rd, dev->config->line_rd);
  }
  if(!DBG_CHECK(dev->config->port_wr)){
    palSetPadMode(dev->config->port_wr,dev->config->line_wr, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->port_wr, dev->config->line_wr);
  }
}

msg_t ad2s_SetResolution(AD2S1210Driver *dev, uint8_t resolution)
{
  if(dev == NULL) return MSG_RESET;
  
  switch(resolution){
  case RES_10b:
    palClearPad(dev->config->port_res0, dev->config->line_res0);
    palClearPad(dev->config->port_res1, dev->config->line_res1);
    break;
  case RES_12b:
    palClearPad(dev->config->port_res0, dev->config->line_res0);
    palSetPad(dev->config->port_res1, dev->config->line_res1);
    break;
  case RES_14b:
    palSetPad(dev->config->port_res0, dev->config->line_res0);
    palClearPad(dev->config->port_res1, dev->config->line_res1);
    break;
  case RES_16b:
    palSetPad(dev->config->port_res0, dev->config->line_res0);
    palSetPad(dev->config->port_res1, dev->config->line_res1);
    break;
  default:
    break;
  }
  return MSG_OK;
}

msg_t ad2s_SetMode(AD2S1210Driver *dev, uint8_t mode)
{
  switch(mode){
  case NM_POSITION:
    palClearPad(dev->config->a0port, dev->config->a0line);
    palClearPad(dev->config->a1port, dev->config->a1line);
    break;
  case NM_VELOCITY:
    palClearPad(dev->config->a0port, dev->config->a0line);
    palSetPad(dev->config->a1port, dev->config->a1line);
    break;
  case NM_CONFIG:
    palSetPad(dev->config->a0port, dev->config->a0line);
    palSetPad(dev->config->a1port, dev->config->a1line);
    break;
  default:
    palSetPad(dev->config->a0port, dev->config->a0line);
    palSetPad(dev->config->a1port, dev->config->a1line);
    break;
  }
  return MSG_OK;
}
msg_t ad2s_direct_position(AD2S1210Driver *dev)
{
  ad2s_SetMode(dev,NM_POSITION);
  
  return MSG_OK;
  
}

msg_t ad2s_direct_velocity(AD2S1210Driver *dev)
{
  
  return MSG_OK;
  
}

void ad2s_reset(AD2S1210Driver *dev)
{
  palClearPad(dev->config->port_rst, dev->config->line_rst);
  delay(10);
  palSetPad(dev->config->port_rst, dev->config->line_rst);
}

msg_t ad2S_Refresh(AD2S1210Driver *dev)
{
  normalRead(dev);
  dev->currentAngle = dev->position * POS_LSB[dev->config->resolution];
  dev->currentSpeed = dev->velocity * VEL_LSB[dev->config->resolution];
  return MSG_OK;
}
