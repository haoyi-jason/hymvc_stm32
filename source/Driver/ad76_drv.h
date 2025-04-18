#ifndef _AD76_DRV_
#define _AD76_DRV_

#include "hal.h"

#define AD76_NOF_CHANNEL        8

typedef struct AD7606Driver AD7606Driver;

enum ADC_ID{
  AD7606,
  AD7608
};

typedef struct{
  SPIDriver *devp;
  const SPIConfig *config;
  ioportid_t    ssport;
  ioportmask_t  sspad;
  ioportid_t    port_convsta;
  ioportmask_t  pad_convsta;
  ioportid_t    port_convstb;
  ioportmask_t  pad_convstb;
  ioportid_t    port_busy;
  ioportmask_t  pad_busy;
  ioportid_t    port_rst;
  ioportmask_t  pad_rst;
  ioportid_t    port_fstdata;
  ioportmask_t  pad_fstdata;
  ioportid_t    port_os0;
  ioportmask_t  pad_os0;
  ioportid_t    port_os1;
  ioportmask_t  pad_os1;
  ioportid_t    port_os2;
  ioportmask_t  pad_os2;
  ioportid_t    port_stby;
  ioportmask_t  pad_stby;
}AD7606Config;

#define _ad7606_data \
  uint8_t chipID; \
  event_source_t ev_drdy; \
  int32_t data[AD76_NOF_CHANNEL];
  
struct AD7606Driver{
  const AD7606Config *config;
  _ad7606_data
};

void ad7606_reset(AD7606Driver *dev);
msg_t ad7606_init(AD7606Driver *dev,const AD7606Config *config);
#endif