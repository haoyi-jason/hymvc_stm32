#ifndef _AD2S_
#define _AD2S_

/* Global Definitions */
#define REG_POSITION            0x80
#define REG_VELOCITY            0x82
#define REG_LOS_THRES           0x88
#define REG_DOS_OR              0x89
#define REG_DOS_MIS             0x8A
#define REG_DOS_RST_MAX         0x8B
#define REG_DOS_RST_MIN         0x8C
#define REG_LOT_HIGH            0x8D
#define REG_LOT_LOW             0x8E
#define REG_EXCI_FREQ           0x91
#define REG_CONTROL             0x92
#define REG_SOFT_RESET          0xF0
#define REG_FAULT               0xFF

enum opMode{
  NM_POSITION,
  NM_VELOCITY,
  NM_RSV,
  NM_CONFIG
};

enum _resolution{
  RES_10b,
  RES_12b,
  RES_14b,
  RES_16b
};




typedef struct AD2S1210Driver AD2S1210Driver;

typedef struct{
  SPIDriver *devp;
  const SPIConfig *config;
  uint8_t reverse;
  uint8_t resolution;
  ioportid_t ssport;
  ioportmask_t ssline;
  ioportid_t soeport;
  ioportmask_t soeline;
  ioportid_t fsyncport;
  ioportmask_t fsyncline;
//  ioportid_t wrport;
//  ioportmask_t wrline;
  ioportid_t a0port;
  ioportmask_t a0line;
  ioportid_t a1port;
  ioportmask_t a1line;
  ioportid_t port_res0;
  ioportmask_t line_res0;
  ioportid_t port_res1;
  ioportmask_t line_res1;
  ioportid_t port_rst;
  ioportmask_t line_rst;
  ioportid_t port_sample;
  ioportmask_t line_sample;
  ioportid_t port_rd;
  ioportmask_t line_rd;
  ioportid_t port_wr;
  ioportmask_t line_wr;
}AD2S1210Config;

#define _ad2s_data \
  uint16_t position; \
  int16_t velocity; \
  uint8_t losThres; \
  uint8_t dosOver; \
  uint8_t dosMismatch; \
  uint8_t dosResetMax; \
  uint8_t dosResetMin; \
  uint16_t lot; \
  uint8_t exci_freq; \
  uint8_t ctrl; \
  uint8_t fault; \
  float currentAngle; \
  float currentSpeed;


struct AD2S1210Driver{
  AD2S1210Config *config;
  _ad2s_data
};

static void sample_trigger(AD2S1210Driver *dev);
msg_t ad2S_init(AD2S1210Driver *dev, AD2S1210Config *config);
msg_t ad2S_start(AD2S1210Driver *dev);
msg_t ad2s_SetResolution(AD2S1210Driver *dev, uint8_t resolution);
msg_t ad2s_SetMode(AD2S1210Driver *dev, uint8_t mode);
void ad2s_reset(AD2S1210Driver *dev);
msg_t ad2S_Refresh(AD2S1210Driver *dev);
#endif