#ifndef _AD57_DRV_
#define _AD57_DRV_

/* GLOBAL DEFINITIONS */
#define AD57_CHANNELS           4
#define AD57_REG_RW_BIT         7
#define AD57_REG_READ           0x80
#define AD57_REG(x)             (x << 3)
#define AD57_DAC(x)             (x << 0)

enum AD57_REG{
  REG_FUNCTION,
  RSV1,
  REG_DATA,
  REG_CORSE_GAIN,
  REG_FINE_GAIN,
  REG_OFFSET
};

enum AD57_DACX{
  DAC_A,
  DAC_B,
  DAC_C,
  DAC_D,
  DAC_ALL
};

enum AD57_FUNCX{
  FUNC_NOP,
  FUNC_GPIO,
  FUNC_CLEAR_DATA=4,
  FUNC_LOAD_DATA
};

enum AD57_CORSE_GAIN{
  CG_B10V,
  CG_B1025V,
  CG_B1052_V
};

typedef struct AD57x4Driver AD57x4Driver;

typedef struct{
  SPIDriver *devp;
  const SPIConfig *config;
  ioportid_t syncport;
  ioportmask_t syncline;
  ioportid_t latchport;
  ioportmask_t latchline;
  ioportid_t rstport;
  ioportmask_t rstline;
  ioportid_t binport;
  ioportmask_t binline;
}AD57Config;

#define _ad57_data \
        uint8_t chip_id; \
        int16_t data[AD57_CHANNELS]; \
        int16_t corse_gain[AD57_CHANNELS]; \
        int16_t fine_gain[AD57_CHANNELS]; \
        int16_t offset[AD57_CHANNELS]; \
        int16_t chipTemp; \
        uint8_t syncUpdate;
        
struct AD57x4Driver{
  AD57Config *config;
  _ad57_data
};

msg_t ad57_init(AD57x4Driver *dev, AD57Config *config);
msg_t ad57_start(AD57x4Driver *dev);
int8_t ad57_stop(AD57x4Driver *dev);
int8_t ad57_set_simulataneous_update(AD57x4Driver *dev, uint8_t flag);
int8_t ad57_set_trigger_update(AD57x4Driver *dev);
int8_t ad57_set_dac(AD57x4Driver *dev, uint8_t ch);
int8_t ad57_get_corse_gain(AD57x4Driver *dev, uint8_t ch);
int8_t ad57_set_corse_gain(AD57x4Driver *dev, uint8_t ch);
int8_t ad57_get_fine_gain(AD57x4Driver *dev);
int8_t ad57_set_fine_gain(AD57x4Driver *dev, uint8_t ch);
int8_t ad57_get_offset(AD57x4Driver *dev);
int8_t ad57_set_offset(AD57x4Driver *dev, uint8_t ch);




#endif