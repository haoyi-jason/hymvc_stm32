#ifndef _DIGITAL_IO_
#define _DIGITAL_IO_

#include "hal.h"

enum _dio_type{
  INPUT_GPIO,
  INPUT_CAPTURE,
  INPUT_COUNTER,
  INPUT_FREQUENCY,
  OUTPUT_GPIO,
  OUTPUT_PWM,
  OUTPUT_FREQUENCY
};

enum _polarity{
  POL_NORMAL,
  POL_REVERSE
};

typedef struct _dio_def{
  uint8_t type;
  ioportid_t port;
  ioportmask_t pad;
  int32_t value;
  uint8_t polarity;
}dio_def;

typedef struct DigitalDriver DigitalDriver;

//typedef struct{
  
void digital_init();
void digital_set_ttl_output(uint32_t value);
void digital_set_ttl_bit(uint8_t bit, uint8_t value);
void digital_set_ttl_port(uint8_t port, uint8_t value);
void digital_set_ttl_port_dir(uint8_t port, uint8_t dir);
uint32_t digital_get_input();
void digital_set_iso_out(uint8_t channel, uint8_t set);
int8_t digital_get_iso_out(uint8_t channel);
int8_t digital_get_iso_in(uint8_t channel);

void digital_set_iso_outw(uint8_t port, uint16_t value);
int8_t digital_get_iso_outw(uint8_t port, uint16_t *val);
int8_t digital_get_iso_inw(uint8_t port, uint16_t *val);

void digital_set_iso_outp(uint8_t channel, uint8_t pol);
void digital_set_iso_outpw(uint8_t port, uint16_t pol);
void digital_set_iso_inp(uint8_t channel, uint8_t pol);
void digital_set_iso_inpw(uint8_t port, uint32_t pol);


#endif