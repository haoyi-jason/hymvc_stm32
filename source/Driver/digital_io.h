#ifndef _DIGITAL_IO_
#define _DIGITAL_IO_

enum _dio_type{
  INPUT_GPIO,
  INPUT_CAPTURE,
  INPUT_COUNTER,
  INPUT_FREQUENCY,
  OUTPUT_GPIO,
  OUTPUT_PWM,
  OUTPUT_FREQUENCY
};

struct _dio_pin{
  uint8_t type;
  int32_t value;
  ioport_t port;
  ioportmask_t pad;
};

typedef struct DigitalDriver DigitalDriver;

//typedef struct{
  

#endif