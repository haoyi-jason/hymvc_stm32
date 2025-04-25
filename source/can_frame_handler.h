#ifndef _CAN_FRAME_HANDLER_
#define _CAN_FRAME_HANDLER_

#include "hal.h"

typedef int8_t(can_pack_handler)(CANRxFrame *prx,CANTxFrame *ptx);

typedef struct can_frame_handler can_frame_handler;

struct can_frame_handler{
  int32_t eid;
  can_pack_handler *handler;
};


struct analog_transfer{
  int32_t raw_low;
  int32_t raw_high;
  float eng_low;
  float eng_high;
  int32_t factor;
};



#endif