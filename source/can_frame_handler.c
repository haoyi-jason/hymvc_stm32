#include "ch.h"
#include "hal.h"
#include "can_frame_handler.h"

int8_t findHandler(int32_t id, CANRxFrame *prx, CANTxFrame *ptx, can_frame_handler *handlers)
{
  int8_t result = -1;
  can_frame_handler handler;
  bool loop = true;
  uint8_t index = 0;
  while(loop){
    handler = handlers[index++];
    if(handler.eid != -1 && handler.eid == id){
      result = handler.handler(prx,ptx);
    }
    if(handler.eid == -1){
      loop = false;
    }
  }
  return result;
}


