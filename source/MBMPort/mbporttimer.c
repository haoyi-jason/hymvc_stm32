/*
 * MODBUS Library: ARM STM32 Port (FWLIB 2.0x)
 * Copyright (c) Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * ARM STM32 Port by Niels Andersen, Elcanic A/S <niels.andersen.elcanic@gmail.com>
 *
 * $Id: mbporttimer.c,v 1.2 2009-01-01 23:37:55 cwalter Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "ch.h"
#include "hal.h"
#include <stdlib.h>

/* ----------------------- Platform includes --------------------------------*/

#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "common/mbtypes.h"
#include "common/mbportlayer.h"
#include "common/mbframe.h"
#include "common/mbutils.h"

/* ----------------------- Defines ------------------------------------------*/
#define MBP_DEBUG_TIMER_PERFORMANCE     ( 0 )

#define MAX_TIMER_HDLS                  ( 2 )
#define IDX_INVALID                     ( 255 )
#define EV_NONE                         ( 0 )

#define TIMER_TIMEOUT_INVALID           ( 65535U )
#define TIMER_PRESCALER                 ( 128U )
#define TIMER_XCLK                      ( 72000000U )

#define TIMER_MS2TICKS( xTimeOut )      ( ( TIMER_XCLK * ( xTimeOut ) ) / ( TIMER_PRESCALER * 1000U ) )

#define RESET_HDL( x ) do { \
    ( x )->dev = NULL; \
    ( x )->ubIdx = IDX_INVALID; \
	( x )->usNTimeOutMS = 0; \
	( x )->usNTimeLeft = TIMER_TIMEOUT_INVALID; \
    ( x )->xMBMHdl = MB_HDL_INVALID; \
    ( x )->pbMBPTimerExpiredFN = NULL; \
    ( x )->gptConfig = NULL; \
} while( 0 );

#define GPT_BASE_CLOCK 1000000

/* ----------------------- Type definitions ---------------------------------*/
typedef struct
{
    GPTDriver*      dev;
    UBYTE           ubIdx;
    USHORT          usNTimeOutMS;
    USHORT          usNTimeLeft;
    xMBHandle       xMBMHdl;
    pbMBPTimerExpiredCB pbMBPTimerExpiredFN;
    GPTConfig       *gptConfig;
} xTimerInternalHandle;

static GPTDriver* gptList[] = {&GPTD3,&GPTD4};

/* ----------------------- Static variables ---------------------------------*/
STATIC xTimerInternalHandle arxTimerHdls[MAX_TIMER_HDLS];
STATIC BOOL     bIsInitalized = FALSE;

/* ----------------------- Static functions ---------------------------------*/
void            prvvMBPTimerISR( void ) __attribute__ ( ( __interrupt__ ) );

/* ----------------------- Start implementation -----------------------------*/



static void gpt_callback(GPTDriver *gptp)          
{
  //(void)gptp;
  
  xTimerInternalHandle *pxTimerIntHdl = NULL;
  chSysLockFromISR(); 
  
  for(uint8_t i=0;i<MB_UTILS_NARRSIZE(arxTimerHdls);i++){
    if(gptp == arxTimerHdls[i].dev){
      pxTimerIntHdl = &arxTimerHdls[i];
    }
  }
  
  if(pxTimerIntHdl != NULL){
    pxTimerIntHdl->usNTimeLeft--;
    if(0 == pxTimerIntHdl->usNTimeLeft){
      pxTimerIntHdl->usNTimeLeft = TIMER_TIMEOUT_INVALID;
      (void)pxTimerIntHdl->pbMBPTimerExpiredFN(pxTimerIntHdl->xMBMHdl);
    }
  }
  
//  if(gptp == arxTimerHdls[0].dev){
//    arxTimerHdls[0].usNTimeLeft--;
//    if(0 == arxTimerHdls[0].usNTimeLeft){
//      arxTimerHdls[0].usNTimeLeft = TIMER_TIMEOUT_INVALID;
//      (void)arxTimerHdls[0].pbMBPTimerExpiredFN(arxTimerHdls[0].xMBMHdl);
//    }
//  }
  
  chSysUnlockFromISR();
}
static GPTConfig gptcfg={
  GPT_BASE_CLOCK,                // GPT_BASE_CLOCK 18000000
  gpt_callback
};

eMBErrorCode
eMBPTimerInit( xMBPTimerHandle * xTimerHdl, USHORT usTimeOut1ms,
               pbMBPTimerExpiredCB pbMBPTimerExpiredFN, xMBHandle xHdl )
{
  //xTimerInternalHandle *pxTimerHdl = (xTimerInternalHandle*)&arxTimerHdls[0];
  eMBErrorCode    eStatus = MB_EPORTERR;
  UBYTE ubIdx;
  if((NULL != xTimerHdl) && (NULL != pbMBPTimerExpiredFN) && (MB_HDL_INVALID != xHdl)){
    if(!bIsInitalized){    
      
      for(ubIdx=0;ubIdx < MB_UTILS_NARRSIZE(arxTimerHdls);ubIdx++){
        RESET_HDL(&arxTimerHdls[ubIdx]);
      }
      gptInit();
      bIsInitalized = TRUE;
    }
    
    for(ubIdx = 0; ubIdx < MB_UTILS_NARRSIZE(arxTimerHdls);ubIdx++){
      if(IDX_INVALID == arxTimerHdls[ubIdx].ubIdx){
        break;
      }
    }
    if(MAX_TIMER_HDLS != ubIdx){      
      arxTimerHdls[ubIdx].ubIdx = ubIdx;
      arxTimerHdls[ubIdx].dev = gptList[ubIdx];
      arxTimerHdls[ubIdx].gptConfig = &gptcfg;
      arxTimerHdls[ubIdx].xMBMHdl = xHdl;
      arxTimerHdls[ubIdx].usNTimeOutMS = usTimeOut1ms ;
      arxTimerHdls[ubIdx].usNTimeLeft= TIMER_TIMEOUT_INVALID;
      arxTimerHdls[ubIdx].pbMBPTimerExpiredFN = pbMBPTimerExpiredFN;
      *xTimerHdl = &arxTimerHdls[ubIdx];

      //gptInterval = usTimeOut1ms+1;
      gptStart(arxTimerHdls[ubIdx].dev,arxTimerHdls[ubIdx].gptConfig);                      //General Purpose Timer,GPTD3:TIM1 
      eStatus = MB_ENOERR;
    }
    else{
      eStatus = MB_ENORES;
    }
  }
  
    return eStatus;
}

void
vMBPTimerClose( xMBPTimerHandle xTimerHdl )
{
    xTimerInternalHandle *pxTimerIntHdl = xTimerHdl;

    if(pxTimerIntHdl->dev != NULL){
      gptStop(pxTimerIntHdl->dev);
    }
}

eMBErrorCode
eMBPTimerSetTimeout( xMBPTimerHandle xTimerHdl, USHORT usTimeOut1ms )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xTimerInternalHandle *pxTimerIntHdl = xTimerHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxTimerIntHdl, arxTimerHdls ) &&
        ( usTimeOut1ms > 0 ) && ( usTimeOut1ms != TIMER_TIMEOUT_INVALID ) )
    {

        pxTimerIntHdl->usNTimeOutMS = usTimeOut1ms;
        eStatus = MB_ENOERR;
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPTimerStart( xMBPTimerHandle xTimerHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xTimerInternalHandle *pxTimerIntHdl = xTimerHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxTimerIntHdl, arxTimerHdls ) )
    {
        pxTimerIntHdl->usNTimeLeft = pxTimerIntHdl->usNTimeOutMS;
//        gptStartContinuous(arxTimerHdls[0].dev,1000);
        gptStartContinuous(pxTimerIntHdl->dev,1000);
        eStatus = MB_ENOERR;
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPTimerStop( xMBPTimerHandle xTimerHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xTimerInternalHandle *pxTimerIntHdl = xTimerHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxTimerIntHdl, arxTimerHdls ) )
    {
        pxTimerIntHdl->usNTimeLeft = TIMER_TIMEOUT_INVALID;
        eStatus = MB_ENOERR;
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

