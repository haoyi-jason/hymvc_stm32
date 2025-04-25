/*
 * MODBUS Library: ARM STM32 Port (FWLIB 2.0x)
 * Copyright (c) Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * ARM STM32 Port by Niels Andersen, Elcanic A/S <niels.andersen.elcanic@gmail.com>
 *
 * $Id: mbportserial.c,v 1.2 2008-12-14 20:30:27 cwalter Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>

/* ----------------------- Platform includes --------------------------------*/
#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "common/mbtypes.h"
#include "common/mbportlayer.h"
#include "common/mbframe.h"
#include "common/mbutils.h"

/* ----------------------- Defines ------------------------------------------*/

#define IDX_INVALID				( 255 )
#define UART_BAUDRATE_MIN		( 300 )
#define UART_BAUDRATE_MAX		( 115200 )


#define UART_1_ENABLED          ( 1 )   /*!< Set this to 1 to enable USART1 */
#define UART_2_ENABLED          ( 0 )   /*!< Set this to 1 to enable USART2 */

#if ( UART_1_ENABLED == 1 ) && ( UART_2_ENABLED == 1 )
#define UART_1_PORT             ( MB_UART_1 )
#define UART_2_PORT             ( MB_UART_2 )
#define UART_1_IDX              ( 0 )
#define UART_2_IDX              ( 1 )
#define NUARTS                  ( 2 )
#elif ( UART_1_ENABLED == 1 )
#define UART_1_PORT             ( MB_UART_1 )
#define UART_1_IDX              ( 0 )
#define NUARTS                  ( 1 )
#elif ( UART_2_ENABLED == 1 )
#define UART_2_PORT             ( MB_UART_2 )
#define UART_2_IDX              ( 0 )
#define NUARTS                  ( 1 )
#else
#define NUARTS                  ( 0 )
#endif

#define RS_485_UART_1_INIT(  )	\
do { \
} while( 0 )

#define RS_485_UART_1_ENABLE_TX(  )	\
do {\
    /* not implemented yet */\
} while( 0 )

#define RS_485_UART_1_DISABLE_TX(  ) \
do { \
    /* not implemented yet */ \
} while( 0 )

#define RS_485_UART_2_INIT(  )\
do { \
    /* not implemented yet */ \
} while( 0 )

#define RS_485_UART_2_ENABLE_TX(  )	\
do { \
    /* not implemented yet */ \
} while( 0 )

#define RS_485_UART_2_DISABLE_TX(  ) \
do { \
    /* not implemented yet */ \
} while( 0 )

/* ----------------------- Defines ------------------------------------------*/
/* ----------------------- Defines (Internal - Don't change) ----------------*/
#define HDL_RESET( x ) do { \
	( x )->ubIdx = IDX_INVALID; \
	( x )->pbMBMTransmitterEmptyFN = NULL; \
	( x )->pvMBMReceiveFN = NULL; \
	( x )->xMBMHdl = MB_HDL_INVALID; \
} while( 0 );

/* ----------------------- Type definitions ---------------------------------*/
typedef struct
{
    UBYTE           ubIdx;
    pbMBPSerialTransmitterEmptyAPIV1CB pbMBMTransmitterEmptyFN;
    pvMBPSerialReceiverAPIV1CB pvMBMReceiveFN;
    xMBHandle       xMBMHdl;
    UARTDriver *dev;
    UARTConfig *uartConfig;
} xSerialHandle;

/* ----------------------- Static variables ---------------------------------*/
STATIC xSerialHandle xSerialHdls[NUARTS];
STATIC BOOL     bIsInitalized = FALSE;

static void txDriverHasRead(UARTDriver *uartp) ;
static void txBufferEmpty(UARTDriver *uartp) ;
static void rxErr(UARTDriver *uartp, uartflags_t e);
static void rxChar(UARTDriver *uartp, uint16_t c);
static void rxEnd(UARTDriver *uartp);
static UARTConfig uartCfg = {
  txDriverHasRead,
  txBufferEmpty,
  rxEnd,
  rxChar,
  rxErr,
  NULL,
  9600,
  0,
  0,
  0
};


/* ----------------------- Static functions ---------------------------------*/
void            vMBPUSART1ISR( void ) __attribute__ ( ( __interrupt__ ) );
STATIC void     prvvMBPUSART1_TXE_ISR( void );
STATIC void     prvvMBPUSART1_TC_ISR( void );
STATIC void     prvvMBPUSART1_RXNE_ISR( void );

void            vMBPUSART2ISR( void ) __attribute__ ( ( __interrupt__ ) );
STATIC void     prvvMBPUSART2_TXE_ISR( void );
STATIC void     prvvMBPUSART2_TC_ISR( void );
STATIC void     prvvMBPUSART2_RXNE_ISR( void );

/* ----------------------- Start implementation -----------------------------*/

eMBErrorCode
eMBPSerialInit( xMBPSerialHandle * pxSerialHdl, UCHAR ucPort, ULONG ulBaudRate,
                UCHAR ucDataBits, eMBSerialParity eParity, UCHAR ucStopBits, xMBHandle xMBMHdl )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    UBYTE           ubIdx;
    xSerialHandle  *pxSerialIntHdl = &xSerialHdls[0];
    MBP_ENTER_CRITICAL_SECTION(  );
    if( !bIsInitalized )
    {
        for( ubIdx = 0; ubIdx < MB_UTILS_NARRSIZE( xSerialHdls ); ubIdx++ )
        {
            HDL_RESET( &xSerialHdls[ubIdx] );
        }
#if UART_1_ENABLED == 1
        RS_485_UART_1_INIT(  );
        RS_485_UART_1_DISABLE_TX(  );
#endif
#if UART_2_ENABLED == 1
        RS_485_UART_2_INIT(  );
        RS_485_UART_2_DISABLE_TX(  );
#endif
        bIsInitalized = TRUE;
    }

    if( ( MB_HDL_INVALID == xMBMHdl ) || ( NULL == pxSerialHdl ) )
    {
        eStatus = MB_EINVAL;
    }
    else
    {
        eStatus = MB_ENORES;

        /* Setup stopbits */
        switch ( ucStopBits )
        {
        case 1:
          uartCfg.cr2 &= ~USART_CR2_STOP_Msk;
          break;
        case 2:
          uartCfg.cr2 &= ~USART_CR2_STOP_Msk;
          uartCfg.cr2 |= USART_CR2_STOP_1;
          break;
        default:
          eStatus = MB_EINVAL;
          break;
        }

        /* For STM32 parity is placed on bit 9 (MSB)
         * Select correct number of databits */
        if( ucDataBits == 8 )
        {
            if( eParity != MB_PAR_NONE )
            {
              uartCfg.cr1 |= USART_CR1_M;
            }
            else
            {
              //  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
            }
        }
        else
        {
            if( eParity != MB_PAR_NONE )
            {
             //   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
            }
            else
            {
                eStatus = MB_EINVAL;
            }
        }

        /* Setup parity */
        switch ( eParity )
        {
        case MB_PAR_NONE:
          break;
        case MB_PAR_ODD:
          //uartCfg.cr1 |= (USART_CR1_PCE | USART_CR1_PS);
          break;
        case MB_PAR_EVEN:
          uartCfg.cr1 |= USART_CR1_PCE;
          break;
        default:
          eStatus = MB_EINVAL;
          break;
        }
        
        if(eStatus != MB_EINVAL){
          xSerialHdls[0].uartConfig = &uartCfg;
          xSerialHdls[0].dev = &UARTD6;
          xSerialHdls[0].xMBMHdl = xMBMHdl;
          *pxSerialHdl = &xSerialHdls[0];
          uartCfg.speed = ulBaudRate;
          uartStart(pxSerialIntHdl->dev,pxSerialIntHdl->uartConfig);
          eStatus = MB_ENOERR;
        }
    }
//    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPSerialClose( xMBPSerialHandle xSerialHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xSerialHandle  *pxSerialIntHdl = xSerialHdl;

    if(pxSerialIntHdl->dev != NULL){
      eStatus = MB_ENOERR;
      uartStop(pxSerialIntHdl->dev);
    }
  
    return eStatus;
}

eMBErrorCode
eMBPSerialTxEnable( xMBPSerialHandle xSerialHdl, pbMBPSerialTransmitterEmptyCB pbMBMTransmitterEmptyFN )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xSerialHandle  *pxSerialIntHdl = xSerialHdl;

    if(pxSerialIntHdl->dev != NULL){
      if(NULL != pbMBMTransmitterEmptyFN){
        pxSerialIntHdl->pbMBMTransmitterEmptyFN = pbMBMTransmitterEmptyFN;
        //pxSerialIntHdl->uartConfig->cr1 |= USART_CR1_TCIE;
        // enable TX End interrupt
        pxSerialIntHdl->dev->usart->CR1 |= (USART_CR1_TCIE);
        BYTE ubTxByte;
        pxSerialIntHdl->pbMBMTransmitterEmptyFN(pxSerialIntHdl->xMBMHdl,&ubTxByte);
        uartStartSend(pxSerialIntHdl->dev, 1, &ubTxByte);      
      }
      else{
        pxSerialIntHdl->pbMBMTransmitterEmptyFN = NULL;
        //pxSerialIntHdl->uartConfig->cr1 &= ~USART_CR1_TCIE;
        // disable TX End interrupt
        pxSerialIntHdl->dev->usart->CR1 &= ~USART_CR1_TCIE;
        //pxSerialIntHdl->dev->usart->CR1 |= (USART_CR1_TCIE);
      }
      eStatus = MB_ENOERR;
    }

    return eStatus;
}

eMBErrorCode
eMBPSerialRxEnable( xMBPSerialHandle xSerialHdl, pvMBPSerialReceiverCB pvMBMReceiveFN )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xSerialHandle  *pxSerialIntHdl = xSerialHdl;
    if(pxSerialIntHdl->dev != NULL){
      if(NULL != pvMBMReceiveFN){
        pxSerialIntHdl->pvMBMReceiveFN = pvMBMReceiveFN;
        pxSerialIntHdl->uartConfig->cr1 |= USART_CR1_RXNEIE;
        pxSerialIntHdl->dev->usart->CR1 |= USART_CR1_RXNEIE;
      }
      else{
        pxSerialIntHdl->pvMBMReceiveFN = NULL;
        pxSerialIntHdl->uartConfig->cr1 &= ~USART_CR1_RXNEIE;
        pxSerialIntHdl->dev->usart->CR1 &= ~USART_CR1_RXNEIE;
      }
      eStatus = MB_ENOERR;
    }
    
    return eStatus;
}
          
          /* End of transmission buffer callback. TC  */
static void txDriverHasRead(UARTDriver *uartp){
  (void) uartp;
  chSysLockFromISR();
  //uartp->usart->CR1 &= ~USART_CR1_TCIE;
  chSysUnlockFromISR();
}

/* Physical end of transmission callback. TE */
void txBufferEmpty(UARTDriver *uartp){
  //(void) uartp;
  BOOL bHasMoreData = TRUE;
  UBYTE ubTxByte;
  chSysLockFromISR();
  for(uint8_t i=0;i<NUARTS;i++){
    xSerialHandle  *pxSerialIntHdl = &xSerialHdls[i];
    if((NULL != pxSerialIntHdl->pbMBMTransmitterEmptyFN) && (uartp == pxSerialIntHdl->dev)){
      bHasMoreData = pxSerialIntHdl->pbMBMTransmitterEmptyFN(pxSerialIntHdl->xMBMHdl,&ubTxByte);
    }
    if(!bHasMoreData){
      pxSerialIntHdl->pbMBMTransmitterEmptyFN = NULL;
      pxSerialIntHdl->uartConfig->cr1 &= ~USART_CR1_TCIE;
      pxSerialIntHdl->dev->usart->CR1 &= ~USART_CR1_TCIE;
    }
    else{
      uartStartSendI (pxSerialIntHdl->dev, 1, &ubTxByte);      
    }
  }

  chSysUnlockFromISR();
}


/* Receive buffer filled callback.  */
void rxEnd(UARTDriver *uartp){
  (void) uartp;
  
}


/* Character received while out if the UART_RECEIVE state.  */
void rxChar(UARTDriver *uartp, uint16_t c){
  
  (void) uartp;
  (void) c;

  UCHAR oneByteAccum = (UCHAR) c;

  vMBPortSetWithinException (TRUE);
  chSysLockFromISR();

  for(uint8_t i=0;i<NUARTS;i++){
    xSerialHandle  *pxSerialIntHdl = &xSerialHdls[i];
    if((NULL != pxSerialIntHdl->pvMBMReceiveFN) && (uartp == pxSerialIntHdl->dev)){
      pxSerialIntHdl->pvMBMReceiveFN(pxSerialIntHdl->xMBMHdl,(UBYTE)c);
    }
  }
    
  chSysUnlockFromISR();  
}

/* Receive error callback.  */
void rxErr(UARTDriver *uartp, uartflags_t e){
  (void) uartp;
  (void) e;
  
  chSysLockFromISR(); 
  if (e & USART_SR_PE) {
    //syslogErrorFromISR ("parity err");
  } else if (e & USART_SR_FE) {
    //syslogErrorFromISR ("framing err");
  } if (e & USART_SR_NE) {
    //syslogErrorFromISR ("noise err");
  } if (e & USART_SR_ORE) {
    //syslogErrorFromISR ("overrun err");
  }  if (e & USART_SR_IDLE) {
    //syslogErrorFromISR ("idle line err");
  } else {
    //syslogErrorFromISR ("uart rx err");
  }
  chSysUnlockFromISR();
  
  
}
         
