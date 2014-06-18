/****************************************************************************
 *   $Id:: uart.c 5751 2010-11-30 23:56:11Z usb00423                        $
 *   Project: NXP LPC17xx UART example
 *
 *   Description:
 *     This file contains UART code example which include UART initialization, 
 *     UART interrupt handler, and APIs for UART access.
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/
#include "LPC17xx.h"
#include "type.h"
#include "uart.h"
#include "HardwareDefinition.h"
#include "Hardwareprofile.h"
#include "i2c.h"
#include "adc.h"
#include "sensorboard.h"
#include "batterymonitor.h"
#include "motorcontroller.h"
#include "inertial.h"
#include "lcd.h"
#include "Gps.h"


volatile uint32_t UART0Status, UART1Status, UART2Status,  UART3Status;
volatile uint8_t UART0TxEmpty = 1, UART1TxEmpty = 1, UART2TxEmpty = 1,UART3TxEmpty = 1;
volatile uint8_t UART0Buffer[UART_BUFSIZE], UART1Buffer[UART_BUFSIZE], UART2Buffer[UART_BUFSIZE], UART3Buffer[UART_BUFSIZE];
volatile uint32_t UART0Count = 0, UART1Count = 0, UART2Count = 0, UART3Count = 0;
volatile uint8_t Tx_Buffer[UART_BUFSIZE];
volatile uint8_t Rx_Buffer[UART_BUFSIZE];
volatile uint8_t UART2_RxBuffer[UART_BUFSIZE];

volatile uint8_t cmd_byte=0;
volatile uint8_t data_byte1=0;
volatile uint8_t data_byte2=0;
volatile uint8_t data_byte3=0;
volatile uint8_t data_byte4=0;
volatile uint8_t data_byte5=0;
volatile uint8_t packet_valid=0;
volatile uint8_t Len=0;
volatile int32_t Temp=0;
volatile uint8_t UART2Flag=0;
volatile uint8_t RxReadLength=0;
volatile uint8_t positionData[10];

volatile uint8_t gpsPacketRx = 0;

extern uint8_t Ultrasonic[8];
extern uint8_t Whiteline[8];
extern uint8_t IR_Proximity[8];
extern uint8_t Accelerometer[8];
extern uint8_t Gyroscope[8];
extern uint8_t Magnetometer[8];
extern uint8_t Battery[8];
extern uint8_t MotorVoltage;
extern uint8_t LeftMotorCurrent;
extern uint8_t RightMotorCurrent;
extern uint8_t AD7998ADC[16];
extern uint32_t PotValue;
extern uint32_t ServoPOD_ADC;
extern uint8_t ResetEncoderCounts;
extern uint8_t UpdateMode;
extern uint8_t UpdateSafety;
extern uint8_t ServoPodULTriggerState;
extern uint8_t updatePosition;
extern int32_t positionLeft;
extern int32_t positionRight;
extern int8_t velocityLeft;
extern int8_t velocityRight;
extern uint8_t updateRemote_2_4GhzCommand;
extern uint8_t remoteControl[6];

extern volatile uint8_t PanAngle;
extern volatile uint8_t TiltAngle;
extern volatile uint8_t AuxAngle;
extern volatile int32_t Left_Count_New_Locked;
extern volatile int32_t Right_Count_New_Locked;
extern volatile uint8_t Mode;
extern volatile uint8_t Safety;

volatile uint8_t Test[200];
volatile uint8_t TestCount=0;

/********************************************/
extern volatile uint8_t My_Button_Status;
/********************************************/


/*****************************************************************************
** Function name:		UART0_IRQHandler
**
** Descriptions:		UART0 interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART0_IRQHandler (void) 
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;
  /*static char c=0;

  if (c==0)
  LED2_ON();
  else
	  LED2_OFF();
  c = !c;*/
	
  IIRValue = LPC_UART0->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_UART0->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{//LED3_ON();
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  UART0Status = LSRValue;
	  Dummy = LPC_UART0->RBR;		/* Dummy read on RX to clear 
							interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */			
	{
	  /* If no error on RLS, normal ready, save into the data buffer. */
	  /* Note: read RBR will clear the interrupt */
	  UART0Buffer[0] = LPC_UART0->RBR;
#ifdef GPS
	  //GPSDecode(UART0Buffer[0]);
	  UARTDecode(0,UART0Buffer[0]);
#endif
	  UART0Count++;
	  if ( UART0Count == UART_BUFSIZE )
	  {
		UART0Count = 0;		/* buffer overflow */
	  }	
	 }
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */
	UART0Buffer[0] = LPC_UART0->RBR;
#ifdef GPS
	  //GPSDecode(UART0Buffer[0]);
	UARTDecode(0,UART0Buffer[0]);
#endif
	UART0Count++;
	if ( UART0Count == UART_BUFSIZE )
	{
	  UART0Count = 0;		/* buffer overflow */
	}
  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	UART0Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {//LED4_ON();
	/* THRE interrupt */
	LSRValue = LPC_UART0->LSR;		/* Check status in the LSR to see if
									valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
	{
	  UART0TxEmpty = 1;
	}
	else
	{
	  UART0TxEmpty = 0;
	}
  }
    
}

/*****************************************************************************
** Function name:		UART1_IRQHandler
**
** Descriptions:		UART1 interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART1_IRQHandler (void) 
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;

  IIRValue = LPC_UART1->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_UART1->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  UART1Status = LSRValue;
	  Dummy = LPC_UART1->RBR;		/* Dummy read on RX to clear 
								interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */			
	{
	  /* If no error on RLS, normal ready, save into the data buffer. */
	  /* Note: read RBR will clear the interrupt */
	  UART1Buffer[0] = LPC_UART1->RBR;
#if	(defined(XBEE) || defined(BLUETOOTH))
	  UARTDecode(WIRELESS_PORT,UART1Buffer[0]);
#endif

#if (defined(WIFI))
	  WirelessModuleDecode(UART1Buffer[0]);
#endif

	  UART1Count++;
	  if ( UART1Count == UART_BUFSIZE )
	  {
		UART1Count = 0;		 //buffer overflow
	  }

	}
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */
	  UART1Buffer[0] = LPC_UART1->RBR;
#if	(defined(XBEE) || defined(BLUETOOTH))
	  UARTDecode(WIRELESS_PORT,UART1Buffer[0]);
#endif

#if (defined(WIFI))
	  WirelessModuleDecode(UART1Buffer[0]);
#endif
	  UART1Count++;
	  if ( UART1Count == UART_BUFSIZE )
	  {
		  UART1Count = 0;		/* buffer overflow */
	  }

  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	UART1Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {
	/* THRE interrupt */
	LSRValue = LPC_UART1->LSR;		/* Check status in the LSR to see if
								valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
	{
	  UART1TxEmpty = 1;
	}
	else
	{
	  UART1TxEmpty = 0;
	}
  }

}


/*****************************************************************************
** Function name:		UART2_IRQHandler
**
** Descriptions:		UART2 interrupt handler
**
** parameters:			None
** Returned value:		None
**
*****************************************************************************/
void UART2_IRQHandler (void)
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;


  IIRValue = LPC_UART2->IIR;

  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_UART2->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  UART2Status = LSRValue;
	  Dummy = LPC_UART2->RBR;		/* Dummy read on RX to clear
							interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */
	{
	  /* If no error on RLS, normal ready, save into the data buffer. */
	  /* Note: read RBR will clear the interrupt */
	  //UART2Buffer[0] = LPC_UART2->RBR;
	  UART2_RxBuffer[UART2Count++] = LPC_UART2->RBR;
	  if ( UART2Count == RxReadLength )
	  {
		UART2Count = 0;		/* buffer overflow */
		RxReadLength = 0;
		UART2Flag = 1;
	  }
	 }
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */
	//UART2Buffer[0] = LPC_UART2->RBR;
	  UART2_RxBuffer[UART2Count++] = LPC_UART2->RBR;
	  if ( UART2Count == RxReadLength )
	  {
		  UART2Count = 0;		/* buffer overflow */
		  RxReadLength = 0;
		  UART2Flag = 1;
	  }
  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	UART2Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {
	/* THRE interrupt */
	LSRValue = LPC_UART2->LSR;		/* Check status in the LSR to see if
									valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
	{
	  UART2TxEmpty = 1;
	}
	else
	{
	  UART2TxEmpty = 0;
	}
  }

}


/*****************************************************************************
** Function name:		UART3_IRQHandler
**
** Descriptions:		UART0 interrupt handler
**
** parameters:			None
** Returned value:		None
**
*****************************************************************************/
void UART3_IRQHandler (void)
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;

  static char c=0;

    if (c==0)
    LED2_ON();
    else
  	  LED2_OFF();
    c = !c;


  IIRValue = LPC_UART3->IIR;

  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_UART3->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{LED3_ON();
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  UART0Status = LSRValue;
	  Dummy = LPC_UART3->RBR;		/* Dummy read on RX to clear
							interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */
	{
	  /* If no error on RLS, normal ready, save into the data buffer. */
	  /* Note: read RBR will clear the interrupt */
	  UART3Buffer[0] = LPC_UART3->RBR;
	  UARTDecode(RS232_PORT,UART3Buffer[0]);
	  UART3Count++;
	  if ( UART3Count == UART_BUFSIZE )
	  {
		UART3Count = 0;		/* buffer overflow */
	  }
	 }
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */
	UART3Buffer[0] = LPC_UART3->RBR;
	UARTDecode(RS232_PORT,UART3Buffer[0]);
	UART3Count++;
	if ( UART3Count == UART_BUFSIZE )
	{
	  UART3Count = 0;		/* buffer overflow */
	}
  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	UART3Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {LED4_ON();
	/* THRE interrupt */
	LSRValue = LPC_UART3->LSR;		/* Check status in the LSR to see if
									valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
	{
	  UART3TxEmpty = 1;
	}
	else
	{
	  UART3TxEmpty = 0;
	}
  }

}

/*****************************************************************************
** Function name:		UARTInit
**
** Descriptions:		Initialize UART port, setup pin select,
**						clock, parity, stop bits, FIFO, etc.
**
** parameters:			portNum(0 or 1) and UART baudrate
** Returned value:		true or false, return false only if the 
**						interrupt handler can't be installed to the 
**						VIC table
** 
*****************************************************************************/
uint32_t UARTInit( uint32_t PortNum, uint32_t baudrate )
{
  uint32_t Fdiv;
  uint32_t pclkdiv, pclk;
  uint32_t GetPriority=0;

  if ( PortNum == 0 )
  {
	LPC_PINCON->PINSEL0 &= ~0x000000F0;
	LPC_PINCON->PINSEL0 |= 0x00000050;  /* RxD0 is P0.3 and TxD0 is P0.2 */
	/* By default, the PCLKSELx value is zero, thus, the PCLK for
	all the peripherals is 1/4 of the SystemFrequency. */
	/* Bit 6~7 is for UART0 */
	pclkdiv = (LPC_SC->PCLKSEL0 >> 6) & 0x03;
	switch ( pclkdiv )
	{
	  case 0x00:
	  default:
		pclk = SystemFrequency/4;
		break;
	  case 0x01:
		pclk = SystemFrequency;
		break; 
	  case 0x02:
		pclk = SystemFrequency/2;
		break; 
	  case 0x03:
		pclk = SystemFrequency/8;
		break;
	}

    LPC_UART0->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
	Fdiv = ( pclk / 16 ) / baudrate ;	/*baud rate */
    LPC_UART0->DLM = Fdiv / 256;							
    LPC_UART0->DLL = Fdiv % 256;
    LPC_UART0->LCR = 0x03;		/* DLAB = 0 */
    LPC_UART0->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */
    //GetPriority=NVIC_GetPriority(UART0_IRQn);
    //NVIC_SetPriority(UART0_IRQn,1);
   	NVIC_EnableIRQ(UART0_IRQn);

    LPC_UART0->IER = IER_RBR | IER_RLS;	/* Enable UART0 interrupt */
    return (TRUE);
  }
  else if ( PortNum == 1 )
  {
	//LPC_PINCON->PINSEL4 &= ~0x0000000F;
	//LPC_PINCON->PINSEL4 |= 0x0000000A;	/* Enable RxD1 P2.1, TxD1 P2.0 */

	LPC_PINCON->PINSEL0 &= ~0xC0000000;
	LPC_PINCON->PINSEL0 |= 0x40000000;

	LPC_PINCON->PINSEL1 &= ~0x00000003;
	LPC_PINCON->PINSEL1 |= 0x000000001;
	
	/* By default, the PCLKSELx value is zero, thus, the PCLK for
	all the peripherals is 1/4 of the SystemFrequency. */
	/* Bit 8,9 are for UART1 */
	pclkdiv = (LPC_SC->PCLKSEL0 >> 8) & 0x03;
	switch ( pclkdiv )
	{
	  case 0x00:
	  default:
		pclk = SystemFrequency/4;
		break;
	  case 0x01:
		pclk = SystemFrequency;
		break; 
	  case 0x02:
		pclk = SystemFrequency/2;
		break; 
	  case 0x03:
		pclk = SystemFrequency/8;
		break;
	}

    LPC_UART1->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
	Fdiv = ( pclk / 16 ) / baudrate ;	/*baud rate */
    LPC_UART1->DLM = Fdiv / 256;							
    LPC_UART1->DLL = Fdiv % 256;
	LPC_UART1->LCR = 0x03;		/* DLAB = 0 */
    LPC_UART1->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

   	NVIC_EnableIRQ(UART1_IRQn);

    LPC_UART1->IER = IER_RBR |IER_RLS;	/* Enable UART1 interrupt */
    return (TRUE);
  }
  else if ( PortNum == 2 )
    {
	  LPC_SC->PCONP|=0x01000000;
	  LPC_PINCON->PINSEL0 &= ~0x00F00000;
	  	LPC_PINCON->PINSEL0 |= 0x00500000;
	  	/* By default, the PCLKSELx value is zero, thus, the PCLK for
	  	all the peripherals is 1/4 of the SystemFrequency. */
	  	/* Bit 6~7 is for UART2 */
	  	pclkdiv = (LPC_SC->PCLKSEL1 >> 16) & 0x03;
	  	switch ( pclkdiv )
	  	{
	  	  case 0x00:
	  	  default:
	  		pclk = SystemFrequency/4;
	  		break;
	  	  case 0x01:
	  		pclk = SystemFrequency;
	  		break;
	  	  case 0x02:
	  		pclk = SystemFrequency/2;
	  		break;
	  	  case 0x03:
	  		pclk = SystemFrequency/8;
	  		break;
	  	}

	      LPC_UART2->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
	  	Fdiv = ( pclk / 16 ) / baudrate ;	/*baud rate */
	      LPC_UART2->DLM = Fdiv / 256;
	      LPC_UART2->DLL = Fdiv % 256;
	      LPC_UART2->LCR = 0x03;		/* DLAB = 0 */
	      LPC_UART2->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

	     	NVIC_EnableIRQ(UART2_IRQn);

	      LPC_UART2->IER = IER_RBR | IER_RLS;	/* Enable UART2 interrupt */
	      return (TRUE);
    }
  else if ( PortNum == 3 )
  {
	  LPC_SC->PCONP|=0x02000000;
	  LPC_PINCON->PINSEL0 &= ~0x0000000F;
	  LPC_PINCON->PINSEL0 |= 0x0000000A;	/* Enable RxD3 P0.1, TxD3 P0.0 */

	  /* By default, the PCLKSELx value is zero, thus, the PCLK for
  	all the peripherals is 1/4 of the SystemFrequency. */
	  /* Bit 8,9 are for UART3 */
	  pclkdiv = (LPC_SC->PCLKSEL1 >> 18) & 0x03;
	  switch ( pclkdiv )
	  {
	  case 0x00:
	  default:
		  pclk = SystemFrequency/4;
		  break;
	  case 0x01:
		  pclk = SystemFrequency;
		  break;
	  case 0x02:
		  pclk = SystemFrequency/2;
		  break;
	  case 0x03:
		  pclk = SystemFrequency/8;
		  break;
	  }

	  LPC_UART3->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
	  Fdiv = ( pclk / 16 ) / baudrate ;	/*baud rate */
	  LPC_UART3->DLM = Fdiv / 256;
	  LPC_UART3->DLL = Fdiv % 256;
	  LPC_UART3->LCR = 0x03;		/* DLAB = 0 */
	  LPC_UART3->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */
	  GetPriority=NVIC_GetPriority(UART3_IRQn);
	  NVIC_SetPriority(UART3_IRQn,1);
	  NVIC_EnableIRQ(UART3_IRQn);
	  NVIC_EnableIRQ(UART3_IRQn);

	  LPC_UART3->IER = IER_RBR | IER_RLS;	/* Enable UART3 interrupt */
	  return (TRUE);
  }
  return( FALSE ); 
}

/*****************************************************************************
** Function name:		UARTSend
**
** Descriptions:		Send a block of data to the UART 0 port based
**						on the data length
**
** parameters:			portNum, buffer pointer, and data length
** Returned value:		None
** 
*****************************************************************************/
void UARTSend( uint32_t portNum, uint8_t *BufferPtr, uint32_t Length )
{
  if ( portNum == 0 )
  {
    while ( Length != 0 )
    {
	  /* THRE status, contain valid data */
	  //while ( !(UART0TxEmpty & 0x01) );
      while((LPC_UART0->LSR & 0x40)==0);
	  LPC_UART0->THR = *BufferPtr;
	  UART0TxEmpty = 0;	/* not empty in the THR until it shifts out */
	  BufferPtr++;
	  Length--;
	}
  }
  else if(portNum == 1)
  {
	while ( Length != 0 )
    {
	  /* THRE status, contain valid data */
	  //while ( !(UART1TxEmpty & 0x01) );
	  while((LPC_UART1->LSR & 0x40)==0);
	  LPC_UART1->THR = *BufferPtr;
	  UART1TxEmpty = 0;	/* not empty in the THR until it shifts out */
	  BufferPtr++;
	  Length--;
    }
  }
  else if ( portNum == 2 )
    {
      while ( Length != 0 )
      {
  	  /* THRE status, contain valid data */
  	  //while ( !(UART0TxEmpty & 0x01) );
        while((LPC_UART2->LSR & 0x40)==0);
  	  LPC_UART2->THR = *BufferPtr;
  	  UART2TxEmpty = 0;	/* not empty in the THR until it shifts out */
  	  BufferPtr++;
  	  Length--;
  	}
    }
  else if(portNum == 3)
  {
	  while ( Length != 0 )
	  {
		  /* THRE status, contain valid data */
		  //while ( !(UART3TxEmpty & 0x01) );
		  while((LPC_UART3->LSR & 0x40)==0);
		  LPC_UART3->THR = *BufferPtr;
		  UART3TxEmpty = 0;	/* not empty in the THR until it shifts out */
		  BufferPtr++;
		  Length--;
	  }
  }
  return;
}


/*===============================================================================
 Function        : GPSDecode(uint8_t data);
 Author			 : Nex Robotics Pvt Ltd
 Parameters		 : data-> Data received in UART0 ISR from GPS modules
 Description 	 : This function buffers the data received in UART0 ISR from the GPS module.
 Preconditions	 : None
===============================================================================*/
#ifdef GPS
void GPSDecode(uint8_t data)
{
	usatr3_count++;

	if(!(rx_start2))
	{
		if(data == 0x24)         // ASCII value of $      // is it start of packet
		{
			rx_start = 1;          // enable rx_start, to receive the next data
			ii = 0;                // initialise it to 0 for array element
			rxtxintchngdbit = 0;   // make this high to indicate communication is established
		}
	}
	if(rx_start)               // is it enabled start rx'ing
	{
		rx_string[ii] = data;    // store received data in buffer array

		rx_start1 = 1;
		rx_start = 0;
		rx_start2 = 1;
		ii++;
	}

	else if(rx_start1)                             // if eabled, start rx'ing the GGA packet
	{
		rx_string[ii] = data;                   // store the received data from GPGGA msg in to Rx_string buffer array
		if(data==0x0A && rx_string[ii-1]==0x0D) // is it end of the receiving packet
		{
			ii = 0;
			rx_start1 = 0;
			rx_start2 = 1;                        // disable this varaible to receive new GPGGA sentence
			gpsPacketRx = 1;						//Indicate Packet is received

		}
		ii++;                                  // incrment the array position
	}
}
#endif

/*===============================================================================
 Function        : WirelessModuleDecode(uint8_t ser_data);
 Author			 : Nex Robotics Pvt Ltd
 Parameters		 : ser_data -> Data received in UART1 ISR from wireless modules
 Description 	 : This function decodes the data received in UART1 ISR
                   and takes necessary action.
 Preconditions	 : None
===============================================================================*/

	/*
	 * NEX1	-> Forward
	 * NEX2 -> Reverse
	 * NEX3 -> Left
	 * NEX4 -> Right
	 * NEX0 -> Stop
	 */
void WirelessModuleDecode(uint8_t ser_data)
{
	if(ser_data==0x20)	//carriage return
	{
		if(Test[0]=='N')
		{
			if(Test[1]=='E')
			{
				if(Test[2]=='X')
				{
					switch(Test[3])
					{
						case '1': {Direction = FORWARD; MotorUpdate = 1; break;}
						case '2': {Direction = REVERSE;	MotorUpdate = 1; break;}
						case '3': {Direction = LEFT;	MotorUpdate = 1; break;}
						case '4': {Direction = RIGHT;   MotorUpdate = 1; break;}
						case '0': {Direction = STOP;    MotorUpdate = 1; break;}
						default:  {Direction = STOP;    MotorUpdate = 1; break;}
					}
				}

			}
		}
		Test[0] = 0x00;
		Test[1] = 0x00;
		Test[2] = 0x00;
		Test[3] = 0x00;
		Test[4] = 0x00;
		TestCount = 0;

	}
	else
	{
		Test[TestCount] = ser_data;
		TestCount++;
	}
}



/*************************ADDITIONS***********************/
#define LASER_PIN			0x04000000
#define LASER_TURN_ON()		LPC_GPIO3->FIOCLR=LASER_PIN
#define LASER_TURN_OFF()	LPC_GPIO3->FIOSET=LASER_PIN
/*********************************************************/

/*************************ADDITIONS***********************/
#define MOTOR_PIN				0x00040000
#define MOTOR_TURN_ON()			LPC_GPIO1->FIOCLR=MOTOR_PIN
#define MOTOR_TURN_OFF()		LPC_GPIO1->FIOSET=MOTOR_PIN
/*********************************************************/

/*************************ADDITIONS***********************/
#define MOTOR1_PIN				0x00400000
#define MOTOR1_TURN_ON()		LPC_GPIO1->FIOCLR=MOTOR1_PIN
#define MOTOR1_TURN_OFF()		LPC_GPIO1->FIOSET=MOTOR1_PIN

#define MOTOR2_PIN				0x00200000
#define MOTOR2_TURN_ON()		LPC_GPIO1->FIOCLR=MOTOR2_PIN
#define MOTOR2_TURN_OFF()		LPC_GPIO1->FIOSET=MOTOR2_PIN

#define MOTOR3_PIN				0x00000002
#define MOTOR3_TURN_ON()		LPC_GPIO2->FIOCLR=MOTOR3_PIN
#define MOTOR3_TURN_OFF()		LPC_GPIO2->FIOSET=MOTOR3_PIN


/*********************************************************/

/*===============================================================================
 Function        : UARTDecode(uint8_t ser_data);
 Author			 : Nex Robotics Pvt Ltd
 Parameters		 : ser_data -> Data received in UART3 ISR from PC or embedded kit
 Description 	 : This function decodes the data received in UART0 ISR
                   and takes necessary action.
 Preconditions	 : None
===============================================================================*/
void UARTDecode(uint8_t portNum,uint8_t ser_data)
{
	static uint8_t packet_data_count = 1;
	uint8_t i=0;

	//////////////////////////////////////////////////////////////////
		//LED4_ON();
	//////////////////////////////////////////////////////////////////

	 if(packet_data_count >= 6)
	 {
	     if(cmd_byte == 0x9C)					// position command
		 {
			 positionData[packet_data_count - 5] = ser_data;
			 packet_data_count++;

			 if(packet_data_count == 15)		// complete packet received for position command
				 packet_valid = 1;
		 }
	     if(cmd_byte == 0x25)					// 2.4GHz remote data
	     {
	    	 remoteControl[packet_data_count - 5] = ser_data;
	    	 packet_data_count++;

	    	 if(packet_data_count == 11)
	    		 packet_valid = 1;
	     }
	 }

	 if(packet_data_count == 5)				// 5th byte of packet
	 {
	     if(cmd_byte == 0x9C)				// position command
		 {
	    	 positionData[0] = ser_data;
			 packet_data_count=6;
		 }
	     else if(cmd_byte == 0x25)			// 2.4 GHz remote control data
	     {
	    	 remoteControl[0] = ser_data;
	    	 packet_data_count = 6;
	     }
	     else
		 {
		     packet_valid = 1;				// complete packet received
	         data_byte1 = ser_data;			// store received data
	         packet_data_count = 5;
		 }
	 }
	 /*****************************************************************/
	 if(packet_data_count == 4)              // 4th byte of packet is command
	 {
	     cmd_byte = ser_data;				// store command byte
		 packet_data_count = 5;				// allow for receiving 5th byte
	 }


	 if(packet_data_count == 3)              // 3rd byte of packet
	 {
		 if(ser_data == 'X')                // is 3rd byte is 'X'
			 packet_data_count = 4;          // allow for receiving 4th byte
		 else
			 packet_data_count = 1;          // allow for new packet to receive i.e 1st byte
	 }

	 if(packet_data_count == 2)              // 2nd byte of packet
	 {
		 if(ser_data=='E')                // is 2nd byte is 'E'
			 packet_data_count = 3;         // allow for receving 3rd byte
		 else
			 packet_data_count = 1;          // allow for new packet to receive i.e 1st byte
	 }

	 if(packet_data_count == 1)              // 1st byte of packet
	 {


		 if(ser_data == 'N')                // is 1st byte is 'N'
			 packet_data_count = 2;          // allow for receving 2nd byte
	 }

	 if(packet_valid)
	 {
		 switch (cmd_byte)
		 {
		 	 case 0x01:           	// command tag for ultrasonic sensors
		 	 {
		 		 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 		 Tx_Buffer[5] = Ultrasonic[data_byte1-1];           // ultrasonic sensor data is stored at index 0 to 7, databyte1 will range from 1 to 8 so...
		 		 Len = 6;
		 		 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);

		 		 break;
		 	 }
		 	 case 0x02:           	// command tag for IR Proximity sensors
		 	 {
		 		 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 		 Tx_Buffer[5] = IR_Proximity[data_byte1-1];         // IR Proximity data is stored at index 0 to 7, databyte1 will range from 1 to 8 so...
		 		 Len = 6;
		 		 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 break;
		 	 }
		 	 case 0x04:           	// command tag for White Line sensors
		 	 {
		 		 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 		 Tx_Buffer[5] = Whiteline[data_byte1-1];     // white line data is stored at index 0 to 7, databyte1 will range from 1 to 8 so...
		 		 Len = 6;
		 		 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 break;
		 	 }

		 	 case 0x05:           	// command tag for sensor group reading
		 	 {
		 		 if(data_byte1 == 0x01)			//Ultrasonic
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 for(i=0;i<8;i++)
		 			 {
		 				 Tx_Buffer[i+5] = Ultrasonic[i];
		 			 }
		 			 Len =13;
		 			 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		 else if(data_byte1 == 0x02)	//IR Proximity
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 for(i=0;i<8;i++)
		 			 {
		 				 Tx_Buffer[i+5] = IR_Proximity[i];
		 			 }
		 			 Len =13;
		 			 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		 else if(data_byte1 == 0x04)	// White Line
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 for(i=0;i<8;i++)
		 			 {
		 				 Tx_Buffer[i+5] = Whiteline[i];
		 			 }
		 			 Len =13;
		 			 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		else if(data_byte1 == 0x05)		// Accelerometer
		 		{
		 			Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			for(i=0;i<6;i++)
		 			{
		 				Tx_Buffer[i+5] = Accelerometer[i];
		 			}
		 			Len =11;
		 			UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}
		 		else if(data_byte1 == 0x06)		// Gyroscope
		 		{
		 			Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			for(i=0;i<6;i++)
		 			{
		 				Tx_Buffer[i+5] = Gyroscope[i];
		 			}
		 			Len =11;
		 			UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}
		 		else if(data_byte1 == 0x07)		// Magnetometer
		 		{
		 			Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			for(i=0;i<6;i++)
		 			{
		 				Tx_Buffer[i+5] = Magnetometer[i];
		 			}
		 			Len =11;
		 			UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}
		 		else if(data_byte1 == 0x08)		// AD7998 ADC
		 		{
		 			Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			for(i=0;i<16;i++)
		 			{
		 				Tx_Buffer[i+5] = AD7998ADC[i];
		 			}
		 			Len =21;
		 			UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}
		 		else if(data_byte1 == 0x09)		// POT
		 		{
		 			Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			Tx_Buffer[5] = PotValue & 0x00FF;
		 			Tx_Buffer[6] = PotValue >> 8;
		 			Len =7;
		 			UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}
		 		else if(data_byte1 == 0x0A)		// Servo Pod Ultrasonic
		 		{
		 			Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			Tx_Buffer[5] = ServoPOD_ADC & 0x00FF;
		 			Tx_Buffer[6] = ServoPOD_ADC >> 8;
		 			Len =7;
		 			UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}
		 		else {;}
		 		 break;
		 	 }
		 	 case 0x06:           	// command tag for servo1 connected to M640 move with specified degrees
		 	 {
		 		 PanAngle = data_byte1;
		 		 ServoUpdate = 1;
		 		 ServoType = PAN;
		 		 break;
		 	 }
		 	 case 0x07:           	// command tag for servo2 connected to M640 move with specified degrees
		 	 {
		 		 TiltAngle = data_byte1;
		 		 ServoUpdate = 1;
		 		 ServoType = TILT;
		 		 break;
		 	 }
		 	 case 0x08:           	// command tag for servo3 connected to M640 move with specified degrees
		 	 {
		 		 AuxAngle = data_byte1;
		 		 ServoUpdate = 1;
		 		 ServoType = AUX;
		 		 break;
		 	 }
		 	 case 0x09:           	// All Sensor Trigger
		 	 {
		 		 if(data_byte1 == 0x01)				//IR Proximity Trigger ON
		 		 {
		 			 IRTrigger=1;
		 			 IRTriggerState=1;
		 		 }
		 		 else if(data_byte1 == 0x02)		//IR Proximity Trigger OFF
		 		 {
		 			 IRTrigger=1;
		 			 IRTriggerState=0;
		 		 }
		 		 else if(data_byte1 == 0x03)		//WL Trigger ON
		 		 {
		 			 WLTrigger = 1;
		 			 WLTriggerState=1;
		 		 }
		 		 else if(data_byte1 == 0x04)		//WL Trigger OFF
		 		 {
		 			 WLTrigger = 1;
		 			 WLTriggerState=0;
		 		 }
		 		 else if(data_byte1 == 0x05)		//Ultrasonic Trigger ON
		 		 {
		 			 ULTrigger=1;
		 			 ULTriggerState=1;
		 		 }
		 		 else if(data_byte1 == 0x06)		//Ultrasonic Trigger OFF
		 		 {
		 			 ULTrigger=1;
		 			 ULTriggerState=0;
		 		 }
		 		 else if(data_byte1 == 0x07)			//Servo Pod Ultrasonic Trigger ON
		 		 {
		 			 UL_TRIG_ON();
		 			 ServoPodULTriggerState = 1;
		 		 }
		 		 else if(data_byte1 == 0x08)			//Servo Pod Ultrasonic Trigger OFF
		 		 {
		 			 UL_TRIG_OFF();
		 			 ServoPodULTriggerState = 0;
		 		 }
		 		 else {;}
		 		 break;
		 	 }
		 	/* case 'S':
		 	 {
		 		WLTrigger=1;
		 		WLTriggerState=1;
		 		break;
		 	 }*/
		 	 case 0x0A:           	// Get sensor trigger status
		 	 {
		 		 if(data_byte1 == 0x01)			//IR Proximity
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 Tx_Buffer[5] = IRTriggerState;
		 			 Len = 6;
		 			 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		 else if(data_byte1 == 0x02)	//WhiteLine
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 Tx_Buffer[5] = WLTriggerState;
		 			 Len = 6;
		 			 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		else if(data_byte1 == 0x03)	//Sensor Board Ultrasonic
		 		{
		 			Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			Tx_Buffer[5] = ULTriggerState;
		 			Len = 6;
		 			UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}
		 		else if(data_byte1 == 0x04)	//Servo POD Ultrasonic
		 		{
		 			Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			Tx_Buffer[5] = ServoPodULTriggerState;
		 			Len = 6;
		 			UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}

		 		 else {;}
		 		 break;
		 	 }

		 	 /*******************************ADDED***********************************************/
		 	case 0x0B:           	// SWITCH STATUS
		 	{
		 		Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		Tx_Buffer[3] = My_Button_Status;/* Tx_Buffer[4] = data_byte1;
		 		Tx_Buffer[5] = IRTriggerState;*/
		 		Len = 4;
		 		UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		My_Button_Status = data_byte1&0xf0;
		 		break;
		 	}

		 	case 0x0C:           	// LASER COMMAND
		 	{
		 		if (data_byte1 == 0)
		 			LASER_TURN_ON();
		 		else
		 			LASER_TURN_OFF();
		 		break;
		 	}
		 	case 0x0D:           	// MOTOR COMMAND
		 	{
		 		if (data_byte1 == 0)
		 			MOTOR_TURN_ON();
		 		else if (data_byte1 == 1)
		 			MOTOR_TURN_OFF();
		 		else if (data_byte1 == 2)
		 			MOTOR1_TURN_ON();
		 		else if (data_byte1 == 3)
		 			MOTOR1_TURN_OFF();
		 		else if (data_byte1 == 4)
		 			MOTOR2_TURN_ON();
		 		else if (data_byte1 == 5)
		 			MOTOR2_TURN_OFF();
		 		else if (data_byte1 == 6)
		 			MOTOR3_TURN_ON();
		 		else if (data_byte1 == 7)
		 			MOTOR3_TURN_OFF();
		 		break;
		 	}
		 	/***********************************************************************************/

		 	 case 0x10:
		 	 {
		 		 if(data_byte1==0x01)			//	Get Acc X axis data
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 Tx_Buffer[5] = Accelerometer[0];
		 			 Tx_Buffer[6] = Accelerometer[1];
		 			 Len = 7;
		 			 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		 else if(data_byte1==0x02)		//	Get Acc Y axis data
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 Tx_Buffer[5] = Accelerometer[2];
		 			 Tx_Buffer[6] = Accelerometer[3];
		 			 Len = 7;
		 			 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		 else if(data_byte1==0x03)		//	Get Acc Z axis data
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 Tx_Buffer[5] = Accelerometer[4];
		 			 Tx_Buffer[6] = Accelerometer[5];
		 			 Len = 7;
		 			 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		 else;
		 		 break;
		 	 }
		 	 case 0x11:
		 	 {
		 		 if(data_byte1==0x01)				//	Get Gyro X axis data
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 Tx_Buffer[5] = Gyroscope[0];
		 			 Tx_Buffer[6] = Gyroscope[1];
		 			 Len = 7;
		 			 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		 else if(data_byte1==0x02)			//	Get Gyro Y axis data
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 Tx_Buffer[5] = Gyroscope[2];
		 			 Tx_Buffer[6] = Gyroscope[3];
		 			 Len = 7;
		 			 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		 else if(data_byte1==0x03)			//	Get Gyro Z axis data
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 Tx_Buffer[5] = Gyroscope[4];
		 			 Tx_Buffer[6] = Gyroscope[5];
		 			 Len = 7;
		 			 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		 else;
		 		 break;
		 	 }

		 	 case 0x12:
		 	 {
		 		 if(data_byte1==0x01)				//	Get Mag X axis data
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 Tx_Buffer[5] = Magnetometer[0];
		 			 Tx_Buffer[6] = Magnetometer[1];
		 			 Len = 7;
		 			 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		 else if(data_byte1==0x02)			//	Get Mag Y axis data
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 Tx_Buffer[5] = Magnetometer[2];
		 			 Tx_Buffer[6] = Magnetometer[3];
		 			 Len = 7;
		 			 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		 else if(data_byte1==0x03)			//	Get Mag Z axis data
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 Tx_Buffer[5] = Magnetometer[4];
		 			 Tx_Buffer[6] = Magnetometer[5];
		 			 Len = 7;
		 			 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		 else;
		 		 break;
		 	 }

		 	 case 0x20:           	// command tag for battery
		 	 {
		 		 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 		 Tx_Buffer[5] = Battery[0];
		 		 Len = 6;
		 		 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 break;
		 	 }
		 	 case 0x21:           	// command tag for current sensor
		 	 {
		 		 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 		 Tx_Buffer[5] = Battery[1];
		 		 Len = 6;
		 		 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 break;
		 	 }
		 	 case 0x22:           	// command tag for Temperature sensor
		 	 {
		 		 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 		 Tx_Buffer[5] = Battery[2];
		 		 Len = 6;
		 		 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 break;
		 	 }

		 	 case 0x23:           	// Read Battery Voltage, Current and Temperature
		 	 {
		 		 if(data_byte1 == 0x00)            // command tag for all battery voltage data reading
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 Tx_Buffer[5] = Battery[0];		// Voltage
		 			 Tx_Buffer[6] = Battery[1];		//Current
		 			 Tx_Buffer[7] = Battery[2];		//Temperature
		 			 Len = 8;
		 			 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		 break;
		 	 }

		 	 case 0x24:           	// command tag for all sensor group reading
		 	 {
		 		 //if(data_byte1 == 0x01)			//Ultrasonic
		 		 {
		 			 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 for(i=0;i<8;i++)
		 			 {
		 				 Tx_Buffer[i+UL_OFFSET] = Ultrasonic[i];
		 			 }
		 			 Len = UL_OFFSET;
		 			 //UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		 //else if(data_byte1 == 0x02)	//IR Proximity
		 		 {
		 			 //Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 //Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 for(i=0;i<8;i++)
		 			 {
		 				 Tx_Buffer[i+IR_OFFSET] = IR_Proximity[i];
		 			 }
		 			 Len += IR_OFFSET;
		 			 //UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		 //else if(data_byte1 == 0x04)	// White Line
		 		 {
		 			 //Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			 //Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			 for(i=0;i<8;i++)
		 			 {
		 				 Tx_Buffer[i+WL_OFFSET] = Whiteline[i];
		 			 }
		 			 Len += WL_OFFSET;
		 			 //UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 }
		 		//else if(data_byte1 == 0x05)		// Accelerometer
		 		{
		 			//Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			//Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			for(i=0;i<6;i++)
		 			{
		 				Tx_Buffer[i+ACC_OFFSET] = Accelerometer[i];
		 			}
		 			Len += ACC_OFFSET;
		 			//UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}
		 		//else if(data_byte1 == 0x06)		// Gyroscope
		 		{
		 			//Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			//Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			for(i=0;i<6;i++)
		 			{
		 				Tx_Buffer[i+GYRO_OFFSET] = Gyroscope[i];
		 			}
		 			Len += GYRO_OFFSET;
		 			//UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}
		 		//else if(data_byte1 == 0x07)		// Magnetometer
		 		{
		 			//Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			//Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			for(i=0;i<6;i++)
		 			{
		 				Tx_Buffer[i+MAGNETO_OFFSET] = Magnetometer[i];
		 			}
		 			Len += MAGNETO_OFFSET;
		 			//UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}
		 		//else if(data_byte1 == 0x08)		// AD7998 ADC
		 		{
		 			//Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			//Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			//for(i=0;i<16;i++)
		 			//{
		 			//	Tx_Buffer[i+AD7998_OFFSET] = AD7998ADC[i];
		 			//}
		 			//Len += AD7998_OFFSET;
		 			//UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}
		 		//else if(data_byte1 == 0x09)		// POT
		 		{
		 			//Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			//Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			Tx_Buffer[POT_OFFSET] = PotValue & 0x00FF;
		 			Tx_Buffer[POT_OFFSET + 1] = PotValue >> 8;
		 			Len += POT_OFFSET;
		 			//UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}
		 		//else if(data_byte1 == 0x0A)		// Servo Pod Ultrasonic
		 		{
		 			//Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			//Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			Tx_Buffer[UL_SERVO_OFFSET] = ServoPOD_ADC & 0x00FF;
		 			Tx_Buffer[UL_SERVO_OFFSET + 1] = ServoPOD_ADC >> 8;
		 			Len += UL_SERVO_OFFSET;
		 			//UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}
				//if(data_byte1 == 0x00)            // command tag for all battery voltage data reading
				{
					//Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
					//Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			for(i=0;i<3;i++)
		 			{
		 				Tx_Buffer[i+BATTERY_OFFSET] = Battery[i];
		 			}
					Len += BATTERY_OFFSET;
					//UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
				}
				Len = BATTERY_OFFSET + 3; // battery data length
				UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);

		 		//else {;}
		 		 break;
		 	 }

		 	 case 0x25:
		 	 {
		 		updateRemote_2_4GhzCommand = 1;
		 	 }

		 	 case 0x89:		// Set Safety mode ON / OFF
		 	 {
		 		 Safety = data_byte1;
		 		 UpdateSafety = 1;
		 		 break;
		 	 }

		 	 case 0x8A:		//Set Max Velocity
		 	 {

		 		 break;
		 	 }

		 	 case 0x8B:		//Get Max Velocity
		 	 {

		 		 break;
		 	 }

		 	 case 0x8C:		//Reset Counts
		 	 {
		 		 //ClearEncoderCounts();
		 		 ResetEncoderCounts = 1;
		 		 break;
		 	 }

		 	 case 0x90:		//Set Motion control mode
		 	 {
		 		 Mode = data_byte1;
		 		 UpdateMode = 1;
		 		 break;
		 	 }

		 	 case 0x91:		//Get Motion control mode
		 	 {
				 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
				 Tx_Buffer[3] = cmd_byte;
				 Tx_Buffer[4] = Mode;
		 		 Len = 5;
		 		 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);

		 		 break;
		 	 }

		 	 case 0x92:		//Get Left Motor Count
		 	 {
		 		 Temp = Left_Count_New_Locked;
		 		 Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		 Tx_Buffer[3] = cmd_byte;
		 		 Tx_Buffer[4] = 0x00;
		 		 Tx_Buffer[5] = ((Temp) >> 24) & 0x000000FF;
		 		 Tx_Buffer[6] = ((Temp) >> 16) & 0x000000FF;
		 		 Tx_Buffer[7] = ((Temp) >> 8) & 0x000000FF;
		 		 Tx_Buffer[8] = Temp  & 0x000000FF;
		 		 Len = 9;
		 		 UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		 break;
		 	 }

		 	case 0x93:		//Get Right Motor Count
		 	{
		 		Temp = Right_Count_New_Locked;
		 		Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		Tx_Buffer[3] = cmd_byte;
		 		Tx_Buffer[4] = 0x00;
		 		Tx_Buffer[5] = ((Temp) >> 24) & 0x000000FF;
		 		Tx_Buffer[6] = ((Temp) >> 16) & 0x000000FF;
		 		Tx_Buffer[7] = ((Temp) >> 8) & 0x000000FF;
		 		Tx_Buffer[8] = Temp  & 0x000000FF;
		 		Len = 9;
		 		UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		break;
		 	}

		 	case 0x94:		//Motor Control FBRL and Stop
		 	{
		 		if     (data_byte1==1) {Direction=FORWARD; MotorUpdate=1;}
		 		else if(data_byte1==2) {Direction=REVERSE; MotorUpdate=1;}
		 		else if(data_byte1==3) {Direction=LEFT;    MotorUpdate=1;}
		 		else if(data_byte1==4) {Direction=RIGHT;   MotorUpdate=1;}
		 		else if(data_byte1==6) {Direction=STOP;    MotorUpdate=1;}
		 		break;
		 	}

		 	case 0x95:		//Left Motor Velocity
		 	{
		 		Left_Velocity=data_byte1;
		 		break;
		 	}

		 	case 0x96:		//Right Motor Velocity
		 	{
		 		Right_Velocity=data_byte1;
		 		break;
		 	}


		 	case 0x97:		//Motor current
		 	{
		 		if(data_byte1==1)			//Left Motor
		 		{
		 			Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			Tx_Buffer[5] = LeftMotorCurrent;
		 			Len = 6;
		 			UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}
		 		else if(data_byte1==2)	//Right Motor
		 		{
		 			Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 			Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 			Tx_Buffer[5] = RightMotorCurrent;
		 			Len = 6;
		 			UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		}
		 		break;
		 	}

		 	case 0x98:		//Set Acceleration
		 	{
		 		AccUpdate=1;
		 		Acc=data_byte1;
		 		break;
		 	}

		 	case 0x99:		//Get Acceleration
		 	{
		 		Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 		Tx_Buffer[5] = Acc;
		 		Len = 6;
		 		UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		break;
		 	}

		 	case 0x9A:		//Get Left Motor Velocity
		 	{
		 		Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 		Tx_Buffer[5] = Left_Velocity;
		 		Len = 6;
		 		UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		break;
		 	}

		 	case 0x9B:		//Get Right Motor Velocity
		 	{
		 		Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 		Tx_Buffer[5] = Right_Velocity;
		 		Len = 6;
		 		UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		break;
		 	}

		 	case 0x9C:		//Set Position
		 	{
		 		positionLeft = 0;
		 		positionRight = 0;

		 		positionLeft |= (positionData[0]<<24);
		 		positionLeft |= (positionData[1]<<16);
		 		positionLeft |= (positionData[2]<<8);
		 		positionLeft |= (positionData[3]);

		 		positionRight |= (positionData[5]<<24);
		 		positionRight |= (positionData[6]<<16);
		 		positionRight |= (positionData[7]<<8);
		 		positionRight |= (positionData[8]);


		 		velocityLeft = (int8_t)positionData[4];

		 		velocityRight = (int8_t)positionData[9];

		 		updatePosition = 1;
		 		break;
		 	}

		 	case 0xAA:		//Get Servo PAN angle
		 	{
		 		Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 		Tx_Buffer[5] = PanAngle;
		 		Len = 6;
		 		UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		break;
		 	}

		 	case 0xAB:		//Get Servo Tilt angle
		 	{
		 		Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		Tx_Buffer[3] = cmd_byte; Tx_Buffer[4] = data_byte1;
		 		Tx_Buffer[5] = TiltAngle;
		 		Len = 6;
		 		UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		break;
		 	}


		 	case 0xFA:		//ID
		 	{
		 		Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		Tx_Buffer[3] = cmd_byte;
		 		Tx_Buffer[4] = ID;
		 		Len = 5;
		 		UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		break;
		 	}

		 	case 0xFB:		//Get Hardware version
		 	{
		 		Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		Tx_Buffer[3] = cmd_byte;
		 		Tx_Buffer[4] = HW_VER;
		 		Len = 5;
		 		UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		break;
		 	}

		 	case 0xFC:		//Get Software version
		 	{
		 		Tx_Buffer[0] = 'F'; Tx_Buffer[1] = 'B'; Tx_Buffer[2] = 'D';
		 		Tx_Buffer[3] = cmd_byte;
		 		Tx_Buffer[4] = SW_VER;
		 		Len = 5;
		 		UARTSend(portNum,(uint8_t *)Tx_Buffer,Len);
		 		break;
		 	}

		 	default:
		 		break;

		 }
		 packet_valid = 0;
		 packet_data_count = 1;
	 }
}

/******************************************************************************
**                            End Of File
******************************************************************************/
