/****************************************************************************
 *   $Id:: uart.h 5751 2010-11-30 23:56:11Z usb00423                        $
 *   Project: NXP LPC17xx UART example
 *
 *   Description:
 *     This file contains UART code header definition.
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
#ifndef __UART_H 
#define __UART_H

#define IER_RBR		0x01
#define IER_THRE	0x02
#define IER_RLS		0x04

#define IIR_PEND	0x01
#define IIR_RLS		0x03
#define IIR_RDA		0x02
#define IIR_CTI		0x06
#define IIR_THRE	0x01

#define LSR_RDR		0x01
#define LSR_OE		0x02
#define LSR_PE		0x04
#define LSR_FE		0x08
#define LSR_BI		0x10
#define LSR_THRE	0x20
#define LSR_TEMT	0x40
#define LSR_RXFE	0x80

#define UART_BUFSIZE		0x40
#define WIRELESS_PORT		1
#define RS232_PORT			3

#define UL_OFFSET	5
#define IR_OFFSET UL_OFFSET + 8
#define WL_OFFSET IR_OFFSET + 8
#define ACC_OFFSET WL_OFFSET + 8
#define GYRO_OFFSET ACC_OFFSET + 6
#define MAGNETO_OFFSET GYRO_OFFSET + 6
//#define AD7998_OFFSET MAGNETO_OFFSET + 6
#define POT_OFFSET MAGNETO_OFFSET + 6
#define UL_SERVO_OFFSET POT_OFFSET + 2
#define BATTERY_OFFSET UL_SERVO_OFFSET + 2


uint32_t UARTInit( uint32_t portNum, uint32_t Baudrate );
void UART0_IRQHandler( void );
void UART1_IRQHandler( void );
void UART2_IRQHandler( void );
void UART3_IRQHandler( void );
void UARTSend( uint32_t portNum, uint8_t *BufferPtr, uint32_t Length );
void UARTDecode(uint8_t portNum,uint8_t ser_data);
void WirelessModuleDecode(uint8_t ser_data);
void GPSDecode(uint8_t data);


extern volatile uint8_t Tx_Buffer[UART_BUFSIZE];
extern volatile uint8_t Rx_Buffer[UART_BUFSIZE];
extern volatile uint8_t UART0Buffer[UART_BUFSIZE];
extern volatile uint8_t UART1Buffer[UART_BUFSIZE];
extern volatile uint8_t UART2Buffer[UART_BUFSIZE];
extern volatile uint8_t UART2_RxBuffer[UART_BUFSIZE];
extern volatile uint8_t UART2Flag;
extern volatile uint8_t RxReadLength;
extern volatile uint8_t gpsPacketRx;

#endif /* end __UART_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
