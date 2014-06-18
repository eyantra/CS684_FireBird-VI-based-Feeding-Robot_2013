/*
===============================================================================
 Name        : sensorboard.c
 Author      : Nex Robotics Pvt Ltd
 Version     : 1.0
 Copyright   : Copyright (C) 2012
 Description : This file contains APIs to exchange data with the sensor module.
===============================================================================
*/
#include <stdint.h>
#include "LPC17xx.h"
#include "type.h"
#include "i2c.h"
#include "HardwareDefinition.h"
#include "Hardwareprofile.h"
#include "sensorboard.h"

volatile uint8_t IRTrigger=0;
volatile uint8_t WLTrigger=0;
volatile uint8_t ULTrigger=0;
volatile uint8_t IRTriggerState=0;
volatile uint8_t WLTriggerState=0;
volatile uint8_t ULTriggerState=0;
volatile uint8_t ServoUpdate = 0;
volatile uint8_t ServoType = 0;


#ifdef SENSORBOARD


/*===============================================================================
 Function        : Sensor_Board_WriteReg();
 Parameters		 : 	 Addr: 1 byte address where data will be written
 	 	 	 	 	 Data: Data to be written
 Description 	 : Sends 1 byte of data to Sensor Module
 Preconditions	 : None
===============================================================================*/
uint32_t Sensor_Board_WriteReg(uint8_t Addr, uint8_t Data)
{
	uint32_t Status;
	I2CWriteLength[SENSOR_BOARD_PORT] = 3;
	I2CReadLength[SENSOR_BOARD_PORT] = 0;
	I2CMasterBuffer[SENSOR_BOARD_PORT][0] = SENSOR_BOARD_ADDR | SENSOR_BOARD_WRITE;
	I2CMasterBuffer[SENSOR_BOARD_PORT][1] = Addr;		/* address */
	I2CMasterBuffer[SENSOR_BOARD_PORT][2] = Data;		/* Data 0 */
	Status = I2CEngine( SENSOR_BOARD_PORT );
	return (Status);
}


/*===============================================================================
 Function        : Sensor_Board_ReadReg();
 Parameters		 : 	 Addr: 1 byte address from where data will be read
 	 	 	 	 	 Length: Length of data to be read
 Description 	 : Reads n number of bytes from sensor module
 Preconditions	 : None
===============================================================================*/
uint32_t Sensor_Board_ReadReg(uint8_t Addr, uint8_t Length)
{
	uint32_t Status;
	I2CWriteLength[SENSOR_BOARD_PORT] = 2;
	I2CReadLength[SENSOR_BOARD_PORT] = Length;
	I2CMasterBuffer[SENSOR_BOARD_PORT][0] = SENSOR_BOARD_ADDR | SENSOR_BOARD_WRITE;
	I2CMasterBuffer[SENSOR_BOARD_PORT][1] = Addr;		/* address */
	I2CMasterBuffer[SENSOR_BOARD_PORT][2] = SENSOR_BOARD_ADDR | SENSOR_BOARD_READ;
	Status = I2CEngine( SENSOR_BOARD_PORT );
	return (Status);
}

#ifdef ULTRASONIC

/*===============================================================================
 Function        : Get_Ultrasonic_Data();
 Parameters		 : *Data -> Pointer to store the data
 Description 	 : Gets 8 bytes of Ultrasonic sensor data from sensor module.
 	 	 	 	   Byte0 = Ultrasonic sensor 1 ... Byte 7 = Ultrasonic Sensor 8
 Preconditions	 : uncomment ULTRASONIC definition in Hardwareprofile.h
===============================================================================*/
uint32_t Get_Ultrasonic_Data(uint8_t *Data)
{
	uint8_t i=0;
	uint32_t Status=FALSE;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][0] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][1] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][2] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][3] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][4] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][5] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][6] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][7] = 0x00;

	if(Sensor_Board_ReadReg(READ_UL_ALL,8)==I2C_OK)
	{
		for(i=0;i<8;i++)
		{
			Data[i] = I2CSlaveBuffer[SENSOR_BOARD_PORT][i];
		}
		Status = TRUE;
	}
	Delay();
	return(Status);
}


/*===============================================================================
 Function        : Set_UL_Trigger();
 Parameters		 : Data -> 0 = Ranging OFF, 1 = Ranging ON
 Description 	 : This function controls the ranging of  8 ultrasnoic sensors
 	 	 	 	   surrounding the robot.
 Preconditions	 : uncomment ULTRASONIC definition in Hardwareprofile.h
===============================================================================*/
uint32_t Set_UL_Trigger(uint8_t Data)
{
	uint32_t Status=FALSE;
	if(Sensor_Board_WriteReg(SET_UL_TRIGGER,Data)==I2C_OK)
	{
		Status=TRUE;
	}
	return(Status);
}


/*===============================================================================
 Function        : ULTriggerUpdate();
 Parameters		 : None
 Description 	 : This function is called in AcquireData(); function in main.c file
 	 	 	 	   to update ultrasonic sensor trigger status and control the ranging.
 	 	 	 	   This function is useful when trigger commands are issued by high level device
 	 	 	 	   like PC or other embedded kit where command are received in UART ISR.
 	 	 	 	   For e.g., if LPC1769 is in the middle of acquiring ultrasonic data
 	 	 	 	   from sensor module and if user requests to turn OFF ranging
 	 	 	 	   from the ISR function will result in to unexpected behaviour.
 Preconditions	 : uncomment ULTRASONIC definition in Hardwareprofile.h
===============================================================================*/
void ULTriggerUpdate(void)
{
	if(ULTrigger)
	{
		if(ULTriggerState==1)
		{
			Set_UL_Trigger(0x01);
		}
		else if(ULTriggerState==0)
		{
			Set_UL_Trigger(0x00);
		}
	}
}

#endif	//ULTRASONIC

#ifdef WHITELINE


/*===============================================================================
 Function        : Get_Whiteline_Data();
 Parameters		 : *Data -> Pointer to store the data
 Description 	 : Gets 8 bytes of Line sensor data from sensor module.
 	 	 	 	   Byte0 = Line sensor 1 ... Byte 7 = Line Sensor 8
 Preconditions	 : uncomment WHITELINE definition in Hardwareprofile.h
===============================================================================*/
uint32_t Get_Whiteline_Data(uint8_t *Data)
{
	uint8_t i=0;
	uint32_t Status=FALSE;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][0] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][1] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][2] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][3] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][4] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][5] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][6] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][7] = 0x00;

	if(Sensor_Board_ReadReg(READ_WL_ALL,8)==I2C_OK)
	{
		for(i=0;i<8;i++)
		{
			Data[i] = I2CSlaveBuffer[SENSOR_BOARD_PORT][i];
		}
		Status = TRUE;
	}
	Delay();
	return(Status);

}


/*===============================================================================
 Function        : Set_WL_Trigger();
 Parameters		 : Data -> 0 = LED OFF, 1 = LED ON
 Description 	 : This function controls the LED of  8 line sensors.
 Preconditions	 : uncomment WHITELINE definition in Hardwareprofile.h
===============================================================================*/
uint32_t Set_WL_Trigger(uint8_t Data)
{
	uint32_t Status=FALSE;
	if(Sensor_Board_WriteReg(SET_WL_TRIGGER,Data)==I2C_OK)
	{
		Status=TRUE;
	}
	return(Status);
}


/*===============================================================================
 Function        : WLTriggerUpdate();
 Parameters		 : None
 Description 	 : This function is called in AcquireData(); function in main.c file
 	 	 	 	   to LEDs of line sensors.
 	 	 	 	   This function is useful when trigger commands are issued by high level device
 	 	 	 	   like PC or other embedded kit where command are received in UART ISR.
 	 	 	 	   For e.g., if LPC1769 is in the middle of acquiring ultrasonic data
 	 	 	 	   from sensor module and if user requests to turn OFF ranging
 	 	 	 	   from the ISR function will result in to unexpected behaviour.
 Preconditions	 : uncomment WHITELINE definition in Hardwareprofile.h
===============================================================================*/
void WLTriggerUpdate(void)
{
	if(WLTrigger)
	{
		if(WLTriggerState==1)
		{
			Set_WL_Trigger(0x01);
		}
		else if(WLTriggerState==0)
		{
			Set_WL_Trigger(0x00);
		}
	}
}

#endif	//WHITELINE


#ifdef IR_PROXIMITY


/*===============================================================================
 Function        : Get_IR_Proximity_Data();
 Parameters		 : *Data -> Pointer to store the data
 Description 	 : Gets 8 bytes of Line sensor data from sensor module.
 	 	 	 	   Byte0 = IR sensor 1 ... Byte 7 = IR Sensor 8
 Preconditions	 : uncomment IR_PROXIMITY definition in Hardwareprofile.h
===============================================================================*/
uint32_t Get_IR_Proximity_Data(uint8_t *Data)
{
	uint8_t i=0;
	uint32_t Status=FALSE;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][0] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][1] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][2] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][3] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][4] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][5] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][6] = 0x00;
	I2CSlaveBuffer[SENSOR_BOARD_PORT][7] = 0x00;

	if(Sensor_Board_ReadReg(READ_IR_ALL,8)==I2C_OK)
	{
		for(i=0;i<8;i++)
		{
			Data[i] = I2CSlaveBuffer[SENSOR_BOARD_PORT][i];
		}
		Status = TRUE;
	}
	Delay();
	return(Status);

}


/*===============================================================================
 Function        : Set_IR_Trigger();
 Parameters		 : Data -> 0 =  IR LED OFF, 1 = IR LED ON
 Description 	 : This function controls the IR LED of 8 proximity sensors.
 Preconditions	 : uncomment IR_PROXIMITY definition in Hardwareprofile.h
===============================================================================*/
uint32_t Set_IR_Trigger(uint8_t Data)
{
	uint32_t Status=FALSE;
	if(Sensor_Board_WriteReg(SET_IR_TRIGGER,Data)==I2C_OK)
	{
		Status=TRUE;
	}
	return(Status);
}


/*===============================================================================
 Function        : IRTriggerUpdate();
 Parameters		 : None
 Description 	 : This function is called in AcquireData(); function in main.c file
 	 	 	 	   to control IR LED of proximity sensors.
 	 	 	 	   This function is useful when trigger commands are issued by high level device
 	 	 	 	   like PC or other embedded kit where command are received in UART ISR.
 	 	 	 	   For e.g., if LPC1769 is in the middle of acquiring ultrasonic data
 	 	 	 	   from sensor module and if user requests to turn OFF ranging
 	 	 	 	   from the ISR function will result in to unexpected behaviour.
 Preconditions	 : uncomment IR_PROXIMITY definition in Hardwareprofile.h
===============================================================================*/
void IRTriggerUpdate(void)
{
	if(IRTrigger)
	{
		if(IRTriggerState==1)
		{
			Set_IR_Trigger(0x01);
		}
		else if(IRTriggerState==0)
		{
			Set_IR_Trigger(0x00);
		}
	}
}
#endif	//IR_PROXIMITY


#ifdef SERVOPOD


/*===============================================================================
 Function        : UpdateServoPos();
 Parameters		 : None
 Description 	 : This function is called in AcquireData(); function in main.c file
 	 	 	 	   to update position of servo motors.
 	 	 	 	   This function is useful when trigger commands are issued by high level device
 	 	 	 	   like PC or other embedded kit where command are received in UART ISR.
 	 	 	 	   For e.g., if LPC1769 is in the middle of acquiring ultrasonic data
 	 	 	 	   from sensor module and if user requests to turn OFF ranging
 	 	 	 	   from the ISR function will result in to unexpected behaviour.
 Preconditions	 : uncomment SERVOPOD definition in Hardwareprofile.h
===============================================================================*/
void UpdateServoPos(uint8_t Angle, uint8_t Servo)
{
	float pwmValue=0;

	if(Angle>180)
		Angle = 180;

	pwmValue = ((float)Angle/1.76) + 36.0;

	switch (Servo)
	{
		case PAN: {Sensor_Board_WriteReg(PAN_SERVO,(uint8_t)pwmValue);break;}
		case TILT:{Sensor_Board_WriteReg(TILT_SERVO,(uint8_t)pwmValue);break;}
		case AUX: {Sensor_Board_WriteReg(AUX_SERVO,(uint8_t)pwmValue);break;}
		default:
			break;
	}

}

#endif		//SERVOPOD

#ifdef EXT_ADC

/*===============================================================================
 Function        : AD7998_WriteReg();
 Parameters		 : 	 Addr: 1 byte address where data will be written
 	 	 	 	 	 Data: Data to be written
 Description 	 : Sends 1 byte of data to External ADC chip
 Preconditions	 : None
===============================================================================*/
uint32_t AD7998_WriteReg(uint8_t Addr, uint16_t Data)
{
	uint32_t Status;
	I2CWriteLength[AD7998_PORT] = 4;
	I2CReadLength[AD7998_PORT] = 0;
	I2CMasterBuffer[AD7998_PORT][0] = AD7998_ADDR | AD7998_WRITE;
	I2CMasterBuffer[AD7998_PORT][1] = Addr;		/* address */
	I2CMasterBuffer[AD7998_PORT][2] = Data >> 8;		/* Data 0 */
	I2CMasterBuffer[AD7998_PORT][3] = Data & 0x00FF;

	Status = I2CEngine( AD7998_PORT );
	return (Status);
}


/*===============================================================================
 Function        : AD7998_Board_ReadReg();
 Parameters		 : 	 Addr: 1 byte address from where data will be read
 	 	 	 	 	 Length: Length of data to be read
 Description 	 : Reads n number of bytes from External ADC chip
 Preconditions	 : None
===============================================================================*/
uint32_t AD7998_Board_ReadReg(uint8_t Addr, uint8_t Length)
{
	uint32_t Status;
	I2CWriteLength[AD7998_PORT] = 2;
	I2CReadLength[AD7998_PORT] = Length;
	I2CMasterBuffer[AD7998_PORT][0] = AD7998_ADDR | AD7998_WRITE;
	I2CMasterBuffer[AD7998_PORT][1] = Addr;		/* address */
	I2CMasterBuffer[AD7998_PORT][2] = AD7998_ADDR | AD7998_READ;
	Status = I2CEngine( AD7998_PORT );
	return (Status);
}


/*===============================================================================
 Function        : Get_AD7998_Data();
 Parameters		 : *Data -> Pointer to store the data
 Description 	 : Gets 16 bytes of data from ADC.
 	 	 	 	   Byte0 = CH0_LSB & Byte 1 = CH0_MSB and so on.
 	 	 	 	   Refer AD7998 datasheet for more details on data format
 Preconditions	 : uncomment EXT_ADC definition in Hardwareprofile.h
===============================================================================*/
uint32_t Get_AD7998_Data(uint8_t *Data)
{
	uint8_t i=0;
	uint32_t Status=FALSE;
	I2CSlaveBuffer[AD7998_PORT][0] = 0x00;
	I2CSlaveBuffer[AD7998_PORT][1] = 0x00;
	I2CSlaveBuffer[AD7998_PORT][2] = 0x00;
	I2CSlaveBuffer[AD7998_PORT][3] = 0x00;
	I2CSlaveBuffer[AD7998_PORT][4] = 0x00;
	I2CSlaveBuffer[AD7998_PORT][5] = 0x00;
	I2CSlaveBuffer[AD7998_PORT][6] = 0x00;
	I2CSlaveBuffer[AD7998_PORT][7] = 0x00;
	I2CSlaveBuffer[AD7998_PORT][8] = 0x00;
	I2CSlaveBuffer[AD7998_PORT][9] = 0x00;
	I2CSlaveBuffer[AD7998_PORT][10] = 0x00;
	I2CSlaveBuffer[AD7998_PORT][11] = 0x00;
	I2CSlaveBuffer[AD7998_PORT][12] = 0x00;
	I2CSlaveBuffer[AD7998_PORT][13] = 0x00;
	I2CSlaveBuffer[AD7998_PORT][14] = 0x00;
	I2CSlaveBuffer[AD7998_PORT][15] = 0x00;

	if(AD7998_Board_ReadReg((MODE2_SEQUENTIAL | AD7998_CONVERSION_REG),16)==I2C_OK)
	{
		for(i=0;i<16;i++)
		{
			Data[i] = I2CSlaveBuffer[AD7998_PORT][i];
		}
		Status = TRUE;
	}
	return(Status);
}
#endif 		//EXT_ADC
#endif		//SENSORBOARD
