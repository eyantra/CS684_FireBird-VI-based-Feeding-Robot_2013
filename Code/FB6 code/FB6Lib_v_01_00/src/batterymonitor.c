/*
===============================================================================
 File Name	 : batterymonitor.c
 Author      : Nex Robotics Pvt Ltd
 Version     :
 Copyright   : Copyright (C) 2012
 Description : This file contains APIs to read and write data to and from
 	 	 	   Power Management module.
===============================================================================
*/
#include "LPC17xx.h"
#include "type.h"
#include "i2c.h"
#include "HardwareDefinition.h"
#include "Hardwareprofile.h"
#include "batterymonitor.h"

#ifdef BATTERYMONITOR

/*===============================================================================
 Function        : Battery_Monitor_WriteReg();
 Parameters		 : 	 Addr: 1 byte address where data will be written
 	 	 	 	 	 Data: Data to be written
 Description 	 : Sends 1 byte of data to power management module
 Preconditions	 : uncomment BATTERYMONITOR definition in Hardwareprofile.h
===============================================================================*/
uint32_t Battery_Monitor_WriteReg(uint8_t Addr, uint8_t Data)
{
	uint32_t Status;
	I2CWriteLength[BATTERY_MONITOR_PORT] = 3;
	I2CReadLength[BATTERY_MONITOR_PORT] = 0;
	I2CMasterBuffer[BATTERY_MONITOR_PORT][0] = BATTERY_MONITOR_ADDR | BATTERY_MONITOR_WRITE;
	I2CMasterBuffer[BATTERY_MONITOR_PORT][1] = Addr;		/* address */
	I2CMasterBuffer[BATTERY_MONITOR_PORT][2] = Data;		/* Data 0 */
	Status = I2CEngine( BATTERY_MONITOR_PORT );
	return (Status);
}


/*===============================================================================
 Function        : Battery_Monitor_ReadReg();
 Parameters		 : 	 Addr: 1 byte address from where data will be read
 	 	 	 	 	 Length: Length of data to be read
 Description 	 : Reads n number of bytes from power management module
 Preconditions	 : uncomment BATTERYMONITOR definition in Hardwareprofile.h
===============================================================================*/
uint32_t Battery_Monitor_ReadReg(uint8_t Addr, uint8_t Length)
{
	uint32_t Status;
	I2CWriteLength[BATTERY_MONITOR_PORT] = 2;
	I2CReadLength[BATTERY_MONITOR_PORT] = Length;
	I2CMasterBuffer[BATTERY_MONITOR_PORT][0] = BATTERY_MONITOR_ADDR | BATTERY_MONITOR_WRITE;
	I2CMasterBuffer[BATTERY_MONITOR_PORT][1] = Addr;		/* address */
	I2CMasterBuffer[BATTERY_MONITOR_PORT][2] = BATTERY_MONITOR_ADDR | BATTERY_MONITOR_READ;
	Status = I2CEngine( BATTERY_MONITOR_PORT );
	return (Status);
}


/*===============================================================================
 Function        : Get_Battery_Status();
 Parameters		 : *Data -> Pointer to store the data
 Description 	 : Gets 3 bytes of data from power management module.
 	 	 	 	   Byte0 = Voltage
 	 	 	 	   Byte1 = Current
 	 	 	 	   Byte2 = Temperature
 Preconditions	 : uncomment BATTERYMONITOR definition in Hardwareprofile.h
===============================================================================*/
uint32_t Get_Battery_Status(uint8_t *Data)
{
	uint8_t i=0;
	uint32_t Status=FALSE;
	I2CSlaveBuffer[BATTERY_MONITOR_PORT][0] = 0x00;
	I2CSlaveBuffer[BATTERY_MONITOR_PORT][1] = 0x00;
	I2CSlaveBuffer[BATTERY_MONITOR_PORT][2] = 0x00;
	if(Battery_Monitor_ReadReg(BATTERY_STATUS,3)==I2C_OK)
	{
		for(i=0;i<3;i++)
		{
			Data[i] = I2CSlaveBuffer[BATTERY_MONITOR_PORT][i];
		}
		Status = TRUE;
	}
	Delay();
	return(Status);
}
#endif
