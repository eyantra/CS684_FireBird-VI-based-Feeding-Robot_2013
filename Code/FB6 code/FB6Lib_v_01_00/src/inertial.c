/*
===============================================================================
 Name        : inertial.c
 Author      : Nex Robotics Pvt Ltd
 Version     : 1.0
 Copyright   : Copyright (C) 2012
 Description : This file contains APIs to exchange data with IMU module of
 	 	 	   Fire Bird VI Robot. The 9 DOF IMU consists of L3G4200D 3 axis digital
 	 	 	   gyroscope and LSM303DLHC 3 axis digital accelerometer and magnetometer
===============================================================================
*/

#include "LPC17xx.h"
#include "type.h"
#include "i2c.h"
#include "HardwareDefinition.h"
#include "Hardwareprofile.h"
#include "inertial.h"

#ifdef INERTIAL


#ifdef GYROSCOPE

/*===============================================================================
 Function        : Init_L3G4200D();
 Parameters		 : None
 Description 	 : Initialises L3G4200D gyroscope module.
 	 	 	 	   It is automatically called in InitPeripherals() function in main.c,
 	 	 	 	   if GYROSCOPE is defined in Hardwareprofile.h
 Preconditions	 : uncomment GYROSCOPE definition in Hardwareprofile.h
===============================================================================*/
void Init_L3G4200D(void)
{
	L3G4200D_WriteReg(L3G4200D_CTRL_REG1,0x0F);	//ODR=100Hz, BW = 12.5,Normal Mode, XYZ enable
	L3G4200D_WriteReg(L3G4200D_CTRL_REG4,0x90);	//BDU update, 500DPS
	Delay();
}


/*===============================================================================
 Function        : L3G4200D_WriteReg();
 Parameters		 : 	 Addr: 1 byte address where data will be written
 	 	 	 	 	 Data: Data to be written
 Description 	 : Sends 1 byte of data to L3G4200D module
 Preconditions	 : uncomment GYROSCOPE definition in Hardwareprofile.h
===============================================================================*/
uint32_t L3G4200D_WriteReg(uint8_t Addr, uint8_t Data)
{
	uint32_t Status;
	I2CWriteLength[L3G4200D_PORT] = 3;
	I2CReadLength[L3G4200D_PORT] = 0;
	I2CMasterBuffer[L3G4200D_PORT][0] = L3G4200D_ADDR | L3G4200D_WRITE;
	I2CMasterBuffer[L3G4200D_PORT][1] = Addr;		/* address */
	I2CMasterBuffer[L3G4200D_PORT][2] = Data;		/* Data 0 */
	Status = I2CEngine( L3G4200D_PORT );
	return (Status);
}


/*===============================================================================
 Function        : L3G4200D_ReadReg();
 Parameters		 : 	 Addr: 1 byte address from where data will be read
 	 	 	 	 	 Length: Length of data to be read
 Description 	 : Reads n number of bytes from L3G4200D module
 Preconditions	 : uncomment GYROSCOPE definition in Hardwareprofile.h
===============================================================================*/
uint32_t L3G4200D_ReadReg(uint8_t Addr, uint8_t Length)
{
	uint32_t Status;
	I2CWriteLength[L3G4200D_PORT] = 2;
	I2CReadLength[L3G4200D_PORT] = Length;
	I2CMasterBuffer[L3G4200D_PORT][0] = L3G4200D_ADDR | L3G4200D_WRITE;
	I2CMasterBuffer[L3G4200D_PORT][1] = Addr;		/* address */
	I2CMasterBuffer[L3G4200D_PORT][2] = L3G4200D_ADDR | L3G4200D_READ;
	Status = I2CEngine( L3G4200D_PORT );
	return (Status);
}


/*===============================================================================
 Function        : Get_XYZ_Rate();
 Parameters		 : *Data ->Pointer to store the data
 Description 	 : Gets 7 bytes of data from L3G4200D module and updates the buffer
 	 	 	 	   only if new data is available by reading status byte.
 	 	 	 	   Byte0 	 = Status
 	 	 	 	   Byte1 & 2 = X lsb & X msb
 	 	 	 	   Byte3 & 4 = Y lsb & Y msb
 	 	 	 	   Byte5 & 6 = Z lsb & Z msb
 Preconditions	 : uncomment GYROSCOPE definition in Hardwareprofile.h
===============================================================================*/
uint32_t Get_XYZ_Rate(uint8_t *Data)
{
	uint8_t i=0;
	uint32_t Status=FALSE;
	I2CSlaveBuffer[L3G4200D_PORT][0] = 0x00;
	I2CSlaveBuffer[L3G4200D_PORT][1] = 0x00;
	I2CSlaveBuffer[L3G4200D_PORT][2] = 0x00;
	I2CSlaveBuffer[L3G4200D_PORT][3] = 0x00;
	I2CSlaveBuffer[L3G4200D_PORT][4] = 0x00;
	I2CSlaveBuffer[L3G4200D_PORT][5] = 0x00;
	I2CSlaveBuffer[L3G4200D_PORT][6] = 0x00;

	if(L3G4200D_ReadReg(0x80 | STATUS_REG,7)==I2C_OK)
	{
		if((I2CSlaveBuffer[L3G4200D_PORT][0] & 0x08)==0x08)
		{
			for(i=0;i<6;i++)
			{
				Data[i] = I2CSlaveBuffer[L3G4200D_PORT][i+1];
			}
			Status = TRUE;
		}

	}
	Delay();
	return(Status);
}
#endif	//GYROSCOPE


#ifdef ACCELEROMETER

/*===============================================================================
 Function        : Init_LSM303DLHC_Accelerometer();
 Parameters		 : None
 Description 	 : Initialises LSM303DLHC accelerometer module.
 	 	 	 	   It is automatically called in InitPeripherals() function in main.c,
 	 	 	 	   if ACCELEROMETER is defined in Hardwareprofile.h
 Preconditions	 : uncomment ACCELEROMETER definition in Hardwareprofile.h
===============================================================================*/
void Init_LSM303DLHC_Accelerometer(void)
{
	LSM303DLHC_Accelerometer_WriteReg(CTRL_REG1_A,0x97);			//Normal Mode, 1344HZ, XYZ enabled
	LSM303DLHC_Accelerometer_WriteReg(CTRL_REG4_A,0xA0);			//BDU update, Full scale selection +-8G
	Delay();
}


/*===============================================================================
 Function        : LSM303DLHC_Accelerometer_WriteReg();
 Parameters		 : 	 Addr: 1 byte address where data will be written
 	 	 	 	 	 Data: Data to be written
 Description 	 : Sends 1 byte of data to LSM303DLHC accelerometer module
 Preconditions	 : uncomment ACCELEROMETER definition in Hardwareprofile.h
===============================================================================*/
uint32_t LSM303DLHC_Accelerometer_WriteReg(uint8_t Addr, uint8_t Data)
{
	uint32_t Status;
	I2CWriteLength[LSM303DLHC_PORT] = 3;
	I2CReadLength[LSM303DLHC_PORT] = 0;
	I2CMasterBuffer[LSM303DLHC_PORT][0] = LSM303DLHC_ACC_ADDR | LSM303DLHC_WRITE;
	I2CMasterBuffer[LSM303DLHC_PORT][1] = Addr;		/* address */
	I2CMasterBuffer[LSM303DLHC_PORT][2] = Data;		/* Data 0 */
	Status = I2CEngine( LSM303DLHC_PORT );
	return (Status);
}


/*===============================================================================
 Function        : LSM303DLHC_Accelerometer_ReadReg();
 Parameters		 : 	 Addr: 1 byte address from where data will be read
 	 	 	 	 	 Length: Length of data to be read
 Description 	 : Reads n number of bytes from LSM303DLHC accelerometer module
 Preconditions	 : uncomment ACCELEROMETER definition in Hardwareprofile.h
===============================================================================*/
uint32_t LSM303DLHC_Accelerometer_ReadReg(uint8_t Addr, uint8_t Length)
{
	uint32_t Status;
	I2CWriteLength[LSM303DLHC_PORT] = 2;
	I2CReadLength[LSM303DLHC_PORT] = Length;
	I2CMasterBuffer[LSM303DLHC_PORT][0] = LSM303DLHC_ACC_ADDR | LSM303DLHC_WRITE;
	I2CMasterBuffer[LSM303DLHC_PORT][1] = Addr;		/* address */
	I2CMasterBuffer[LSM303DLHC_PORT][2] = LSM303DLHC_ACC_ADDR | LSM303DLHC_READ;
	Status = I2CEngine( LSM303DLHC_PORT );
	return (Status);
}

/*===============================================================================
 Function        : Get_XYZ_Acceleration();
 Parameters		 : *Data ->Pointer to store the data
 Description 	 : Gets 7 bytes of data from LSM303DLHC accelerometer module
 	 	 	 	   and updates the buffer only if new data is available by reading status byte.
 	 	 	 	   Byte0 	 = Status
 	 	 	 	   Byte1 & 2 = X lsb & X msb
 	 	 	 	   Byte3 & 4 = Y lsb & Y msb
 	 	 	 	   Byte5 & 6 = Z lsb & Z msb
 Preconditions	 : uncomment ACCELEROMETER definition in Hardwareprofile.h
===============================================================================*/
uint32_t Get_XYZ_Acceleration(uint8_t *Data)
{
	uint8_t i=0;
	uint32_t Status=FALSE;
	I2CSlaveBuffer[LSM303DLHC_PORT][0] = 0x00;
	I2CSlaveBuffer[LSM303DLHC_PORT][1] = 0x00;
	I2CSlaveBuffer[LSM303DLHC_PORT][2] = 0x00;
	I2CSlaveBuffer[LSM303DLHC_PORT][3] = 0x00;
	I2CSlaveBuffer[LSM303DLHC_PORT][4] = 0x00;
	I2CSlaveBuffer[LSM303DLHC_PORT][5] = 0x00;
	I2CSlaveBuffer[LSM303DLHC_PORT][6] = 0x00;

	if(LSM303DLHC_Accelerometer_ReadReg(0x80 | STATUS_REG_A,7)==I2C_OK)
	{
		if((I2CSlaveBuffer[LSM303DLHC_PORT][0] & 0x08)==0x08)
		{
			for(i=0;i<6;i++)
			{
				Data[i] = I2CSlaveBuffer[LSM303DLHC_PORT][i+1];
			}
			Status = TRUE;
		}

	}
	Delay();
	return(Status);
}
#endif	//ACCELEROMETER

#ifdef MAGNETOMETER

/*===============================================================================
 Function        : Init_LSM303DLHC_Magnetometer();
 Parameters		 : None
 Description 	 : Initialises LSM303DLHC magnetometer module.
 	 	 	 	   It is automatically called in InitPeripherals() function in main.c,
 	 	 	 	   if GYROSCOPE is defined in Hardwareprofile.h
 Preconditions	 : uncomment MAGNETOMETER definition in Hardwareprofile.h
===============================================================================*/
void Init_LSM303DLHC_Magnetometer(void)
{
	LSM303DLHC_Magnetometer_WriteReg(CRA_REG_M,0x10);	  			//15Hz Output Rate, Temp Disabled
	LSM303DLHC_Magnetometer_WriteReg(CRB_REG_M,0x20);				//+-1.3 Guass,
	LSM303DLHC_Magnetometer_WriteReg(MR_REG_M,0x00);				//Continuous Conversion Mode
	Delay();
}

/*===============================================================================
 Function        : LSM303DLHC_Magnetometer_WriteReg;
 Parameters		 : 	 Addr: 1 byte address where data will be written
 	 	 	 	 	 Data: Data to be written
 Description 	 : Sends 1 byte of data to LSM303DLHC magnetometer module
 Preconditions	 : uncomment MAGNETOMETER definition in Hardwareprofile.h
===============================================================================*/
uint32_t LSM303DLHC_Magnetometer_WriteReg(uint8_t Addr, uint8_t Data)
{
	uint32_t Status;
	I2CWriteLength[LSM303DLHC_PORT] = 3;
	I2CReadLength[LSM303DLHC_PORT] = 0;
	I2CMasterBuffer[LSM303DLHC_PORT][0] = LSM303DLHC_MAG_ADDR | LSM303DLHC_WRITE;
	I2CMasterBuffer[LSM303DLHC_PORT][1] = Addr;		/* address */
	I2CMasterBuffer[LSM303DLHC_PORT][2] = Data;		/* Data 0 */
	Status = I2CEngine( LSM303DLHC_PORT );
	return (Status);
}


/*===============================================================================
 Function        : LSM303DLHC_Magnetometer_ReadReg();
 Parameters		 : 	 Addr: 1 byte address from where data will be read
 	 	 	 	 	 Length: Length of data to be read
 Description 	 : Reads n number of bytes from LSM303DLHC magnetometer module
 Preconditions	 : uncomment MAGNETOMETER definition in Hardwareprofile.h
===============================================================================*/
uint32_t LSM303DLHC_Magnetometer_ReadReg(uint8_t Addr, uint8_t Length)
{
	uint32_t Status;
	I2CWriteLength[LSM303DLHC_PORT] = 2;
	I2CReadLength[LSM303DLHC_PORT] = Length;
	I2CMasterBuffer[LSM303DLHC_PORT][0] = LSM303DLHC_MAG_ADDR | LSM303DLHC_WRITE;
	I2CMasterBuffer[LSM303DLHC_PORT][1] = Addr;		/* address */
	I2CMasterBuffer[LSM303DLHC_PORT][2] = LSM303DLHC_MAG_ADDR | LSM303DLHC_READ;
	Status = I2CEngine( LSM303DLHC_PORT );
	return (Status);
}


/*===============================================================================
 Function        : Get_XYZ_Magnetometer();
 Parameters		 : *Data ->Pointer to store the data
 Description 	 : Gets 7 bytes of data from LSM303DLHC magnetometer module
 	 	 	 	   and updates the buffer only if new data is available by reading status byte.
 	 	 	 	   Byte0 	 = Status
 	 	 	 	   Byte1 & 2 = X msb & X lsb
 	 	 	 	   Byte3 & 4 = Y msb & Y lsb
 	 	 	 	   Byte5 & 6 = Z msb & Z lsb
 Preconditions	 : uncomment MAGNETOMETER definition in Hardwareprofile.h
===============================================================================*/
uint32_t Get_XYZ_Magnetometer(uint8_t *Data)
{
	uint8_t i=0;
	uint32_t Status=FALSE;
	I2CSlaveBuffer[LSM303DLHC_PORT][0] = 0x00;
	I2CSlaveBuffer[LSM303DLHC_PORT][1] = 0x00;
	I2CSlaveBuffer[LSM303DLHC_PORT][2] = 0x00;
	I2CSlaveBuffer[LSM303DLHC_PORT][3] = 0x00;
	I2CSlaveBuffer[LSM303DLHC_PORT][4] = 0x00;
	I2CSlaveBuffer[LSM303DLHC_PORT][5] = 0x00;
	I2CSlaveBuffer[LSM303DLHC_PORT][6] = 0x00;

	if(LSM303DLHC_Magnetometer_ReadReg(MR_REG_M,7)==I2C_OK)
	{
			for(i=0;i<6;i++)
			{
				Data[i] = I2CSlaveBuffer[LSM303DLHC_PORT][i+1];
			}
			Status = TRUE;


	}
	Delay();
	return(Status);
}
#endif	//MAGNETOMETER
#endif	//INERTIAL
