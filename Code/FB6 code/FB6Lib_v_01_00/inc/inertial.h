/*
===============================================================================
 Name        : inertial.h
 Author      : Nex Robotics Pvt Ltd
 Version     : 1.0
 Copyright   : Copyright (C) 2012
 Description : This file contains IMU code header definition.
===============================================================================
*/

#ifndef INERTIAL_H_
#define INERTIAL_H_

void Init_L3G4200D(void);
void Init_LSM303DLHC_Accelerometer(void);
void Init_LSM303DLHC_Magnetometer(void);
uint32_t L3G4200D_WriteReg(uint8_t Addr, uint8_t Data);
uint32_t L3G4200D_ReadReg(uint8_t Addr, uint8_t Length);
uint32_t LSM303DLHC_Accelerometer_WriteReg(uint8_t Addr, uint8_t Data);
uint32_t LSM303DLHC_Accelerometer_ReadReg(uint8_t Addr, uint8_t Length);
uint32_t LSM303DLHC_Magnetometer_WriteReg(uint8_t Addr, uint8_t Data);
uint32_t LSM303DLHC_Magnetometer_ReadReg(uint8_t Addr, uint8_t Length);
uint32_t Get_XYZ_Rate(uint8_t *Data);
uint32_t Get_XYZ_Acceleration(uint8_t *Data);
uint32_t Get_XYZ_Magnetometer(uint8_t *Data);
extern void Delay(void);

#endif /* INERTIAL_H_ */
