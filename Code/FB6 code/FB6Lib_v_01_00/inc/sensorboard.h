/*
===============================================================================
 Name        : sensorboard.h
 Author      : Nex Robotics Pvt Ltd
 Version     : 1.0
 Copyright   : Copyright (C) 2012
 Description : This file contains Sensor Module code header definition.
===============================================================================
*/

#ifndef SENSORBOARD_H_
#define SENSORBOARD_H_

uint32_t Sensor_Board_WriteReg(uint8_t Addr, uint8_t Data);
uint32_t Sensor_Board_ReadReg(uint8_t Addr, uint8_t Length);
uint32_t Set_WL_Trigger(uint8_t Data);
uint32_t Set_IR_Trigger(uint8_t Data);
uint32_t Set_UL_Trigger(uint8_t Data);
uint32_t Get_Ultrasonic_Data(uint8_t *Data);
uint32_t Get_Whiteline_Data(uint8_t *Data);
uint32_t Get_IR_Proximity_Data(uint8_t *Data);
uint32_t AD7998_WriteReg(uint8_t Addr, uint16_t Data);
uint32_t AD7998_Board_ReadReg(uint8_t Addr, uint8_t Length);
uint32_t Get_AD7998_Data(uint8_t *Data);
void UpdateServoPos(uint8_t Angle, uint8_t Servo);
void ULTriggerUpdate(void);
void WLTriggerUpdate(void);
void IRTriggerUpdate(void);
extern void Delay(void);

extern volatile uint8_t IRTrigger;
extern volatile uint8_t WLTrigger;
extern volatile uint8_t ULTrigger;
extern volatile uint8_t IRTriggerState;
extern volatile uint8_t WLTriggerState;
extern volatile uint8_t ULTriggerState;
extern volatile uint8_t ServoUpdate;
extern volatile uint8_t ServoType;

#endif /* SENSORBOARD_H_ */
