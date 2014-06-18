/*
===============================================================================
 Name        : motorcontroller.h
 Author      : Nex Robotics Pvt Ltd
 Version     : 1.0
 Copyright   : Copyright (C) 2012
 Description : This file contains Motion controller code header definition.
===============================================================================
*/

#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#define FORWARD		1
#define REVERSE		2
#define RIGHT		3
#define LEFT		4
#define STOP		6


uint8_t Get_Motor_Voltage(void);
uint8_t Get_LeftMotor_Current(void);
uint8_t Get_RightMotor_Current(void);
uint32_t GetLeftMotorCount(void);
uint32_t GetRightMotorCount(void);
void ClearEncoderCounts(void);
void DisableSpeedRegulation(void);
void EnableSpeedRegulation(void);
void DisableTimeout(void);
void EnableTimeout(void);
void InitMotorController(void);
void Get_MotorStatus(void);
void SetMode(uint8_t Mode);
uint8_t GetMode(void);
void Move(uint8_t LeftSpeed, uint8_t RightSpeed);
void Stop(void);
void SetAcceleration(uint8_t Acc);
void MotorControl(void);
void AccelerationUpdate(void);
void SetPosition(int32_t positionLeft, int8_t velocityLeft, int32_t positionRight, int8_t velocityRight);
void MoveForwardDistanceMM(uint8_t Speed,uint32_t Distance);
void MoveReverseDistanceMM(uint8_t Speed,uint32_t Distance);
void MoveRotate(uint8_t Speed, int32_t Degrees);
void EnableSafety();
void DisableSafety();


extern volatile uint8_t Right_Velocity;
extern volatile uint8_t Left_Velocity;
extern volatile uint8_t MotorUpdate;
extern volatile uint8_t Direction;
extern volatile uint8_t AccUpdate;
extern volatile uint8_t Acc;
#endif /* MOTORCONTROLLER_H_ */
