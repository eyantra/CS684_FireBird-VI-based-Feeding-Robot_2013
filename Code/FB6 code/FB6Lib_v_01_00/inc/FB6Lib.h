

#ifndef FB6LIB_H_
#define FB6LIB_H_

// TODO: insert other include files here
#include "HardwareDefinition.h"
#include "Hardwareprofile.h"
#include "uart.h"
#include "i2c.h"
#include "adc.h"
#include "sensorboard.h"
#include "batterymonitor.h"
#include "motorcontroller.h"
#include "inertial.h"
#include "lcd.h"
#include "Gps.h"


// TODO: insert other definitions and declarations here
/******************Global Variables****************************/
uint8_t Ultrasonic[8]={0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8};
uint8_t Whiteline[8]={0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8};
uint8_t IR_Proximity[8]={0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8};
uint8_t Accelerometer[6]={0xD1,0xD2,0xD3,0xD4,0xD5,0xD6};
uint8_t Gyroscope[6]={0xE1,0xE2,0xE3,0xE4,0xE5,0xE6};
uint8_t Magnetometer[6]={0xF1,0xF2,0xF3,0xF4,0xF5,0xF6};
uint8_t Battery[8]={0xAA,0xAB,0xAC};
uint8_t MotorVoltage=123;
uint8_t LeftMotorCurrent=100;
uint8_t RightMotorCurrent=100;
uint8_t AD7998ADC[16]={0x71,0x72,0x73,0x74,0x75,0x76,0x78,0x79};
uint32_t PotValue=0;
uint32_t ServoPOD_ADC = 0;
uint8_t ResetEncoderCounts=0;
uint8_t	UpdateMode = 0;
uint8_t UpdateSafety = 0;
uint8_t ServoPodULTriggerState = 0;
uint8_t updatePosition = 0;
int32_t positionLeft = 0;
int32_t positionRight = 0;
int8_t velocityLeft = 0;
int8_t velocityRight = 0;
uint8_t updateRemote_2_4GhzCommand = 0;
uint8_t remoteControl[6] = {0xA1,0xA2,0xA3,0xA4,0xA5,0xA6};

volatile uint8_t PanAngle=0;
volatile uint8_t TiltAngle=0;
volatile uint8_t AuxAngle=0;
volatile int32_t Left_Count_New_Locked;
volatile int32_t Right_Count_New_Locked;
volatile uint8_t Mode;
volatile uint8_t Safety;

/********************************************/
volatile uint8_t My_Button_Status;
//volatile uint8_t My_L_Status;
/********************************************/


#endif // FB6LIB_H_

