/*
===============================================================================
 Name        : main.c
 Author      : 
 Version     :
 Copyright   : Copyright (C) 
 Description : main definition
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#include "type.h"
#endif

#include <cr_section_macros.h>

// TODO: insert other include files here
#include "FB6Lib.h"

/**************************************************************/

/******************Function Prototypes****************************/
/*void InitLeds(void);
void InitSwitch(void);
void InitPeripherals(void);
void Delay(void);
void BlinkLEDs(void);
void Delay1s(void);
void AcquireData(void);
void DelayRotate90(void);*/
/*****************************************************************/

/*===============================================================================
 Function        : InitLeds()
 Parameters		 : None
 Description 	 : Sets direction of LED IO pins
 Preconditions	 : Uncomment LED definition in Hardwareprofile.h
===============================================================================*/
/*void InitLeds(void)
{
	LPC_GPIO1->FIODIR&=	!(LED1 | LED2 | LED3 | LED4);
	LPC_GPIO1->FIODIR|= (LED1 | LED2 | LED3 | LED4);
}*/

/*===============================================================================
 Name        	 : InitSwitch()
 Parameters		 : None
 Description 	 : Sets direction of Switch IO pins
 Preconditions	 : Uncomment SWITCH definition in Hardwareprofile.h
===============================================================================*/
/*void InitSwitch(void)
{
	LPC_GPIO2->FIODIR&= !(SW1 | SW2 | SW3 | SW4);
}*/


/*===============================================================================
 Name        	 : InitPeripherals()
 Parameters		 : None
 Description 	 : This function initializes the peripherals of LPC1769 microcontroller
 	 	 	 	   and modules of Fire Bird VI robot as per the definitions in
 	 	 	 	   Hardwareprofile.h
 Preconditions	 : None
===============================================================================*/
/*void InitPeripherals(void)
{
#if (defined(MOTORCONTROLLER))
	int i = 0;
	// delay to allow motor controller to initialize
	for (i=0;i<1000;i++)
		Delay();
	UARTInit(2, 19200);
	InitMotorController();
	Stop();
#endif

#ifdef LED
	InitLeds();
	BlinkLEDs();
#endif

#ifdef SWITCH
	InitSwitch();
#endif

#if (defined(POT) || defined(SERVOPOD))
	ADCInit(ADC_CLK);
#if (defined(SERVOPOD))
	LPC_GPIO1->FIODIR|= P1_29;				//Set Direction of trigger pin
	UL_TRIG_OFF();							//Initially ServoPod Ultrasonic Trigger is set OFF
#endif
#endif

#ifdef SENSORBOARD
	ResetI2C0();
	I2C0Init();
	I2CStop(0);

#if (defined(WHITELINE))
	//Write Code here
#endif

#if (defined(ULTRASONIC))
	//Write Code here
#endif

#if (defined(IR_PROXIMITY))
	//Write Code here
#endif

#if (defined(EXT_ADC))
	AD7998_WriteReg(AD7998_CONFIG,0x0FF8);		//Convert Channel 1, Filter On
#endif
#endif

#if (defined(BATTERYMONITOR) || defined(INERTIAL) || defined(LCD))
	ResetI2C1();
	I2C1Init();
	I2CStop(1);

#if (defined(BATTERYMONITOR))
	//Write Code here
#endif

#if (defined(GYROSCOPE))
		Init_L3G4200D();
#endif

#if (defined(ACCELEROMETER))
		Init_LSM303DLHC_Accelerometer();
#endif

#if (defined(MAGNETOMETER))
		Init_LSM303DLHC_Magnetometer();
#endif

#if (defined(LCD))
		Delay1s();
		InitLCD();
#endif

#endif


#if (defined(BEAGLE))
	UARTInit(3, 115200);
#endif

#if (defined(GPS))
	UARTInit(0, 9600);
#endif

#if (defined(WIRELESS_COMMUNICATION))
	UARTInit(1,9600);
#if (defined(BLUETOOTH))
	//Write Code here
#endif

#if (defined(XBEE))
	//Write Code here
#endif

#if (defined(WIFI))
	//Write Code here
#endif
#endif
}*/

/*===============================================================================
 Name        	 : Delay();
 Parameters		 : None
 Description 	 : Generates delay of very small amount
 Preconditions	 : None
===============================================================================*/
/*void Delay(void)
{
	uint32_t i=0;
	//for(i=0;i<10000;i++);
	for(i=0;i<100;i++);
}*/

/*===============================================================================
 Name        	 : Delay1s();
 Parameters		 : None
 Description 	 : Generates delay of approximately 1 Second
 Preconditions	 : None
===============================================================================*/
/*void Delay1s(void)
{
	volatile uint32_t i=0;
	volatile uint32_t k=0;
	volatile uint32_t j=0;
	for(k=0;k<110;k++)
	{
		for(i=0;i<60000;i++)
		{
			j++;
		}
	}
}*/

/*void DelayRotate90(void)
{
	volatile uint32_t i=0;
	volatile uint32_t k=0;
	volatile uint32_t j=0;
	for(k=0;k<45;k++)
	{
		for(i=0;i<60000;i++)
		{
			j++;
		}
	}
}*/

/*===============================================================================
 Name        	 : BlinkLEDs();
 Parameters		 : None
 Description 	 : This function blinks the LEDs on GPIO panel
 Preconditions	 : None
===============================================================================*/
/*void BlinkLEDs(void)
{
	LED1_ON();
	Delay1s();
	LED1_OFF();
	LED2_ON();
	Delay1s();
	LED2_OFF();
	LED3_ON();
	Delay1s();
	LED3_OFF();
	LED4_ON();
	Delay1s();
	LED4_OFF();
}*/


/*===============================================================================
 Name        	 : AcquireData();
 Parameters		 : None
 Description 	 : This function acquires data from sensor module, power management module, IMU
 	 	 	 	   and sets actuation signals for traction motors, servo motors, etc.
 Preconditions	 : None
===============================================================================*/
/*void AcquireData(void)
{
#ifdef BATTERYMONITOR
	Get_Battery_Status(Battery);			//Battery
#endif

#ifdef WHITELINE
	WLTriggerUpdate();
	Get_Whiteline_Data(Whiteline);			//Whiteline
#endif

#ifdef ULTRASONIC
	ULTriggerUpdate();
	Get_Ultrasonic_Data(Ultrasonic);		//Ultrasonic
#endif

#ifdef IR_PROXIMITY
	IRTriggerUpdate();
	Get_IR_Proximity_Data(IR_Proximity);	//IR_Proximity
#endif

#ifdef GYROSCOPE
	Get_XYZ_Rate(Gyroscope);				//Gyroscope
#endif

#ifdef ACCELEROMETER
	Get_XYZ_Acceleration(Accelerometer);	//Accelerometer
#endif

#ifdef MAGNETOMETER
	Get_XYZ_Magnetometer(Magnetometer);		//Magnetometer
#endif

#ifdef MOTORCONTROLLER
	AccelerationUpdate();
	MotorControl();
	if(ResetEncoderCounts==1)
	{
		ClearEncoderCounts();
		ResetEncoderCounts=0;
	}
	Left_Count_New_Locked=GetLeftMotorCount();
	Right_Count_New_Locked = GetRightMotorCount();
#endif

#ifdef POT
	PotValue = ADCRead(5);
#endif

#ifdef SERVOPOD
	ServoPOD_ADC = ADCRead(1);
	if(ServoUpdate==1)
	{
		if		(ServoType==PAN)	{UpdateServoPos(PanAngle,PAN);}
		else if (ServoType==TILT)	{UpdateServoPos(TiltAngle,TILT);}
		else if (ServoType==AUX)	{UpdateServoPos(AuxAngle,AUX);}
	}
#endif

#ifdef LED
	if(!SW1_PRESSED){LED1_ON();}
	else			{LED1_OFF();}
	if(!SW2_PRESSED){LED2_ON();	}
	else			{LED2_OFF();}
	if(!SW3_PRESSED){LED3_ON();}
	else			{LED3_OFF();}
	if(!SW4_PRESSED){LED4_ON();}
	else			{LED4_OFF();}
#endif

#ifdef EXT_ADC
	Get_AD7998_Data(AD7998ADC);
#endif
}*/

