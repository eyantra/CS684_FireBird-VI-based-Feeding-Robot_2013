/*
===============================================================================
 Name        : motorcontroller.c
 Author      : Nex Robotics Pvt Ltd
 Version     : 1.0
 Copyright   : Copyright (C) 2012
 Description : This file contains APIs to set motor velocity, get encoder counts,
 	 	 	   setup motor control parameters, etc.
===============================================================================
*/
#include <stdlib.h>
#include "LPC17xx.h"
#include "type.h"
#include "i2c.h"
#include "uart.h"
#include "HardwareDefinition.h"
#include "Hardwareprofile.h"
#include "motorcontroller.h"


volatile uint8_t Left_Velocity=0;
volatile uint8_t Right_Velocity=0;
volatile uint8_t MotorUpdate=0;
volatile uint8_t Direction=0;
volatile uint8_t AccUpdate=0;
volatile uint8_t Acc=0;

#ifdef MOTORCONTROLLER

/*===============================================================================
 Name        	 : Delay_ms;
 Parameters		 : None
 Description 	 : Generates delay of milliseconds as per the argument passed
 Preconditions	 : LPC1769 running at 120 MHz
===============================================================================*/
void Delay_ms(uint32_t ms)
{
	volatile uint32_t i=0;
	volatile uint32_t k=0;
	volatile uint32_t j=0;
	for(k=0;k<11*ms;k++)
	{
		for(i=0;i<675;i++)
		{
			j++;
		}
	}
}

/*===============================================================================
 Function        : InitMotorController();
 Parameters		 : None
 Description 	 : Initialises motor control module by setting mode
 	 	 	 	   and disabling time outs.
 	 	 	 	   It is automatically called in InitPeripherals() function in main.c,
 	 	 	 	   if MOTORCONTROLLER is defined in Hardwareprofile.h
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void InitMotorController(void)
{
	DisableTimeout();
}

/*===============================================================================
 Function        : EnableSafety();
 Parameters		 : None
 Description 	 : Enables safety mode and restricts maximum velocity..
 Returns		 : None
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void EnableSafety()
{
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = SAFETY_COMMAND;
	UART2Buffer[4] = 0x01;

	UARTSend(MOTOR_CONTROLLER_PORT,(uint8_t *)UART2Buffer,5);
}

/*===============================================================================
 Function        : DisableSafety();
 Parameters		 : None
 Description 	 : Disables safety mode and removes restriction on maximum velocity.
 Returns		 : None
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void DisableSafety()
{
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = SAFETY_COMMAND;
	UART2Buffer[4] = 0x00;

	UARTSend(MOTOR_CONTROLLER_PORT,(uint8_t *)UART2Buffer,5);

}

/*===============================================================================
 Function        : Get_Motor_Voltage();
 Parameters		 : None
 Description 	 : Gets Motor controller voltage status.
 	 	 	 	   It reads 10 times the motor voltage. i.e. value 120 means 12.0V
 Returns		 : 1 byte data containing motor voltage
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
uint8_t Get_Motor_Voltage(void)
{
	uint8_t Motor_Voltage=0;

	UART2_RxBuffer[0]=0;
	//UART2Flag = 0;
	RxReadLength = 1;
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = GET_MOTOR_VOLTS;
	UARTSend(MOTOR_CONTROLLER_PORT,(uint8_t *)UART2Buffer,4);

	//while(UART2Flag==0);
	Delay_ms(RxReadLength);	// wait for data to arrive

	Motor_Voltage = UART2_RxBuffer[0];
	return (Motor_Voltage);
}


/*===============================================================================
 Function        : Get_LeftMotor_Current();
 Parameters		 : None
 Description 	 : Gets Left motor current value. It reads 10 times number of amps
 	 	 	 	   i.e. value 025 indicates 2.5 Amps
 Returns		 : 1 byte data containing left motor current
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
uint8_t Get_LeftMotor_Current(void)
{
	uint8_t Motor_Current=0;

	UART2_RxBuffer[0]=0;
	//UART2Flag = 0;
	RxReadLength = 1;
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = GET_CURRENT1;
	UARTSend(MOTOR_CONTROLLER_PORT,(uint8_t *)UART2Buffer,4);

	//while(UART2Flag==0);
	Delay_ms(RxReadLength);	// wait for data to arrive

	Motor_Current = UART2_RxBuffer[0];
	return (Motor_Current);
}


/*===============================================================================
 Function        : Get_RightMotor_Current();
 Parameters		 : None
 Description 	 : Gets Right motor current value. It reads 10 times number of amps
 	 	 	 	   i.e. value 025 indicates 2.5 Amps
 Returns		 : 1 byte data containing left motor current
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
uint8_t Get_RightMotor_Current(void)
{
	uint8_t Motor_Current=0;

	UART2_RxBuffer[0]=0;
	//UART2Flag = 0;
	RxReadLength = 1;
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = GET_CURRENT2;
	UARTSend(MOTOR_CONTROLLER_PORT,(uint8_t *)UART2Buffer,4);

	//while(UART2Flag==0);
	Delay_ms(RxReadLength);	// wait for data to arrive

	Motor_Current = UART2_RxBuffer[0];
	return (Motor_Current);
}


/*===============================================================================
 Function        : GetLeftMotorCount();
 Parameters		 : None
 Description 	 : This function gets 32 bit encoder counts of left motor
 Returns		 : 32 bit value containing left motor encoder counts
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
uint32_t GetLeftMotorCount(void)
{
	uint32_t LeftMotorCount=0;
	UART2_RxBuffer[0]=0;	UART2_RxBuffer[1]=0;	UART2_RxBuffer[2]=0;	UART2_RxBuffer[3]=0;

	//UART2Flag = 0;
	RxReadLength = 4;
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = GET_ENCODER1;
	UARTSend(MOTOR_CONTROLLER_PORT,(uint8_t *)UART2Buffer,4);

	//while(UART2Flag==0);
	Delay_ms(RxReadLength);	// wait for data to arrive

	LeftMotorCount = (uint32_t)(UART2_RxBuffer[0]) << 24;
	LeftMotorCount|= (uint32_t)(UART2_RxBuffer[1]) << 16;
	LeftMotorCount|= (uint32_t)(UART2_RxBuffer[2]) << 8;
	LeftMotorCount|= (uint32_t)(UART2_RxBuffer[3]);

	return (LeftMotorCount);
}


/*===============================================================================
 Function        : GetRightMotorCount();
 Parameters		 : None
 Description 	 : This function gets 32 bit encoder counts of right motor
 Returns		 : 32 bit value containing right motor enocder counts
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
uint32_t GetRightMotorCount(void)
{
	uint32_t RightMotorCount=0;
	UART2_RxBuffer[0]=0;	UART2_RxBuffer[1]=0;	UART2_RxBuffer[2]=0;	UART2_RxBuffer[3]=0;

	//UART2Flag = 0;
	RxReadLength = 4;
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = GET_ENCODER2;
	UARTSend(MOTOR_CONTROLLER_PORT,(uint8_t *)UART2Buffer,4);

	//while(UART2Flag==0);
	Delay_ms(RxReadLength);	// wait for data to arrive

	RightMotorCount = (uint32_t)(UART2_RxBuffer[0]) << 24;
	RightMotorCount|= (uint32_t)(UART2_RxBuffer[1]) << 16;
	RightMotorCount|= (uint32_t)(UART2_RxBuffer[2]) << 8;
	RightMotorCount|= (uint32_t)(UART2_RxBuffer[3]);

	return (RightMotorCount);
}


/*===============================================================================
 Function        : ClearEncoderCounts();
 Parameters		 : None
 Description 	 : Clears left & right motos encoder counts and sets it to zero
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void ClearEncoderCounts(void)
{
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = RESET_ENC_COUNTS;
	UARTSend(MOTOR_CONTROLLER_PORT,(uint8_t *)UART2Buffer,4);
}


/*===============================================================================
 Function        : DisableSpeedRegulation();
 Parameters		 : None
 Description 	 : Disables Automatic speed regulation.
 	 	 	 	 	 Speed regulation is enabled by default
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void DisableSpeedRegulation(void)
{
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = DISABLE_SPEED_CONTROL;
	UARTSend(MOTOR_CONTROLLER_PORT,(uint8_t *)UART2Buffer,4);
}


/*===============================================================================
 Function        : EnableSpeedRegulation();
 Parameters		 : None
 Description 	 : Enables Automatic speed regulation.
 	 	 	 	 	 Speed regulation is enabled by default
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void EnableSpeedRegulation(void)
{
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = ENABLE_SPEED_CONTROL;
	UARTSend(MOTOR_CONTROLLER_PORT,(uint8_t *)UART2Buffer,4);
}


/*===============================================================================
 Function        : DisableTimeout();
 Parameters		 : None
 Description 	 : Disables Automatic Timeout.
 	 	 	 	   Automatic timeout stops motor if no UART communication is
 	 	 	 	   observed for 2 seconds
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void DisableTimeout(void)
{
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = DISABLE_TIMEOUT;
	UARTSend(MOTOR_CONTROLLER_PORT, (uint8_t *) UART2Buffer, 4);
}


/*===============================================================================
 Function        : EnableTimeout();
 Parameters		 : None
 Description 	 : Enables Automatic Timeout.
 	 	 	 	   Automatic timeout stops motor if no UART communication is
 	 	 	 	   observed for 2 seconds
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void EnableTimeout(void)
{
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = ENABLE_TIMEOUT;
	UARTSend(MOTOR_CONTROLLER_PORT, (uint8_t *) UART2Buffer, 4);
}


/*===============================================================================
 Function        : SetMode();
 Parameters		 : Mode -> 0,1 or 2
 Description 	 : Sets Motor controller operating mode.
				   This function is called during motor controller initialization sequence.
 	 	 	 	   Read motion controller manual for more details
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void SetMode(uint8_t Mode)
{
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = SET_MODE;
	UART2Buffer[4] = Mode;
	UARTSend(MOTOR_CONTROLLER_PORT, (uint8_t *) UART2Buffer, 5);
}

/*===============================================================================
 Function        : GetMode();
 Parameters		 : None
 Description 	 : Gets Motor controller operating mode.
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
uint8_t GetMode(void)
{
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = GET_MODE;
	UARTSend(MOTOR_CONTROLLER_PORT, (uint8_t *) UART2Buffer, 4);

	//UART2Flag = 0;
	RxReadLength = 1;

	//while(UART2Flag==0);
	Delay_ms(1);

	return UART2_RxBuffer[0];
}
/*===============================================================================
 Function        : Move(uint8_t LeftSpeed, uint8_t RightSpeed);
 Parameters		 : LeftSpeed  -> 0-Full forward,128-Stop, 255-Full reverse
 	 	 	 	   RightSpeed -> 0-Full forward,128-Stop, 255-Full reverse
 Description 	 : These function controls the locomotion of robot. It takes speed
 	 	 	 	   parameters as arguments.
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void Move(uint8_t LeftSpeed, uint8_t RightSpeed)
{
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = SET_SPEED2;
	UART2Buffer[4] = RightSpeed;
	UARTSend(MOTOR_CONTROLLER_PORT, (uint8_t *) UART2Buffer, 5);

	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = SET_SPEED1;
	UART2Buffer[4] = LeftSpeed;			//Left Motor Speed
	UARTSend(MOTOR_CONTROLLER_PORT, (uint8_t *) UART2Buffer, 5);

}


/*===============================================================================
 Function        : Stop();
 Parameters		 : None
 Description 	 : These function is used to stop the robot. It is equivalent to
 	 	           Move(128,128);
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void Stop(void)
{
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = SET_SPEED2;
	UART2Buffer[4] = 0x80;
	UARTSend(MOTOR_CONTROLLER_PORT, (uint8_t *) UART2Buffer, 5);

	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = SET_SPEED1;
	UART2Buffer[4] = 0x80;
	UARTSend(MOTOR_CONTROLLER_PORT, (uint8_t *) UART2Buffer, 5);
}


/*===============================================================================
 Function        : SetAcceleration();
 Parameters		 : Acc -> 1 to 10 decimal
 Description 	 : Sets the acceleration parameter of the motion controller.
 	 	 	 	   Its value ranges from 1(Least acceleration) to 10(full acceleration)
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void SetAcceleration(uint8_t Acc)
{
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = SET_ACCELERATION;
	UART2Buffer[4] = Acc;
	UARTSend(MOTOR_CONTROLLER_PORT, (uint8_t *) UART2Buffer, 5);
}


/*===============================================================================
 Function        : MotorControl();
 Parameters		 : None
 Description 	 : This function is called in AcquireData(); function in main.c file
 	 	 	 	   to update speed or direction of robot. This function is useful
 	 	 	 	   when speed & direction commands are issued by high level device
 	 	 	 	   like PC or other embedded kit where command are received in UART ISR.
 	 	 	 	   For e.g., if LPC1769 is in the middle of acquiring encoder counts
 	 	 	 	   from motor controller and if user requests to change speed of motors
 	 	 	 	   from the ISR function will result in to unexpected behaviour.
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void MotorControl(void)
{
	if(MotorUpdate)
	{
		switch(Direction)
		{
			case FORWARD:	{Move(Left_Velocity,Right_Velocity);	break;}
			case REVERSE:	{Move(~Left_Velocity,~Right_Velocity);	break;}
			case LEFT:		{Move(~Left_Velocity,Right_Velocity);	break;}
			case RIGHT:		{Move(Left_Velocity,~Right_Velocity);	break;}
			case STOP:		{Stop();								break;}
			default:
				break;
		}
		MotorUpdate=0;
	}
}


/*===============================================================================
 Function        : AccelerationUpdate();
 Parameters		 : None
 Description 	 : This function is called in AcquireData(); function in main.c file
 	 	 	 	   to update acceleration parameter of motion controller. This function is useful
 	 	 	 	   when acceleration commands are issued by high level device
 	 	 	 	   like PC or other embedded kit where command are received in UART ISR.
 	 	 	 	   For e.g., if LPC1769 is in the middle of acquiring encoder counts
 	 	 	 	   from motor controller and if user requests to change the acceleration
 	 	 	 	   from the ISR function will result in to unexpected behaviour.
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void AccelerationUpdate(void)
{
	if(AccUpdate)
	{
		AccUpdate=0;
		SetAcceleration(Acc);
	}

}

/*===============================================================================
 Function        : SetPosition(int32_t positionLeft, int8_t velocityLeft, int32_t positionRight, int8_t velocityRight);
 Parameters		 : positionLeft -> target position count of left motor encoder
 	 	 	 	   velocityLeft -> velocity of left motor
 	 	 	 	   positionRight -> target position count of right motor encoder
 	 	 	 	   velocityRight -> velocity of right motor
 Description 	 : This function is used to move robot by specified number of counts.
 	 	 	 	   The direction of motion for each motor is decided by velocity.
 	 	 	 	   The number of encoder counts to travel is always taken
 	 	 	 	   as absolute value of signed target count specified.
 	 	 	 	   On executing this command, the encoder count registers for both the motors are reset and the
 	 	 	 	   position counting always starts from zero.
 Preconditions	 : 1. uncomment MOTORCONTROLLER definition in Hardwareprofile.h
 	 	 	 	   2. Set Mode value to 2
===============================================================================*/
void SetPosition(int32_t positionLeft, int8_t velocityLeft, int32_t positionRight, int8_t velocityRight)
{
	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = 0x40;	// position control command
	UART2Buffer[4] = (uint8_t)(positionLeft>>24 & 0xFF);
	UART2Buffer[5] = (uint8_t)(positionLeft>>16 & 0xFF);
	UART2Buffer[6] = (uint8_t)(positionLeft>>8  & 0xFF);
	UART2Buffer[7] = (uint8_t)(positionLeft     & 0xFF);
	UART2Buffer[8] = (uint8_t)velocityLeft;
	UART2Buffer[9] = (uint8_t)(positionRight>>24 & 0xFF);
	UART2Buffer[10] = (uint8_t)(positionRight>>16 & 0xFF);
	UART2Buffer[11] = (uint8_t)(positionRight>>8  & 0xFF);
	UART2Buffer[12] = (uint8_t)(positionRight     & 0xFF);
	UART2Buffer[13] = (uint8_t)velocityRight;
	UARTSend(MOTOR_CONTROLLER_PORT, (uint8_t *) UART2Buffer, 14);

}

/*===============================================================================
 Function        : MoveForwardDistanceMM(uint8_t Speed,uint32_t Distance);
 Parameters		 : Speed -> specifies speed of the robot
 	 	 	 	   Distance-> specifies distance to be traveled
 Description 	 : This function is used to move robot in forward direction only upto
 	 	 	 	   specified distance in millimeters. This function is blocking type
 	 	 	 	   and it will not return until specified distance is traveled by the robot.

 	 	 	 	   No. of count per 360 deg rotation of wheel = 360
 	 	 	 	   wheel diameter = 100 mm
 	 	 	 	   wheel circumference = 2.pi.100/2 = 314 mm
 	 	 	 	   Distance traveled per count = wheel circumference/No. of count per 360 deg rotation of wheel
 	 	 	 	   	   	   	   	   	   	   	   = 314/360
 	 	 	 	   	   	   	   	   	   	   	   = 0.872 mm/count
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void MoveForwardDistanceMM(uint8_t Speed,uint32_t Distance)
{
	float req_counts;
	uint32_t req_counts_int=0;
	req_counts = (float)Distance/0.8722;
	req_counts_int = (uint32_t)req_counts;

	if(Speed>127)
			Speed=127;

	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = 0x40;	// position control command
	UART2Buffer[4] = (uint8_t)(req_counts_int>>24 & 0xFF);
	UART2Buffer[5] = (uint8_t)(req_counts_int>>16 & 0xFF);
	UART2Buffer[6] = (uint8_t)(req_counts_int>>8  & 0xFF);
	UART2Buffer[7] = (uint8_t)(req_counts_int     & 0xFF);
	UART2Buffer[8] = Speed;
	UART2Buffer[9] = (uint8_t)(req_counts_int>>24 & 0xFF);
	UART2Buffer[10] = (uint8_t)(req_counts_int>>16 & 0xFF);
	UART2Buffer[11] = (uint8_t)(req_counts_int>>8  & 0xFF);
	UART2Buffer[12] = (uint8_t)(req_counts_int     & 0xFF);
	UART2Buffer[13] = Speed;
	UARTSend(MOTOR_CONTROLLER_PORT, (uint8_t *) UART2Buffer, 14);

}


/*===============================================================================
 Function        : MoveReverseDistanceMM(uint8_t Speed,uint32_t Distance);
 Parameters		 : Speed -> specifies speed of the robot
 	 	 	 	   Distance-> specifies distance to be traveled
 Description 	 : This function is used to move robot in reverse direction only upto
 	 	 	 	   specified distance in millimeters. This function is blocking type
 	 	 	 	   and it will not return until specified distance is traveled by the robot.

 	 	 	 	   No. of count per 360 deg rotation of wheel = 360
 	 	 	 	   wheel diameter = 100 mm
 	 	 	 	   wheel circumference = 2.pi.100/2 = 314 mm
 	 	 	 	   Distance traveled per count = wheel circumference/No. of count per 360 deg rotation of wheel
 	 	 	 	   	   	   	   	   	   	   	   = 314/360
 	 	 	 	   	   	   	   	   	   	   	   = 0.872 mm/count
 	 	 	 	   	   	   	   	   	   	   	   or 1.15 counts/mm

 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void MoveReverseDistanceMM(uint8_t Speed,uint32_t Distance)
{
	float req_counts;
	uint32_t req_counts_int=0;

	req_counts = (float)Distance/0.8722;
	req_counts_int = (uint32_t)req_counts;

	if(Speed>127)
			Speed=127;

	UART2Buffer[0] = 'N';
	UART2Buffer[1] = 'E';
	UART2Buffer[2] = 'X';
	UART2Buffer[3] = 0x40;	// position control command
	UART2Buffer[4] = (uint8_t)(req_counts_int>>24 & 0xFF);
	UART2Buffer[5] = (uint8_t)(req_counts_int>>16 & 0xFF);
	UART2Buffer[6] = (uint8_t)(req_counts_int>>8  & 0xFF);
	UART2Buffer[7] = (uint8_t)(req_counts_int     & 0xFF);
	UART2Buffer[8] = ~Speed;
	UART2Buffer[9] = (uint8_t)(req_counts_int>>24 & 0xFF);
	UART2Buffer[10] = (uint8_t)(req_counts_int>>16 & 0xFF);
	UART2Buffer[11] = (uint8_t)(req_counts_int>>8  & 0xFF);
	UART2Buffer[12] = (uint8_t)(req_counts_int     & 0xFF);
	UART2Buffer[13] = ~Speed;
	UARTSend(MOTOR_CONTROLLER_PORT, (uint8_t *) UART2Buffer, 14);
}


/*===============================================================================
 Function        : MoveRotate(uint8_t Speed, int32_t Degrees);
 Parameters		 : Speed -> specifies speed of the robot
 	 	 	 	   Degrees-> specifies degrees of rotation, Ranges from -180 to 180.
 Description 	 : This function is used to rotate robot in left or right direction only upto
 	 	 	 	   specified degrees of rotation. This function is blocking type
 	 	 	 	   and it will not return until robot rotates to specified degrees.

				   Distance between rear wheels(internal)  			 : 256 mm
				   Circumference traced by robots caster wheel		 : 803.84 mm
				   No of counts / 360 deg of rotation				 : Circumference traced by robots caster wheel/ No.of counts per 1 mm distance traveled
				   													  = 803.84 * 1.1464
				   													  = 921.52 count/360 deg
				   													  = 2.56 counts/deg
 Preconditions	 : uncomment MOTORCONTROLLER definition in Hardwareprofile.h
===============================================================================*/
void MoveRotate(uint8_t Speed, int32_t Degrees)
{
	float req_counts;
	uint32_t req_counts_int=0;

	if(Degrees>180)
	{
		Degrees = 180;
	}
	else if(Degrees<-180)
	{
		Degrees = -180;
	}
	req_counts = abs((float)Degrees * 2.56);
	req_counts_int = (uint32_t)req_counts;

	if(Degrees>0)
	{
		UART2Buffer[0] = 'N';
		UART2Buffer[1] = 'E';
		UART2Buffer[2] = 'X';
		UART2Buffer[3] = 0x40;	// position control command
		UART2Buffer[4] = (uint8_t)(req_counts_int>>24 & 0xFF);
		UART2Buffer[5] = (uint8_t)(req_counts_int>>16 & 0xFF);
		UART2Buffer[6] = (uint8_t)(req_counts_int>>8  & 0xFF);
		UART2Buffer[7] = (uint8_t)(req_counts_int     & 0xFF);
		UART2Buffer[8] = Speed;
		UART2Buffer[9] = (uint8_t)(req_counts_int>>24 & 0xFF);
		UART2Buffer[10] = (uint8_t)(req_counts_int>>16 & 0xFF);
		UART2Buffer[11] = (uint8_t)(req_counts_int>>8  & 0xFF);
		UART2Buffer[12] = (uint8_t)(req_counts_int     & 0xFF);
		UART2Buffer[13] = ~Speed;
		UARTSend(MOTOR_CONTROLLER_PORT, (uint8_t *) UART2Buffer, 14);
	}
	else
	{
		UART2Buffer[0] = 'N';
		UART2Buffer[1] = 'E';
		UART2Buffer[2] = 'X';
		UART2Buffer[3] = 0x40;	// position control command
		UART2Buffer[4] = (uint8_t)(req_counts_int>>24 & 0xFF);
		UART2Buffer[5] = (uint8_t)(req_counts_int>>16 & 0xFF);
		UART2Buffer[6] = (uint8_t)(req_counts_int>>8  & 0xFF);
		UART2Buffer[7] = (uint8_t)(req_counts_int     & 0xFF);
		UART2Buffer[8] = ~Speed;
		UART2Buffer[9] = (uint8_t)(req_counts_int>>24 & 0xFF);
		UART2Buffer[10] = (uint8_t)(req_counts_int>>16 & 0xFF);
		UART2Buffer[11] = (uint8_t)(req_counts_int>>8  & 0xFF);
		UART2Buffer[12] = (uint8_t)(req_counts_int     & 0xFF);
		UART2Buffer[13] = Speed;
		UARTSend(MOTOR_CONTROLLER_PORT, (uint8_t *) UART2Buffer, 14);
	}
}

#endif	//  MOTORCONTROLLER
