/*
===============================================================================
 Name        : HardwareDefinition.h
 Author      :
 Version     :
 Copyright   : Copyright (C) 2012
 Description : main definition
===============================================================================
*/

#ifndef HARDWAREDEFINITION_H_
#define HARDWAREDEFINITION_H_

/***********************LED Definitions********************/
		//LED1 -> P1.25
		//LED2 -> P1.26
		//LED3 -> P1.27
		//LED4 -> P1.28

#define LED1	0x02000000
#define LED2	0x04000000
#define LED3	0x08000000
#define LED4	0x10000000

#define LED_ON(val)		LPC_GPIO1->FIOSET = val
#define LED_OFF(val)	LPC_GPIO1->FIOCLR = val

#define LED1_ON()		LPC_GPIO1->FIOSET = LED1
#define LED2_ON()		LPC_GPIO1->FIOSET = LED2
#define LED3_ON()		LPC_GPIO1->FIOSET = LED3
#define LED4_ON()		LPC_GPIO1->FIOSET = LED4

#define LED1_OFF()		LPC_GPIO1->FIOCLR = LED1
#define LED2_OFF()		LPC_GPIO1->FIOCLR = LED2
#define LED3_OFF()		LPC_GPIO1->FIOCLR = LED3
#define LED4_OFF()		LPC_GPIO1->FIOCLR = LED4

#define ON				1
#define OFF				0

#define P1_29			0x20000000
#define UL_TRIG_ON()	LPC_GPIO1->FIOSET = P1_29
#define UL_TRIG_OFF()	LPC_GPIO1->FIOCLR = P1_29

#define PAN				1
#define TILT			2
#define AUX				3

/**********************************************************/

/***********************Switch Definitions********************/
		//SW1 -> P2.10(EINT0)
		//SW2 -> P2.11(EINT1)
		//SW3 -> P2.12(EINT2)
		//SW4 -> P2.13(EINT3)

#define SW1		0x00000400
#define SW2		0x00000800
#define SW3		0x00001000
#define SW4		0x00002000

#define SW1_PRESSED		(LPC_GPIO2->FIOPIN & 0x00000400)
#define SW2_PRESSED		(LPC_GPIO2->FIOPIN & 0x00000800)
#define SW3_PRESSED		(LPC_GPIO2->FIOPIN & 0x00001000)
#define SW4_PRESSED		(LPC_GPIO2->FIOPIN & 0x00002000)
/**********************************************************/

/***********************L3G4200D 3 axis Gyroscope Definitions********************/
#define L3G4200D_PORT			1
#define L3G4200D_ADDR 			0xD2	//L3G4200D I2C Address
#define L3G4200D_READ 			1
#define L3G4200D_WRITE 			0

#define L3G4200D_WHO_AM_I 		0x0F
#define L3G4200D_CTRL_REG1 		0x20
#define L3G4200D_CTRL_REG2 		0x21
#define L3G4200D_CTRL_REG3 		0x22
#define L3G4200D_CTRL_REG4 		0x23
#define L3G4200D_CTRL_REG5 		0x24
#define REFERENCE_L3G4200D 		0x25
#define OUT_TEMP 				0x26
#define STATUS_REG 				0x27
#define OUT_X_L 				0x28
#define OUT_X_H 				0x29
#define OUT_Y_L 				0x2A
#define OUT_Y_H 				0x2B
#define OUT_Z_L 				0x2C
#define OUT_Z_H 				0x2D
#define FIFO_CTRL_REG 			0x2E
#define FIFO_SRC_REG 			0x2F
#define INT1_CFG 				0x30
#define INT1_SRC 				0x31
#define INT1_TSH_XH 			0x32
#define INT1_TSH_XL 			0x33
#define INT1_TSH_YH 			0x34
#define INT1_TSH_YL 			0x35
#define INT1_TSH_ZH 			0x36
#define INT1_TSH_ZL 			0x37
#define INT1_DURATION 			0x38

/**********************************************************/

/*********MMA8451 3 axis Accelerometer Definitions*********/

#define MMA8451_PORT			0
#define MMA8451_ADDR 			0x38
#define MMA8451_READ 			1
#define MMA8451_WRITE 			0

#define STATUS					0x00
#define OUT_X_MSB				0x01
#define OUT_X_LSB				0x02
#define OUT_Y_MSB				0x03
#define OUT_Y_LSB				0x04
#define OUT_Z_MSB				0x05
#define OUT_Z_LSB				0x06
#define F_SETUP					0x09
#define MMA8451_WHO_AM_I		0x0D
#define XYZ_DATA_CFG			0x0E
#define MMA8451_CTRL_REG1		0x2A
#define MMA8451_CTRL_REG2		0x2B

/***********************************************************************/


/*********LSM303DLHC 3 axis Accelerometer & 3 axis Magnetometer*********/
#define LSM303DLHC_PORT			1
#define LSM303DLHC_READ 		1
#define LSM303DLHC_WRITE 		0


/****************Accelerometer Register Definitions*************/

#define LSM303DLHC_ACC_ADDR 	0x32		//LSM303DLHC Accelerometer Address

#define CTRL_REG1_A 			0x20
#define CTRL_REG2_A 			0x21
#define CTRL_REG3_A 			0x22
#define CTRL_REG4_A 			0x23
#define CTRL_REG5_A 			0x24
#define CTRL_REG6_A 			0x25
#define REFERENCE_LSM303DLHC_A 	0x26
#define STATUS_REG_A 			0x27
#define OUT_X_L_A 				0x28
#define OUT_X_H_A 				0x29
#define OUT_Y_L_A 				0x2A
#define OUT_Y_H_A 				0x2B
#define OUT_Z_L_A 				0x2C
#define OUT_Z_H_A 				0x2D
#define FIFO_CTRL_REG_A 		0x2E
#define FIFO_SRC_REG_A 			0x2F
#define INT1_CFG_A 				0x30
#define INT1_SRC_A 				0x31
#define INT1_TSH_A 				0x32
#define INT1_DURATION_A 		0x33
#define INT2_CFG_A 				0x34
#define INT2_SRC_A 				0x35
#define INT2_THS_A 				0x36
#define INT2_DURATION_A 		0x37
#define CLICK_CFG_A				0x38
#define CLICK_SRC_A				0x39
#define CLICK_THS_A				0x3A
#define TIME_LIMIT_A 			0x3B
#define TIME_LATENCY_A			0x3C
#define TIME_WINDOW_A			0x3D
/***************************************************************/

/******************Magnetometer Definitions*********************/

#define LSM303DLHC_MAG_ADDR		0x3C		//LSM303DLHC Magnetometer Address

#define CRA_REG_M				0x00
#define CRB_REG_M				0x01
#define MR_REG_M				0x02
#define OUT_X_H_M				0x03
#define OUT_X_L_M				0x04
#define OUT_Z_H_M				0x05
#define OUT_Z_L_M				0x06
#define OUT_Y_H_M				0x07
#define OUT_Y_L_M				0x08
#define SR_REG_MG				0x09
#define IRA_REG_M				0x0A
#define IRB_REG_M				0x0B
#define IRC_REG_M				0x0C
#define TEMP_OUT_H_M			0x31
#define TEMP_OUT_L_M			0x32
/***************************************************************/

/***********************************************************************/



/*********Sensor Board Definitions**************************************/

#define SENSOR_BOARD_PORT		0
#define SENSOR_BOARD_ADDR		0xE8
#define SENSOR_BOARD_READ		1
#define SENSOR_BOARD_WRITE		0

#define	SENSOR_WHO_AM_I			0x20
#define	STATUS_WL				0x22
#define	STATUS_UL				0x23
#define	STATUS_IR				0x24
//_____________________________________________________________
#define	READ_WL1				0x31
#define	READ_WL2				0x32
#define	READ_WL3				0x33
#define	READ_WL4				0x34
#define	READ_WL5				0x35
#define	READ_WL6				0x36
#define	READ_WL7				0x37
#define	READ_WL8				0x38
#define	READ_WL_ALL				0x39
#define	READ_WL_DIGITAL			0x3A

#define	SET_WL_TRIGGER			0x3B
#define	WL1_TH					0x41
#define	WL2_TH					0x42
#define	WL3_TH					0x43
#define	WL4_TH					0x44
#define	WL5_TH					0x45
#define	WL6_TH					0x46
#define	WL7_TH					0x47
#define	WL8_TH					0x48
//_____________________________________________________________
#define	READ_UL1				0x51
#define	READ_UL2				0x52
#define	READ_UL3				0x53
#define	READ_UL4				0x54
#define	READ_UL5				0x55
#define	READ_UL6				0x56
#define	READ_UL7				0x57
#define	READ_UL8				0x58
#define	READ_UL_ALL				0x59
#define	READ_UL_DIGITAL			0x5A

#define	SET_UL_TRIGGER			0x5B
#define	UL1_TH					0x61
#define	UL2_TH					0x62
#define	UL3_TH					0x63
#define	UL4_TH					0x64
#define	UL5_TH					0x65
#define	UL6_TH					0x66
#define	UL7_TH					0x67
#define	UL8_TH					0x68
//_____________________________________________________________
#define	READ_IR1				0x71
#define	READ_IR2				0x72
#define	READ_IR3				0x73
#define	READ_IR4				0x74
#define	READ_IR5				0x75
#define	READ_IR6				0x76
#define	READ_IR7				0x77
#define	READ_IR8				0x78
#define	READ_IR_ALL				0x79
#define	READ_IR_DIGITAL			0x7A

#define	SET_IR_TRIGGER			0x7B
#define	IR1_TH					0x81
#define	IR2_TH					0x82
#define	IR3_TH					0x83
#define	IR4_TH					0x84
#define	IR5_TH					0x85
#define	IR6_TH					0x86
#define	IR7_TH					0x87
#define	IR8_TH					0x88

#define PAN_SERVO				0xA1
#define TILT_SERVO				0xA2
#define AUX_SERVO				0xA3

/***********************************************************************/


/*********Battery Monitor Board definitions**************************************/
#define BATTERY_MONITOR_PORT	1
#define BATTERY_MONITOR_ADDR	0xF8
#define BATTERY_MONITOR_READ	1
#define BATTERY_MONITOR_WRITE	0

#define	BATTERY_WHO_AM_I		0x20
#define	BATTERY_STATUS			0x31
#define	BATTERY_VOLTAGE			0x32
#define	BATTERY_CURRENT			0x33
#define BATTERY_TEMPERATURE		0x34
/***********************************************************************/


/*******************Motion Controller Definitions***********************/
#define MOTOR_CONTROLLER_PORT	2

#define SYNC					0x00
#define	SET_SPEED1				0x31
#define	SET_SPEED2				0x32
#define	SET_TURN				0x32
#define	GET_ENCODER1			0x23
#define	GET_ENCODER2			0x24
#define GET_ENCODERS			0x25
#define	GET_MOTOR_VOLTS			0x26
#define	GET_CURRENT1			0x27
#define	GET_CURRENT2			0x28
#define	SW_REV					0x29
#define	SET_ACCELERATION		0x33
#define	SET_MODE				0x34
#define GET_VI_PARAMS			0x2C

#define RESET_ENC_COUNTS		0x35
#define DISABLE_SPEED_CONTROL	0x36
#define ENABLE_SPEED_CONTROL	0x37
#define DISABLE_TIMEOUT			0x38
#define ENABLE_TIMEOUT			0x39
/***********************************************************************/

/***************************I2C LCD Definitions*************************/
#define LCD_PORT				1
#define LCD_ADDR				0xC6
#define LCD_READ				1
#define LCD_WRITE				0

#define LCD_NULL				0
#define	CURSOR_HOME				1
#define	CURSOR_POS				2
#define	SET_CURSOR				3
#define	HIDE_CURSOR				4
#define	UNDERLINE_CURSOR		5
#define	BLINK_CURSOR			6
#define	BACKSPACCE				8
#define	HT						9
#define	LF						10
#define	VT						11
#define	CLS						12
#define	CR						13
#define	CLR_COL					17
#define	SET_TAB					18
#define	BL_ON					19
#define	BL_OFF					20
#define	CHANGE_ADD				25
#define	CUSTOM_CHAR				27

/***********************************************************************/

/***********AD7997/78 8ch 10/12 bit ADC Definitions*********************/

#define AD7998_PORT				0
#define AD7998_ADDR				0x46
#define AD7998_READ				1
#define AD7998_WRITE			0

#define AD7998_CONVERSION_REG	0x00
#define AD7998_ALERT_STAT		0x01
#define AD7998_CONFIG			0x02
#define AD7998 CYCLE_TIME		0x03
#define LIMIT_CH1_HIGH			0x04
#define LIMIT_CH1_LOW			0x05
#define LIMIT_CH1_HYSTERSIS		0x06
#define LIMIT_CH2_HIGH			0x07
#define LIMIT_CH2_LOW			0x08
#define LIMIT_CH2_HYSTERSIS		0x09
#define LIMIT_CH3_HIGH			0x0A
#define LIMIT_CH3_LOW			0x0B
#define LIMIT_CH3_HYSTERSIS		0x0C
#define LIMIT_CH4_HIGH			0x0D
#define LIMIT_CH4_LOW			0x0E
#define LIMIT_CH4_HYSTERSIS		0x0F

#define MODE2_OFF				0x00
#define	MODE2_CH1				0x80
#define	MODE2_CH2				0x90
#define	MODE2_CH3				0xA0
#define	MODE2_CH4				0xB0
#define	MODE2_CH5				0xC0
#define	MODE2_CH6				0xD0
#define	MODE2_CH7				0xE0
#define MODE2_CH8				0xF0
#define	MODE2_SEQUENTIAL		0x70
/***********************************************************************/


#endif /* HARDWAREINIT_H_ */
