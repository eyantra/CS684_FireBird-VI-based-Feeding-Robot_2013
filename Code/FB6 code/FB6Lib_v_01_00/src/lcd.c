/*
===============================================================================
 Name        : lcd.c
 Author      : Nex Robotics Pvt Ltd
 Version     : 1.0
 Copyright   : Copyright (C) 2012
 Description : This file contains APIs to display data on LCD screen.
===============================================================================
*/

#include "LPC17xx.h"
#include "type.h"
#include "i2c.h"
#include "HardwareDefinition.h"
#include "Hardwareprofile.h"
#include "lcd.h"


#ifdef LCD
/*===============================================================================
 Name        	 : Delay();
 Parameters		 : None
 Description 	 : Generates delay of very small amount
 Preconditions	 : None
===============================================================================*/
void Delay_LCD(void)
{
	uint32_t i=0;
	for(i=0;i<100;i++);
}

/*===============================================================================
 Function        : InitLCD();
 Parameters		 : None
 Description 	 : Initializes LCD module. Clears any data on the screen, turns ON backlight
 	 	 	 	   and hides cursor.
 	 	 	 	   It is automatically called in InitPeripherals() function in main.c,
 	 	 	 	   if LCD is defined in Hardwareprofile.h
 Preconditions	 : uncomment LCD definition in Hardwareprofile.h
===============================================================================*/
void InitLCD(void)
{
	LCDClearScreen();
	Delay_LCD();
	LCDSetBackLight(1);
	Delay_LCD();
	LCD_WriteCmd(HIDE_CURSOR);
	Delay_LCD();
}


/*===============================================================================
 Function        : LCD_WriteCmd(uint8_t cmd);
 Parameters		 : cmd -> Command to be executed
 Description 	 : This function sets certain parameters of LCD module like
 	 	 	 	   hide cursor, unhide cursor underline cursor, blinking cursor.
 	 	 	 	   Refer HardwareDefinition.h for commands.
 Preconditions	 : uncomment LCD definition in Hardwareprofile.h
===============================================================================*/
uint32_t LCD_WriteCmd(uint8_t cmd)
{
	uint32_t Status;
	I2CWriteLength[LCD_PORT] = 2;
	I2CReadLength[LCD_PORT] = 0;
	I2CMasterBuffer[LCD_PORT][0] = LCD_ADDR | LCD_WRITE;
	I2CMasterBuffer[LCD_PORT][1] = cmd;		/* address */
	Status = I2CEngine( LCD_PORT );
	return (Status);
}


/*===============================================================================
 Function        : LCD_WriteStr(uint8_t Len,uint8_t *Data);
 Parameters		 : Len -> Length of the string to be displayed
 	 	 	 	   *Data-> Pointer to the string to be displayed
 Description 	 : This function displays a string characters of specified length
 	 	 	 	   on LCD screen.
 Preconditions	 : uncomment LCD definition in Hardwareprofile.h
===============================================================================*/
uint32_t LCD_WriteStr(uint8_t Len,uint8_t *Data)
{
	uint32_t Status;
	uint32_t i=0;
	I2CWriteLength[LCD_PORT] = Len + 1;
	I2CReadLength[LCD_PORT] = 0;
	I2CMasterBuffer[LCD_PORT][0] = LCD_ADDR | LCD_WRITE;
	for(i=0;i<Len;i++)
	{
		I2CMasterBuffer[LCD_PORT][i+1]= *Data;
		Data++;
	}
	Status = I2CEngine( LCD_PORT );
	return (Status);
}


/*===============================================================================
 Function        : LCD_PrintData(uint8_t Row,uint8_t Col,uint32_t Number,uint8_t Digits);
 Parameters		 : Row -> LCD row address (1-4)
 	 	 	 	   Col -> LCD column address (1-20)
 	 	 	 	   Number-> Decimal number to be converted into ASCII
 	 	 	 	   Digits -> Number of digits to be converted
 	 	 	 	   (For e.g dec data 123 will have digit field as 3)
 Description 	 : This function converts decimal data into ASCII characters and
 	 	 	 	   displays them at specified location on LCD screen.
 Preconditions	 : uncomment LCD definition in Hardwareprofile.h
===============================================================================*/

uint32_t LCD_PrintData(uint8_t Row,uint8_t Col,uint32_t Number,uint8_t Digits)
{
	uint32_t Status = FALSE;
	uint32_t Temp=0;
	uint8_t String[10];
	uint8_t Flag=0;
	uint8_t count=0;

	LCDSetCursorPosition(Row,Col);

	if(Digits>5)
		return(Status);
	if(Digits==5 || Flag==1)
	{
		String[count++] = Number/10000 + 48;
		Flag=1;
	}
	if(Digits==4 || Flag==1)
	{
		Temp = Number/1000;
		String[count++] = (Temp % 10) + 48;
		Flag=1;
	}
	if(Digits==3 || Flag==1)
	{
		Temp = Number/100;
		String[count++] = (Temp % 10) + 48;
		Flag=1;
	}
	if(Digits==2 || Flag==1)
	{
		Temp = Number/10;
		String[count++] = (Temp % 10) + 48;
		Flag=1;
	}
	if(Digits==1 || Flag==1)
	{
		String[count++] = (Number%10) + 48;
	}

	if(LCD_WriteStr(Digits,&String[0])==I2C_OK)
		Status = TRUE;
	return (Status);
}


/*===============================================================================
 Function        : LCDSetCursorPosition(uint8_t Row, uint8_t Col);
 Parameters		 : Row -> LCD row address (1-4)
 	 	 	 	   Col -> LCD column address (1-20)
 Description 	 : This function sets cursor position to the specified address.
 Preconditions	 : uncomment LCD definition in Hardwareprofile.h
===============================================================================*/
uint32_t LCDSetCursorPosition(uint8_t Row, uint8_t Col)
{
	uint32_t Status;
	I2CWriteLength[LCD_PORT] = 4;
	I2CReadLength[LCD_PORT] = 0;
	I2CMasterBuffer[LCD_PORT][0] = LCD_ADDR | LCD_WRITE;
	I2CMasterBuffer[LCD_PORT][1] = SET_CURSOR;
	I2CMasterBuffer[LCD_PORT][2] = Row;
	I2CMasterBuffer[LCD_PORT][3] = Col;
	Status = I2CEngine( LCD_PORT );
	return (Status);
}


/*===============================================================================
 Function        : LCDSetCursorHome();
 Parameters		 : None
 Description 	 : This function sets cursor to Home position i.e. Row=Col=1
 Preconditions	 : uncomment LCD definition in Hardwareprofile.h
===============================================================================*/
uint32_t LCDSetCursorHome(void)
{
	uint32_t Status;
	I2CWriteLength[LCD_PORT] = 2;
	I2CReadLength[LCD_PORT] = 0;
	I2CMasterBuffer[LCD_PORT][0] = LCD_ADDR | LCD_WRITE;
	I2CMasterBuffer[LCD_PORT][1] = CURSOR_HOME;		/* address */
	Status = I2CEngine( LCD_PORT );
	return (Status);
}


/*===============================================================================
 Function        : LCDSetBackLight(uint8_t Data);
 Parameters		 : Data-> 1=ON, 0=OFF
 Description 	 : Controls backlight of LCD display
 Preconditions	 : uncomment LCD definition in Hardwareprofile.h
===============================================================================*/
uint32_t LCDSetBackLight(uint8_t Data)
{
	uint32_t Status;
	I2CWriteLength[LCD_PORT] = 2;
	I2CReadLength[LCD_PORT] = 0;
	I2CMasterBuffer[LCD_PORT][0] = LCD_ADDR | LCD_WRITE;
	if(Data==1)
		I2CMasterBuffer[LCD_PORT][1] = BL_ON;		/* address */
	else
		I2CMasterBuffer[LCD_PORT][1] = BL_OFF;

	Status = I2CEngine( LCD_PORT );
	return (Status);
}


/*===============================================================================
 Function        : LCDClearScreen();
 Parameters		 : None
 Description 	 : This function clears the contents of LCD screen and sets cursor
 	 	 	 	   to home position.
 Preconditions	 : uncomment LCD definition in Hardwareprofile.h
===============================================================================*/
uint32_t LCDClearScreen(void)
{
	uint32_t Status;
	I2CWriteLength[LCD_PORT] = 2;
	I2CReadLength[LCD_PORT] = 0;
	I2CMasterBuffer[LCD_PORT][0] = LCD_ADDR | LCD_WRITE;
	I2CMasterBuffer[LCD_PORT][1] = CLS;		/* address */
	Status = I2CEngine( LCD_PORT );
	return (Status);
}

#endif//LCD
