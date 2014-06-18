/*
===============================================================================
 Name        : lcd.h
 Author      : Nex Robotics Pvt Ltd
 Version     : 1.0
 Copyright   : Copyright (C) 2012
 Description : This file contains LCD module code header definition.
===============================================================================
*/

#ifndef LCD_H_
#define LCD_H_

void InitLCD(void);
uint32_t LCD_WriteCmd(uint8_t Addr);
uint32_t LCD_WriteStr(uint8_t Len,uint8_t *Data);
uint32_t LCDSetCursorPosition(uint8_t Row, uint8_t Col);
uint32_t LCDSetCursorHome(void);
uint32_t LCDSetBackLight(uint8_t Data);
uint32_t LCDClearScreen(void);
uint32_t LCD_PrintData(uint8_t Row,uint8_t Col,uint32_t Number,uint8_t Digits);
void Delay_LCD(void);


#endif /* LCD_H_ */
