//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//中景园电子
//店铺地址：http://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//  文 件 名   : main.c
//  版 本 号   : v2.0
//  作    者   : Evk123
//  生成日期   : 2014-0101
//  最近修改   : 
//  功能描述   : 0.69寸OLED 接口演示例程(STM32F103ZE系列IIC)
//              说明: 
//              ----------------------------------------------------------------
//              GND   电源地
//              VCC   接5V或3.3v电源
//              SCL   接PD6（SCL）
//              SDA   接PD7（SDA）            
//              ----------------------------------------------------------------
//Copyright(C) 中景园电子2014/3/16
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////
#ifndef __OLED_H
#define __OLED_H			  	 
#include "stdlib.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#define u8 uint8_t
#define u32 uint32_t
#define OLED_MODE 1
#define SIZE 8
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    						  
//-----------------OLED IIC端口定义----------------  					   

#define OLED_SCLK_PIN GPIO_PIN_8
#define OLED_SDIN_PIN GPIO_PIN_9

#define OLED_SCLK_Clr() HAL_GPIO_WritePin(GPIOB,OLED_SCLK_PIN,GPIO_PIN_RESET);
#define OLED_SCLK_Set() HAL_GPIO_WritePin(GPIOB,OLED_SCLK_PIN,GPIO_PIN_SET)


#define OLED_SDIN_Clr() HAL_GPIO_WritePin(GPIOB,OLED_SDIN_PIN,GPIO_PIN_RESET)//DIN
#define OLED_SDIN_Set() HAL_GPIO_WritePin(GPIOB,OLED_SDIN_PIN,GPIO_PIN_SET)

 		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据




//OLED控制用函数
void OLED_WR_Byte(unsigned dat,unsigned cmd);  
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x,u8 y,u8 no);
void fill_picture(unsigned char fill_Data);
void Picture(void);
void OLED_IIC_Start(void);
void OLED_IIC_Stop(void);
void OLED_Write_IIC_Command(unsigned char IIC_Command);
void OLED_Write_IIC_Data(unsigned char IIC_Data);
void OLED_Write_IIC_Byte(unsigned char IIC_Byte);
void OLED_IIC_Wait_Ack(void);
void OLED_fuhao_write(unsigned char x,unsigned char y,unsigned char asc);
void OLED_Num_write(unsigned char x,unsigned char y,unsigned char asc) ;
void OLED_Float(unsigned char Y,unsigned char X,double real,unsigned char N);
void OLED_Float2(unsigned char Y,unsigned char X,double real,unsigned char N1,unsigned char N2);
void OLED_Num2(unsigned char x,unsigned char y, int number);
void OLED_Num3(unsigned char x,unsigned char y,int number); 
void OLED_Num4(unsigned char x,unsigned char y, int number);
void OLED_Num5(unsigned char x,unsigned char y,unsigned int number);


//常用
extern unsigned char Limit[] ;

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);


void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 Char_Size);

void OLED_ShowStringG4(uint8_t x, uint8_t y, uint8_t *str, uint8_t mode);
void OLED_ShowCharG4(uint8_t x, uint8_t y, uint8_t chr, uint8_t mode);
void OLED_printf(uint8_t mode ,uint8_t x, uint8_t y, const char *fmt, ...);
void OLED_Refresh_Gram(void);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
#endif  
