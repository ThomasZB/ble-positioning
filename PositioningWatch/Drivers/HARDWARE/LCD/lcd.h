#ifndef __LCD_H
#define __LCD_H

#include "main.h"


#define USE_HORIZONTAL 1  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 240
#define LCD_H 280

#else
#define LCD_W 280
#define LCD_H 240
#endif


//画笔颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE           	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //棕色
#define BRRED 			 0XFC07 //棕红色
#define GRAY  			 0X8430 //灰色
#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
#define LIGHTGREEN     	 0X841F //浅绿色
#define LGRAY 			 0XC618 //浅灰色(PANNEL),窗体背景色
#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)


/**
 * @breif LCD地址结构体，地址0是读/写命令，1是读/写数据
 */
typedef struct
{
	volatile uint8_t LCD_REG;
	volatile uint8_t LCD_RAM;
} LCD_TypeDef;

/* 定义LCD，读写LCD就是读写其地址 */
#define LCD_BASE 		((uint32_t)0x6C000000)
#define LCD             ((LCD_TypeDef *) LCD_BASE)



/* LCD驱动 */
void LCD_Init(void);
void LCD_Reset(void);
void LCD_BLK_Set(uint8_t level);
void LCD_WR_DATA(volatile uint16_t data);
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);

/* LCD绘图 */
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend, uint16_t yend, uint16_t color);



#endif

