/******************************************************************************
***************************Intermediate driver layer***************************
* | file      :	OLED_Driver.c
* |	version		:	V1.0
* | date			:	2020-06-17
* | function	:	SH1107 Drive function
	
note:
Image scanning:
Please use progressive scanning to generate images or fonts			
******************************************************************************/
#ifndef __OLED_DRIVER_H
#define __OLED_DRIVER_H		

#ifdef __cplusplus
extern "C" {
#endif

#include "DEV_Config.h"

/********************************************************************************
function:	
		Define the full screen height length of the display
********************************************************************************/
#define OLED_WIDTH  128//OLED width
#define OLED_HEIGHT 64 //OLED height

//function
void OLED_Init(void);
void OLED_WriteReg(uint8_t Reg);
void OLED_WriteData(uint8_t Data);
void OLED_Clear(void);
void OLED_Display(UBYTE *Image);
UBYTE reverse(UBYTE temp);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  
	 
