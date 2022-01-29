/******************************************************************************
**************************Hardware interface layer*****************************
* | file      	:	DEV_Config.c
* |	version			:	V1.0
* | date				:	2020-06-17
* | function		:	Provide the hardware underlying interface	
******************************************************************************/
#include "DEV_Config.h"
#include "stm32g0xx_hal_i2c.h"
//#include "i2c.h"

#include "stm32g0xx_hal_spi.h"
//#include "spi.h"

//#include "usart.h"
#include <stdio.h>		//printf()
#include <string.h>
#include <stdlib.h>

extern I2C_HandleTypeDef H_I2C;
extern SPI_HandleTypeDef H_SPI;

/********************************************************************************
function:	System Init
note:
	Initialize the communication method
********************************************************************************/
uint8_t System_Init(void)
{
#if USE_SPI_4W
  printf("USE 4wire spi\r\n");
#elif USE_IIC
  printf("USE i2c\r\n");
	OLED_DC_0;//DC = 1 >> Address = 0x3d 
#endif
  return 0;
}

void System_Exit(void)
{
}

/********************************************************************************
function:	Hardware interface
note:
	SPI4W_Write_Byte(value) : 
		HAL library hardware SPI
		Register hardware SPI
		Gpio analog SPI
	I2C_Write_Byte(value, cmd):
		HAL library hardware I2C
********************************************************************************/
uint8_t SPI4W_Write_Byte(uint8_t value)
{
#if 1
    HAL_SPI_Transmit(&H_SPI, &value, 1, 500);
#elif 0
    char i;
    for(i = 0; i < 8; i++) {
        SPI_SCK_0;
        if(value & 0X80)
            SPI_MOSI_1;
        else
            SPI_MOSI_0;
        Driver_Delay_us(10);
        SPI_SCK_1;
        Driver_Delay_us(10);
        value = (value << 1);
    }
#else
    __HAL_SPI_ENABLE(&H_SPI);
    H_SPI.Instance->CR2 |= (1) << 12;

    while((H_SPI.Instance->SR & (1 << 1)) == 0)
        ;

    *((__IO uint8_t *)(&H_SPI.Instance->DR)) = value;

    while(H_SPI.Instance->SR & (1 << 7)) ; //Wait for not busy

    while((H_SPI.Instance->SR & (1 << 0)) == 0) ; // Wait for the receiving area to be empty

    return *((__IO uint8_t *)(&H_SPI.Instance->DR));
#endif
}

void I2C_Write_Byte(uint8_t value, uint8_t Cmd)
{
    int Err = 0;
    uint8_t W_Buf[2] ;
    W_Buf[0] = Cmd;
    W_Buf[1] = value;
    if(HAL_I2C_Master_Transmit(&H_I2C, (0X3C << 1) | 0X00, W_Buf, 2, 0x10) != HAL_OK) {
        Err++;
        if(Err == 1000) {
            printf("send error\r\n");
            return ;
        }
    }
}

/********************************************************************************
function:	Delay function
note:
	Driver_Delay_ms(xms) : Delay x ms
	Driver_Delay_us(xus) : Delay x us
********************************************************************************/
void Driver_Delay_ms(uint32_t xms)
{
    HAL_Delay(xms);
}

void Driver_Delay_us(uint32_t xus)
{
    int j;
    for(j=xus; j > 0; j--);
}
