#ifndef __EEPROM24CW_H
#define __EEPROM24CW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"

HAL_StatusTypeDef EEPROM24CW_WriteByte(I2C_HandleTypeDef* hi2c, uint8_t devAddress, uint16_t memAddress, uint8_t value);
HAL_StatusTypeDef EEPROM24CW_WriteBytes(I2C_HandleTypeDef* hi2c, uint8_t devAddress, uint16_t memAddress, uint8_t* pData, uint16_t length);
HAL_StatusTypeDef EEPROM24CW_ReadBytes(I2C_HandleTypeDef* hi2c, uint8_t devAddress, uint16_t memAddress, uint8_t* pData, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM24CW_H */
