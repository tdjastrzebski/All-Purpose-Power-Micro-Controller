#include "EEPROM24CW.h"

#include "string.h"  // for memcpy()

#define PAGE_SIZE 32
#define TIMEOUT 1000

// Possible improvements:
// * delay before next write operation, rather than after if elapsed time < 5ms (check uwTick / use GetTick())
// * refactor and create a separate static method for addrH/addrL calculation
// * add addres range check (memAddress/length) based on EEPROM size (16/32/64k)
//
// Note:
// * only one Read method is implemented since random read is same as sequential read
// * HAL_I2C_Mem_Write() method is used in a non-standard way

HAL_StatusTypeDef EEPROM24CW_WriteByte(I2C_HandleTypeDef* hi2c, uint8_t devAddress, uint16_t memAddress, uint8_t value) {
	// The first bit of the first address byte allows access to Configuration Registers
	// See: DS20005772A, 3.3.2 WORD ADDRESS BYTES
	// See: DS20005772A, 6.1 Byte Write
	uint8_t addrH = (memAddress & 0x7F00) >> 8;
	uint8_t addrL = (memAddress & 0x00FF);
	uint8_t data[2] = {addrL, value};
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, devAddress, addrH, I2C_MEMADD_SIZE_8BIT, data, 2, TIMEOUT);
	if (status != HAL_OK) return status;
	HAL_Delay(5);  // delay 5ms after each page is written to allow for the internal write cycle (TWC)
	return HAL_OK;
}

HAL_StatusTypeDef EEPROM24CW_WriteBytes(I2C_HandleTypeDef* hi2c, uint8_t devAddress, uint16_t memAddress, uint8_t* pData, uint16_t length) {
	// See: DS20005772A, 6.2 Page Write
	uint16_t pageAddress = (memAddress / PAGE_SIZE) * PAGE_SIZE;
	uint8_t byteCount;  // number of bytes to be written on the current page
	static uint8_t data[PAGE_SIZE + 1];

	do {
		if (memAddress + length >= pageAddress + PAGE_SIZE) {
			// data spans over more than one page
			byteCount = pageAddress + PAGE_SIZE - memAddress;
		} else {
			byteCount = length;
		}
		uint8_t addrH = (memAddress & 0x7F00) >> 8;
		uint8_t addrL = (memAddress & 0x00FF);
		data[0] = addrL;
		memcpy(data + 1, pData, byteCount);
		HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, devAddress, addrH, I2C_MEMADD_SIZE_8BIT, data, byteCount + 1, TIMEOUT);
		if (status != HAL_OK) return status;
		length -= byteCount;
		memAddress += byteCount;
		pageAddress += PAGE_SIZE;
		HAL_Delay(5);  // delay 5ms after each page is written to allow for the internal write cycle (TWC)
	} while (length > 0);

	return HAL_OK;
}

HAL_StatusTypeDef EEPROM24CW_ReadBytes(I2C_HandleTypeDef* hi2c, uint8_t devAddress, uint16_t memAddress, uint8_t* pData, uint16_t length) {
	// See: DS20005772A, 7.2 Random Read
	// See: DS20005772A, 7.3 Sequential Read
	uint8_t addrH = (memAddress & 0x7F00) >> 8;
	uint8_t addrL = (memAddress & 0x00FF);
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, devAddress, addrH, I2C_MEMADD_SIZE_8BIT, &addrL, 1, TIMEOUT);
	if (status != HAL_OK) return status;
	status = HAL_I2C_Master_Receive(hi2c, devAddress, pData, length, TIMEOUT);
	if (status != HAL_OK) return status;
	return HAL_OK;
}
