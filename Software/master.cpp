/*---------------------------------------------------------------------------------------------
 *  Copyright (c) 2022 Tomasz Jastrzębski. All rights reserved.
 *  Licensed under the MIT License. See License.txt in the project root for license information.
 *--------------------------------------------------------------------------------------------*/

#include "master.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <cstdlib>

#include "EEPROM24CW.h"
#include "GUI_Paint.h"
#include "OLED_Driver.h"
#include "lv_font.h"
#include "main.h"
#include "stm32g0xx_hal_adc_ex.h"
#include "stm32g0xx_hal_tim.h"

#define PWM_FREQUENCY 100          // Hz
#define ADC_ACTIVE_CHANEL_COUNT 2  // number of conversions
#define TEMP_MIN 200
#define TEMP_MAX 450
#define VIN_MIN 22
#define VIN_MAX 26
#define LOOP_DELAY 100    // ms
#define PERSIST_DELAY 30  // N * LOOP_DELAY
#define UART_TX_TIMEOUT 0xFFFFFFFF
#define BLINK_PATTERN_SLOW 0x0000FFFF
#define BLINK_PATTERN_FAST 0xAAAAAAAA
#define BLINK_PATTERN_MED 0xCCCCCCCC
#define EEPROM_SIZE (64 * 1024)  // 64k
#define EEPROM_ADDRESS 0xA0

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern DAC_HandleTypeDef hdac1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;
extern PCD_HandleTypeDef hpcd_USB_DRD_FS;

extern lv_font_t liberation_mono;

static lv_font_t* _pFont = &liberation_mono;
static uint16_t _adcBuffer[ADC_ACTIVE_CHANEL_COUNT];  // buffer for store the results of the ADC conversion
static uint16_t _tempSetCurrent = TEMP_MIN;
static uint16_t _tempSetPrevious = 0;
static bool _isDirty = false;  // if (true) display needs refresh
static bool _isPwmOn = false;
static bool _isPwmPaused = false;
static bool _startPwmFlag = false;  // PWM should start
static bool _stopPwmFlag = false;   // PWM should stop
static bool _vMode = true;          // voltage (true) or temp (false) display mode
static char _displayTextBuffer[32];
static int16_t _tempCurrent = 0;           // measured temperature [°C]
static int16_t _tempPrevious = -1;         // last measured temperature [°C]
static int16_t _vInCurrent = 0;            // measured input voltage [V]
static int16_t _vInPrevious = -1;          // last input voltage [V]
static int16_t _persistDelayCounter = -1;  // time remaining to persist temp settings (N * LOOP_DELAY)
static uint16_t _pwmCounterPeriod;
static uint16_t _storageAddress = 0;
static uint8_t* _oledImage;

static uint8_t DrawLvString(uint8_t x, uint8_t y, const char* str, lv_font_t* _pFont);
static void PwmStart(void);
static void PwmStop(void);
static void PwmPause(void);
static void UartTransmit(const char* pString, ...);

void PreInit() {}

void SysInit() {}

void Init() {}

void PostInit(void) {
	// calibrate ADC before start
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	// start ADC in DMA mode and declare the buffer where store the results
	// note: Length param is a number of conversions, not bytes as incorrectly stated in HAL_ADC_Start_DMA() function docs
	// if (HAL_ADC_Start_IT(&hadc1)!= HAL_OK) {
	// 	Error_Handler();
	// }
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)_adcBuffer, ADC_ACTIVE_CHANEL_COUNT) != HAL_OK) {
		Error_Handler();
	}

	// set PWM timer Period and frequency (ARR autoreload registry)
	uint16_t prescaler = htim1.Instance->PSC + 1;
	uint32_t timerClockFrequency = TIMERS_CLOCK_FREQ / prescaler;
	_pwmCounterPeriod = (timerClockFrequency / PWM_FREQUENCY);
	__HAL_TIM_SET_AUTORELOAD(&htim1, _pwmCounterPeriod - 1);          // set ARR: counter period (autoreload) registry
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, _pwmCounterPeriod);  // set CCR1: capture compare registry

	// initialize OLED display
	OLED_Init();
	Driver_Delay_ms(500);
	uint16_t imagesize = ((OLED_WIDTH % 8 == 0) ? (OLED_WIDTH / 8) : (OLED_WIDTH / 8 + 1)) * OLED_HEIGHT;
	if ((_oledImage = (uint8_t*)malloc(imagesize)) == NULL) {
		printf("Failed to allocate OLED memory buffer.\r\n");
	}
	Paint_NewImage(_oledImage, OLED_WIDTH, OLED_HEIGHT, 180, BLACK);
	Paint_SelectImage(_oledImage);

	// display "???""
	Paint_Clear(BLACK);
	DrawLvString(0, 0, "???", _pFont);
	OLED_Display(_oledImage);

	// restore persisted temp setting
	_storageAddress = 0;
	uint16_t persistedValue;
	while (true) {
		// find the first non-zero value in EEPROM memory range
		EEPROM24CW_ReadBytes(&hi2c2, EEPROM_ADDRESS, _storageAddress, (uint8_t*)&persistedValue, 2);  // note: do not care about endianness as long as it is consistent
		if (persistedValue != 0) {
			// non-zero value found
			if (persistedValue > TEMP_MAX || persistedValue < TEMP_MIN) persistedValue = TEMP_MIN;
			_tempSetCurrent = persistedValue;
			break;
		}
		_storageAddress += 2;
		if (_storageAddress >= EEPROM_SIZE) {
			// end of EEPROM address space reached
			_storageAddress = 0;
			break;
		}
	};

	__HAL_TIM_SET_COUNTER(&htim1, _tempSetCurrent);
	// start 1Hz ADC trigger
	// note: no need to start this timer in interrupt mode
	if (HAL_TIM_Base_Start(&htim6) != HAL_OK) {
		Error_Handler();
	}
}

void MainLoop(void) {
	static uint32_t blinkPattern = BLINK_PATTERN_SLOW;
	static uint8_t blinkPatternShift = 0;

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (blinkPattern >> blinkPatternShift) & 0x01 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	++blinkPatternShift %= 32;

	if (_vInCurrent < VIN_MIN || _vInCurrent > VIN_MAX) {
		// VIn too high or too low to operate
		if (!_vMode) _isDirty = true;
		_vMode = true;

		if (_vInCurrent != _vInPrevious) {
			if (_vInCurrent < VIN_MIN) {
				sprintf(_displayTextBuffer, "%dV", _vInCurrent);
			} else {
				sprintf(_displayTextBuffer, "%dV", _vInCurrent);
			}
			_isDirty = true;
		}
		if (_isPwmOn) {
			PwmPause();
			_isDirty = true;
		}
	} else {
		// Vin within operating limits
		if (_vMode) _isDirty = true;
		_vMode = false;

		if (_startPwmFlag) {
			PwmStart();
			sprintf(_displayTextBuffer, "%d", _tempSetCurrent);
			_isDirty = true;
		} else if (_stopPwmFlag) {
			PwmStop();
			sprintf(_displayTextBuffer, "---");
			_isDirty = true;
		} else if (_isPwmOn) {
			if (_isPwmPaused) {
				// PWM as paused - restart
				PwmStart();
				sprintf(_displayTextBuffer, "%d", _tempSetCurrent);
				_isDirty = true;
			} else if (_tempSetPrevious != _tempSetCurrent) {
				sprintf(_displayTextBuffer, "%d", _tempSetCurrent);
				_isDirty = true;
			}
		} else if (!_isPwmOn && _isDirty) {
			// pwm went off
			sprintf(_displayTextBuffer, "---");
		}
	}

	if (_isPwmOn && !_isPwmPaused) {
		if (_tempPrevious != _tempCurrent || _tempSetPrevious != _tempSetCurrent) {
			// adjust PWM duty cycle
			int16_t tempDiff = _tempSetCurrent - _tempCurrent;

			if (tempDiff <= 2) {
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);  // set duty cycle to 0%
				blinkPattern = BLINK_PATTERN_SLOW;
			} else if (tempDiff <= 20) {
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, round(_pwmCounterPeriod * 0.4));  // set duty cycle 40%
				blinkPattern = BLINK_PATTERN_FAST;
			} else {
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, _pwmCounterPeriod);  // set duty cycle 100%
				blinkPattern = BLINK_PATTERN_FAST;
			}
		}
	} else {
		// PWM off or paused
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);  // set duty cycle to 0%
		blinkPattern = BLINK_PATTERN_SLOW;
	}

	_vInPrevious = _vInCurrent;
	_tempPrevious = _tempCurrent;
	_tempSetPrevious = _tempSetCurrent;
	_startPwmFlag = false;
	_stopPwmFlag = false;

	if (_isDirty) {
		Paint_Clear(BLACK);
		DrawLvString(0, 6, _displayTextBuffer, _pFont);
		OLED_Display(_oledImage);
		_isDirty = false;
	}

	if (_persistDelayCounter < 0) {
		// do nothing
	} else if (_persistDelayCounter == 0) {
		// delay time elapsed - persist set temperature at the next storage address
		if (_storageAddress + 2 >= EEPROM_SIZE) {
			EEPROM24CW_WriteBytes(&hi2c2, EEPROM_ADDRESS, 0x00, (uint8_t*)&_tempSetCurrent, 2);
		} else {
			EEPROM24CW_WriteBytes(&hi2c2, EEPROM_ADDRESS, _storageAddress + 2, (uint8_t*)&_tempSetCurrent, 2);
		}
		// zero the value stored at the current address
		const uint16_t zero = 0;
		EEPROM24CW_WriteBytes(&hi2c2, EEPROM_ADDRESS, _storageAddress, (uint8_t*)&zero, 2);
		_storageAddress += 2;
		if (_storageAddress >= EEPROM_SIZE) _storageAddress = 0;
		_persistDelayCounter = -1;
	} else if (_persistDelayCounter > 0) {
		// count down time to persist settings after the last temp adjust
		_persistDelayCounter--;
	}

	HAL_Delay(LOOP_DELAY);  // delay 100ms
}

/**
  * @brief  Draw string using LVGL library fonts.
  * @retval drawn string width
  */
uint8_t DrawLvString(uint8_t posX, uint8_t posY, const char* str, lv_font_t* pFont) {
	lv_font_glyph_dsc_t dsc;
	uint8_t newPosX = posX;

	while (*str) {
		// for each char in the string
		uint32_t c = *str;         // current char
		uint32_t cn = *(str + 1);  // next char
		bool ret = lv_font_get_glyph_dsc(pFont, &dsc, c, cn);

		if (!ret) {
			// char not found - draw '?'
			c = 63;
			// get glyph descriptor
			ret = lv_font_get_glyph_dsc(pFont, &dsc, c, 0);
			if (!ret) {
				// '?' char not found either
				str++;
				continue;
			}
		}

		// get glyph bitmap
		const uint8_t* bitmap = lv_font_get_glyph_bitmap(pFont, c);
		// get bitmap vertical offset
		int16_t offsetY = pFont->line_height - pFont->base_line - dsc.ofs_y - dsc.box_h;  // box offset from the top

		uint8_t z = 0;
		// draw glyph bitmap
		for (uint16_t y = offsetY; y < dsc.box_h + offsetY; y++) {
			for (uint16_t x = dsc.ofs_x; x < dsc.box_w + dsc.ofs_x; x++) {
				Paint_SetPixel(newPosX + x, posY + y, ((*bitmap << z) & 0x80) ? WHITE : BLACK);
				z++;
				z %= 8;
				if (z == 0) bitmap++;
			}
		}

		str++;
		newPosX += dsc.adv_w;
	}

	return newPosX - posX;
}

/**
  * @brief  Input Capture callback in non-blocking mode - called when encoder is rotated
  * @param  htim TIM IC handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
	static uint16_t tempSetCurrent;

	if (htim == &htim3) {
		tempSetCurrent = __HAL_TIM_GET_COUNTER(htim);
		// note: check __HAL_TIM_GET_COUNTER, this event fires also without counter change
		if (tempSetCurrent == _tempSetCurrent) return;  // no counter change - exit
		_tempSetCurrent = tempSetCurrent;
		_persistDelayCounter = PERSIST_DELAY;

		if (_tempSetCurrent < TEMP_MIN) {
			_tempSetCurrent = TEMP_MIN;
			__HAL_TIM_SET_COUNTER(htim, _tempSetCurrent);
		} else if (_tempSetCurrent > TEMP_MAX) {
			_tempSetCurrent = TEMP_MAX;
			__HAL_TIM_SET_COUNTER(htim, _tempSetCurrent);
		}

		UartTransmit("Enc counter: %u\n", _tempSetCurrent);
		_isDirty = true;
	}
}

/**
  * @brief  EXTI line detection callbacks - called when encoder button is pressed
  * @param  GPIO_Pin Specifies the pins connected to the EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == BTN_Pin) {
		_isDirty = true;
		if (_isPwmOn) {
			_startPwmFlag = false;
			_stopPwmFlag = true;
		} else {
			_startPwmFlag = true;
			_stopPwmFlag = false;
		}
	}
}

/**
  * @brief  Conversion complete callback in non-blocking mode.
  * @param  hadc ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	// ADC_A (IN0) measured V - temp
	uint16_t adc = _adcBuffer[0];
	const float G = 5.0 + (80000.0 / 3160.0);  // V/V gain
	const float Coef = 0.072285781;            // Ω/°C
	const float Offset = 20.47139855;          // Ω
	float uR8 = 1.5 - (adc * 3.0 / (4095.0 * G)) + (1.5 / G);
	float temp = (1.0 / Coef) * ((3.0 * 330.0 / uR8) - (294.0 + 330.0 + Offset));
	UartTransmit("ADC_A = %lu (%.2f°C)\n", adc, temp);
	_tempCurrent = round(temp);

	// ADC_B (IN1) VCC
	adc = _adcBuffer[1];
	float vIn = (adc / 4095.0) * 3.0 * 11.0;
	UartTransmit("ADC_B = %lu (%.2fV)\n", adc, vIn);
	_vInCurrent = round(vIn);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc) {
	UartTransmit("ADC ERROR, state: %lu, code: %lu\n", hadc->State, hadc->ErrorCode);
}

void PwmStart(void) {
	if (_isPwmOn) return;
	_isPwmOn = true;
	_isPwmPaused = false;

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TIMERS_CLOCK_FREQ / 1000 * _tempSetCurrent / 10);

	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
		//Error_Handler();
	}

	if (HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2) != HAL_OK) {
		//Error_Handler();
	}
}

void PwmStop(void) {
	if (!_isPwmOn) return;
	_isPwmOn = false;
	if (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2) != HAL_OK) {
		//Error_Handler();
	}

	if (HAL_TIM_Encoder_Stop_IT(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2) != HAL_OK) {
		//Error_Handler();
	}
}

void PwmPause(void) {
	if (!_isPwmOn | _isPwmPaused) return;
	_isPwmPaused = true;
	PwmStop();
}

void UartTransmit(const char* format_msg, ...) {
	static char textBuffer[64];
	va_list args;
	va_start(args, format_msg);
	int len = vsprintf(textBuffer, format_msg, args);
	va_end(args);
	HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart1, (uint8_t*)textBuffer, len, UART_TX_TIMEOUT);
	if (ret != HAL_OK) {
		Error_Handler();
	}
}
