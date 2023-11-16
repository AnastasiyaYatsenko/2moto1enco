/*
  AMT22.h - STM32 library for ATM22 series absolute encoders by CUI Devices.
  Created by Simone Di Blasi, December 2020.
*/


#ifndef AMT22_H_
#define AMT22_H_

#include "stdint.h"
#include "DWT_Delay.h"
#define 	STM32F1
#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70

#define RES12           12
#define RES14           14

#define docDelay  3
#define docDelayLong 50 //min 40

#define ENCODER_RESOLUTION_12_BIT 4096
#define ENCODER_RESOLUTION_14_BIT 16384

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_tim.h"
#include "DWT_Delay.h"
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

int startDWT();
float calculateAngle(uint16_t encoderValue, uint8_t bitDepth);
void setCSLine (GPIO_TypeDef *encoderPort, uint16_t encoderPin, GPIO_PinState csLine);
uint8_t spiWriteRead(SPI_HandleTypeDef *hspi, uint8_t sendByte, GPIO_TypeDef* encoderPort, uint16_t encoderPin, uint8_t releaseLine);
uint16_t getPositionSPI(SPI_HandleTypeDef *hspi, GPIO_TypeDef *encoderPort,	uint16_t encoderPin, uint8_t resolution);
void setZeroSPI(SPI_HandleTypeDef *hspi, GPIO_TypeDef* encoderPort, uint16_t encoderPin);
void resetAMT22(SPI_HandleTypeDef *hspi, GPIO_TypeDef* encoderPort, uint16_t encoderPin);
void delay(uint32_t delayTime);

#ifdef __cplusplus
}
#endif


#endif /* SRC_AMT22_H_ */
