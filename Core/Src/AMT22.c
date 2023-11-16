#include "AMT22.h"


void setCSLine(GPIO_TypeDef *encoderPort, uint16_t encoderPin,
		GPIO_PinState csLine) {
	HAL_GPIO_WritePin(encoderPort, encoderPin, csLine);

}


uint8_t spiWriteRead(SPI_HandleTypeDef *hspi, uint8_t sendByte,
		GPIO_TypeDef *encoderPort, uint16_t encoderPin, uint8_t releaseLine) {
	uint8_t data;
	setCSLine(encoderPort, encoderPin, GPIO_PIN_RESET);
	delay(docDelay);
	HAL_SPI_TransmitReceive(hspi, &sendByte, &data, 1, 10);
	delay(docDelay);
	setCSLine(encoderPort, encoderPin, releaseLine);
	return data;
}

uint16_t getPositionSPI(SPI_HandleTypeDef *hspi, GPIO_TypeDef *encoderPort,
		uint16_t encoderPin, uint8_t resolution) {

	DWT_Delay_Init();

	uint16_t currentPosition = 0;
	uint8_t binaryArray[16];
	currentPosition = spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 0) << 8;
	delay(docDelay);
	currentPosition |= spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 1);
	for (int i = 0; i < 16; i++)
		binaryArray[i] = (0x01) & (currentPosition >> (i));
	if ((binaryArray[15]
			== !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9]
					^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3]
					^ binaryArray[1]))
			&& (binaryArray[14]
					== !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8]
							^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2]
							^ binaryArray[0]))) {
		currentPosition &= 0x3FFF;
	} else {
		currentPosition = 0xFFFF; //bad position
	}

	//If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
	if ((resolution == RES12) && (currentPosition != 0xFFFF))
		currentPosition = currentPosition >> 2;
	return currentPosition;
}

void setZeroSPI(SPI_HandleTypeDef *hspi, GPIO_TypeDef *encoderPort,
		uint16_t encoderPin) {

	spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 0);
	delay( docDelay);
	spiWriteRead(hspi, AMT22_ZERO, encoderPort, encoderPin, 1);
	delay( docDelayLong + docDelayLong);

}

float calculateAngle(uint16_t encoderValue, uint8_t bitDepth) {
	float angle = 0.0;
	if (bitDepth == 12) {
		angle = ((float) encoderValue * 360.00)/ ENCODER_RESOLUTION_12_BIT;
	} else if (bitDepth == 14) {
		angle = ((float) encoderValue * 360.00)/ ENCODER_RESOLUTION_14_BIT;
	}

	// до 0.2 градусів за документацією
	//angle = roundf(angle * 100.0) / 100.0;
	return angle;
}

void resetAMT22(SPI_HandleTypeDef *hspi, GPIO_TypeDef *encoderPort,
		uint16_t encoderPin) {
	spiWriteRead((SPI_HandleTypeDef*) &hspi, AMT22_NOP, encoderPort, encoderPin,
			0);
	delay(docDelay);
	spiWriteRead((SPI_HandleTypeDef*) &hspi, AMT22_RESET, encoderPort,	encoderPin, 1);
	delay(docDelayLong);
}

//блокуючий хуйовий метод, треба через неблокуючий мабуть
void delay(uint32_t delayTime) {

	DWT_Delay_us(delayTime);

//
//	__HAL_RCC_TIM2_CLK_ENABLE();
//	HAL_TIM_Base_Start(timer);
//	uint32_t start_time = __HAL_TIM_GET_COUNTER(timer);
//	while ((__HAL_TIM_GET_COUNTER(timer) - start_time) < delayTime) {
//		// wait suka
//	}
//	HAL_TIM_Base_Stop(timer);
//	__HAL_RCC_TIM2_CLK_DISABLE();
}

int startDWT(){

	 DWT_Delay_Init();

}

