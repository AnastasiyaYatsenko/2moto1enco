/*
 * RoboArm.h
 *
 *  Created on: Oct 3, 2023
 *      Author: admin
 */

#ifndef ROBOARM_H_
#define ROBOARM_H_

#include "stm32f1xx_hal.h"
#include <math.h>
#include "AMT22.h"
//#include "TMC2209.h"

#define drvMicroSteps 8
//#define drvMicroSteps 16
#define spoolStep 20
#define motorStep 200
#define beltRatio 2

#define encAmendment 0
//#define encAmendment 5461
//#define encAmendment 10923

class RoboArm {
public:

	float linearStepsMil = motorStep * drvMicroSteps / ( beltRatio * spoolStep);
	//Settings for moto/rs
	TIM_HandleTypeDef *htim1M1;
	TIM_HandleTypeDef *htim2M2;
//	UART_HandleTypeDef *huart_tmc;

	GPIO_TypeDef *Dir1_GPIO_Port_M1;
	uint16_t Dir1_Pin_M1;
	GPIO_TypeDef *Dir2_GPIO_Port_M2;
	uint16_t Dir2_Pin_M2;
	GPIO_TypeDef *EN1_GPIO_Port_M1;
	uint16_t EN1_Pin_M1;
	GPIO_TypeDef *EN2_GPIO_Port_M2;
	uint16_t EN2_Pin_M2;

	float distPsteps = 0, anglePsteps = 0;

	//ENCODERS
	uint8_t ResolutionEncoders = 14;
	SPI_HandleTypeDef *arm_hspi1;
	uint16_t CS_Pin_Enc1;
	GPIO_TypeDef *CS_GPIO_Port_Enc1;
	uint16_t CS_Pin_Enc2;
	GPIO_TypeDef *CS_GPIO_Port_Enc2;

	uint32_t posNowEnc1, posNowEnc2;

	//TMC2209 drivers
//	TMC2209 tmcd_lin;
//	TMC2209 tmcd_ang;

	// 124 мм лыныйне перемышення   добавить к линейному перемщению при сбросе
	//Записать последние данные в память флеш

	//entity
	float posNowAngle;
	uint16_t posNowDistance;
	bool stateMoveM1 = false, stateMoveM2 = false;
	uint16_t defaultAngle, defaultDistanse; //стандартний кут //0 120 240 та дистанція 124 мм
	bool stateMovement[2];

	RoboArm(uint8_t, uint8_t);
	int OpenGripper();  //Open Gripper
	int CloseGripper();  // Close Gripper
	int GetLastPosition();                 //set last positions to encoder value
	int Move2MotorsSimu(float, float);  //move 2 mottors simultaneously
	int correctPosition();
	int correctPositionSimu();
	int MoveAngle(uint16_t angle); 			  //move angle motors
	int MoveDistanse(uint16_t dist);  		  //move distance motor
//	int SetMicrosteps(uint16_t microsteps_per_step); //set microsteps per step
	int SetSettEncoders(SPI_HandleTypeDef &arm_hspi1T,
			GPIO_TypeDef *CS_GPIO_Port_Enc1T, uint16_t CS_Pin_Enc1T,
			GPIO_TypeDef *CS_GPIO_Port_Enc2T, uint16_t CS_Pin_Enc2T,
			uint8_t ResolutionEncodersT);     //settings for encoders
	uint32_t GetPosEncoders(uint8_t); 	//get actually position encoders 1 or 2
	int SetZeroEncoders();					  //set zero position all encoders
	int SetSoftwareZero();                    //memorize current position as zero position
	float ShiftZeroInputAng(float);           //converts user angle into actual angle
	float ShiftZeroInputLin(float);     //converts user distance into actual distance
	float ShiftZeroOutputAng(float);          //converts actual angle into user angle
	float ShiftZeroOutputLin(float);    //converts actual distance into user distance
	float GetAngleEncoders(uint32_t);	      //get calculated Angle - pos value
	uint32_t GetPosTactEncoders(uint32_t);	  //get calculated position

	int setPrintState(bool); // flag to send status to uart
	bool getPrintState();

	int EmergencyStop();
	int SetSettMotors(TIM_HandleTypeDef &htim1, TIM_HandleTypeDef &htim2,
			GPIO_TypeDef *Dir1_GPIO_Port_M1T, uint16_t Dir1_Pin_M1T,
			GPIO_TypeDef *Dir2_GPIO_Port_M2T, uint16_t Dir2_Pin_M2T,
			GPIO_TypeDef *EN1_GPIO_Port_M1T, uint16_t EN1_Pin_M1T,
			GPIO_TypeDef *EN2_GPIO_Port_M2T, uint16_t EN2_Pin_M2T);
//			UART_HandleTypeDef &huart_tmcT);

	int SetEnable(uint16_t numMotor, bool state);

	int saveDatatoFlash();
	int readDataonFlash();
	int checkPosition();
	int factoryReset();
	int getStateArm();

	int cringeFunction(bool); //set log state movement

private:

	bool PrintAllState = false;

	// Данные о текущем положении моторов
	uint16_t lastPosLinear = 0;
	float lastPosAngle = 0;
	bool lastPosGripper = false;

	//zero position info
	float ang_zero = 0;
	float lin_zero = 0;
};

#endif /* ROBOARM_H_ */
