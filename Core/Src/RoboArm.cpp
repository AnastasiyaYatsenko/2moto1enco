#include "RoboArm.h"

RoboArm::RoboArm(uint8_t defaultAngleT, uint8_t defaultDistanseT) {
	defaultAngle = defaultAngleT;
	defaultDistanse = defaultDistanseT;
	startDWT();
}

int RoboArm::CloseGripper() {
	return 0;
}

int RoboArm::EmergencyStop() {

	HAL_GPIO_WritePin(EN1_GPIO_Port_M1, EN1_Pin_M1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN2_GPIO_Port_M2, EN2_Pin_M2, GPIO_PIN_SET);

	HAL_TIM_PWM_Stop(htim1M1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(htim2M2, TIM_CHANNEL_3);
	HAL_TIM_Base_Stop_IT(htim1M1);
	HAL_TIM_Base_Stop_IT(htim2M2);

	return 0;
}

int RoboArm::correctPosition() {
	HAL_TIM_PWM_Stop(htim1M1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(htim2M2, TIM_CHANNEL_3);
	HAL_TIM_Base_Stop_IT(htim1M1);
	HAL_TIM_Base_Stop_IT(htim2M2);

	float actualAngle = GetAngleEncoders(GetPosEncoders(1)); //angle
	uint16_t actualDistance = (GetAngleEncoders(GetPosEncoders(2)) * 6.45)
			/ (linearStepsMil * 360 / (motorStep * drvMicroSteps));

	if (lastPosAngle < actualAngle) {
		HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_SET);
	}

	if (lastPosLinear < actualDistance) {
		HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir2_Pin_M2, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir2_Pin_M2, GPIO_PIN_SET);
	}

	float difAngle = abs(actualAngle - lastPosAngle); //різниця між поточним кутом та попередньо встановленим
	uint16_t difDistance = abs(actualDistance - lastPosLinear); //різниця між поточним положенням в міліметрах та попередньо встановленним

	anglePsteps = (difAngle * (8 * motorStep * drvMicroSteps)) / 360; //angle to steps
	distPsteps = difDistance * linearStepsMil;

//	lastPosAngle = actualAngle;
//	lastPosLinear = actualDistance;

// 1, 2, 3, 4, 6, 8, 9, 12, 18, 24, 36 и 72 - Це можлива обрана максимальна швидкість для мотора з більшої кількістю кроків. Це дільник таймера

	float periodM1 = 1200;
	uint32_t psc = 72;

	float delimiter=1;
	float mnoj=1;

	if (anglePsteps > distPsteps) {

		htim1M1->Instance->PSC = psc;
		htim1M1->Instance->ARR = periodM1;
		htim1M1->Instance->CCR3 = periodM1/2;

		delimiter = anglePsteps / distPsteps;
		mnoj = ceil(periodM1 * delimiter);

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = mnoj;
		htim2M2->Instance->CCR3 = mnoj / 2;

	} else if (anglePsteps < distPsteps) {

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = periodM1;
		htim2M2->Instance->CCR3 = periodM1 / 2;

		delimiter = distPsteps / anglePsteps;
		mnoj = ceil(periodM1 * delimiter);

		htim1M1->Instance->PSC = psc;
		htim1M1->Instance->ARR = mnoj;
		htim1M1->Instance->CCR3 = mnoj / 2;
	}

	stateMoveM1 = true;
	stateMoveM2 = true;

	SetEnable(1, true);
	SetEnable(2, true);

	HAL_TIM_PWM_Start(htim1M1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim2M2, TIM_CHANNEL_3);
	HAL_TIM_Base_Start_IT(htim1M1);
	HAL_TIM_Base_Start_IT(htim2M2);


	return 0;

}

int RoboArm::Move2MotorsSimu(float angle, uint16_t distance) {

	// TIM1 Х  enc1 -  угол 360  -  8 оборотов движка на 1 оборот энкодера
	// TIM2  Y  enc2 - линейный -  6,4516129 оборотов движка (это целое линейное перемещение с запасом) на 1 оборот энкодера

	HAL_TIM_PWM_Stop(htim1M1, TIM_CHANNEL_3);      //остановили PWM таймера
	HAL_TIM_PWM_Stop(htim2M2, TIM_CHANNEL_3);
	HAL_TIM_Base_Stop_IT(htim1M1);				// остановили прерывание таймеров
	HAL_TIM_Base_Stop_IT(htim2M2);

	SetEnable(1, false);
	SetEnable(2, false);
	/* выставили в каку сторону ехать мотору*/

	if (lastPosAngle < angle) {
		HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_SET);
	} else if (lastPosAngle > angle) {
		HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_RESET);
	}
	if (lastPosLinear < distance) {
		HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir2_Pin_M2, GPIO_PIN_SET);
	} else if (lastPosLinear > distance) {
		HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir2_Pin_M2, GPIO_PIN_RESET);
	}

	float actualPosAngle = abs(lastPosAngle - angle);
	float actualPosDistance = abs(lastPosLinear - distance);

	anglePsteps = (actualPosAngle * (8 * motorStep * drvMicroSteps)) / 360; //angle to steps
	distPsteps = actualPosDistance * linearStepsMil; //steps to distanse

	float distPangle = ((distPsteps / (motorStep * drvMicroSteps)) * 360
			/ 6.45);

	lastPosAngle = angle;
	lastPosLinear = distance;

// 1, 2, 3, 4, 6, 8, 9, 12, 18, 24, 36 и 72 - Це можлива обрана максимальна швидкість для мотора з більшої кількістю кроків. Це дільник таймера

	float periodM1 = 1200;
	uint32_t psc = 72-1;

	float delimiter=1;
	float mnoj=1;

	if (anglePsteps > distPsteps) {

		htim1M1->Instance->PSC = psc;
		htim1M1->Instance->ARR = periodM1;
		htim1M1->Instance->CCR3 = periodM1/2;

		delimiter = anglePsteps / distPsteps;
		mnoj = ceil(periodM1 * delimiter);

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = mnoj;
		htim2M2->Instance->CCR3 = mnoj / 2;

	} else if (anglePsteps < distPsteps) {

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = periodM1;
		htim2M2->Instance->CCR3 = periodM1 / 2;

		delimiter = distPsteps / anglePsteps;
		mnoj = ceil(periodM1 * delimiter);

		htim1M1->Instance->PSC = psc;
		htim1M1->Instance->ARR = mnoj;
		htim1M1->Instance->CCR3 = mnoj / 2;
	}

	stateMoveM1 = true;
	stateMoveM2 = true;

	SetEnable(1, true);
	SetEnable(2, true);

	HAL_TIM_PWM_Start(htim1M1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim2M2, TIM_CHANNEL_3);
	HAL_TIM_Base_Start_IT(htim1M1);
	HAL_TIM_Base_Start_IT(htim2M2);

	return 0;
}

int RoboArm::factoryReset() {
	SetZeroEncoders();
	posNowAngle = defaultAngle;
	posNowDistance = 124;
}

int RoboArm::OpenGripper() {
	return 0;
}

int RoboArm::setPrintState(bool state) {
	if (state) {
		PrintAllState = true;
	} else {
		PrintAllState = false;
	}
	return 0;
}

bool RoboArm::getPrintState() {
	if (PrintAllState) {
		return true;
	} else {
		return false;
	}
}

int RoboArm::cringeFunction(bool state) {
	stateMovement[0] = stateMovement[1];
	stateMovement[1] = state;
}

int RoboArm::SetSettEncoders(SPI_HandleTypeDef &arm_hspi1T,
		GPIO_TypeDef *CS_GPIO_Port_Enc1T, uint16_t CS_Pin_Enc1T,
		GPIO_TypeDef *CS_GPIO_Port_Enc2T, uint16_t CS_Pin_Enc2T,
		uint8_t ResolutionEncodersT) {

	arm_hspi1 = &arm_hspi1T;
	CS_GPIO_Port_Enc1 = CS_GPIO_Port_Enc1T;
	CS_Pin_Enc1 = CS_Pin_Enc1T;
	CS_Pin_Enc2 = CS_Pin_Enc2T;
	CS_GPIO_Port_Enc2 = CS_GPIO_Port_Enc2T;
	ResolutionEncoders = ResolutionEncodersT;
	return 0;
}

float RoboArm::GetAngleEncoders(uint32_t encoderValue) {
	//	https://www.cuidevices.com/product/resource/amt22.pdf
	return calculateAngle(encoderValue, ResolutionEncoders);
}

uint32_t RoboArm::GetPosEncoders(uint8_t numEnc) {
	switch (numEnc) {
	case 1:
		posNowEnc1 = getPositionSPI(arm_hspi1, CS_GPIO_Port_Enc1, CS_Pin_Enc1,
				ResolutionEncoders);
		return posNowEnc1;
		break;
	case 2:
		posNowEnc2 = getPositionSPI(arm_hspi1, CS_GPIO_Port_Enc2, CS_Pin_Enc2,
				ResolutionEncoders);
		return posNowEnc2;
		break;
	default:
		return 1;
		break;
	}
	return 0;
}

int RoboArm::SetZeroEncoders() {

	HAL_Delay(300);
	getPositionSPI(arm_hspi1, CS_GPIO_Port_Enc1, CS_Pin_Enc1,
			ResolutionEncoders);
	getPositionSPI(arm_hspi1, CS_GPIO_Port_Enc2, CS_Pin_Enc2,
			ResolutionEncoders);
	//	resetAMT22();
	setZeroSPI(arm_hspi1, CS_GPIO_Port_Enc1, CS_Pin_Enc1);
	setZeroSPI(arm_hspi1, CS_GPIO_Port_Enc2, CS_Pin_Enc2);
	HAL_Delay(250);
	return 0;
}

int RoboArm::SetSettMotors(TIM_HandleTypeDef &htim1, TIM_HandleTypeDef &htim2,
		GPIO_TypeDef *Dir1_GPIO_Port_M1T, uint16_t Dir1_Pin_M1T,
		GPIO_TypeDef *Dir2_GPIO_Port_M2T, uint16_t Dir2_Pin_M2T,
		GPIO_TypeDef *EN1_GPIO_Port_M1T, uint16_t EN1_Pin_M1T,
		GPIO_TypeDef *EN2_GPIO_Port_M2T, uint16_t EN2_Pin_M2T) {
	htim1M1 = &htim1;
	htim2M2 = &htim2;

	Dir1_GPIO_Port_M1 = Dir1_GPIO_Port_M1T;
	Dir1_Pin_M1 = Dir1_Pin_M1T;
	Dir2_GPIO_Port_M2 = Dir2_GPIO_Port_M2T;
	Dir2_Pin_M2 = Dir2_Pin_M2T;
	EN1_GPIO_Port_M1 = EN1_GPIO_Port_M1T;
	EN1_Pin_M1 = EN1_Pin_M1T;
	EN2_GPIO_Port_M2 = EN2_GPIO_Port_M2T;
	EN2_Pin_M2 = EN2_Pin_M2T;

	SetEnable(1, false);
	SetEnable(2, false);

	return 0;
}

int RoboArm::SetEnable(uint16_t numMotor, bool state) {

	GPIO_PinState pinSet;

	if (state) {
		pinSet = GPIO_PIN_RESET;
	} else {
		pinSet = GPIO_PIN_SET;
	}

	if (numMotor == 1) {
		HAL_GPIO_WritePin(EN1_GPIO_Port_M1, EN1_Pin_M1, pinSet);
	} else if (numMotor == 2) {
		HAL_GPIO_WritePin(EN2_GPIO_Port_M2, EN2_Pin_M2, pinSet);
	} else {

	}
}

int RoboArm::MoveAngle(uint16_t angle) {

	return 0;
}

int RoboArm::MoveDistanse(uint16_t angle) {

	return 0;
}

