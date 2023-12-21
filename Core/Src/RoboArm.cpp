#include "RoboArm.h"
//#include "TMC2209.h"

RoboArm::RoboArm(uint8_t defaultAngleT, uint8_t defaultDistanseT) {
	defaultAngle = defaultAngleT;
	defaultDistanse = defaultDistanseT;


	startDWT();
}

int RoboArm::CloseGripper() {
	return 0;
}

int RoboArm::EmergencyStop() {

	HAL_GPIO_WritePin(En1_GPIO_Port_M1, En1_Pin_M1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(En2_GPIO_Port_M2, En2_Pin_M2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(En3_GPIO_Port_M3, En3_Pin_M3, GPIO_PIN_SET);

	HAL_TIM_PWM_Stop(htim1M1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(htim2M2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(htim3M3, TIM_CHANNEL_3);

	HAL_TIM_Base_Stop_IT(htim1M1);
	HAL_TIM_Base_Stop_IT(htim2M2);
	HAL_TIM_Base_Stop_IT(htim3M3);

	return 0;
}

/*
int RoboArm::MoveCorrectPosition() {
	HAL_TIM_PWM_Stop(htim1M1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(htim2M2, TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT(htim1M1);
	HAL_TIM_Base_Stop_IT(htim2M2);

	float actualAngle = GetAngleEncoders(GetPosEncoders(1));// + defaultAngle; //angle
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
		htim1M1->Instance->CCR1 = periodM1/2;

		delimiter = anglePsteps / distPsteps;
		mnoj = ceil(periodM1 * delimiter);

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = mnoj;
		htim2M2->Instance->CCR2 = mnoj / 2;

	} else if (anglePsteps < distPsteps) {

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = periodM1;
		htim2M2->Instance->CCR2 = periodM1 / 2;

		delimiter = distPsteps / anglePsteps;
		mnoj = ceil(periodM1 * delimiter);

		htim1M1->Instance->PSC = psc;
		htim1M1->Instance->ARR = mnoj;
		htim1M1->Instance->CCR1 = mnoj / 2;
	}

	stateMoveM1 = true;
	stateMoveM2 = true;

	SetEnable(1, true);
	SetEnable(2, true);

	HAL_TIM_PWM_Start(htim1M1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim2M2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(htim1M1);
	HAL_TIM_Base_Start_IT(htim2M2);


	return 0;

}
*/

int RoboArm::GetLastPosition() {
	int attempts = 0;
	uint32_t posnowT_ang = GetPosEncoders(1);
	while(posnowT_ang == 0xFFFF && ++attempts < 3)
		posnowT_ang = GetPosEncoders(1);
	lastPosAngle_Enc = GetAngleEncoders(posnowT_ang);
	attempts = 0;
	uint32_t posnowT_lin = GetPosEncoders(2);
	while(posnowT_lin == 0xFFFF && ++attempts < 3)
		posnowT_lin = GetPosEncoders(2);
	float pos = GetAngleEncoders(posnowT_lin);
	lastPosLinear_Enc = pos*distMax/360.0;
	return 0;
}

int RoboArm::Move2Motors(float angle, float distance) {

	// TIM1 Х  enc1 -  угол 360  -  8 оборотов движка на 1 оборот энкодера
	// TIM2  Y  enc2 - линейный -  6,4516129 оборотов движка (это целое линейное перемещение с запасом) на 1 оборот энкодера

	HAL_TIM_PWM_Stop(htim1M1, TIM_CHANNEL_1);      //остановили PWM таймера
	HAL_TIM_PWM_Stop(htim2M2, TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT(htim1M1);				// остановили прерывание таймеров
	HAL_TIM_Base_Stop_IT(htim2M2);

	SetEnable(1, false);
	SetEnable(2, false);

	GetLastPosition(); //update -> lastPosAngle lastPosLinear from ENCODER
	float lastPosAngle = ShiftZeroAng(lastPosAngle_Enc);
	float lastPosLinear = ShiftZeroLin(lastPosLinear_Enc);

	float pos_ang = abs(lastPosAngle - angle);
	float inverse_pos_ang = abs(360.0 - pos_ang);
	float actualPosAngle;

	/* виставили в яку сторону ехать мотору*/
	if (inverse_pos_ang < pos_ang) {
		actualPosAngle = inverse_pos_ang;
		if (lastPosAngle < angle) {
			HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_RESET);
//			tmcd_angle.enableInverseMotorDirection();
		} else if (lastPosAngle > angle) {
			HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_SET);
//			tmcd_angle.disableInverseMotorDirection();
		}
	}
	else {
		actualPosAngle = pos_ang;
		if (lastPosAngle < angle) {
			HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_SET);
//			tmcd_angle.disableInverseMotorDirection();
		} else if (lastPosAngle > angle) {
			HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_RESET);
//			tmcd_angle.enableInverseMotorDirection();
		}
	}

//	if (lastPosLinear < distance) {
//		HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir2_Pin_M2, GPIO_PIN_SET);
//	} else if (lastPosLinear > distance) {
//		HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir2_Pin_M2, GPIO_PIN_RESET);
//	}
	if (lastPosLinear < distance) {
//			HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir2_Pin_M2, GPIO_PIN_RESET);
			tmcd_linear.enableInverseMotorDirection();
		} else if (lastPosLinear > distance) {
//			HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir2_Pin_M2, GPIO_PIN_SET);
			tmcd_linear.disableInverseMotorDirection();
		}

//	actualPosAngle = abs(lastPosAngle - angle);
	float actualPosDistance = abs(lastPosLinear - distance);

	//set microstepping TODO
	anglePsteps = (actualPosAngle * (8 * motorStep * drvMicroSteps)) / 360; //angle to steps
//	anglePsteps = anglePsteps+(anglePsteps*0.05);
	distPsteps = actualPosDistance * linearStepsMil; //steps to distanse
//	distPsteps = distPsteps+ (distPsteps*0.05);

//	float distPangle = ((distPsteps / (motorStep * drvMicroSteps)) * 360/ 6.4516129);

	//TODO last position from encoder
//	lastPosAngle = angle;
//	lastPosLinear = distance;

// 1, 2, 3, 4, 6, 8, 9, 12, 18, 24, 36 и 72 - Це можлива обрана максимальна швидкість для мотора з більшої кількістю кроків. Це дільник таймера

//	uint32_t periodM1 = 1200; //reduse to 600-400 mks for 32 microsteps
	uint32_t periodM1 = 600;
	uint32_t psc = 72-1;

	float delimiter=1;
	float mnoj=1;

	if (anglePsteps > distPsteps) {

		htim1M1->Instance->PSC = psc;
		htim1M1->Instance->ARR = periodM1;
		htim1M1->Instance->CCR1 = periodM1/2;

		delimiter = float(anglePsteps) / float(distPsteps);
		mnoj = ceil(periodM1 * delimiter);

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = mnoj;
		htim2M2->Instance->CCR2 = mnoj / 2;

	} else if (anglePsteps < distPsteps) {

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = periodM1;
		htim2M2->Instance->CCR2 = periodM1 / 2;

		delimiter = float(distPsteps) / float(anglePsteps);
		mnoj = ceil(periodM1 * delimiter);

		htim1M1->Instance->PSC = psc;
		htim1M1->Instance->ARR = mnoj;
		htim1M1->Instance->CCR1 = mnoj / 2;
	}
//	uint32_t delimiter=1;
//	uint32_t mnoj=1;
//
//	if (anglePsteps > distPsteps) {
//
//		htim1M1->Instance->PSC = psc;
//		htim1M1->Instance->ARR = periodM1;
//		htim1M1->Instance->CCR1 = periodM1/2;
//
//		delimiter = anglePsteps / distPsteps;
//		mnoj = periodM1 * delimiter;
//
//		htim2M2->Instance->PSC = psc;
//		htim2M2->Instance->ARR = mnoj;
//		htim2M2->Instance->CCR2 = mnoj / 2;
//
//	} else if (anglePsteps < distPsteps) {
//
//		htim2M2->Instance->PSC = psc;
//		htim2M2->Instance->ARR = periodM1;
//		htim2M2->Instance->CCR2 = periodM1 / 2;
//
//		delimiter = distPsteps / anglePsteps;
//		mnoj = periodM1 * delimiter;
//
//		htim1M1->Instance->PSC = psc;
//		htim1M1->Instance->ARR = mnoj;
//		htim1M1->Instance->CCR1 = mnoj / 2;
//	}

	stateMoveM1 = true;
	stateMoveM2 = true;

	SetEnable(1, true);
	SetEnable(2, true);

	HAL_TIM_PWM_Start(htim1M1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim2M2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(htim1M1);
	HAL_TIM_Base_Start_IT(htim2M2);

	return 0;
}

int MoveLinear(float Dist) {



	return 0;
}
int MoveAngle(float Angl){



	return 0;
}

/*

int RoboArm::Move2MotorsSimu(float angle, uint16_t distance) {

	// TIM1 Х  enc1 -  угол 360  -  8 оборотов движка на 1 оборот энкодера
	// TIM2  Y  enc2 - линейный -  6,4516129 оборотов движка (это целое линейное перемещение с запасом) на 1 оборот энкодера

	HAL_TIM_PWM_Stop(htim1M1, TIM_CHANNEL_1);      //остановили PWM таймера
	HAL_TIM_PWM_Stop(htim2M2, TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT(htim1M1);				// остановили прерывание таймеров
	HAL_TIM_Base_Stop_IT(htim2M2);

	SetEnable(1, false);
	SetEnable(2, false);

	GetLastPosition();


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

	//set microstepping TODO
	anglePsteps = (actualPosAngle * (8 * motorStep * drvMicroSteps)) / 360; //angle to steps
	distPsteps = actualPosDistance * linearStepsMil; //steps to distanse

	float distPangle = ((distPsteps / (motorStep * drvMicroSteps)) * 360
			/ 6.45);

	//TODO last position from encoder
//	lastPosAngle = angle;
//	lastPosLinear = distance;

// 1, 2, 3, 4, 6, 8, 9, 12, 18, 24, 36 и 72 - Це можлива обрана максимальна швидкість для мотора з більшої кількістю кроків. Це дільник таймера

	float periodM1 = 1200;
	uint32_t psc = 72-1;

	float delimiter=1;
	float mnoj=1;

	if (anglePsteps > distPsteps) {

		htim1M1->Instance->PSC = psc;
		htim1M1->Instance->ARR = periodM1;
		htim1M1->Instance->CCR1 = periodM1/2;

		delimiter = anglePsteps / distPsteps;
		mnoj = ceil(periodM1 * delimiter);

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = mnoj;
		htim2M2->Instance->CCR2 = mnoj / 2;

	} else if (anglePsteps < distPsteps) {

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = periodM1;
		htim2M2->Instance->CCR2 = periodM1 / 2;

		delimiter = distPsteps / anglePsteps;
		mnoj = ceil(periodM1 * delimiter);

		htim1M1->Instance->PSC = psc;
		htim1M1->Instance->ARR = mnoj;
		htim1M1->Instance->CCR1 = mnoj / 2;
	}

	stateMoveM1 = true;
	stateMoveM2 = true;

	SetEnable(1, true);
	SetEnable(2, true);

	HAL_TIM_PWM_Start(htim1M1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim2M2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(htim1M1);
	HAL_TIM_Base_Start_IT(htim2M2);

	return 0;
}
*/

int RoboArm::factoryReset() {
	SetZeroEncoders();
	posNowAngle = defaultAngle;
	posNowDistance = defaultDistanse;

	return 0;
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


int RoboArm::SetMicrosteps4All(uint8_t microsteps_per_step){

	tmcd_angle.setMicrostepsPerStepPowerOfTwo(microsteps_per_step);
	tmcd_gripper.setMicrostepsPerStepPowerOfTwo(microsteps_per_step);
	tmcd_linear.setMicrostepsPerStepPowerOfTwo(microsteps_per_step);

	return 0;
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

int RoboArm::SetSoftwareZero() {
//	ang_zero = GetPosEncoders(1);
//	lin_zero = GetPosEncoders(2);

	int attempts = 0;
	uint32_t posnowT_1 = GetPosEncoders(1);

	while(posnowT_1 == 0xFFFF && ++attempts < 3)
		posnowT_1 = GetPosEncoders(1); //try again

	attempts = 0;

	//			float ang = posnowT*360/16384;
	ang_zero = GetAngleEncoders(posnowT_1) - defaultAngle;// - defaultAngle; //0, 120, 240
	if (ang_zero < 0.0)
		ang_zero = 360.0 + ang_zero;
	//			un_send.params.ang = angleT;

	uint32_t posnowT_2 = GetPosEncoders(2);
	while(posnowT_2 == 0xFFFF && ++attempts < 3)
		posnowT_2 = GetPosEncoders(2); //try again
	float ang_pos = GetAngleEncoders(posnowT_2);
	lin_zero = ang_pos*distMax/360.0 - defaultDistanse;
	if (lin_zero < 0.0)
		lin_zero = distMax + lin_zero;

	return 0;
}

float RoboArm::UnshiftZeroAng(float angle){
	float ang_actual = ang_zero + angle;
	if (ang_actual > 360.0)
		ang_actual -= 360.0;
	return ang_actual;
}

float RoboArm::UnshiftZeroLin(float distance){
	float lin_actual = lin_zero + distance;
	if (lin_actual > distMax)
		lin_actual -= distMax;
	return lin_actual;
}

float RoboArm::ShiftZeroAng(float ang_actual){
	float ang = ang_actual - ang_zero;
	if (ang < 0.0)
		ang = 360.0 + ang;
	return ang;
}

float RoboArm::ShiftZeroLin(float lin_actual){
	float lin = lin_actual - lin_zero;
	if (lin < 0.0)
		lin = distMax + lin;
	return lin;
}

int RoboArm::SetSettMotors(UART_HandleTypeDef &huartTmc,TIM_HandleTypeDef &htim1, TIM_HandleTypeDef &htim2, TIM_HandleTypeDef &htim3,
		GPIO_TypeDef *Dir1_GPIO_Port_M1T, uint16_t Dir1_Pin_M1T,
		GPIO_TypeDef *Dir2_GPIO_Port_M2T, uint16_t Dir2_Pin_M2T,
		GPIO_TypeDef *Dir3_GPIO_Port_M3T, uint16_t Dir3_Pin_M3T,
		GPIO_TypeDef *En1_GPIO_Port_M1T, uint16_t En1_Pin_M1T,
		GPIO_TypeDef *En2_GPIO_Port_M2T, uint16_t En2_Pin_M2T,
		GPIO_TypeDef *En3_GPIO_Port_M3T, uint16_t En3_Pin_M3T){
//		UART_HandleTypeDef &huart_tmcT) {
	htim1M1 = &htim1;
	htim2M2 = &htim2;
	htim3M3 = &htim3;

	Dir1_GPIO_Port_M1 = Dir1_GPIO_Port_M1T;
	Dir1_Pin_M1 = Dir1_Pin_M1T;
	Dir2_GPIO_Port_M2 = Dir2_GPIO_Port_M2T;
	Dir2_Pin_M2 = Dir2_Pin_M2T;
	Dir3_GPIO_Port_M3 = Dir3_GPIO_Port_M3T;
	Dir3_Pin_M3 = Dir3_Pin_M3T;

	En1_GPIO_Port_M1 = En1_GPIO_Port_M1T;
	En1_Pin_M1 = En1_Pin_M1T;
	En2_GPIO_Port_M2 = En2_GPIO_Port_M2T;
	En2_Pin_M2 = En2_Pin_M2T;
	En3_GPIO_Port_M3 = En3_GPIO_Port_M3T;
	En3_Pin_M3 = En3_Pin_M3T;

	SetEnable(1, true);
	SetEnable(2, true);
	SetEnable(3, true);

	tmcd_angle.setup(&huartTmc, 115200, tmcd_angle.SERIAL_ADDRESS_0);
	tmcd_gripper.setup(&huartTmc, 115200, tmcd_gripper.SERIAL_ADDRESS_1);
	tmcd_linear.setup(&huartTmc, 115200, tmcd_linear.SERIAL_ADDRESS_3);

	tmcd_angle.enable();

//	tmcd_angle.disableAutomaticCurrentScaling();
//	tmcd_angle.disableAutomaticGradientAdaptation();

	tmcd_gripper.enable();

//	tmcd_gripper.disableAutomaticCurrentScaling();
//	tmcd_gripper.disableAutomaticGradientAdaptation();

	tmcd_linear.enable();

//	tmcd_linear.disableAutomaticCurrentScaling();
//	tmcd_linear.disableAutomaticGradientAdaptation();

	SetMicrosteps4All(5);



	SetEnable(1, false);
	SetEnable(2, false);
	SetEnable(3, false);

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
		HAL_GPIO_WritePin(En1_GPIO_Port_M1, En1_Pin_M1, pinSet);
	} else if (numMotor == 2) {
		HAL_GPIO_WritePin(En2_GPIO_Port_M2, En2_Pin_M2, pinSet);
	} else if (numMotor == 3) {
		HAL_GPIO_WritePin(En3_GPIO_Port_M3, En3_Pin_M3, pinSet);
	}

	return 0;
}
