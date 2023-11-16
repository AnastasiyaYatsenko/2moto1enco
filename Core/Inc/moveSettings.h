/*
 * moveSettings.h
 *
 *  Created on: Oct 2, 2023
 *      Author: admin
 */

#ifndef INC_MOVESETTINGS_H_
#define INC_MOVESETTINGS_H_


/*
 *motor0 - зацеп
 * - у первого 6,4516129 оборотов движка (это целое линейное перемещение с запасом) на 1 оборот энкодера
- у второго 8 оборотов движка на 1 оборот энкодера
 */

#define M1_stepsPerLap 3200
#define M1_stepsPerLap 3200

#define M1_Limits_min 120
#define M1_Limits_max 2400
uint32_t M1_EncoderState=0;
uint32_t M1_AngleState=0;

#define M2_Limits_min 200
#define M2_Limits_max 2000
uint32_t M2_EncoderState=0;
uint32_t M2_AngleState=0;

#define M3_Steps 200s
bool M3_HookState=false;

#endif /* INC_MOVESETTINGS_H_ */
