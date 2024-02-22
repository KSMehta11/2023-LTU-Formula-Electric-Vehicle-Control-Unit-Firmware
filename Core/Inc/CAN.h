/*
 * CAN.h
 *
 *  Created on: Oct 31, 2022
 *      Author: kmehta
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stm32f4xx_hal.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;

void startNode();
void clearErrors();
void nodeGuarding();
void syncMessage();
void torqueControlMessage(int *torqueRefLim, const int *speedRefLimit);
void orionFilterConfig();
void orionFilterConfig_1();
void motorAndControllerTempFilterConfig();
void motorRPMFilterConfig();

#endif /* INC_CAN_H_ */
