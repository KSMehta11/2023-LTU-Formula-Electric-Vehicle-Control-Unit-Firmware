/*
 * CAN.c
 *
 *  Created on: Oct 31, 2022
 *      Author: kmehta
 */

#include "CAN.h"

void startNode()
{
	uint8_t data[8] = { 0x01, 0x7A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	uint32_t mailbox = 0;

	CAN_TxHeaderTypeDef txHeader;

	txHeader.DLC = 2;
	txHeader.ExtId = 0;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = 0x000;
	txHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &mailbox) != HAL_OK)
	{
		Error_Handler();
	}

	while (HAL_CAN_IsTxMessagePending(&hcan1, mailbox));

	return;
}

void clearErrors()
{
	uint8_t data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00 };

	uint32_t mailbox = 0;

	CAN_TxHeaderTypeDef txHeader;

	txHeader.DLC = 6;
	txHeader.ExtId = 0;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = 0x47A;
	txHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &mailbox) != HAL_OK)
	{
		Error_Handler();
	}

	while (HAL_CAN_IsTxMessagePending(&hcan1, mailbox));

	return;
}

void nodeGuarding()
{
	uint8_t data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	uint32_t mailbox = 0;

	CAN_TxHeaderTypeDef txHeader;

	txHeader.DLC = 8;
	txHeader.ExtId = 0;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_REMOTE;
	txHeader.StdId = 0x77A;
	txHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &mailbox) != HAL_OK)
	{
		Error_Handler();
	}

	while (HAL_CAN_IsTxMessagePending(&hcan1, mailbox));

	return;
}

void syncMessage()
{
	uint8_t data[8] = { 0x00, 0x7A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	uint32_t mailbox = 0;

	CAN_TxHeaderTypeDef txHeader;

	txHeader.DLC = 2;
	txHeader.ExtId = 0;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = 0x080;
	txHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &mailbox) != HAL_OK)
	{
		Error_Handler();
	}

	while (HAL_CAN_IsTxMessagePending(&hcan1, mailbox));
}

void torqueControlMessage(int* torqueRefLim, const int* speedRefLimit)
{
	uint8_t data[8] = { *torqueRefLim & 0xFF, (*torqueRefLim >> 8) & 0xFF,
					 *speedRefLimit & 0xFF, (*speedRefLimit >> 8) & 0xFF,
					 0x01, 0x00,
					 0x00, 0x00 };

	uint32_t mailbox = 0;

	CAN_TxHeaderTypeDef txHeader;

	txHeader.DLC = 6;
	txHeader.ExtId = 0;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = 0x47A;
	txHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &mailbox) != HAL_OK)
	{
		Error_Handler();
	}

	while (HAL_CAN_IsTxMessagePending(&hcan1, mailbox));
}

void orionFilterConfig()
{
	CAN_FilterTypeDef orionFilter;

	orionFilter.FilterActivation = CAN_FILTER_ENABLE;
	orionFilter.FilterBank = 0;
	orionFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	orionFilter.FilterIdHigh = 0x6CA<<5;
	orionFilter.FilterIdLow = 0x000;
	orionFilter.FilterMaskIdHigh = 0x6CA<<5;
	orionFilter.FilterMaskIdLow = 0x000;
	orionFilter.FilterMode = CAN_FILTERMODE_IDLIST;
	orionFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	orionFilter.SlaveStartFilterBank = 20;

	if (HAL_CAN_ConfigFilter(&hcan1, &orionFilter) != HAL_OK)
	{
		Error_Handler();
	}
}

void orionFilterConfig_1()
{
	CAN_FilterTypeDef orionFilter;

	orionFilter.FilterActivation = CAN_FILTER_ENABLE;
	orionFilter.FilterBank = 3;
	orionFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	orionFilter.FilterIdHigh = 0x6B1<<5;
	orionFilter.FilterIdLow = 0x000;
	orionFilter.FilterMaskIdHigh = 0x6B1<<5;
	orionFilter.FilterMaskIdLow = 0x000;
	orionFilter.FilterMode = CAN_FILTERMODE_IDLIST;
	orionFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	orionFilter.SlaveStartFilterBank = 20;

	if (HAL_CAN_ConfigFilter(&hcan1, &orionFilter) != HAL_OK)
	{
		Error_Handler();
	}
}

void motorAndControllerTempFilterConfig()
{
	CAN_FilterTypeDef tempFilter;

	tempFilter.FilterActivation = CAN_FILTER_ENABLE;
	tempFilter.FilterBank = 1;
	tempFilter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	tempFilter.FilterIdHigh = 0x1BA<<5;
	tempFilter.FilterIdLow = 0x000;
	tempFilter.FilterMaskIdHigh = 0x1BA<<5;
	tempFilter.FilterMaskIdLow = 0x000;
	tempFilter.FilterMode = CAN_FILTERMODE_IDLIST;
	tempFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	tempFilter.SlaveStartFilterBank = 20;

	if (HAL_CAN_ConfigFilter(&hcan1, &tempFilter) != HAL_OK)
	{
		Error_Handler();
	}
}

void motorRPMFilterConfig()
{
	CAN_FilterTypeDef rpmFilter;

	rpmFilter.FilterActivation = CAN_FILTER_ENABLE;
	rpmFilter.FilterBank = 2;
	rpmFilter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	rpmFilter.FilterIdHigh = 0x3FA<<5;
	rpmFilter.FilterIdLow = 0x000;
	rpmFilter.FilterMaskIdHigh = 0x3FA<<5;
	rpmFilter.FilterMaskIdLow = 0x000;
	rpmFilter.FilterMode = CAN_FILTERMODE_IDLIST;
	rpmFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	rpmFilter.SlaveStartFilterBank = 20;

	if (HAL_CAN_ConfigFilter(&hcan1, &rpmFilter) != HAL_OK)
	{
		Error_Handler();
	}
}
