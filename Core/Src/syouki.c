/*
 * syouki.c
 *
 *  Created on: Aug 29, 2024
 *      Author: yosaha
 */
#include "syouki.h"

float syouki_map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void syouki_CAN_Write(CAN_HandleTypeDef *hCANx, uint8_t CAN_ID, uint8_t send_data[]){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint8_t TxData[8];

	if(0 < HAL_CAN_GetTxMailboxesFreeLevel(hCANx)){
		TxHeader.StdId = CAN_ID;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.DLC = 8;
		TxHeader.TransmitGlobalTime = DISABLE;
		TxData[0] = send_data[0];
		TxData[1] = send_data[1];
		TxData[2] = send_data[2];
		TxData[3] = send_data[3];
		TxData[4] = send_data[4];
		TxData[5] = send_data[5];
		TxData[6] = send_data[6];
		TxData[7] = send_data[7];
		if (HAL_CAN_AddTxMessage(hCANx, &TxHeader, TxData, &TxMailbox)!= HAL_OK){
			Error_Handler();
		}
	}
}

void syouki_printf_Init(){
	setbuf(stdout, NULL);
}

void syouki_omni_calculation(int vx_value, int vy_value,int role_value, int *fl_value, int *fr_value, int *rl_value, int *rr_value){
	int vx = vy_value;
	int vy = -vx_value;

	if(15 > vx && -15 < vx){
		vx = 0;
	}
	if(15 > vy && -15 < vy){
		vy = 0;
	}

	fl_value = vx - vy - role_value;
	fr_value = vx + vy + role_value;
	rl_value = vx + vy - role_value;
	rr_value = vx - vy + role_value;
}
