/*
 * syouki.h
 *
 *  Created on: Aug 29, 2024
 *      Author: yosaha
 */

#ifndef INC_SYOKI_H_
#define INC_SYOKI_H_

#include "stdio.h"
#include "string.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_hal_can.h"
#include "stm32l4xx_hal_conf.h"

#endif /* INC_SYOKI_H_ */

float syouki_map(float x, float in_min, float in_max, float out_min, float out_max);
void syouki_CAN_Write(CAN_HandleTypeDef *hCANx, uint8_t CAN_ID, uint8_t send_data[]);
void syouki_printf_Init();
extern void syouki_omni_calculation(int vx_value, int vy_value,int role_value,
										int *fl_value, int *fr_value, int *rl_value, int *rr_value);
