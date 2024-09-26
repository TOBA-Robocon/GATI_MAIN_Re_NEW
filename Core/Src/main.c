/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
//
//                       _oo0oo_
//                      o8888888o
//                      88" . "88
//                      (| -_- |)
//                      0\  =  /0
//                    ___/`---'\___
//                  .' \\|     |// '.
//                 / \\|||  :  |||// \
//                / _||||| -:- |||||- \
//               |   | \\\  -  /// |   |
//               | \_|  ''\---/''  |_/ |
//               \  .-\__  '-'  ___/-. /
//             ___'. .'  /--.--\  `. .'___
//          ."" '<  `.___\_<|>_/___.' >' "".
//         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
//         \  \ `_.   \_ __\ /__ _/   .-` /  /
//     =====`-.____`.___ \_____/___.-`___.-'=====
//                       `=---='
//
//
//     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//               佛祖保佑         永无BUG
//
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
#include "stdbool.h"
#include "syouki.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//can_ID
#define AB_ID 0x003
#define CD_ID 0x004

//発射機構前
#define FFIRING_ID 0x005

//発射機構後ろ
//#define BFIRING_ID 0x006

//グライダー
//#define GFIRING_ID 0x007

//receive_ID
uint32_t fId1 = 0x000 << 5; // フィルターID1
uint32_t fId2 = 0x100 << 5; // フィルターID2
uint32_t fId3 = 0x200 << 5; // フィルターID3
uint32_t fId4 = 0x000 << 5; // フィルターID4

//スティックのデッドゾーン
#define HAJIKU 50
#define MAX 250
#define MIN -250

//足回りの正面変更
float root = 0.7071;

//半自動化
//int path = 0;

//can_RX
uint32_t id;
uint32_t dlc;
uint32_t stm_id;

//発射機構のモーターの出力
int s_power = 0;
int direction = 0;

//足回りモーターの方向指定
int state = 0;
int fl_state, fr_state, bl_state, br_state;

//足回りのモーターの出力
int turning;
int fl_power, fr_power, bl_power, br_power;

//角度変更後の足回りのモーターの出力
int vx, vy;
int new_vx, new_vy;

//前のサーボの角度指定
int Fservo_angle = 0;

//後ろのサーボの角度指定
//int Bservo_angle = 0;

//PS4からのデータ受け取り用配列
uint8_t DualShock_data[3][8] = { 0 };

//発射用ばねハブからのデータ送受信用
uint8_t FilingMechanism_data[3][8] = { 0 };

//電源基盤受信用
uint8_t Electrical_data[8] = { 0 };

//map関数
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//printf用関数
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 10);
	return len;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	setbuf(stdout, NULL);
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_CAN1_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	// CANスタート
	HAL_CAN_Start(&hcan1);
	// 割り込み有効
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		printf("main..%d  ", Electrical_data[0]);
		printf("sub..%d\r\n", Electrical_data[1]);
//      動作確認用LED
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);

//		旋回:Y:X方向の出力の計算
//		Lスティック
		turning = DualShock_data[2][5] - DualShock_data[2][6];
		vy = DualShock_data[0][1] - DualShock_data[0][2];
		vx = DualShock_data[0][3] - DualShock_data[0][4];

//		Y:X方向の値の最低:最大値変更
		vy = map(vy, -128, 128, -255, 255);
		vx = map(vx, -128, 128, -255, 255);

//		正面の向きを45度変更
		new_vx = root * (vx + vy);
		new_vy = root * (-vx + vy);

//		それぞれのオムニの出力の計算
		fl_power = -new_vx - new_vy - turning;
		fr_power = new_vx - new_vy + turning;
		bl_power = -new_vx + new_vy + turning;
		br_power = new_vx + new_vy - turning;

		if (DualShock_data[1][3]) {
			fl_power = 0;
			fr_power = 250;
			bl_power = -250;
			br_power = 0;
		} else if (DualShock_data[1][5]) {
			fl_power = 0;
			fr_power = -250;
			bl_power = 250;
			br_power = 0;
		} else {
			fl_power = -new_vx - new_vy - turning;
			fr_power = new_vx - new_vy + turning;
			bl_power = -new_vx + new_vy + turning;
			br_power = new_vx + new_vy - turning;
		}

//		値を弾く
		fl_power = flip(fl_power);
		fr_power = flip(fr_power);
		bl_power = flip(bl_power);
		br_power = flip(br_power);

//		値からモーターの方向指定
		fl_state = all_state(fl_power);
		fr_state = all_state(fr_power);
		bl_state = all_state(bl_power);
		br_state = all_state(br_power);

//		サーボモーターの角度指定
//		〇ボタン
		if (DualShock_data[1][6]) {
			Fservo_angle = 0;
		} else {
			Fservo_angle = 20;
		}

//		サーボモーターの角度指定
//		〇ボタン
		if (DualShock_data[1][6]) {
			Fservo_angle = 0;
		} else {
			Fservo_angle = 20;
		}

//		サーボモーターの角度指定
//		□ボタン
//		if (DualShock_data[2][2]) {
//			Bservo_angle = 0;
//		} else {
//			Bservo_angle = 20;
//		}

//		サーボモーターの角度指定
//		△ボタン
//		if (DualShock_data[2][1]) {
//			Gservo_angle = 0;
//		} else {
//			Gservo_angle = 20;
//		}

//      発射機構正面手動ver(リミットスイッチによる制限付き)
//		十字キー上ボタン
		if (DualShock_data[1][2]) {
//			printf("%d \r\n",FilingMechanism_data[0][1]);
			if (FilingMechanism_data[0][1]) {
				s_power = 0;
				direction = 1;
			} else {
				s_power = 175;
				direction = 1;
			}
		}
//		十字キー下ボタン
		else if (DualShock_data[1][4]) {
//			printf("%d \r\n",FilingMechanism_data[0][2]);
//			後ろのリミットスイッチ
			if (FilingMechanism_data[0][2]) {
				s_power = 0;
				direction = 0;
			} else {
				s_power = 175;
				direction = 0;
			}
		}
//		想定していない状態なら止める
		else {
			s_power = 0;
			direction = 0;
		}

//      足回り前
		CAN_TX(AB_ID, abs(fl_power), fl_state, abs(fr_power), fr_state, 0);
//      足回り後ろ
		CAN_TX(CD_ID, abs(bl_power), bl_state, abs(br_power), br_state, 0);
//		発射機構前
		CAN_TX(FFIRING_ID, 0, 0, s_power, direction, Fservo_angle);
//		発射機構後ろ
//		CAN_TX(BFIRING_ID, 0, 0, s_power, direction, Bservo_angle);
//		グライダー
//		CAN_TX(GFIRING_ID, 0, 0, 0, 0, Gservo_angle);

		HAL_Delay(5);
	}

//発射機構自動化ver
//		printf("limit1..%d  ", FilingMechanism_data[0][1]);
//		printf("limit2..%d\r\n", FilingMechanism_data[0][2]);
//		printf("path..%d\r\n", path);
//		if (path == 0) {
//			s_power = 50;
//			direction = 0;
//			if (FilingMechanism_data[0][2]) {
//				path = 1;
//			}
//		} else if (path == 1) {
//			s_power = 0;
//			direction = 0;
//			if (DualShock_data[1][6]) {
//				path = 2;
//			}
//		} else if (path == 2) {
//			s_power = 50;
//			direction = 1;
//			if (FilingMechanism_data[0][1]) {
//				path = 3;
//			}
//		} else if (path == 3) {
//			s_power = 0;
//			direction = 0;
//			if (DualShock_data[2][2]) {
//				path = 0;
//			}
//		} else {
//			s_power = 0;
//			direction = 0;
//		}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 15;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 3;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */
	CAN_FilterTypeDef filter;
	filter.FilterIdHigh = fId1;                  // フィルターID1
	filter.FilterIdLow = fId2;                  // フィルターID2
	filter.FilterMaskIdHigh = fId3;                  // フィルターID3
	filter.FilterMaskIdLow = fId4;                  // フィルターID4
	filter.FilterScale = CAN_FILTERSCALE_16BIT; // 16モード
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;      // FIFO0へ格納
	filter.FilterBank = 0;
	filter.FilterMode = CAN_FILTERMODE_IDLIST; // IDリストモード
	filter.SlaveStartFilterBank = 14;
	filter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &filter);
	/* USER CODE END CAN1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x60101017;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, SMPS_EN_Pin | SMPS_V1_Pin | SMPS_SW_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED1_Pin | LD4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LED3_Pin | LED4_Pin | LED2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SMPS_EN_Pin SMPS_V1_Pin SMPS_SW_Pin */
	GPIO_InitStruct.Pin = SMPS_EN_Pin | SMPS_V1_Pin | SMPS_SW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SMPS_PG_Pin */
	GPIO_InitStruct.Pin = SMPS_PG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED1_Pin LD4_Pin */
	GPIO_InitStruct.Pin = LED1_Pin | LD4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : LED3_Pin LED4_Pin LED2_Pin */
	GPIO_InitStruct.Pin = LED3_Pin | LED4_Pin | LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//値弾く関数
int flip(int flip_num) {
	if (flip_num <= HAJIKU && -HAJIKU <= flip_num) {
		flip_num = 0;
	} else if (flip_num <= MIN) {
		flip_num = MIN;
	} else if (flip_num >= MAX) {
		flip_num = MAX;
	}
	return flip_num;
}

//モーターの方向指定関数
int all_state(int num) {
	if (num >= 0) {
		state = 0;
	} else {
		state = 1;
	}
	return state;
}

//can_RX
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
	int i = 0;
	if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData)
			== HAL_OK) {
		id = RxHeader.StdId; // ID
		if (id == 0x000) {
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
			if (RxData[0] == 1) {
				for (i = 0; i <= 7; i++) {
					DualShock_data[0][i] = RxData[i];
				}
			} else if (RxData[0] == 2) {
				for (i = 0; i <= 7; i++) {
					DualShock_data[1][i] = RxData[i];
				}
			} else if (RxData[0] == 3) {
				for (i = 0; i <= 7; i++) {
					DualShock_data[2][i] = RxData[i];
				}
			}
		}
		if (id == 0x100) {
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
			if (RxData[0] == 5) {
				for (i = 0; i <= 7; i++) {
					FilingMechanism_data[0][i] = RxData[i];
				}
			}
			if (RxData[0] == 6) {
				for (i = 0; i <= 7; i++) {
					FilingMechanism_data[1][i] = RxData[i];
				}
			}
			if (RxData[0] == 7) {
				for (i = 0; i <= 7; i++) {
					FilingMechanism_data[2][i] = RxData[i];
				}
			}
		}
		if (id == 0x200) {
			for (i = 0; i <= 7; i++) {
				Electrical_data[i] = RxData[i];
			}
		}
	}
}

//can_TX
void CAN_TX(int stm_id, int m1, int direction1, int m2, int direction2,
		int servo_angle) {
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, SET);
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint8_t TxData[8];
	if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)) {
		TxHeader.StdId = stm_id;                 // CAN ID
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.DLC = 8;
		TxHeader.TransmitGlobalTime = DISABLE;  // ???
		TxData[0] = m1;
		TxData[1] = direction1;
		TxData[2] = m2;
		TxData[3] = direction2;
		TxData[4] = servo_angle;
		TxData[5] = servo_angle;
		TxData[6] = 0;
		TxData[7] = 0;
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox)
				!= HAL_OK) {
			Error_Handler();
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
