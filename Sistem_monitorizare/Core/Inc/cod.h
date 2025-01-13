/*
 * cod.h
 *
 *  Created on: Jan 11, 2025
 *      Author: Zbook
 */

#ifndef INC_COD_H_
#define INC_COD_H_

#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
#include <stdio.h>
#include "lwip/sockets.h"
#include <string.h>

// Function prototypes
void Fan_ON(void);
void Fan_OFF(void);
void ReadSensors(void);
void StartSensorTask(void *argument);
void StartCommTask(void *argument);
void SendDustAlert(void);
void BluetoothReceiveTask(void *argument);

// External handles
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;


// Semaphore handle
extern osSemaphoreId_t alertSemaphoreHandle;

// Shared global variables
extern volatile float lm35_1_temp;
extern volatile float lm35_2_temp;
extern volatile float dust_density;

#endif /* INC_COD_H_ */
