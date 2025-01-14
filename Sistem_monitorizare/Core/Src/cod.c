#include "cod.h"
#include "main.h"

// Constants
#define ADC_RESOLUTION 4095.0f
#define ADC_REF_VOLTAGE 3.3f
#define LM35_VOLTAGE_TO_TEMP_CONV 0.01f // 10 mV/°C
#define DUST_THRESHOLD 50.0f
#define FAN_THRESHOLD 20.0f // Temperature threshold for fan activation
#define FAN_GPIO_PORT GPIOA
#define FAN_GPIO_PIN GPIO_PIN_6

// Global variables (volatile for shared access between tasks)
volatile float lm35_1_temp = 0.0f;
volatile float lm35_2_temp = 0.0f;
volatile float dust_density = 0.0f;

// Turn on the fan
void Fan_ON(void) {
    HAL_GPIO_WritePin(FAN_GPIO_PORT, FAN_GPIO_PIN, GPIO_PIN_SET);
}

// Turn off the fan
void Fan_OFF(void) {
    HAL_GPIO_WritePin(FAN_GPIO_PORT, FAN_GPIO_PIN, GPIO_PIN_RESET);
}

// Read sensor values
void ReadSensors(void) {
    uint32_t raw_value;

    // Start ADC
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        printf("Error: ADC Start failed!\r\n");
        return;
    }

    // LM35_1 (Channel 10)
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        raw_value = HAL_ADC_GetValue(&hadc1);
        lm35_1_temp = ((float)raw_value / ADC_RESOLUTION) * ADC_REF_VOLTAGE / LM35_VOLTAGE_TO_TEMP_CONV;
    }

    // LM35_2 (Channel 15)
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        raw_value = HAL_ADC_GetValue(&hadc1);
        lm35_2_temp = ((float)raw_value / ADC_RESOLUTION) * ADC_REF_VOLTAGE / LM35_VOLTAGE_TO_TEMP_CONV;
    }

    // Dust Sensor (Channel 5)
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        raw_value = HAL_ADC_GetValue(&hadc1);
        dust_density = ((float)raw_value / ADC_RESOLUTION) * ADC_REF_VOLTAGE * 100.0f; // Adjust scaling factor if needed
    }

    // Stop ADC
    HAL_ADC_Stop(&hadc1);

    // Display sensor readings
    printf("LM35_1: %.2f C | LM35_2: %.2f C | Dust: %.2f μg/m^3\r\n", lm35_1_temp, lm35_2_temp, dust_density);
}

void SendDustAlert(void) {
    char message[] = "ALERT\n"; // Message to send

    // Transmit the message via Bluetooth (USART2)
    if (HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, strlen(message)) != HAL_OK) {
        printf("Error: Bluetooth message transmission failed!\r\n");
    } else {
        printf("Bluetooth message sent: %s\r\n", message);
    }
}

// UART Transmission Complete Callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // This function is triggered after the transmission completes
        printf("Bluetooth transmission completed.\r\n");
    }
}

// Task for reading sensors and controlling the fan
void mainTask(void *argument) {
    for (;;) {
        printf("Starting measurement cycle...\r\n");
        ReadSensors();  // Perform sensor measurements

        // Control fan based on average temperature
        float avg_temp = (lm35_1_temp + lm35_2_temp) / 2.0f;
        if (avg_temp > FAN_THRESHOLD) {
            Fan_ON();
            printf("Fan ON (Avg Temp: %.2f C)\r\n", avg_temp);
        } else {
            Fan_OFF();
            printf("Fan OFF (Avg Temp: %.2f C)\r\n", avg_temp);
        }

        // Release semaphore if dust density exceeds threshold
        if (dust_density > DUST_THRESHOLD) {
            printf("Dust density exceeded threshold! Releasing semaphore...\r\n");
            osSemaphoreRelease(alertSemaphoreHandle);
        }

        osDelay(2000);  // Delay for 2 seconds
    }
}

// Task for sending alerts via Bluetooth
void secondTask(void *argument) {
    for (;;) {
        // Wait for semaphore to be released when dust density exceeds threshold
        if (osSemaphoreAcquire(alertSemaphoreHandle, osWaitForever) == osOK) {
            SendDustAlert(); // Send "ALERT" message via HC-05
        }

        osDelay(100); // Allow other tasks to execute
    }
}
