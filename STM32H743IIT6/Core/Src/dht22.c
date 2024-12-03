#include "dht22.h"

// Helper function for microsecond delays
static void delay_us(uint16_t us) {
	HAL_Delay(1);
}

// Configure the GPIO pin as output
static void DHT22_SetPinOutput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStruct);
}

// Configure the GPIO pin as input
static void DHT22_SetPinInput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStruct);
}

// Initialize the DHT22 pin
void DHT22_Init(void) {
    DHT22_SetPinOutput();
    HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET); // Keep pin high initially
}

// Read data from DHT22
uint8_t DHT22_Read(DHT22_Data_t *data) {
    uint8_t buffer[5] = {0}; // Buffer to store the 5 bytes of data
    uint8_t i, j;

    // Start signal
    DHT22_SetPinOutput();
    HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_RESET);
    HAL_Delay(1); // Hold low for at least 1ms
    HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET);
    delay_us(30); // Release line for 20-40us

    // Wait for DHT22 response
    DHT22_SetPinInput();
    if (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) != GPIO_PIN_RESET) return 1; // No response
    delay_us(80);
    if (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) != GPIO_PIN_SET) return 1; // No high response
    delay_us(80);

    // Read 40 bits of data
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_RESET); // Wait for low
            delay_us(40); // Measure high pulse
            if (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_SET) {
                buffer[i] |= (1 << (7 - j)); // Set bit if high pulse > 40us
            }
            while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_SET); // Wait for high to end
        }
    }

    // Checksum validation
    if (buffer[4] != (buffer[0] + buffer[1] + buffer[2] + buffer[3])) return 2;

    // Convert data
    data->Humidity = ((buffer[0] << 8) | buffer[1]) / 10.0;
    data->Temperature = ((buffer[2] << 8) | buffer[3]) / 10.0;

    return 0; // Success
}/*
 * dht22.c
 *
 *  Created on: Dec 3, 2024
 *      Author: thund
 */


