/*
 * dht22.h
 *
 *  Created on: Dec 3, 2024
 *      Author: thund
 */

#ifndef INC_DHT22_H_
#define INC_DHT22_H_

#include "stm32H7xx_hal.h"  // Update this include based on your STM32 series

#define DHT22_PORT GPIOA
#define DHT22_PIN  GPIO_PIN_1

typedef struct {
    float Temperature;
    float Humidity;
} DHT22_Data_t;

void DHT22_Init(void);
uint8_t DHT22_Read(DHT22_Data_t *data);

#endif /* INC_DHT22_H_ */
