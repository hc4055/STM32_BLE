/*
 * DHT11.h
 *
 *  Created on: Feb 5, 2025
 *      Author: PC
 */


#ifndef DHT11_H
#define DHT11_H

#include "stm32f4xx_hal.h"

#define OUTPUT 1
#define INPUT 0

/**
 * @brief DHT11 struct
 */
typedef struct {
    GPIO_TypeDef* port;        ///< GPIO Port (e.g., GPIOA)
    uint16_t pin;              ///< GPIO Pin (e.g., GPIO_PIN_2)
    TIM_HandleTypeDef *htim;   ///< Timer handle for delay (e.g., &htim2)
    uint8_t temperature;       ///< Last read temperature value
    uint8_t humidity;          ///< Last read humidity value
} dht11_t;

void init_dht11(dht11_t *dht, TIM_HandleTypeDef *htim, GPIO_TypeDef* port, uint16_t pin);
void set_dht11_gpio_mode(dht11_t *dht, uint8_t mode);
uint8_t readDHT11(dht11_t *dht);

#endif /* SRC_DHT11_H_ */
