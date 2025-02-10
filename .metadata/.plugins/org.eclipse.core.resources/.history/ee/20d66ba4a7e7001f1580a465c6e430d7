/*
 * DHT11.c
 *
 *  Created on: Feb 6, 2025
 *      Author: PC
 */

#include "dht11.h"


/**
 * @brief Initialize the DHT11 sensor
 * @param dht Pointer to dht11_t struct
 * @param htim Timer handle for delay (should be configured for 1 µs resolution)
 * @param port GPIO port (e.g., GPIOA)
 * @param pin GPIO pin (e.g., GPIO_PIN_2)
 */
void init_dht11(dht11_t *dht, TIM_HandleTypeDef *htim, GPIO_TypeDef* port, uint16_t pin) {
    dht->htim = htim;
    dht->port = port;
    dht->pin = pin;
}

/**
 * @brief Set DHT11 pin as input or output
 * @param dht Pointer to dht11_t struct
 * @param pMode GPIO Mode (INPUT or OUTPUT)
 */
void set_dht11_gpio_mode(dht11_t *dht, uint8_t pMode) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = dht->pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    if (pMode == OUTPUT) {
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    } else {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    }

    HAL_GPIO_Init(dht->port, &GPIO_InitStruct);
}

/**
 * @brief Reads temperature and humidity from DHT11 sensor
 * @param dht Pointer to dht11_t struct
 * @return 1 if read successful, 0 if error occurs
 */
uint8_t readDHT11(dht11_t *dht) {
    uint16_t mTime1 = 0, mTime2 = 0;
    //uint8_t mBit = 0;
    uint8_t humVal = 0, tempVal = 0, parityVal = 0, genParity = 0;
    uint8_t mData[40] = {0};

    // Step 1: Send Start Signal
    set_dht11_gpio_mode(dht, OUTPUT);
    HAL_GPIO_WritePin(dht->port, dht->pin, GPIO_PIN_RESET);
    HAL_Delay(18);
    HAL_GPIO_WritePin(dht->port, dht->pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(dht->htim, 0);
    HAL_TIM_Base_Start(dht->htim);
    set_dht11_gpio_mode(dht, INPUT);

    // Step 2: Wait for response from DHT11
    __disable_irq();

    while (HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_SET) {
        if (__HAL_TIM_GET_COUNTER(dht->htim) > 500) {
            __enable_irq();
            return 0;
        }
    }

    __HAL_TIM_SET_COUNTER(dht->htim, 0);
    while (HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_RESET) {
        if (__HAL_TIM_GET_COUNTER(dht->htim) > 500) {
            __enable_irq();
            return 0;
        }
    }

    mTime1 = __HAL_TIM_GET_COUNTER(dht->htim);

    __HAL_TIM_SET_COUNTER(dht->htim, 0);
    while (HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_SET) {
        if (__HAL_TIM_GET_COUNTER(dht->htim) > 500) {
            __enable_irq();
            return 0;
        }
    }

    mTime2 = __HAL_TIM_GET_COUNTER(dht->htim);

    // Step 3: Validate response timing
    if (mTime1 < 75 || mTime1 > 85 || mTime2 < 75 || mTime2 > 85) {
        __enable_irq();
        return 0;
    }

    // Step 4: Read 40-bit Data
    for (int j = 0; j < 40; j++) {
        __HAL_TIM_SET_COUNTER(dht->htim, 0);
        while (HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_RESET) {
            if (__HAL_TIM_GET_COUNTER(dht->htim) > 500) {
                __enable_irq();
                return 0;
            }
        }

        __HAL_TIM_SET_COUNTER(dht->htim, 0);
        while (HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_SET) {
            if (__HAL_TIM_GET_COUNTER(dht->htim) > 500) {
                __enable_irq();
                return 0;
            }
        }

        mTime1 = __HAL_TIM_GET_COUNTER(dht->htim);
        mData[j] = (mTime1 > 50) ? 1 : 0;
    }

    HAL_TIM_Base_Stop(dht->htim);
    __enable_irq();

    // Step 5: Convert Data to Temperature and Humidity
    for (int i = 0; i < 8; i++) {
        humVal = (humVal << 1) | mData[i];
        tempVal = (tempVal << 1) | mData[i + 16];
        parityVal = (parityVal << 1) | mData[i + 32];
    }

    genParity = humVal + tempVal;

    dht->temperature = tempVal;
    dht->humidity = humVal;

    return (genParity == parityVal) ? 1 : 0;
}
