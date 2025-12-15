//
//    FILE: dht.h
//  AUTHOR: Rob Tillaart (Modified for STM32 HAL)
// VERSION: 0.2.0 (STM32 HAL Port)
// PURPOSE: DHT Temperature & Humidity Sensor library for STM32 HAL
//     URL: http://arduino.cc/playground/Main/DHTLib
//
// HISTORY:
// 0.2.0 - Ported to STM32 HAL library
// see dht.c file
//

#ifndef dht_h
#define dht_h

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "stm32f1xx_hal.h"
#include <stdint.h>

#define DHT_LIB_VERSION "0.2.0"

#define DHTLIB_OK                   0
#define DHTLIB_ERROR_CHECKSUM       -1
#define DHTLIB_ERROR_TIMEOUT        -2
#define DHTLIB_ERROR_CONNECT        -3
#define DHTLIB_ERROR_ACK_L          -4
#define DHTLIB_ERROR_ACK_H          -5

#define DHTLIB_DHT11_WAKEUP         18
#define DHTLIB_DHT_WAKEUP           1

#define DHTLIB_DHT11_LEADING_ZEROS  1
#define DHTLIB_DHT_LEADING_ZEROS    6

// Timeout value for STM32 (adjusted for 72MHz)
// DHT protocol timing requires ~100us timeout
#define DHTLIB_TIMEOUT 7200  // 72MHz / 10000 = ~100us timeout

/* DHT sensor structure */
typedef struct {
    double humidity;
    double temperature;
    uint8_t bits[5];  // buffer to receive data
} DHT_DataTypeDef;

/* Public function prototypes */
// Return values:
// DHTLIB_OK
// DHTLIB_ERROR_CHECKSUM
// DHTLIB_ERROR_TIMEOUT
// DHTLIB_ERROR_CONNECT
// DHTLIB_ERROR_ACK_L
// DHTLIB_ERROR_ACK_H

int DHT_Read11(DHT_DataTypeDef *dht, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
int DHT_Read(DHT_DataTypeDef *dht, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

// Aliases for different DHT sensor models
#define DHT_Read21(dht, port, pin) DHT_Read(dht, port, pin)
#define DHT_Read22(dht, port, pin) DHT_Read(dht, port, pin)
#define DHT_Read33(dht, port, pin) DHT_Read(dht, port, pin)
#define DHT_Read44(dht, port, pin) DHT_Read(dht, port, pin)

/* Microsecond delay using DWT (Data Watchpoint and Trace) */
void DWT_Delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif
//
// END OF FILE
//