//
//    FILE: dht.c
//  AUTHOR: Rob Tillaart (Modified for STM32 HAL)
// VERSION: 0.2.0 (STM32 HAL Port)
// PURPOSE: DHT Temperature & Humidity Sensor library for STM32 HAL
//     URL: http://arduino.cc/playground/Main/DHTLib
//
// HISTORY:
// 0.2.0 - Ported to STM32 HAL library
//        - Changed from C++ class to C struct
//        - Replaced Arduino GPIO with STM32 HAL GPIO
//        - Added DWT-based microsecond delay
//        - Direct register access for fast GPIO reading
//
// Released to the public domain
//

#include "dht.h"

/* Private function prototypes */
static int DHT_ReadSensor(DHT_DataTypeDef *dht, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,
                          uint8_t wakeupDelay, uint8_t leadingZeroBits);
static void GPIO_SetOutput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
static void GPIO_SetInput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/* DWT (Data Watchpoint and Trace) functions for microsecond delay */
// DWT counter must be enabled in CoreDebug for this to work
// Call DWT_Init() once at startup

static uint8_t DWT_Initialized = 0;

// Initialize DWT for microsecond delays
static void DWT_Init(void)
{
    if (!DWT_Initialized)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable TRC
        DWT->CYCCNT = 0; // Reset counter
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable counter
        DWT_Initialized = 1;
    }
}

// Microsecond delay using DWT cycle counter
// Note: DWT->CYCCNT is a 32-bit counter that wraps around
// The subtraction works correctly due to unsigned integer arithmetic
void DWT_Delay_us(uint32_t us)
{
    if (!DWT_Initialized)
    {
        DWT_Init();
    }

    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);

    // Wait until the difference reaches delayTicks
    // This handles counter overflow correctly due to unsigned arithmetic
    while ((DWT->CYCCNT - startTick) < delayTicks);
}

/* GPIO helper functions */
// Set GPIO pin as output (Open-Drain for DHT single-wire protocol)
static void GPIO_SetOutput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // Open-Drain for single-wire
    GPIO_InitStruct.Pull = GPIO_NOPULL;  // External pull-up required
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// Set GPIO pin as input
static void GPIO_SetInput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;  // DHT22 requires external pull-up resistor
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// Fast GPIO read using direct register access
static inline GPIO_PinState GPIO_ReadPin_Fast(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    return ((GPIOx->IDR & GPIO_Pin) != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/////////////////////////////////////////////////////
//
// PUBLIC
//

int DHT_Read11(DHT_DataTypeDef *dht, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    // READ VALUES
    int result = DHT_ReadSensor(dht, GPIOx, GPIO_Pin, DHTLIB_DHT11_WAKEUP, DHTLIB_DHT11_LEADING_ZEROS);

    // these bits are always zero, masking them reduces errors.
    dht->bits[0] &= 0x7F;
    dht->bits[2] &= 0x7F;

    // CONVERT AND STORE
    dht->humidity = dht->bits[0];  // bits[1] == 0;
    dht->temperature = dht->bits[2];  // bits[3] == 0;

    // TEST CHECKSUM
    // bits[1] && bits[3] both 0
    uint8_t sum = dht->bits[0] + dht->bits[2];
    if (dht->bits[4] != sum)
    {
        return DHTLIB_ERROR_CHECKSUM;
    }
    return result;
}

int DHT_Read(DHT_DataTypeDef *dht, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    // READ VALUES
    int result = DHT_ReadSensor(dht, GPIOx, GPIO_Pin, DHTLIB_DHT_WAKEUP, DHTLIB_DHT_LEADING_ZEROS);

    // these bits are always zero, masking them reduces errors.
    dht->bits[0] &= 0x03;
    dht->bits[2] &= 0x83;

    // CONVERT AND STORE
    // Equivalent to Arduino's word(MSB, LSB) macro
    uint16_t raw_humidity = ((uint16_t)dht->bits[0] << 8) | dht->bits[1];
    uint16_t raw_temperature = ((uint16_t)(dht->bits[2] & 0x7F) << 8) | dht->bits[3];

    dht->humidity = raw_humidity * 0.1;
    dht->temperature = raw_temperature * 0.1;

    if (dht->bits[2] & 0x80)  // negative temperature
    {
        dht->temperature = -dht->temperature;
    }

    // TEST CHECKSUM
    uint8_t sum = dht->bits[0] + dht->bits[1] + dht->bits[2] + dht->bits[3];
    if (dht->bits[4] != sum)
    {
        return DHTLIB_ERROR_CHECKSUM;
    }
    return result;
}

/////////////////////////////////////////////////////
//
// PRIVATE
//

static int DHT_ReadSensor(DHT_DataTypeDef *dht, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,
                          uint8_t wakeupDelay, uint8_t leadingZeroBits)
{
    // INIT BUFFERVAR TO RECEIVE DATA
    uint8_t mask = 128;
    uint8_t idx = 0;
    uint8_t data = 0;
    GPIO_PinState state = GPIO_PIN_RESET;
    GPIO_PinState pstate = GPIO_PIN_RESET;
    uint16_t zeroLoop = DHTLIB_TIMEOUT;
    uint16_t delta = 0;

    // Clear bits array to avoid stale data
    for (uint8_t i = 0; i < 5; i++) {
        dht->bits[i] = 0;
    }

    leadingZeroBits = 40 - leadingZeroBits; // reverse counting...

    // REQUEST SAMPLE
    GPIO_SetOutput(GPIOx, GPIO_Pin);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // Pull low
    HAL_Delay(wakeupDelay); // Wait wakeup time (ms)
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // Pull high
    DWT_Delay_us(40); // Wait 40us
    GPIO_SetInput(GPIOx, GPIO_Pin); // Set as input

    uint16_t loopCount = DHTLIB_TIMEOUT * 2;  // 200uSec max
    // Wait for pin to go low
    while (GPIO_ReadPin_Fast(GPIOx, GPIO_Pin) != GPIO_PIN_RESET)
    {
        if (--loopCount == 0) return DHTLIB_ERROR_CONNECT;
    }

    // GET ACKNOWLEDGE or TIMEOUT
    loopCount = DHTLIB_TIMEOUT;
    // Wait for pin to go high (T-rel)
    while (GPIO_ReadPin_Fast(GPIOx, GPIO_Pin) == GPIO_PIN_RESET)
    {
        if (--loopCount == 0) return DHTLIB_ERROR_ACK_L;
    }

    loopCount = DHTLIB_TIMEOUT;
    // Wait for pin to go low (T-reh)
    while (GPIO_ReadPin_Fast(GPIOx, GPIO_Pin) != GPIO_PIN_RESET)
    {
        if (--loopCount == 0) return DHTLIB_ERROR_ACK_H;
    }

    loopCount = DHTLIB_TIMEOUT;

    // READ THE OUTPUT - 40 BITS => 5 BYTES
    for (uint8_t i = 40; i != 0; )
    {
        // WAIT FOR FALLING EDGE
        state = GPIO_ReadPin_Fast(GPIOx, GPIO_Pin);
        if (state == GPIO_PIN_RESET && pstate != GPIO_PIN_RESET)
        {
            if (i > leadingZeroBits) // DHT22 first 6 bits are all zero !!   DHT11 only 1
            {
                if (loopCount < zeroLoop)
                {
                    zeroLoop = loopCount;
                }
                delta = (DHTLIB_TIMEOUT - zeroLoop) / 4;
            }
            else if (loopCount <= (zeroLoop - delta)) // long -> one
            {
                data |= mask;
            }
            mask >>= 1;
            if (mask == 0)   // next byte
            {
                mask = 128;
                dht->bits[idx] = data;
                idx++;
                data = 0;
            }
            // next bit
            --i;

            // reset timeout flag
            loopCount = DHTLIB_TIMEOUT;
        }
        pstate = state;
        // Check timeout
        if (--loopCount == 0)
        {
            return DHTLIB_ERROR_TIMEOUT;
        }
    }

    // Release the bus
    GPIO_SetOutput(GPIOx, GPIO_Pin);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);

    return DHTLIB_OK;
}

// User
static uint32_t last_dht_time = 0;
static DHT_DataTypeDef dht22;
static double humidity = 0;
static double temperature = 0;

uint8_t DHT22_update(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    if (HAL_GetTick() - last_dht_time >= 2000) {
        if (DHT_Read22(&dht22, GPIOx, GPIO_Pin) == DHTLIB_OK) {
            humidity = dht22.humidity;
            temperature = dht22.temperature;
        }
        last_dht_time = HAL_GetTick();
        return 1;
    }
    return 0;
}

double get_humidity() {
    return humidity;
}

double get_temperature() {
    return temperature;
}

//
// END OF FILE
//
