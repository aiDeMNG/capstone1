#pragma once
#include "stm32f1xx_hal.h"
#include <string.h>

#define HC05_BUFFER_SIZE 256

void HC05_Init(UART_HandleTypeDef* huart);
void HC05_SendString(const char* str);
void HC05_SendData(uint8_t* data, uint16_t len);
uint16_t HC05_Available(void);
uint16_t HC05_Read(uint8_t* buffer, uint16_t max_len);
void HC05_ClearBuffer(void);
uint8_t HC05_SendATCommand(const char* cmd, char* response, uint16_t timeout_ms);
