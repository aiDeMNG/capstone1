#pragma once
#include "stm32f1xx_hal.h"

void MQ135_init(ADC_HandleTypeDef* hadc);
void MQ135_stop(ADC_HandleTypeDef* hadc);
uint16_t get_MQ135_raw();
uint16_t get_MQ135_voltage();
uint8_t air_quality_is_bad();