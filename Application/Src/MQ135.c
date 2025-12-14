#include "MQ135.h"

#define VREF 5.0f

static volatile uint16_t adc_raw = 0;

void MQ135_init(ADC_HandleTypeDef* hadc) {
    HAL_ADC_Start_DMA(hadc, (uint32_t*)&adc_raw, 1);
}

void MQ135_stop(ADC_HandleTypeDef* hadc) {
    HAL_ADC_Stop_DMA(hadc);
}

uint16_t get_MQ135_raw() {
    return adc_raw;
}

uint16_t get_MQ135_voltage() {
    uint16_t raw = get_MQ135_raw();
    return (uint16_t)((raw / 4095.0f) * VREF * 1000);
}

uint8_t air_quality_is_bad() {
    uint16_t voltage = get_MQ135_voltage();
    if (voltage > 3000) 
        return 1;
    else
        return 0;
}