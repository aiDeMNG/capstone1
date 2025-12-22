/**
 * @file    temp_humidity_control.h
 * @brief   温湿度监测与自动控制模块
 * @note    监测DHT22温湿度传感器，根据温湿度自动调节窗户开度
 *          优先级: 温湿度控制 > 光照控制
 *          控制策略:
 *          - 温度过低(< 18°C): 关窗保温
 *          - 温度过高(> 28°C): 开窗散热
 *          - 湿度过高(> 70%): 开窗除湿（50%开度）
 *          - 决策优先级: 温度过低 > 湿度控制 > 温度过高
 */

#ifndef __TEMP_HUMIDITY_CONTROL_H
#define __TEMP_HUMIDITY_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include "control_priority.h"
#include "dht.h"
#include "motor_uln2003.h"

/* ==================== 温湿度阈值配置 ==================== */

#define TEMP_LOW_THRESHOLD          18.0f   // 温度过低阈值 (°C)
#define TEMP_HIGH_THRESHOLD         28.0f   // 温度过高阈值 (°C)
#define TEMP_HYSTERESIS             1.0f    // 温度回差 (°C)

#define HUMIDITY_LOW_THRESHOLD      30.0f   // 湿度过低阈值 (%RH)
#define HUMIDITY_HIGH_THRESHOLD     70.0f   // 湿度过高阈值 (%RH)
#define HUMIDITY_HYSTERESIS         5.0f    // 湿度回差 (%RH)

/* ==================== 窗户开度定义 ==================== */

#define WINDOW_OPEN_NONE            0       // 关窗 (0%)
#define WINDOW_OPEN_HALF            50      // 半开 (50%)
#define WINDOW_OPEN_FULL            100     // 全开 (100%)

/* ==================== 温湿度控制状态 ==================== */

typedef enum {
    TEMP_HUM_CTRL_IDLE = 0,     // 空闲（温湿度适宜）
    TEMP_HUM_CTRL_DETECTING,    // 检测中
    TEMP_HUM_CTRL_REGULATING    // 调节中（温度或湿度异常）
} Temp_Humidity_Control_State;

/* ==================== 温湿度控制句柄 ==================== */

typedef struct {
    /* 传感器和执行器 */
    GPIO_TypeDef *dht_port;                     // DHT22 GPIO端口
    uint16_t dht_pin;                           // DHT22 GPIO引脚
    Motor_ULN2003_HandleTypeDef *hmotor_window; // 窗户电机句柄

    /* 控制状态 */
    Temp_Humidity_Control_State state;          // 控制状态
    Control_Priority current_priority;          // 当前优先级

    /* 传感器数据 */
    float temperature;                          // 当前温度 (°C)
    float humidity;                             // 当前湿度 (%RH)

    /* 控制决策 */
    uint8_t temp_requires_open;                 // 温度要求开窗标志
    uint8_t humidity_requires_open;             // 湿度要求开窗标志
    uint8_t target_window_opening;              // 目标窗户开度 (0-100%)

    /* 标志位 */
    uint8_t window_controlled_by_temp_hum;      // 窗户是否由温湿度控制

} Temp_Humidity_Control_HandleTypeDef;

/* ==================== 函数声明 ==================== */

/**
 * @brief  初始化温湿度控制模块
 * @param  hctrl: 控制句柄
 * @param  dht_port: DHT22 GPIO端口
 * @param  dht_pin: DHT22 GPIO引脚
 * @param  hmotor_window: 窗户电机句柄
 */
void TempHumidityControl_Init(Temp_Humidity_Control_HandleTypeDef *hctrl,
                               GPIO_TypeDef *dht_port,
                               uint16_t dht_pin,
                               Motor_ULN2003_HandleTypeDef *hmotor_window);

/**
 * @brief  温湿度控制处理函数（主循环调用）
 * @param  hctrl: 控制句柄
 * @param  external_priority: 外部优先级（如空气质量控制的优先级）
 * @note   定期调用此函数进行温湿度检测和自动控制
 *         如果external_priority >= PRIORITY_TEMP_HUMIDITY，则温湿度控制会被抑制
 */
void TempHumidityControl_Process(Temp_Humidity_Control_HandleTypeDef *hctrl,
                                  Control_Priority external_priority);

/**
 * @brief  手动停止温湿度控制
 * @param  hctrl: 控制句柄
 * @note   用于手动控制时强制停止自动温湿度调节
 */
void TempHumidityControl_Stop(Temp_Humidity_Control_HandleTypeDef *hctrl);

/**
 * @brief  获取控制状态
 * @param  hctrl: 控制句柄
 * @return 控制状态
 */
Temp_Humidity_Control_State TempHumidityControl_GetState(Temp_Humidity_Control_HandleTypeDef *hctrl);

/**
 * @brief  获取当前优先级
 * @param  hctrl: 控制句柄
 * @return 当前优先级
 */
Control_Priority TempHumidityControl_GetPriority(Temp_Humidity_Control_HandleTypeDef *hctrl);

/**
 * @brief  获取当前温度
 * @param  hctrl: 控制句柄
 * @return 温度值 (°C)
 */
float TempHumidityControl_GetTemperature(Temp_Humidity_Control_HandleTypeDef *hctrl);

/**
 * @brief  获取当前湿度
 * @param  hctrl: 控制句柄
 * @return 湿度值 (%RH)
 */
float TempHumidityControl_GetHumidity(Temp_Humidity_Control_HandleTypeDef *hctrl);

/**
 * @brief  获取目标窗户开度
 * @param  hctrl: 控制句柄
 * @return 窗户开度 (0-100%)
 */
uint8_t TempHumidityControl_GetTargetOpening(Temp_Humidity_Control_HandleTypeDef *hctrl);

#ifdef __cplusplus
}
#endif

#endif /* __TEMP_HUMIDITY_CONTROL_H */
