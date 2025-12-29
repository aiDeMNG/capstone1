/**
 * @file    air_quality_control.h
 * @brief   空气质量监测与自动通风控制模块
 * @note    监测MQ135传感器，当空气质量差时自动打开窗户并启动风扇通风
 *          优先级: 空气质量控制 > 光照控制
 */

#ifndef __AIR_QUALITY_CONTROL_H
#define __AIR_QUALITY_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include "control_priority.h"
#include "MQ135.h"
#include "motor_uln2003.h"
#include "motor_a4988.h"
#include "fan.h"

/* ==================== 空气质量控制状态 ==================== */

typedef enum {
    AIR_CTRL_IDLE = 0,          // 空闲（空气质量正常）
    AIR_CTRL_DETECTING,         // 检测中
    AIR_CTRL_VENTILATING        // 通风中（空气质量差）
} Air_Quality_Control_State;

/* ==================== 空气质量控制句柄 ==================== */

typedef struct {
    /* 传感器和执行器 */
    Motor_ULN2003_HandleTypeDef *hmotor_window;  // 窗户电机句柄
    Motor_A4988_HandleTypeDef *hmotor_curtain;   // 窗帘电机句柄
    Fan_HandleTypeDef *hfan;                     // 风扇句柄

    /* 控制状态 */
    Air_Quality_Control_State state;             // 控制状态
    Control_Priority current_priority;           // 当前优先级

    /* 标志位 */
    uint8_t air_quality_bad;                     // 空气质量差标志
    uint8_t window_opened_by_air_ctrl;           // 窗户是否由空气质量控制打开
    uint8_t curtain_opened_by_air_ctrl;          // 窗帘是否由空气质量控制打开
    uint8_t fan_started_by_air_ctrl;             // 风扇是否由空气质量控制启动

} Air_Quality_Control_HandleTypeDef;

/* ==================== 函数声明 ==================== */

/**
 * @brief  初始化空气质量控制模块
 * @param  hctrl: 控制句柄
 * @param  hmotor_window: 窗户电机句柄
 * @param  hmotor_curtain: 窗帘电机句柄
 * @param  hfan: 风扇句柄
 */
void AirQualityControl_Init(Air_Quality_Control_HandleTypeDef *hctrl,
                            Motor_ULN2003_HandleTypeDef *hmotor_window,
                            Motor_A4988_HandleTypeDef *hmotor_curtain,
                            Fan_HandleTypeDef *hfan);

/**
 * @brief  空气质量控制处理函数（主循环调用）
 * @param  hctrl: 控制句柄
 * @param  external_priority: 外部优先级（如光照控制的优先级）
 * @note   定期调用此函数进行空气质量检测和自动控制
 *         如果external_priority >= PRIORITY_AIR_QUALITY，则空气质量控制会被抑制
 */
void AirQualityControl_Process(Air_Quality_Control_HandleTypeDef *hctrl,
                               Control_Priority external_priority);

/**
 * @brief  手动停止通风
 * @param  hctrl: 控制句柄
 * @note   用于手动控制时强制停止自动通风
 */
void AirQualityControl_StopVentilation(Air_Quality_Control_HandleTypeDef *hctrl);

/**
 * @brief  获取控制状态
 * @param  hctrl: 控制句柄
 * @return 控制状态
 */
Air_Quality_Control_State AirQualityControl_GetState(Air_Quality_Control_HandleTypeDef *hctrl);

/**
 * @brief  获取当前优先级
 * @param  hctrl: 控制句柄
 * @return 当前优先级
 */
Control_Priority AirQualityControl_GetPriority(Air_Quality_Control_HandleTypeDef *hctrl);

/**
 * @brief  检查是否正在通风
 * @param  hctrl: 控制句柄
 * @return 1=正在通风, 0=未通风
 */
uint8_t AirQualityControl_IsVentilating(Air_Quality_Control_HandleTypeDef *hctrl);

#ifdef __cplusplus
}
#endif

#endif /* __AIR_QUALITY_CONTROL_H */
