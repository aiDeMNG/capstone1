/**
 * @file    light_control.h
 * @brief   光照自动控制模块
 * @note    监测BH1750光照传感器，自动控制窗户和窗帘
 *          窗户：相对模式（开/关）
 *          窗帘：位置模式（全开/半开/全关）
 */

#ifndef __LIGHT_CONTROL_H
#define __LIGHT_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include "control_priority.h"
#include "gy_30.h"
#include "motor_uln2003.h"

/* ==================== 光照控制状态 ==================== */

typedef enum {
    LIGHT_CTRL_IDLE = 0,        // 空闲
    LIGHT_CTRL_PROCESSING,      // 处理中
    LIGHT_CTRL_SUPPRESSED       // 被抑制（优先级低）
} Light_Control_State;

/* ==================== 光照控制句柄 ==================== */

typedef struct {
    /* 传感器和执行器 */
    LightSensor_HandleTypeDef *hlsensor;        // 光照传感器句柄
    Motor_ULN2003_HandleTypeDef *hmotor_window; // 窗户电机句柄
    Motor_ULN2003_HandleTypeDef *hmotor_curtain;// 窗帘电机句柄

    /* 控制状态 */
    Light_Control_State state;                  // 控制状态
    Control_Priority current_priority;          // 当前优先级

} Light_Control_HandleTypeDef;

/* ==================== 函数声明 ==================== */

/**
 * @brief  初始化光照控制模块
 * @param  hctrl: 控制句柄
 * @param  hlsensor: 光照传感器句柄
 * @param  hmotor_window: 窗户电机句柄
 * @param  hmotor_curtain: 窗帘电机句柄
 */
void LightControl_Init(Light_Control_HandleTypeDef *hctrl,
                       LightSensor_HandleTypeDef *hlsensor,
                       Motor_ULN2003_HandleTypeDef *hmotor_window,
                       Motor_ULN2003_HandleTypeDef *hmotor_curtain);

/**
 * @brief  光照控制处理函数（主循环调用）
 * @param  hctrl: 控制句柄
 * @param  external_priority: 外部优先级（如空气质量控制的优先级）
 * @note   定期调用此函数进行光照检测和自动控制
 *         如果external_priority >= PRIORITY_LIGHT，则光照对窗户的控制会被抑制
 *         窗帘控制不受优先级影响
 */
void LightControl_Process(Light_Control_HandleTypeDef *hctrl,
                         Control_Priority external_priority);

/**
 * @brief  获取控制状态
 * @param  hctrl: 控制句柄
 * @return 控制状态
 */
Light_Control_State LightControl_GetState(Light_Control_HandleTypeDef *hctrl);

/**
 * @brief  获取当前优先级
 * @param  hctrl: 控制句柄
 * @return 当前优先级
 */
Control_Priority LightControl_GetPriority(Light_Control_HandleTypeDef *hctrl);

#ifdef __cplusplus
}
#endif

#endif /* __LIGHT_CONTROL_H */
