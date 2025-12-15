/**
 * @file    gy_30.h
 * @brief   GY-30 光照控制模块
 * @note    根据光照强度自动控制窗户/窗帘开关
 */

#ifndef __GY_30_H
#define __GY_30_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "bh1750.h"

/* 光照控制状态 */
typedef enum {
    LIGHT_CTRL_IDLE = 0,      // 空闲状态
    LIGHT_CTRL_OPENING,       // 正在打开窗帘
    LIGHT_CTRL_CLOSING,       // 正在关闭窗帘
    LIGHT_CTRL_OPEN,          // 窗帘已打开
    LIGHT_CTRL_CLOSED         // 窗帘已关闭
} LightCtrl_State;

/* 光照控制模式 */
typedef enum {
    LIGHT_MODE_AUTO = 0,      // 自动模式
    LIGHT_MODE_MANUAL         // 手动模式
} LightCtrl_Mode;

/* 光照控制句柄结构体 */
typedef struct {
    BH1750_HandleTypeDef *hbh1750;    // BH1750 传感器句柄
    float lux_threshold_high;          // 光照上限阈值 (lx) - 超过则关闭窗帘
    float lux_threshold_low;           // 光照下限阈值 (lx) - 低于则打开窗帘
    float current_lux;                 // 当前光照值
    LightCtrl_State state;             // 当前状态
    LightCtrl_Mode mode;               // 控制模式
    uint8_t hysteresis;                // 滞后量 (防止频繁切换)
} LightCtrl_HandleTypeDef;

/* 函数声明 */

/**
 * @brief  初始化光照控制模块
 * @param  hlctrl: 光照控制句柄指针
 * @param  hbh1750: BH1750 传感器句柄指针
 * @param  threshold_high: 光照上限阈值 (lx)
 * @param  threshold_low: 光照下限阈值 (lx)
 * @retval None
 */
void LightCtrl_Init(LightCtrl_HandleTypeDef *hlctrl, BH1750_HandleTypeDef *hbh1750,
                    float threshold_high, float threshold_low);

/**
 * @brief  光照控制处理函数 (需在主循环中调用)
 * @param  hlctrl: 光照控制句柄指针
 * @retval None
 */
void LightCtrl_Process(LightCtrl_HandleTypeDef *hlctrl);

/**
 * @brief  设置光照阈值
 * @param  hlctrl: 光照控制句柄指针
 * @param  threshold_high: 光照上限阈值 (lx)
 * @param  threshold_low: 光照下限阈值 (lx)
 * @retval None
 */
void LightCtrl_SetThreshold(LightCtrl_HandleTypeDef *hlctrl,
                            float threshold_high, float threshold_low);

/**
 * @brief  设置控制模式
 * @param  hlctrl: 光照控制句柄指针
 * @param  mode: 控制模式 (LIGHT_MODE_AUTO / LIGHT_MODE_MANUAL)
 * @retval None
 */
void LightCtrl_SetMode(LightCtrl_HandleTypeDef *hlctrl, LightCtrl_Mode mode);

/**
 * @brief  手动打开窗帘
 * @param  hlctrl: 光照控制句柄指针
 * @retval None
 */
void LightCtrl_OpenCurtain(LightCtrl_HandleTypeDef *hlctrl);

/**
 * @brief  手动关闭窗帘
 * @param  hlctrl: 光照控制句柄指针
 * @retval None
 */
void LightCtrl_CloseCurtain(LightCtrl_HandleTypeDef *hlctrl);

/**
 * @brief  停止窗帘运动
 * @param  hlctrl: 光照控制句柄指针
 * @retval None
 */
void LightCtrl_StopCurtain(LightCtrl_HandleTypeDef *hlctrl);

/**
 * @brief  获取当前光照值
 * @param  hlctrl: 光照控制句柄指针
 * @retval 当前光照值 (lx)
 */
float LightCtrl_GetLux(LightCtrl_HandleTypeDef *hlctrl);

/**
 * @brief  获取当前状态
 * @param  hlctrl: 光照控制句柄指针
 * @retval 当前状态
 */
LightCtrl_State LightCtrl_GetState(LightCtrl_HandleTypeDef *hlctrl);

#ifdef __cplusplus
}
#endif

#endif /* __GY_30_H */
