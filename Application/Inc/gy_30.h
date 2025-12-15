/**
 * @file    gy_30.h
 * @brief   GY-30 (BH1750) 光照传感器模块
 * @note    只负责读取光照值和设置控制标志位
 *          电机控制由 motor.c 根据标志位执行
 */

#ifndef __GY_30_H
#define __GY_30_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "bh1750.h"
#include <stdint.h>

/* ==================== 控制标志位定义 ==================== */

/* 窗户控制标志 */
typedef enum {
    WINDOW_FLAG_NONE = 0,       // 无动作
    WINDOW_FLAG_OPEN,           // 需要打开窗户
    WINDOW_FLAG_CLOSE           // 需要关闭窗户
} Window_Flag;

/* 窗帘控制标志 */
typedef enum {
    CURTAIN_FLAG_NONE = 0,      // 无动作
    CURTAIN_FLAG_OPEN,          // 需要打开窗帘
    CURTAIN_FLAG_CLOSE          // 需要关闭窗帘
} Curtain_Flag;

/* 控制模式 */
typedef enum {
    LIGHT_MODE_AUTO = 0,        // 自动模式
    LIGHT_MODE_MANUAL           // 手动模式
} Light_Mode;

/* ==================== 光照阈值配置 ==================== */

#define LUX_WINDOW_OPEN_TH      500.0f      // 窗户打开阈值 (光照低于此值)
#define LUX_WINDOW_CLOSE_TH     2000.0f     // 窗户关闭阈值 (光照高于此值)
#define LUX_CURTAIN_OPEN_TH     300.0f      // 窗帘打开阈值 (光照低于此值)
#define LUX_CURTAIN_CLOSE_TH    1500.0f     // 窗帘关闭阈值 (光照高于此值)
#define LUX_HYSTERESIS          50.0f       // 滞后量 (防止频繁切换)

/* ==================== 光照控制句柄 ==================== */

typedef struct {
    BH1750_HandleTypeDef *hbh1750;      // BH1750 传感器句柄
    float current_lux;                   // 当前光照值 (lx)

    /* 阈值配置 */
    float window_open_th;                // 窗户打开阈值
    float window_close_th;               // 窗户关闭阈值
    float curtain_open_th;               // 窗帘打开阈值
    float curtain_close_th;              // 窗帘关闭阈值
    float hysteresis;                    // 滞后量

    /* 控制标志位 */
    Window_Flag window_flag;             // 窗户控制标志
    Curtain_Flag curtain_flag;           // 窗帘控制标志

    /* 当前状态 (用于滞后判断) */
    uint8_t window_is_open;              // 窗户当前是否打开
    uint8_t curtain_is_open;             // 窗帘当前是否打开

    /* 控制模式 */
    Light_Mode mode;                     // 自动/手动模式

    /* 更新时间控制 */
    uint32_t last_read_tick;             // 上次读取时间
    uint16_t read_interval;              // 读取间隔 (ms)
} LightSensor_HandleTypeDef;

/* ==================== 函数声明 ==================== */

/**
 * @brief  初始化光照传感器模块
 * @param  hlsensor: 光照传感器句柄
 * @param  hbh1750: BH1750 句柄
 */
void LightSensor_Init(LightSensor_HandleTypeDef *hlsensor, BH1750_HandleTypeDef *hbh1750);

/**
 * @brief  光照传感器处理函数 (主循环调用)
 * @param  hlsensor: 光照传感器句柄
 * @note   读取光照值并更新控制标志位
 */
void LightSensor_Process(LightSensor_HandleTypeDef *hlsensor);

/**
 * @brief  获取当前光照值
 * @param  hlsensor: 光照传感器句柄
 * @return 光照值 (lx)
 */
float LightSensor_GetLux(LightSensor_HandleTypeDef *hlsensor);

/**
 * @brief  获取窗户控制标志
 * @param  hlsensor: 光照传感器句柄
 * @return 窗户控制标志
 */
Window_Flag LightSensor_GetWindowFlag(LightSensor_HandleTypeDef *hlsensor);

/**
 * @brief  获取窗帘控制标志
 * @param  hlsensor: 光照传感器句柄
 * @return 窗帘控制标志
 */
Curtain_Flag LightSensor_GetCurtainFlag(LightSensor_HandleTypeDef *hlsensor);

/**
 * @brief  清除窗户控制标志 (电机处理完后调用)
 * @param  hlsensor: 光照传感器句柄
 */
void LightSensor_ClearWindowFlag(LightSensor_HandleTypeDef *hlsensor);

/**
 * @brief  清除窗帘控制标志 (电机处理完后调用)
 * @param  hlsensor: 光照传感器句柄
 */
void LightSensor_ClearCurtainFlag(LightSensor_HandleTypeDef *hlsensor);

/**
 * @brief  设置控制模式
 * @param  hlsensor: 光照传感器句柄
 * @param  mode: 控制模式
 */
void LightSensor_SetMode(LightSensor_HandleTypeDef *hlsensor, Light_Mode mode);

/**
 * @brief  设置光照阈值
 * @param  hlsensor: 光照传感器句柄
 * @param  window_open: 窗户打开阈值
 * @param  window_close: 窗户关闭阈值
 * @param  curtain_open: 窗帘打开阈值
 * @param  curtain_close: 窗帘关闭阈值
 */
void LightSensor_SetThreshold(LightSensor_HandleTypeDef *hlsensor,
                               float window_open, float window_close,
                               float curtain_open, float curtain_close);

/**
 * @brief  更新窗户状态 (电机完成动作后调用)
 * @param  hlsensor: 光照传感器句柄
 * @param  is_open: 是否打开
 */
void LightSensor_UpdateWindowState(LightSensor_HandleTypeDef *hlsensor, uint8_t is_open);

/**
 * @brief  更新窗帘状态 (电机完成动作后调用)
 * @param  hlsensor: 光照传感器句柄
 * @param  is_open: 是否打开
 */
void LightSensor_UpdateCurtainState(LightSensor_HandleTypeDef *hlsensor, uint8_t is_open);

#ifdef __cplusplus
}
#endif

#endif /* __GY_30_H */
