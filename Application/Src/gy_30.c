/**
 * @file    gy_30.c
 * @brief   GY-30 (BH1750) 光照传感器模块
 * @note    只负责读取光照值和设置控制标志位
 *          电机控制由 motor.c 根据标志位执行
 */

#include "gy_30.h"

/**
 * @brief  初始化光照传感器模块
 */
void LightSensor_Init(LightSensor_HandleTypeDef *hlsensor, BH1750_HandleTypeDef *hbh1750)
{
    if (hlsensor == NULL) {
        return;
    }

    hlsensor->hbh1750 = hbh1750;
    hlsensor->current_lux = 0;

    /* 默认阈值 */
    hlsensor->window_high_th = LUX_WINDOW_HIGH_TH;
    hlsensor->window_low_th = LUX_WINDOW_LOW_TH;
    hlsensor->curtain_high_th = LUX_CURTAIN_HIGH_TH;
    hlsensor->curtain_low_th = LUX_CURTAIN_LOW_TH;
    hlsensor->hysteresis = LUX_HYSTERESIS;

    /* 清除标志位 */
    hlsensor->window_flag = WINDOW_FLAG_NONE;
    hlsensor->curtain_flag = CURTAIN_FLAG_NONE;

    /* 初始状态 - 假设初始都是关闭状态 */
    hlsensor->window_current_state = WINDOW_FLAG_CLOSE;
    hlsensor->curtain_current_state = CURTAIN_FLAG_CLOSE;

    /* 自动模式 */
    hlsensor->mode = LIGHT_MODE_AUTO;

    /* 读取间隔 */
    hlsensor->last_read_tick = 0;
    hlsensor->read_interval = 500;  // 500ms
}

/**
 * @brief  光照传感器处理函数
 * @note   读取光照值，根据阈值设置控制标志位
 *         窗户三态：强光关窗、适中开窗、夜晚关窗
 *         窗帘三态：强光全关、适中半开、弱光全开
 */
void LightSensor_Process(LightSensor_HandleTypeDef *hlsensor)
{
    if (hlsensor == NULL || hlsensor->hbh1750 == NULL) {
        return;
    }

    uint32_t current_tick = HAL_GetTick();

    /* 检查是否到达读取间隔 */
    if (current_tick - hlsensor->last_read_tick < hlsensor->read_interval) {
        return;
    }
    hlsensor->last_read_tick = current_tick;

    /* 读取光照值 */
    if (BH1750_ReadLight(hlsensor->hbh1750, &hlsensor->current_lux) != HAL_OK) {
        return;
    }

    /* 手动模式下不自动设置标志位 */
    if (hlsensor->mode == LIGHT_MODE_MANUAL) {
        return;
    }

    /* ========== 窗户控制逻辑（三态）========== */
    Window_Flag window_target = WINDOW_FLAG_NONE;

    if (hlsensor->current_lux > hlsensor->window_high_th + hlsensor->hysteresis) {
        /* 强光 → 关窗 */
        window_target = WINDOW_FLAG_CLOSE;
    } else if (hlsensor->current_lux < hlsensor->window_low_th - hlsensor->hysteresis) {
        /* 夜晚 → 关窗 */
        window_target = WINDOW_FLAG_CLOSE;
    } else if (hlsensor->current_lux > hlsensor->window_low_th + hlsensor->hysteresis &&
               hlsensor->current_lux < hlsensor->window_high_th - hlsensor->hysteresis) {
        /* 适中光照 → 开窗 */
        window_target = WINDOW_FLAG_OPEN;
    }

    /* 仅在状态变化时设置标志位 */
    if (window_target != WINDOW_FLAG_NONE && window_target != hlsensor->window_current_state) {
        hlsensor->window_flag = window_target;
    }

    /* ========== 窗帘控制逻辑（三态）========== */
    Curtain_Flag curtain_target = CURTAIN_FLAG_NONE;

    if (hlsensor->current_lux > hlsensor->curtain_high_th + hlsensor->hysteresis) {
        /* 强光 → 全关 */
        curtain_target = CURTAIN_FLAG_CLOSE;
    } else if (hlsensor->current_lux < hlsensor->curtain_low_th - hlsensor->hysteresis) {
        /* 弱光 → 全开 */
        curtain_target = CURTAIN_FLAG_OPEN;
    } else if (hlsensor->current_lux > hlsensor->curtain_low_th + hlsensor->hysteresis &&
               hlsensor->current_lux < hlsensor->curtain_high_th - hlsensor->hysteresis) {
        /* 适中光照 → 半开 */
        curtain_target = CURTAIN_FLAG_HALF;
    }

    /* 仅在状态变化时设置标志位 */
    if (curtain_target != CURTAIN_FLAG_NONE && curtain_target != hlsensor->curtain_current_state) {
        hlsensor->curtain_flag = curtain_target;
    }
}

/**
 * @brief  获取当前光照值
 */
float LightSensor_GetLux(LightSensor_HandleTypeDef *hlsensor)
{
    if (hlsensor == NULL) {
        return 0;
    }
    return hlsensor->current_lux;
}

/**
 * @brief  获取窗户控制标志
 */
Window_Flag LightSensor_GetWindowFlag(LightSensor_HandleTypeDef *hlsensor)
{
    if (hlsensor == NULL) {
        return WINDOW_FLAG_NONE;
    }
    return hlsensor->window_flag;
}

/**
 * @brief  获取窗帘控制标志
 */
Curtain_Flag LightSensor_GetCurtainFlag(LightSensor_HandleTypeDef *hlsensor)
{
    if (hlsensor == NULL) {
        return CURTAIN_FLAG_NONE;
    }
    return hlsensor->curtain_flag;
}

/**
 * @brief  清除窗户控制标志
 */
void LightSensor_ClearWindowFlag(LightSensor_HandleTypeDef *hlsensor)
{
    if (hlsensor == NULL) {
        return;
    }
    hlsensor->window_flag = WINDOW_FLAG_NONE;
}

/**
 * @brief  清除窗帘控制标志
 */
void LightSensor_ClearCurtainFlag(LightSensor_HandleTypeDef *hlsensor)
{
    if (hlsensor == NULL) {
        return;
    }
    hlsensor->curtain_flag = CURTAIN_FLAG_NONE;
}

/**
 * @brief  设置控制模式
 */
void LightSensor_SetMode(LightSensor_HandleTypeDef *hlsensor, Light_Mode mode)
{
    if (hlsensor == NULL) {
        return;
    }
    hlsensor->mode = mode;

    /* 切换到手动模式时清除标志位 */
    if (mode == LIGHT_MODE_MANUAL) {
        hlsensor->window_flag = WINDOW_FLAG_NONE;
        hlsensor->curtain_flag = CURTAIN_FLAG_NONE;
    }
}

/**
 * @brief  设置光照阈值
 */
void LightSensor_SetThreshold(LightSensor_HandleTypeDef *hlsensor,
                               float window_high, float window_low,
                               float curtain_high, float curtain_low)
{
    if (hlsensor == NULL) {
        return;
    }
    hlsensor->window_high_th = window_high;
    hlsensor->window_low_th = window_low;
    hlsensor->curtain_high_th = curtain_high;
    hlsensor->curtain_low_th = curtain_low;
}

/**
 * @brief  更新窗户状态
 */
void LightSensor_UpdateWindowState(LightSensor_HandleTypeDef *hlsensor, Window_Flag state)
{
    if (hlsensor == NULL) {
        return;
    }
    hlsensor->window_current_state = state;
}

/**
 * @brief  更新窗帘状态
 */
void LightSensor_UpdateCurtainState(LightSensor_HandleTypeDef *hlsensor, Curtain_Flag state)
{
    if (hlsensor == NULL) {
        return;
    }
    hlsensor->curtain_current_state = state;
}
