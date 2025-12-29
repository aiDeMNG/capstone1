/**
 * @file    light_control.c
 * @brief   光照自动控制模块
 * @note    监测BH1750光照传感器，自动控制窗户和窗帘
 *          窗户：ULN2003驱动，相对模式（开/关）- 受优先级影响
 *          窗帘：A4988驱动，位置模式（全开/半开/全关）- 不受优先级影响
 */

#include "light_control.h"

/* ==================== 私有函数声明 ==================== */

static void ProcessWindowControl(Light_Control_HandleTypeDef *hctrl, uint8_t suppressed);
static void ProcessCurtainControl(Light_Control_HandleTypeDef *hctrl);

/* ==================== 公共函数实现 ==================== */

/**
 * @brief  初始化光照控制模块
 */
void LightControl_Init(Light_Control_HandleTypeDef *hctrl,
                       LightSensor_HandleTypeDef *hlsensor,
                       Motor_ULN2003_HandleTypeDef *hmotor_window,
                       Motor_A4988_HandleTypeDef *hmotor_curtain)
{
    if (hctrl == NULL) {
        return;
    }

    hctrl->hlsensor = hlsensor;
    hctrl->hmotor_window = hmotor_window;
    hctrl->hmotor_curtain = hmotor_curtain;
    hctrl->state = LIGHT_CTRL_IDLE;
    hctrl->current_priority = PRIORITY_LIGHT;
}

/**
 * @brief  光照控制处理函数
 */
void LightControl_Process(Light_Control_HandleTypeDef *hctrl,
                         Control_Priority external_priority)
{
    if (hctrl == NULL) {
        return;
    }

    hctrl->state = LIGHT_CTRL_PROCESSING;

    /* 检查是否被更高优先级抑制 */
    uint8_t window_suppressed = (external_priority >= PRIORITY_AIR_QUALITY) ? 1 : 0;

    if (window_suppressed) {
        hctrl->state = LIGHT_CTRL_SUPPRESSED;
    }

    /* 处理窗户控制（可能被抑制） */
    ProcessWindowControl(hctrl, window_suppressed);

    /* 处理窗帘控制（不受优先级影响） */
    ProcessCurtainControl(hctrl);

    /* 更新窗户状态反馈给光照传感器模块 */
    if (hctrl->hmotor_window != NULL) {
        Motor_ULN2003_State motor_state = Motor_ULN2003_GetState(hctrl->hmotor_window);
        if (motor_state == MOTOR_ULN2003_OPEN) {
            LightSensor_UpdateWindowState(hctrl->hlsensor, WINDOW_FLAG_OPEN);
        }
        else if (motor_state == MOTOR_ULN2003_CLOSED) {
            LightSensor_UpdateWindowState(hctrl->hlsensor, WINDOW_FLAG_CLOSE);
        }
    }
}

/**
 * @brief  获取控制状态
 */
Light_Control_State LightControl_GetState(Light_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return LIGHT_CTRL_IDLE;
    }
    return hctrl->state;
}

/**
 * @brief  获取当前优先级
 */
Control_Priority LightControl_GetPriority(Light_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return PRIORITY_NONE;
    }
    return hctrl->current_priority;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief  处理窗户控制
 * @param  hctrl: 控制句柄
 * @param  suppressed: 是否被抑制（1=被抑制，0=正常工作）
 */
static void ProcessWindowControl(Light_Control_HandleTypeDef *hctrl, uint8_t suppressed)
{
    if (hctrl == NULL || hctrl->hlsensor == NULL || hctrl->hmotor_window == NULL) {
        return;
    }

    /* 如果被抑制，清除窗户标志位并返回 */
    if (suppressed) {
        LightSensor_ClearWindowFlag(hctrl->hlsensor);
        return;
    }

    /* 正常处理窗户控制 */
    Window_Flag wflag = LightSensor_GetWindowFlag(hctrl->hlsensor);

    if (wflag == WINDOW_FLAG_OPEN &&
        hctrl->hmotor_window->state != MOTOR_ULN2003_OPENING)
    {
        Motor_ULN2003_Open(hctrl->hmotor_window);
        LightSensor_UpdateWindowState(hctrl->hlsensor, WINDOW_FLAG_OPEN);
        LightSensor_ClearWindowFlag(hctrl->hlsensor);
    }
    else if (wflag == WINDOW_FLAG_CLOSE &&
             hctrl->hmotor_window->state != MOTOR_ULN2003_CLOSING)
    {
        Motor_ULN2003_Close(hctrl->hmotor_window);
        LightSensor_UpdateWindowState(hctrl->hlsensor, WINDOW_FLAG_CLOSE);
        LightSensor_ClearWindowFlag(hctrl->hlsensor);
    }
}

/**
 * @brief  处理窗帘控制
 * @param  hctrl: 控制句柄
 * @note   窗帘控制不受优先级影响
 */
static void ProcessCurtainControl(Light_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL || hctrl->hlsensor == NULL || hctrl->hmotor_curtain == NULL) {
        return;
    }

    Curtain_Flag cflag = LightSensor_GetCurtainFlag(hctrl->hlsensor);

    if (cflag == CURTAIN_FLAG_OPEN &&
        hctrl->hmotor_curtain->state != MOTOR_A4988_MOVING &&
        hctrl->hmotor_curtain->target_position != hctrl->hmotor_curtain->max_position)
    {
        Motor_A4988_MoveToOpen(hctrl->hmotor_curtain);  // 全开 = max_position
        LightSensor_UpdateCurtainState(hctrl->hlsensor, CURTAIN_FLAG_OPEN);
        LightSensor_ClearCurtainFlag(hctrl->hlsensor);
    }
    else if (cflag == CURTAIN_FLAG_HALF &&
             hctrl->hmotor_curtain->state != MOTOR_A4988_MOVING &&
             hctrl->hmotor_curtain->target_position != hctrl->hmotor_curtain->max_position / 2)
    {
        Motor_A4988_MoveToHalf(hctrl->hmotor_curtain);  // 半开 = max_position / 2
        LightSensor_UpdateCurtainState(hctrl->hlsensor, CURTAIN_FLAG_HALF);
        LightSensor_ClearCurtainFlag(hctrl->hlsensor);
    }
    else if (cflag == CURTAIN_FLAG_CLOSE &&
             hctrl->hmotor_curtain->state != MOTOR_A4988_MOVING &&
             hctrl->hmotor_curtain->target_position != 0)
    {
        Motor_A4988_MoveToClose(hctrl->hmotor_curtain);  // 全关 = 0
        LightSensor_UpdateCurtainState(hctrl->hlsensor, CURTAIN_FLAG_CLOSE);
        LightSensor_ClearCurtainFlag(hctrl->hlsensor);
    }
}
