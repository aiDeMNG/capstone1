/**
 * @file    air_quality_control.c
 * @brief   空气质量监测与自动通风控制模块
 * @note    监测MQ135传感器，当空气质量差时自动打开窗户并启动风扇通风
 *          优先级: 空气质量控制 > 光照控制
 */

#include "air_quality_control.h"

/* ==================== 私有函数声明 ==================== */

static void StartVentilation(Air_Quality_Control_HandleTypeDef *hctrl);
static void StopVentilation(Air_Quality_Control_HandleTypeDef *hctrl);
static uint8_t IsWindowOpen(Air_Quality_Control_HandleTypeDef *hctrl);

/* ==================== 公共函数实现 ==================== */

/**
 * @brief  初始化空气质量控制模块
 */
void AirQualityControl_Init(Air_Quality_Control_HandleTypeDef *hctrl,
                            Motor_ULN2003_HandleTypeDef *hmotor_window,
                            Fan_HandleTypeDef *hfan)
{
    if (hctrl == NULL) {
        return;
    }

    hctrl->hmotor_window = hmotor_window;
    hctrl->hfan = hfan;
    hctrl->state = AIR_CTRL_IDLE;
    hctrl->current_priority = PRIORITY_NONE;
    hctrl->air_quality_bad = 0;
    hctrl->window_opened_by_air_ctrl = 0;
    hctrl->fan_started_by_air_ctrl = 0;
}

/**
 * @brief  空气质量控制处理函数
 */
void AirQualityControl_Process(Air_Quality_Control_HandleTypeDef *hctrl,
                               Control_Priority external_priority)
{
    if (hctrl == NULL) {
        return;
    }

    /* 检查是否被更高优先级抑制（手动控制） */
    if (external_priority >= PRIORITY_MANUAL) {
        /* 如果正在通风，停止由空气质量控制启动的设备 */
        if (hctrl->state == AIR_CTRL_VENTILATING) {
            StopVentilation(hctrl);
        }
        hctrl->state = AIR_CTRL_IDLE;
        hctrl->current_priority = PRIORITY_NONE;
        return;
    }

    /* 检测空气质量 */
    hctrl->state = AIR_CTRL_DETECTING;
    hctrl->air_quality_bad = air_quality_is_bad();

    /* 空气质量差，需要通风 */
    if (hctrl->air_quality_bad) {
        /* 设置高优先级 */
        hctrl->current_priority = PRIORITY_AIR_QUALITY;

        /* 如果还未开始通风，启动通风 */
        if (hctrl->state != AIR_CTRL_VENTILATING) {
            StartVentilation(hctrl);
        }

        hctrl->state = AIR_CTRL_VENTILATING;
    }
    /* 空气质量正常，立即停止通风 */
    else {
        /* 如果正在通风，立即停止 */
        if (hctrl->state == AIR_CTRL_VENTILATING) {
            StopVentilation(hctrl);
        }

        hctrl->state = AIR_CTRL_IDLE;
        hctrl->current_priority = PRIORITY_NONE;
    }
}

/**
 * @brief  手动停止通风
 */
void AirQualityControl_StopVentilation(Air_Quality_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }

    StopVentilation(hctrl);
    hctrl->state = AIR_CTRL_IDLE;
    hctrl->current_priority = PRIORITY_NONE;
}

/**
 * @brief  获取控制状态
 */
Air_Quality_Control_State AirQualityControl_GetState(Air_Quality_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return AIR_CTRL_IDLE;
    }
    return hctrl->state;
}

/**
 * @brief  获取当前优先级
 */
Control_Priority AirQualityControl_GetPriority(Air_Quality_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return PRIORITY_NONE;
    }
    return hctrl->current_priority;
}

/**
 * @brief  检查是否正在通风
 */
uint8_t AirQualityControl_IsVentilating(Air_Quality_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return 0;
    }
    return (hctrl->state == AIR_CTRL_VENTILATING) ? 1 : 0;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief  启动通风
 * @note   1. 检查窗户是否打开
 *         2. 如果窗户关闭，打开窗户
 *         3. 启动风扇恒速运转
 */
static void StartVentilation(Air_Quality_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }

    /* 检查窗户状态 */
    if (!IsWindowOpen(hctrl)) {
        /* 窗户未打开，打开窗户 */
        if (hctrl->hmotor_window != NULL) {
            /* 根据电机模式选择合适的打开方法 */
            if (hctrl->hmotor_window->mode == MOTOR_MODE_RELATIVE) {
                /* 相对模式：执行打开动作 */
                Motor_ULN2003_Open(hctrl->hmotor_window);
            } else {
                /* 位置模式：移动到全开位置 */
                Motor_ULN2003_MoveToOpen(hctrl->hmotor_window);
            }
            hctrl->window_opened_by_air_ctrl = 1;
        }
    }

    /* 启动风扇恒速运转 */
    if (hctrl->hfan != NULL) {
        Fan_StartConstant(hctrl->hfan);
        hctrl->fan_started_by_air_ctrl = 1;
    }
}

/**
 * @brief  停止通风
 * @note   仅停止由空气质量控制启动的设备
 */
static void StopVentilation(Air_Quality_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }

    /* 停止风扇（如果是由空气质量控制启动的） */
    if (hctrl->fan_started_by_air_ctrl && hctrl->hfan != NULL) {
        Fan_Stop(hctrl->hfan);
        hctrl->fan_started_by_air_ctrl = 0;
    }

    /* 注意：不自动关闭窗户，让用户或其他控制逻辑决定何时关闭 */
    /* 如果需要自动关闭窗户，可以取消下面的注释 */
    /*
    if (hctrl->window_opened_by_air_ctrl && hctrl->hmotor_window != NULL) {
        if (hctrl->hmotor_window->mode == MOTOR_MODE_RELATIVE) {
            Motor_ULN2003_Close(hctrl->hmotor_window);
        } else {
            Motor_ULN2003_MoveToClose(hctrl->hmotor_window);
        }
        hctrl->window_opened_by_air_ctrl = 0;
    }
    */

    hctrl->window_opened_by_air_ctrl = 0;
}

/**
 * @brief  检查窗户是否打开
 * @return 1=窗户打开, 0=窗户关闭
 */
static uint8_t IsWindowOpen(Air_Quality_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL || hctrl->hmotor_window == NULL) {
        return 0;
    }

    Motor_ULN2003_State motor_state = Motor_ULN2003_GetState(hctrl->hmotor_window);

    /* 根据电机模式判断窗户状态 */
    if (hctrl->hmotor_window->mode == MOTOR_MODE_RELATIVE) {
        /* 相对模式：检查状态是否为OPEN或OPENING */
        return (motor_state == MOTOR_ULN2003_OPEN ||
                motor_state == MOTOR_ULN2003_OPENING) ? 1 : 0;
    } else {
        /* 位置模式：检查当前位置是否接近全开位置(0) */
        int32_t current_pos = Motor_ULN2003_GetPosition(hctrl->hmotor_window);
        /* 认为位置在0-100范围内都算打开状态 */
        return (current_pos <= 100) ? 1 : 0;
    }
}
