/**
 * @file    air_quality_control.c
 * @brief   空气质量监测与自动通风控制模块
 * @note    监测MQ135传感器，当空气质量差时自动打开窗户、窗帘并启动风扇通风
 *          空气质量恢复后，自动关闭窗户和窗帘
 *          优先级: 空气质量控制 > 光照控制
 */

#include "air_quality_control.h"

/* ==================== 私有函数声明 ==================== */

static void StartVentilation(Air_Quality_Control_HandleTypeDef *hctrl);
static void StopVentilation(Air_Quality_Control_HandleTypeDef *hctrl);
static uint8_t IsWindowOpen(Air_Quality_Control_HandleTypeDef *hctrl);
static uint8_t IsCurtainFullyOpen(Air_Quality_Control_HandleTypeDef *hctrl);

/* ==================== 公共函数实现 ==================== */

/**
 * @brief  初始化空气质量控制模块
 */
void AirQualityControl_Init(Air_Quality_Control_HandleTypeDef *hctrl,
                            Servo_SG90_HandleTypeDef *hservo_window,
                            Motor_A4988_HandleTypeDef *hmotor_curtain,
                            Fan_HandleTypeDef *hfan)
{
    if (hctrl == NULL) {
        return;
    }

    hctrl->hservo_window = hservo_window;
    hctrl->hmotor_curtain = hmotor_curtain;
    hctrl->hfan = hfan;
    hctrl->state = AIR_CTRL_IDLE;
    hctrl->current_priority = PRIORITY_NONE;
    hctrl->air_quality_bad = 0;
    hctrl->window_opened_by_air_ctrl = 0;
    hctrl->curtain_opened_by_air_ctrl = 0;
    hctrl->fan_started_by_air_ctrl = 0;
    hctrl->saved_window_angle = 0;
    hctrl->saved_curtain_position = 0;
    hctrl->state_saved = 0;
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
    hctrl->air_quality_bad = air_quality_is_bad();

    /* 空气质量差，需要通风 */
    if (hctrl->air_quality_bad) {
        /* 设置高优先级 */
        hctrl->current_priority = PRIORITY_AIR_QUALITY;

        /* 如果还未开始通风，启动通风 */
        if (hctrl->state != AIR_CTRL_VENTILATING) {
            StartVentilation(hctrl);
            hctrl->state = AIR_CTRL_VENTILATING;
        }
    }
    /* 空气质量正常，立即停止通风 */
    else {
        /* 如果正在通风，立即停止 */
        if (hctrl->state == AIR_CTRL_VENTILATING) {
            StopVentilation(hctrl);
            hctrl->state = AIR_CTRL_IDLE;
        }

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
 * @note   1. 检查窗户是否打开，如果未打开则打开窗户
 *         2. 打开窗帘到全开位置
 *         3. 启动风扇恒速运转
 */
static void StartVentilation(Air_Quality_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }

    /* 检查窗户状态 */
    if (!IsWindowOpen(hctrl)) {
        /* 窗户未打开，打开窗户到全开位置 (90度) */
        if (hctrl->hservo_window != NULL) {
            Servo_SG90_OpenWindow(hctrl->hservo_window);
            hctrl->window_opened_by_air_ctrl = 1;
        }
    }

    /* 窗帘移动到全开位置 */
    if (hctrl->hmotor_curtain != NULL) {
        /* 每次空气质量差时都尝试打开窗帘 */
        /* 如果窗帘已经在全开位置，MoveToOpen不会触发移动 */
        Motor_A4988_MoveToOpen(hctrl->hmotor_curtain);
        hctrl->curtain_opened_by_air_ctrl = 1;
    }

    /* 启动风扇恒速运转 */
    if (hctrl->hfan != NULL) {
        Fan_StartConstant(hctrl->hfan);
        hctrl->fan_started_by_air_ctrl = 1;
    }
}

/**
 * @brief  停止通风
 * @note   1. 停止风扇
 *         2. 关闭窗户和窗帘
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

    /* 关闭窗户（如果是由空气质量控制打开的） */
    if (hctrl->window_opened_by_air_ctrl && hctrl->hservo_window != NULL) {
        Servo_SG90_CloseWindow(hctrl->hservo_window);  // 关闭到180度
        hctrl->window_opened_by_air_ctrl = 0;
    }

    /* 关闭窗帘（如果是由空气质量控制打开的） */
    if (hctrl->curtain_opened_by_air_ctrl && hctrl->hmotor_curtain != NULL) {
        Motor_A4988_MoveToClose(hctrl->hmotor_curtain);  // 关闭到位置0
        hctrl->curtain_opened_by_air_ctrl = 0;
    }
}

/**
 * @brief  检查窗户是否打开
 * @return 1=窗户打开, 0=窗户关闭
 */
static uint8_t IsWindowOpen(Air_Quality_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL || hctrl->hservo_window == NULL) {
        return 0;
    }

    Servo_SG90_State servo_state = Servo_SG90_GetState(hctrl->hservo_window);

    /* 检查舵机状态是否为打开或半开 */
    return (servo_state == SERVO_SG90_OPEN ||
            servo_state == SERVO_SG90_HALF ||
            servo_state == SERVO_SG90_OPENING) ? 1 : 0;
}

/**
 * @brief  检查窗帘是否全开
 * @return 1=窗帘全开, 0=窗帘未全开
 */
static uint8_t IsCurtainFullyOpen(Air_Quality_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL || hctrl->hmotor_curtain == NULL) {
        return 0;
    }

    /* A4988窗帘使用位置模式，全开位置为max_position（10圈，32000步） */
    int32_t current_pos = Motor_A4988_GetPosition(hctrl->hmotor_curtain);
    int32_t max_pos = hctrl->hmotor_curtain->max_position;

    /* 认为位置在max_position±100范围内都算全开状态 */
    return (current_pos >= max_pos - 100 && current_pos <= max_pos) ? 1 : 0;
}
