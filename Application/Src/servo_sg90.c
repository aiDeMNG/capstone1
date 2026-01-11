/**
 * @file    servo_sg90.c
 * @brief   SG90舵机驱动模块
 * @note    使用PWM控制SG90舵机
 *          PA6 - TIM3_CH1 (PWM输出)
 */

#include "servo_sg90.h"

/* ==================== 私有函数声明 ==================== */

static uint16_t Servo_AngleToCCR(uint8_t angle);
static uint8_t Servo_CCRToAngle(uint16_t ccr);
static void Servo_UpdatePWM(Servo_SG90_HandleTypeDef *hservo, uint16_t ccr);

/* ==================== 基础函数实现 ==================== */

/**
 * @brief  初始化SG90舵机
 */
void Servo_SG90_Init(Servo_SG90_HandleTypeDef *hservo,
                     TIM_HandleTypeDef *htim,
                     uint32_t channel)
{
    if (hservo == NULL || htim == NULL) {
        return;
    }

    /* 硬件配置 */
    hservo->htim = htim;
    hservo->channel = channel;

    /* 初始化为关闭状态 (180度) */
    hservo->state = SERVO_SG90_CLOSED;
    hservo->current_angle = SERVO_WINDOW_CLOSED_ANGLE;
    hservo->target_angle = SERVO_WINDOW_CLOSED_ANGLE;
    hservo->current_ccr = SERVO_WINDOW_CLOSED_CCR;
    hservo->target_ccr = SERVO_WINDOW_CLOSED_CCR;

    /* 运动控制 */
    hservo->is_moving = 0;
    hservo->last_update_time = 0;
    hservo->move_delay_ms = 15;  // 默认15ms更新一次，实现平滑运动

    /* 启动PWM并设置初始位置 */
    HAL_TIM_PWM_Start(hservo->htim, hservo->channel);
    Servo_UpdatePWM(hservo, hservo->current_ccr);
}

/**
 * @brief  设置舵机角度 (0-180度)
 */
void Servo_SG90_SetAngle(Servo_SG90_HandleTypeDef *hservo, uint8_t angle)
{
    if (hservo == NULL) {
        return;
    }

    /* 限制角度范围 */
    if (angle > 180) {
        angle = 180;
    }

    hservo->target_angle = angle;
    hservo->target_ccr = Servo_AngleToCCR(angle);

    /* 直接设置到目标位置 (如果需要平滑运动，在Process函数中处理) */
    hservo->current_angle = angle;
    hservo->current_ccr = hservo->target_ccr;

    /* 更新PWM输出 */
    Servo_UpdatePWM(hservo, hservo->current_ccr);

    /* 更新状态 */
    if (angle == SERVO_WINDOW_CLOSED_ANGLE) {
        hservo->state = SERVO_SG90_CLOSED;
    } else if (angle == SERVO_WINDOW_HALF_ANGLE) {
        hservo->state = SERVO_SG90_HALF;
    } else if (angle == SERVO_WINDOW_OPEN_ANGLE) {
        hservo->state = SERVO_SG90_OPEN;
    } else {
        hservo->state = SERVO_SG90_MOVING;
    }
}

/**
 * @brief  设置舵机CCR值 (直接控制脉宽)
 */
void Servo_SG90_SetCCR(Servo_SG90_HandleTypeDef *hservo, uint16_t ccr)
{
    if (hservo == NULL) {
        return;
    }

    /* 限制CCR范围 */
    if (ccr < SERVO_SG90_ANGLE_0_CCR) {
        ccr = SERVO_SG90_ANGLE_0_CCR;
    } else if (ccr > SERVO_SG90_ANGLE_180_CCR) {
        ccr = SERVO_SG90_ANGLE_180_CCR;
    }

    hservo->target_ccr = ccr;
    hservo->current_ccr = ccr;
    hservo->target_angle = Servo_CCRToAngle(ccr);
    hservo->current_angle = hservo->target_angle;

    Servo_UpdatePWM(hservo, ccr);
}

/**
 * @brief  舵机处理函数 (主循环调用)
 * @note   用于实现平滑运动（可选功能，当前直接设置）
 */
void Servo_SG90_Process(Servo_SG90_HandleTypeDef *hservo)
{
    if (hservo == NULL) {
        return;
    }

    /* 当前实现为直接设置，如果需要平滑运动可在此处实现 */
    /* 平滑运动逻辑示例：
     * if (hservo->is_moving) {
     *     if (HAL_GetTick() - hservo->last_update_time >= hservo->move_delay_ms) {
     *         // 逐步移动到目标位置
     *         hservo->last_update_time = HAL_GetTick();
     *     }
     * }
     */
}

/**
 * @brief  停止舵机 (停止PWM输出)
 */
void Servo_SG90_Stop(Servo_SG90_HandleTypeDef *hservo)
{
    if (hservo == NULL) {
        return;
    }

    HAL_TIM_PWM_Stop(hservo->htim, hservo->channel);
    hservo->is_moving = 0;
    hservo->state = SERVO_SG90_IDLE;
}

/**
 * @brief  启动舵机PWM输出
 */
void Servo_SG90_Start(Servo_SG90_HandleTypeDef *hservo)
{
    if (hservo == NULL) {
        return;
    }

    HAL_TIM_PWM_Start(hservo->htim, hservo->channel);
    Servo_UpdatePWM(hservo, hservo->current_ccr);
}

/**
 * @brief  获取舵机状态
 */
Servo_SG90_State Servo_SG90_GetState(Servo_SG90_HandleTypeDef *hservo)
{
    if (hservo == NULL) {
        return SERVO_SG90_IDLE;
    }
    return hservo->state;
}

/**
 * @brief  获取当前角度
 */
uint8_t Servo_SG90_GetAngle(Servo_SG90_HandleTypeDef *hservo)
{
    if (hservo == NULL) {
        return 0;
    }
    return hservo->current_angle;
}

/* ==================== 窗户控制函数实现 ==================== */

/**
 * @brief  打开窗户 (设置为全开角度90度)
 */
void Servo_SG90_OpenWindow(Servo_SG90_HandleTypeDef *hservo)
{
    if (hservo == NULL) {
        return;
    }

    Servo_SG90_SetAngle(hservo, SERVO_WINDOW_OPEN_ANGLE);
    hservo->state = SERVO_SG90_OPEN;
}

/**
 * @brief  半开窗户 (设置为半开角度135度)
 */
void Servo_SG90_HalfWindow(Servo_SG90_HandleTypeDef *hservo)
{
    if (hservo == NULL) {
        return;
    }

    Servo_SG90_SetAngle(hservo, SERVO_WINDOW_HALF_ANGLE);
    hservo->state = SERVO_SG90_HALF;
}

/**
 * @brief  关闭窗户 (设置为全关角度180度)
 */
void Servo_SG90_CloseWindow(Servo_SG90_HandleTypeDef *hservo)
{
    if (hservo == NULL) {
        return;
    }

    Servo_SG90_SetAngle(hservo, SERVO_WINDOW_CLOSED_ANGLE);
    hservo->state = SERVO_SG90_CLOSED;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief  角度转换为CCR值
 * @param  angle: 角度 (0-180)
 * @return CCR值
 * @note   线性映射：0度=1000, 180度=5000
 *         CCR = 1000 + (angle / 180) * 4000
 */
static uint16_t Servo_AngleToCCR(uint8_t angle)
{
    if (angle > 180) {
        angle = 180;
    }

    /* 线性映射：CCR = 1000 + (angle * 4000) / 180 */
    uint32_t ccr = SERVO_SG90_ANGLE_0_CCR +
                   ((uint32_t)angle * (SERVO_SG90_ANGLE_180_CCR - SERVO_SG90_ANGLE_0_CCR)) / 180;

    return (uint16_t)ccr;
}

/**
 * @brief  CCR值转换为角度
 * @param  ccr: CCR值
 * @return 角度 (0-180)
 * @note   反向映射：angle = (CCR - 1000) * 180 / 4000
 */
static uint8_t Servo_CCRToAngle(uint16_t ccr)
{
    if (ccr < SERVO_SG90_ANGLE_0_CCR) {
        ccr = SERVO_SG90_ANGLE_0_CCR;
    } else if (ccr > SERVO_SG90_ANGLE_180_CCR) {
        ccr = SERVO_SG90_ANGLE_180_CCR;
    }

    /* 反向映射：angle = (ccr - 1000) * 180 / 4000 */
    uint32_t angle = ((uint32_t)(ccr - SERVO_SG90_ANGLE_0_CCR) * 180) /
                     (SERVO_SG90_ANGLE_180_CCR - SERVO_SG90_ANGLE_0_CCR);

    return (uint8_t)angle;
}

/**
 * @brief  更新PWM输出
 * @param  hservo: 舵机句柄
 * @param  ccr: CCR值
 */
static void Servo_UpdatePWM(Servo_SG90_HandleTypeDef *hservo, uint16_t ccr)
{
    if (hservo == NULL) {
        return;
    }

    /* 设置PWM占空比 */
    __HAL_TIM_SET_COMPARE(hservo->htim, hservo->channel, ccr);
}
