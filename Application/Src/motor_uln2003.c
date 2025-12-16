/**
 * @file    motor_uln2003.c
 * @brief   ULN2003步进电机驱动模块
 * @note    控制ULN2003驱动板的步进电机 (28BYJ-48)
 *          使用4相8拍驱动方式
 *          ULN2003反向驱动: 低电平导通
 *          支持两种控制模式：相对模式、位置模式
 */

#include "motor_uln2003.h"

/* 步进电机相位表 (4相8拍) */
static const uint8_t phase_table[MOTOR_ULN2003_PHASE_COUNT] = {
    0x01,   // 0001 - A
    0x03,   // 0011 - AB
    0x02,   // 0010 - B
    0x06,   // 0110 - BC
    0x04,   // 0100 - C
    0x0C,   // 1100 - CD
    0x08,   // 1000 - D
    0x09    // 1001 - DA
};

/* ==================== 私有函数声明 ==================== */

static void Motor_ULN2003_SetPhase(Motor_ULN2003_HandleTypeDef *hmotor, uint8_t phase);
static void Motor_ULN2003_StepForward(Motor_ULN2003_HandleTypeDef *hmotor);
static void Motor_ULN2003_StepBackward(Motor_ULN2003_HandleTypeDef *hmotor);
static void Motor_ULN2003_Release(Motor_ULN2003_HandleTypeDef *hmotor);

static void Motor_Process_RelativeMode(Motor_ULN2003_HandleTypeDef *hmotor);
static void Motor_Process_PositionMode(Motor_ULN2003_HandleTypeDef *hmotor);

/* ==================== 基础函数实现 ==================== */

/**
 * @brief  初始化ULN2003电机
 */
void Motor_ULN2003_Init(Motor_ULN2003_HandleTypeDef *hmotor,
                        GPIO_TypeDef *port,
                        uint16_t pin_in1,
                        uint16_t pin_in2,
                        uint16_t pin_in3,
                        uint16_t pin_in4)
{
    if (hmotor == NULL) {
        return;
    }

    /* GPIO配置 */
    hmotor->port = port;
    hmotor->pin_in1 = pin_in1;
    hmotor->pin_in2 = pin_in2;
    hmotor->pin_in3 = pin_in3;
    hmotor->pin_in4 = pin_in4;

    /* 默认为相对模式 */
    hmotor->mode = MOTOR_MODE_RELATIVE;
    hmotor->state = MOTOR_ULN2003_CLOSED;

    /* 相位 */
    hmotor->current_phase = 0;

    /* 相对模式初始化 */
    hmotor->step_count = 0;
    hmotor->relative_steps = MOTOR_STEPS_QUARTER;  // 默认1/4圈

    /* 位置模式初始化 */
    hmotor->current_position = 0;
    hmotor->target_position = 0;
    hmotor->max_position = MOTOR_STEPS_FULL;  // 默认一圈
    hmotor->is_moving = 0;

    /* 释放电机 (初始不通电) */
    Motor_ULN2003_Release(hmotor);
}

/**
 * @brief  设置电机控制模式
 */
void Motor_ULN2003_SetMode(Motor_ULN2003_HandleTypeDef *hmotor, Motor_ULN2003_Mode mode)
{
    if (hmotor == NULL) {
        return;
    }

    /* 切换模式前停止电机 */
    Motor_ULN2003_Stop(hmotor);

    hmotor->mode = mode;

    /* 根据模式初始化状态 */
    if (mode == MOTOR_MODE_RELATIVE) {
        hmotor->state = MOTOR_ULN2003_CLOSED;
    } else {
        hmotor->state = MOTOR_ULN2003_AT_POSITION;
    }
}

/**
 * @brief  设置最大位置（仅位置模式有效）
 */
void Motor_ULN2003_SetMaxPosition(Motor_ULN2003_HandleTypeDef *hmotor, int32_t max_pos)
{
    if (hmotor == NULL || max_pos <= 0) {
        return;
    }
    hmotor->max_position = max_pos;
}

/**
 * @brief  设置相对模式的单次移动步数
 */
void Motor_ULN2003_SetRelativeSteps(Motor_ULN2003_HandleTypeDef *hmotor, int32_t steps)
{
    if (hmotor == NULL || steps <= 0) {
        return;
    }
    hmotor->relative_steps = steps;
}

/**
 * @brief  电机处理函数 (主循环调用)
 */
void Motor_ULN2003_Process(Motor_ULN2003_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return;
    }

    /* 根据控制模式调用对应的处理函数 */
    if (hmotor->mode == MOTOR_MODE_RELATIVE) {
        Motor_Process_RelativeMode(hmotor);
    } else {
        Motor_Process_PositionMode(hmotor);
    }
}

/**
 * @brief  停止电机
 */
void Motor_ULN2003_Stop(Motor_ULN2003_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return;
    }

    hmotor->step_count = 0;
    hmotor->is_moving = 0;
    hmotor->state = MOTOR_ULN2003_IDLE;
    Motor_ULN2003_Release(hmotor);
}

/**
 * @brief  获取电机状态
 */
Motor_ULN2003_State Motor_ULN2003_GetState(Motor_ULN2003_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return MOTOR_ULN2003_IDLE;
    }
    return hmotor->state;
}

/* ==================== 相对模式函数实现 ==================== */

/**
 * @brief  启动电机打开动作（相对模式）
 */
void Motor_ULN2003_Open(Motor_ULN2003_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return;
    }
    hmotor->state = MOTOR_ULN2003_OPENING;
    hmotor->step_count = hmotor->relative_steps;
}

/**
 * @brief  启动电机关闭动作（相对模式）
 */
void Motor_ULN2003_Close(Motor_ULN2003_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return;
    }
    hmotor->state = MOTOR_ULN2003_CLOSING;
    hmotor->step_count = hmotor->relative_steps;
}

/**
 * @brief  启动电机半开动作（相对模式）
 */
void Motor_ULN2003_Half(Motor_ULN2003_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return;
    }
    hmotor->state = MOTOR_ULN2003_OPENING;
    hmotor->step_count = hmotor->relative_steps / 2;
}

/* ==================== 位置模式函数实现 ==================== */

/**
 * @brief  移动到指定位置（位置模式）
 */
void Motor_ULN2003_MoveToPosition(Motor_ULN2003_HandleTypeDef *hmotor, int32_t position)
{
    if (hmotor == NULL || hmotor->mode != MOTOR_MODE_POSITION) {
        return;
    }

    /* 限制位置范围 */
    if (position < 0) {
        position = 0;
    } else if (position > hmotor->max_position) {
        position = hmotor->max_position;
    }

    hmotor->target_position = position;

    /* 如果已经在目标位置，不需要移动 */
    if (hmotor->current_position == hmotor->target_position) {
        hmotor->is_moving = 0;
        hmotor->state = MOTOR_ULN2003_AT_POSITION;
        return;
    }

    /* 开始移动 */
    hmotor->is_moving = 1;
    hmotor->state = MOTOR_ULN2003_MOVING;
}

/**
 * @brief  移动到全开位置（位置模式）
 */
void Motor_ULN2003_MoveToOpen(Motor_ULN2003_HandleTypeDef *hmotor)
{
    Motor_ULN2003_MoveToPosition(hmotor, 0);
}

/**
 * @brief  移动到半开位置（位置模式）
 */
void Motor_ULN2003_MoveToHalf(Motor_ULN2003_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return;
    }
    Motor_ULN2003_MoveToPosition(hmotor, hmotor->max_position / 2);
}

/**
 * @brief  移动到全关位置（位置模式）
 */
void Motor_ULN2003_MoveToClose(Motor_ULN2003_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return;
    }
    Motor_ULN2003_MoveToPosition(hmotor, hmotor->max_position);
}

/**
 * @brief  获取当前位置（位置模式）
 */
int32_t Motor_ULN2003_GetPosition(Motor_ULN2003_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return 0;
    }
    return hmotor->current_position;
}

/**
 * @brief  复位位置为0（位置模式）
 */
void Motor_ULN2003_ResetPosition(Motor_ULN2003_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return;
    }

    hmotor->current_position = 0;
    hmotor->target_position = 0;
    hmotor->state = MOTOR_ULN2003_AT_POSITION;
    hmotor->is_moving = 0;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief  相对模式处理函数
 */
static void Motor_Process_RelativeMode(Motor_ULN2003_HandleTypeDef *hmotor)
{
    if (hmotor->step_count <= 0) {
        return;
    }

    /* 执行步进 */
    if (hmotor->state == MOTOR_ULN2003_OPENING) {
        Motor_ULN2003_StepForward(hmotor);
    } else if (hmotor->state == MOTOR_ULN2003_CLOSING) {
        Motor_ULN2003_StepBackward(hmotor);
    }

    HAL_Delay(MOTOR_ULN2003_STEP_DELAY_MS);
    hmotor->step_count--;

    /* 检查是否完成 */
    if (hmotor->step_count <= 0) {
        if (hmotor->state == MOTOR_ULN2003_OPENING) {
            hmotor->state = MOTOR_ULN2003_OPEN;
        } else if (hmotor->state == MOTOR_ULN2003_CLOSING) {
            hmotor->state = MOTOR_ULN2003_CLOSED;
        }
        Motor_ULN2003_Release(hmotor);
    }
}

/**
 * @brief  位置模式处理函数
 */
static void Motor_Process_PositionMode(Motor_ULN2003_HandleTypeDef *hmotor)
{
    if (!hmotor->is_moving) {
        return;
    }

    /* 检查是否到达目标位置 */
    if (hmotor->current_position == hmotor->target_position) {
        hmotor->is_moving = 0;
        hmotor->state = MOTOR_ULN2003_AT_POSITION;
        Motor_ULN2003_Release(hmotor);
        return;
    }

    /* 根据方向移动一步 */
    if (hmotor->current_position < hmotor->target_position) {
        /* 需要正转 (关闭方向) */
        Motor_ULN2003_StepForward(hmotor);
        hmotor->current_position++;
    } else {
        /* 需要反转 (打开方向) */
        Motor_ULN2003_StepBackward(hmotor);
        hmotor->current_position--;
    }

    /* 步进延时 */
    HAL_Delay(MOTOR_ULN2003_STEP_DELAY_MS);
}

/**
 * @brief  设置ULN2003电机相位输出
 */
static void Motor_ULN2003_SetPhase(Motor_ULN2003_HandleTypeDef *hmotor, uint8_t phase)
{
    uint8_t seq = phase_table[phase & 0x07];

    /* ULN2003反向驱动: 低电平导通 */
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_in1, (seq & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_in2, (seq & 0x02) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_in3, (seq & 0x04) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_in4, (seq & 0x08) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief  ULN2003电机正转一步
 */
static void Motor_ULN2003_StepForward(Motor_ULN2003_HandleTypeDef *hmotor)
{
    hmotor->current_phase++;
    if (hmotor->current_phase >= MOTOR_ULN2003_PHASE_COUNT) {
        hmotor->current_phase = 0;
    }
    Motor_ULN2003_SetPhase(hmotor, hmotor->current_phase);
}

/**
 * @brief  ULN2003电机反转一步
 */
static void Motor_ULN2003_StepBackward(Motor_ULN2003_HandleTypeDef *hmotor)
{
    if (hmotor->current_phase == 0) {
        hmotor->current_phase = MOTOR_ULN2003_PHASE_COUNT - 1;
    } else {
        hmotor->current_phase--;
    }
    Motor_ULN2003_SetPhase(hmotor, hmotor->current_phase);
}

/**
 * @brief  释放ULN2003电机 (断电)
 */
static void Motor_ULN2003_Release(Motor_ULN2003_HandleTypeDef *hmotor)
{
    /* 所有引脚输出高电平 (ULN2003: 高电平=断开) */
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_in1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_in2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_in3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_in4, GPIO_PIN_SET);
}
