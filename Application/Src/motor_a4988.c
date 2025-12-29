/**
 * @file    motor_a4988.c
 * @brief   A4988步进电机驱动模块
 * @note    控制A4988驱动板的步进电机
 *          使用STEP/DIR控制方式
 *          支持细分模式配置 (当前使用1/16步进)
 *          EN引脚接地(常使能)
 *          支持两种控制模式：相对模式、位置模式
 */

#include "motor_a4988.h"

/* ==================== 私有函数声明 ==================== */

static void Motor_A4988_SetDir(Motor_A4988_HandleTypeDef *hmotor, uint8_t dir);
static void Motor_A4988_Step(Motor_A4988_HandleTypeDef *hmotor);
static void delay_us(uint32_t us);

static void Motor_Process_RelativeMode(Motor_A4988_HandleTypeDef *hmotor);
static void Motor_Process_PositionMode(Motor_A4988_HandleTypeDef *hmotor);

/* ==================== 基础函数实现 ==================== */

/**
 * @brief  初始化A4988电机
 */
void Motor_A4988_Init(Motor_A4988_HandleTypeDef *hmotor,
                      GPIO_TypeDef *port,
                      uint16_t pin_step,
                      uint16_t pin_dir,
                      uint16_t pin_ms1,
                      uint16_t pin_ms2,
                      uint16_t pin_ms3)
{
    if (hmotor == NULL)
    {
        return;
    }

    /* GPIO配置 */
    hmotor->port = port;
    hmotor->pin_step = pin_step;
    hmotor->pin_dir = pin_dir;
    hmotor->pin_ms1 = pin_ms1;
    hmotor->pin_ms2 = pin_ms2;
    hmotor->pin_ms3 = pin_ms3;

    /* 默认为相对模式 */
    hmotor->mode = MOTOR_A4988_MODE_RELATIVE;
    hmotor->state = MOTOR_A4988_CLOSED;

    /* 相对模式初始化 */
    hmotor->step_count = 0;
    hmotor->relative_steps = MOTOR_A4988_STEPS_QUARTER; // 默认1/4圈

    /* 位置模式初始化 */
    hmotor->current_position = 0;
    hmotor->target_position = 0;
    hmotor->max_position = MOTOR_A4988_STEPS_FULL; // 默认一圈
    hmotor->is_moving = 0;

    /* 设置细分模式: MS1=HIGH, MS2=HIGH, MS3=HIGH -> 1/16步进 */
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_ms1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_ms2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_ms3, GPIO_PIN_SET);

    /* STEP和DIR初始状态 */
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_step, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_dir, GPIO_PIN_RESET);
}

/**
 * @brief  设置电机控制模式
 */
void Motor_A4988_SetMode(Motor_A4988_HandleTypeDef *hmotor, Motor_A4988_Mode mode)
{
    if (hmotor == NULL)
    {
        return;
    }

    /* 切换模式前停止电机 */
    Motor_A4988_Stop(hmotor);

    hmotor->mode = mode;

    /* 根据模式初始化状态 */
    if (mode == MOTOR_A4988_MODE_RELATIVE)
    {
        hmotor->state = MOTOR_A4988_CLOSED;
    }
    else
    {
        hmotor->state = MOTOR_A4988_AT_POSITION;
    }
}

/**
 * @brief  设置最大位置（仅位置模式有效）
 */
void Motor_A4988_SetMaxPosition(Motor_A4988_HandleTypeDef *hmotor, int32_t max_pos)
{
    if (hmotor == NULL || max_pos <= 0)
    {
        return;
    }
    hmotor->max_position = max_pos;
}

/**
 * @brief  设置相对模式的单次移动步数
 */
void Motor_A4988_SetRelativeSteps(Motor_A4988_HandleTypeDef *hmotor, int32_t steps)
{
    if (hmotor == NULL || steps <= 0)
    {
        return;
    }
    hmotor->relative_steps = steps;
}

/**
 * @brief  电机处理函数 (主循环调用)
 */
void Motor_A4988_Process(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL)
    {
        return;
    }

    /* 根据控制模式调用对应的处理函数 */
    if (hmotor->mode == MOTOR_A4988_MODE_RELATIVE)
    {
        Motor_Process_RelativeMode(hmotor);
    }
    else
    {
        Motor_Process_PositionMode(hmotor);
    }
}

/**
 * @brief  停止电机
 */
void Motor_A4988_Stop(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL)
    {
        return;
    }

    hmotor->step_count = 0;
    hmotor->is_moving = 0;
    hmotor->state = MOTOR_A4988_IDLE;
}

/**
 * @brief  获取电机状态
 */
Motor_A4988_State Motor_A4988_GetState(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL)
    {
        return MOTOR_A4988_IDLE;
    }
    return hmotor->state;
}

/* ==================== 相对模式函数实现 ==================== */

/**
 * @brief  启动电机打开动作（相对模式）
 */
void Motor_A4988_Open(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL)
    {
        return;
    }
    hmotor->state = MOTOR_A4988_OPENING;
    hmotor->step_count = hmotor->relative_steps;
}

/**
 * @brief  启动电机关闭动作（相对模式）
 */
void Motor_A4988_Close(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL)
    {
        return;
    }
    hmotor->state = MOTOR_A4988_CLOSING;
    hmotor->step_count = hmotor->relative_steps;
}

/**
 * @brief  启动电机半开动作（相对模式）
 */
void Motor_A4988_Half(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL)
    {
        return;
    }
    hmotor->state = MOTOR_A4988_OPENING;
    hmotor->step_count = hmotor->relative_steps / 2;
}

/* ==================== 位置模式函数实现 ==================== */

/**
 * @brief  移动到指定位置（位置模式）
 */
void Motor_A4988_MoveToPosition(Motor_A4988_HandleTypeDef *hmotor, int32_t position)
{
    if (hmotor == NULL || hmotor->mode != MOTOR_A4988_MODE_POSITION)
    {
        return;
    }

    /* 限制位置范围 */
    if (position < 0)
    {
        position = 0;
    }
    else if (position > hmotor->max_position)
    {
        position = hmotor->max_position;
    }

    hmotor->target_position = position;

    /* 如果已经在目标位置，不需要移动 */
    if (hmotor->current_position == hmotor->target_position)
    {
        hmotor->is_moving = 0;
        hmotor->state = MOTOR_A4988_AT_POSITION;
        return;
    }

    /* 开始移动 */
    hmotor->is_moving = 1;
    hmotor->state = MOTOR_A4988_MOVING;
}

/**
 * @brief  移动到全开位置（位置模式）
 * @note   全开位置 = max_position（10圈，32000步）
 */
void Motor_A4988_MoveToOpen(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL)
    {
        return;
    }
    Motor_A4988_MoveToPosition(hmotor, hmotor->max_position);  // 全开 = 10圈
}

/**
 * @brief  移动到半开位置（位置模式）
 * @note   半开位置 = max_position / 2（5圈，16000步）
 */
void Motor_A4988_MoveToHalf(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL)
    {
        return;
    }
    Motor_A4988_MoveToPosition(hmotor, hmotor->max_position / 2);  // 半开 = 5圈
}

/**
 * @brief  移动到全关位置（位置模式）
 * @note   全关位置 = 0
 */
void Motor_A4988_MoveToClose(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL)
    {
        return;
    }
    Motor_A4988_MoveToPosition(hmotor, 0);  // 全关 = 0
}

/**
 * @brief  获取当前位置（位置模式）
 */
int32_t Motor_A4988_GetPosition(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL)
    {
        return 0;
    }
    return hmotor->current_position;
}

/**
 * @brief  复位位置为0（位置模式）
 */
void Motor_A4988_ResetPosition(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL)
    {
        return;
    }

    hmotor->current_position = 0;
    hmotor->target_position = 0;
    hmotor->state = MOTOR_A4988_AT_POSITION;
    hmotor->is_moving = 0;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief  相对模式处理函数
 */
static void Motor_Process_RelativeMode(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor->step_count <= 0)
    {
        return;
    }

    /* 设置方向并执行步进 */
    if (hmotor->state == MOTOR_A4988_OPENING)
    {
        Motor_A4988_SetDir(hmotor, 1); // 正向
    }
    else if (hmotor->state == MOTOR_A4988_CLOSING)
    {
        Motor_A4988_SetDir(hmotor, 0); // 反向
    }

    Motor_A4988_Step(hmotor);
    delay_us(MOTOR_A4988_DELAY_US);
    hmotor->step_count--;

    /* 检查是否完成 */
    if (hmotor->step_count <= 0)
    {
        if (hmotor->state == MOTOR_A4988_OPENING)
        {
            hmotor->state = MOTOR_A4988_OPEN;
        }
        else if (hmotor->state == MOTOR_A4988_CLOSING)
        {
            hmotor->state = MOTOR_A4988_CLOSED;
        }
    }
}

/**
 * @brief  位置模式处理函数
 */
static void Motor_Process_PositionMode(Motor_A4988_HandleTypeDef *hmotor)
{
    if (!hmotor->is_moving)
    {
        return;
    }

    /* 检查是否到达目标位置 */
    if (hmotor->current_position == hmotor->target_position)
    {
        hmotor->is_moving = 0;
        hmotor->state = MOTOR_A4988_AT_POSITION;
        return;
    }

    /* 根据方向移动一步 */
    if (hmotor->current_position < hmotor->target_position)
    {
        /* 需要正转 (关闭方向) */
        Motor_A4988_SetDir(hmotor, 0);
        Motor_A4988_Step(hmotor);
        hmotor->current_position++;
    }
    else
    {
        /* 需要反转 (打开方向) */
        Motor_A4988_SetDir(hmotor, 1);
        Motor_A4988_Step(hmotor);
        hmotor->current_position--;
    }

    /* 步进延时 */
    delay_us(MOTOR_A4988_DELAY_US);
}

/**
 * @brief  设置A4988电机方向
 * @param  dir: 1=正向(HIGH), 0=反向(LOW)
 */
static void Motor_A4988_SetDir(Motor_A4988_HandleTypeDef *hmotor, uint8_t dir)
{
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_dir, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  A4988执行一步
 * @note   产生一个STEP脉冲
 */
static void Motor_A4988_Step(Motor_A4988_HandleTypeDef *hmotor)
{
    /* 产生上升沿脉冲 */
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_step, GPIO_PIN_SET);
    delay_us(MOTOR_A4988_PULSE_US);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_step, GPIO_PIN_RESET);
}

/**
 * @brief  简单微秒延时
 * @note   基于循环的粗略延时, 需根据系统时钟调整
 */
static void delay_us(uint32_t us)
{
    /* 假设72MHz时钟, 每次循环约14个时钟周期 */
    volatile uint32_t count = us * 5;
    while (count--)
    {
        __NOP();
    }
}
