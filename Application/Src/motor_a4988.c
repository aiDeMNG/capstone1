/**
 * @file    motor_a4988.c
 * @brief   A4988步进电机驱动模块
 * @note    控制A4988驱动板的步进电机
 *          使用STEP/DIR控制方式
 *          支持细分模式配置 (当前使用1/16步进)
 *          EN引脚接地(常使能)
 */

#include "motor_a4988.h"

/* ==================== 私有函数声明 ==================== */

static void Motor_A4988_SetDir(Motor_A4988_HandleTypeDef *hmotor, uint8_t dir);
static void Motor_A4988_Step(Motor_A4988_HandleTypeDef *hmotor);
static void delay_us(uint32_t us);

/* ==================== 公共函数实现 ==================== */

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
    if (hmotor == NULL) {
        return;
    }

    hmotor->state = MOTOR_A4988_CLOSED;
    hmotor->step_count = 0;
    hmotor->port = port;
    hmotor->pin_step = pin_step;
    hmotor->pin_dir = pin_dir;
    hmotor->pin_ms1 = pin_ms1;
    hmotor->pin_ms2 = pin_ms2;
    hmotor->pin_ms3 = pin_ms3;

    /* 设置细分模式: MS1=HIGH, MS2=HIGH, MS3=HIGH -> 1/16步进 */
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_ms1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_ms2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_ms3, GPIO_PIN_SET);

    /* STEP和DIR初始状态 */
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_step, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_dir, GPIO_PIN_RESET);
}

/**
 * @brief  启动电机打开动作
 */
void Motor_A4988_Open(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return;
    }
    Motor_A4988_SetDir(hmotor, 1);  // 设置方向为正向
    hmotor->state = MOTOR_A4988_OPENING;
    hmotor->step_count = MOTOR_A4988_STEPS;
}

/**
 * @brief  启动电机关闭动作
 */
void Motor_A4988_Close(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return;
    }
    Motor_A4988_SetDir(hmotor, 0);  // 设置方向为反向
    hmotor->state = MOTOR_A4988_CLOSING;
    hmotor->step_count = MOTOR_A4988_STEPS;
}

/**
 * @brief  停止电机
 */
void Motor_A4988_Stop(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return;
    }
    hmotor->step_count = 0;
    hmotor->state = MOTOR_A4988_IDLE;
}

/**
 * @brief  电机处理函数 (主循环调用)
 */
void Motor_A4988_Process(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL || hmotor->step_count <= 0) {
        return;
    }

    /* 执行步进 - 方向已在Open/Close函数中设置 */
    Motor_A4988_Step(hmotor);
    delay_us(MOTOR_A4988_DELAY_US);
    hmotor->step_count--;

    /* 检查是否完成 */
    if (hmotor->step_count <= 0) {
        if (hmotor->state == MOTOR_A4988_OPENING) {
            hmotor->state = MOTOR_A4988_OPEN;
        } else if (hmotor->state == MOTOR_A4988_CLOSING) {
            hmotor->state = MOTOR_A4988_CLOSED;
        }
    }
}

/**
 * @brief  获取电机状态
 */
Motor_A4988_State Motor_A4988_GetState(Motor_A4988_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return MOTOR_A4988_IDLE;
    }
    return hmotor->state;
}

/* ==================== 私有函数实现 ==================== */

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
    while (count--) {
        __NOP();
    }
}
