/**
 * @file    ceshi.c
 * @brief   电机测试程序
 * @note    测试两个步进电机:
 *          - ULN2003 (窗户电机): PA2, PA3, PA4, PA5 - 转2圈后反转2圈，循环
 *          - A4988 (窗帘电机): PA8(STEP), PA9(DIR), PA10(MS1), PA11(MS2), PA12(MS3) - 转5圈后反转5圈，循环
 *          EN引脚接地(常使能)
 *
 *          A4988细分模式: MS1=H, MS2=H, MS3=H -> 1/16步进 (3200步/圈)
 */

#include "ceshi.h"

/* ==================== ULN2003相位表 (4相8拍) ==================== */
static const uint8_t uln2003_phase_table[8] = {
    0x01,   // 0001 - A
    0x03,   // 0011 - AB
    0x02,   // 0010 - B
    0x06,   // 0110 - BC
    0x04,   // 0100 - C
    0x0C,   // 1100 - CD
    0x08,   // 1000 - D
    0x09    // 1001 - DA
};

/* 当前相位 */
static uint8_t uln2003_phase = 0;

/* ==================== 私有函数声明 ==================== */

/* 延时函数 */
static void ceshi_delay_us(uint32_t us);

/* ULN2003函数 */
static void ceshi_uln2003_set_phase(uint8_t phase);
static void ceshi_uln2003_step_forward(void);
static void ceshi_uln2003_step_backward(void);
static void ceshi_uln2003_release(void);
static void ceshi_uln2003_rotate(int32_t steps);

/* A4988函数 */
static void ceshi_a4988_init(void);
static void ceshi_a4988_set_dir(uint8_t dir);
static void ceshi_a4988_step(void);
static void ceshi_a4988_rotate(uint32_t steps, uint8_t dir);

/* ==================== 公共函数实现 ==================== */

/**
 * @brief  测试程序初始化
 */
void Ceshi_Init(void)
{
    /* 释放ULN2003电机 */
    ceshi_uln2003_release();

    /* 初始化A4988 */
    ceshi_a4988_init();
}

/**
 * @brief  测试程序主循环 - 同时测试两个电机
 * @note   由于两个电机需要同时运行，这里采用交替步进的方式
 */
void Ceshi_Run(void)
{
    uint32_t uln2003_total_steps = CESHI_ULN2003_STEPS_PER_REV * CESHI_ULN2003_REVS;
    uint32_t a4988_total_steps = CESHI_A4988_STEPS_PER_REV * CESHI_A4988_REVS;

    while (1) {
        /* ===== 正转阶段 ===== */
        ceshi_a4988_set_dir(1);  // A4988正转方向

        uint32_t uln2003_count = 0;
        uint32_t a4988_count = 0;

        /* 同时运行两个电机正转 */
        while (uln2003_count < uln2003_total_steps || a4988_count < a4988_total_steps) {
            /* ULN2003步进 */
            if (uln2003_count < uln2003_total_steps) {
                ceshi_uln2003_step_forward();
                uln2003_count++;
            }

            /* A4988步进 (速度更快，可能需要多步) */
            if (a4988_count < a4988_total_steps) {
                ceshi_a4988_step();
                a4988_count++;
            }

            /* 延时控制速度 */
            HAL_Delay(CESHI_ULN2003_STEP_DELAY_MS);
        }

        /* 释放ULN2003电机，短暂停顿 */
        ceshi_uln2003_release();
        HAL_Delay(500);

        /* ===== 反转阶段 ===== */
        ceshi_a4988_set_dir(0);  // A4988反转方向

        uln2003_count = 0;
        a4988_count = 0;

        /* 同时运行两个电机反转 */
        while (uln2003_count < uln2003_total_steps || a4988_count < a4988_total_steps) {
            /* ULN2003步进 */
            if (uln2003_count < uln2003_total_steps) {
                ceshi_uln2003_step_backward();
                uln2003_count++;
            }

            /* A4988步进 */
            if (a4988_count < a4988_total_steps) {
                ceshi_a4988_step();
                a4988_count++;
            }

            /* 延时控制速度 */
            HAL_Delay(CESHI_ULN2003_STEP_DELAY_MS);
        }

        /* 释放ULN2003电机，短暂停顿 */
        ceshi_uln2003_release();
        HAL_Delay(500);
    }
}

/**
 * @brief  仅测试ULN2003电机
 */
void Ceshi_ULN2003_Test(void)
{
    uint32_t total_steps = CESHI_ULN2003_STEPS_PER_REV * CESHI_ULN2003_REVS;

    while (1) {
        /* 正转2圈 */
        ceshi_uln2003_rotate(total_steps);
        ceshi_uln2003_release();
        HAL_Delay(500);

        /* 反转2圈 */
        ceshi_uln2003_rotate(-total_steps);
        ceshi_uln2003_release();
        HAL_Delay(500);
    }
}

/**
 * @brief  仅测试A4988电机
 * @note   简化版本，用于调试 - 慢速转动便于观察
 */
void Ceshi_A4988_Test(void)
{
    /* 设置方向为正转 */
    HAL_GPIO_WritePin(motor2A9_GPIO_Port, motor2A9_Pin, GPIO_PIN_SET);  // DIR = HIGH

    while (1) {
        /* 产生STEP脉冲 - 每个脉冲电机转一步 */
        HAL_GPIO_WritePin(motor2_GPIO_Port, motor2_Pin, GPIO_PIN_SET);   // STEP高
        HAL_Delay(1);  // 1ms
        HAL_GPIO_WritePin(motor2_GPIO_Port, motor2_Pin, GPIO_PIN_RESET); // STEP低
        HAL_Delay(5);  // 5ms - 慢速转动，便于观察
    }
}

/* ==================== ULN2003驱动实现 ==================== */

/**
 * @brief  设置ULN2003相位输出
 */
static void ceshi_uln2003_set_phase(uint8_t phase)
{
    uint8_t seq = uln2003_phase_table[phase & 0x07];

    /* ULN2003反向驱动: 低电平导通 */
    HAL_GPIO_WritePin(motor1_GPIO_Port, motor1_Pin,    (seq & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET);  // PA2
    HAL_GPIO_WritePin(motor1A3_GPIO_Port, motor1A3_Pin, (seq & 0x02) ? GPIO_PIN_RESET : GPIO_PIN_SET); // PA3
    HAL_GPIO_WritePin(motor1A4_GPIO_Port, motor1A4_Pin, (seq & 0x04) ? GPIO_PIN_RESET : GPIO_PIN_SET); // PA4
    HAL_GPIO_WritePin(motor1A5_GPIO_Port, motor1A5_Pin, (seq & 0x08) ? GPIO_PIN_RESET : GPIO_PIN_SET); // PA5
}

/**
 * @brief  ULN2003正转一步
 */
static void ceshi_uln2003_step_forward(void)
{
    uln2003_phase++;
    if (uln2003_phase >= 8) {
        uln2003_phase = 0;
    }
    ceshi_uln2003_set_phase(uln2003_phase);
}

/**
 * @brief  ULN2003反转一步
 */
static void ceshi_uln2003_step_backward(void)
{
    if (uln2003_phase == 0) {
        uln2003_phase = 7;
    } else {
        uln2003_phase--;
    }
    ceshi_uln2003_set_phase(uln2003_phase);
}

/**
 * @brief  释放ULN2003电机
 */
static void ceshi_uln2003_release(void)
{
    /* 所有引脚输出高电平 (断开) */
    HAL_GPIO_WritePin(motor1_GPIO_Port, motor1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor1A3_GPIO_Port, motor1A3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor1A4_GPIO_Port, motor1A4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor1A5_GPIO_Port, motor1A5_Pin, GPIO_PIN_SET);
}

/**
 * @brief  ULN2003转动指定步数
 * @param  steps: 正数=正转, 负数=反转
 */
static void ceshi_uln2003_rotate(int32_t steps)
{
    uint32_t abs_steps = (steps >= 0) ? steps : -steps;

    for (uint32_t i = 0; i < abs_steps; i++) {
        if (steps >= 0) {
            ceshi_uln2003_step_forward();
        } else {
            ceshi_uln2003_step_backward();
        }
        HAL_Delay(CESHI_ULN2003_STEP_DELAY_MS);
    }
}

/* ==================== A4988驱动实现 ==================== */

/**
 * @brief  初始化A4988
 */
static void ceshi_a4988_init(void)
{
    /* 设置细分模式: MS1=HIGH, MS2=HIGH, MS3=HIGH -> 1/16步进 */
    HAL_GPIO_WritePin(motor2A10_GPIO_Port, motor2A10_Pin, GPIO_PIN_SET);  // MS1 = HIGH (PA10)
    HAL_GPIO_WritePin(motor2A11_GPIO_Port, motor2A11_Pin, GPIO_PIN_SET);  // MS2 = HIGH (PA11)
    HAL_GPIO_WritePin(motor2A12_GPIO_Port, motor2A12_Pin, GPIO_PIN_SET);  // MS3 = HIGH (PA12)

    /* STEP和DIR初始状态 */
    HAL_GPIO_WritePin(motor2_GPIO_Port, motor2_Pin, GPIO_PIN_RESET);      // STEP = LOW (PA8)
    HAL_GPIO_WritePin(motor2A9_GPIO_Port, motor2A9_Pin, GPIO_PIN_RESET);  // DIR = LOW (PA9)
}

/**
 * @brief  设置A4988方向
 * @param  dir: 1=正向, 0=反向
 */
static void ceshi_a4988_set_dir(uint8_t dir)
{
    HAL_GPIO_WritePin(motor2A9_GPIO_Port, motor2A9_Pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  A4988执行一步
 */
static void ceshi_a4988_step(void)
{
    /* 产生上升沿脉冲 */
    HAL_GPIO_WritePin(motor2_GPIO_Port, motor2_Pin, GPIO_PIN_SET);
    ceshi_delay_us(CESHI_A4988_PULSE_US);
    HAL_GPIO_WritePin(motor2_GPIO_Port, motor2_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  A4988转动指定步数
 * @param  steps: 步数
 * @param  dir: 方向 (1=正转, 0=反转)
 */
static void ceshi_a4988_rotate(uint32_t steps, uint8_t dir)
{
    ceshi_a4988_set_dir(dir);

    for (uint32_t i = 0; i < steps; i++) {
        ceshi_a4988_step();
        ceshi_delay_us(CESHI_A4988_STEP_DELAY_US);
    }
}

/* ==================== 延时函数 ==================== */

/**
 * @brief  微秒延时
 */
static void ceshi_delay_us(uint32_t us)
{
    /* 假设72MHz时钟 */
    volatile uint32_t count = us * 5;
    while (count--) {
        __NOP();
    }
}
