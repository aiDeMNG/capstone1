/**
 * @file    motor.c
 * @brief   步进电机驱动模块
 * @note    控制两个步进电机 (均使用42步进电机):
 *          - Motor1 (窗户): ULN2003驱动板, PA2, PA3, PA4, PA5
 *          - Motor2 (窗帘): A4988驱动板, PA8(STEP), PA9(DIR), PA10(MS1), PA11(MS2), PA12(MS3)
 *          EN引脚接地(常使能), 根据 gy_30 模块的标志位控制电机
 *
 *          A4988细分模式配置 (当前使用1/16步进):
 *          MS1=HIGH, MS2=HIGH, MS3=HIGH -> 1/16步进 (3200步/圈)
 */

#include "motor.h"

/* 步进电机相位表 (4相8拍) - 仅ULN2003使用 */
static const uint8_t phase_table[MOTOR_PHASE_COUNT] = {
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

/* ULN2003 驱动函数 */
static void Motor_ULN2003_SetPhase(Motor_HandleTypeDef *hmotor, uint8_t phase);
static void Motor_ULN2003_StepForward(Motor_HandleTypeDef *hmotor);
static void Motor_ULN2003_StepBackward(Motor_HandleTypeDef *hmotor);
static void Motor_ULN2003_Release(Motor_HandleTypeDef *hmotor);

/* A4988 驱动函数 */
static void Motor_A4988_Init(Motor_HandleTypeDef *hmotor);
static void Motor_A4988_SetDir(Motor_HandleTypeDef *hmotor, uint8_t dir);
static void Motor_A4988_Step(Motor_HandleTypeDef *hmotor);

/* 通用处理函数 */
static void Motor_Release(Motor_HandleTypeDef *hmotor);
static void Motor_ProcessSingle(Motor_HandleTypeDef *hmotor);

/* 简单延时函数 */
static void delay_us(uint32_t us);

/* ==================== 公共函数实现 ==================== */

/**
 * @brief  初始化电机控制系统
 */
void Motor_Init(MotorCtrl_HandleTypeDef *hctrl, LightSensor_HandleTypeDef *hlsensor)
{
    if (hctrl == NULL) {
        return;
    }

    hctrl->hlsensor = hlsensor;

    /* 初始化窗户电机 (Motor1: ULN2003, PA2, PA3, PA4, PA5) */
    hctrl->window.state = MOTOR_CLOSED;
    hctrl->window.driver_type = DRIVER_ULN2003;
    hctrl->window.current_phase = 0;
    hctrl->window.step_count = 0;
    hctrl->window.port = GPIOA;
    hctrl->window.pin_a = motor1_Pin;       // PA2 - IN1
    hctrl->window.pin_b = motor1A3_Pin;     // PA3 - IN2
    hctrl->window.pin_c = motor1A4_Pin;     // PA4 - IN3
    hctrl->window.pin_d = motor1A5_Pin;     // PA5 - IN4
    hctrl->window.pin_ms3 = 0;              // 不使用

    /* 初始化窗帘电机 (Motor2: A4988, PA8, PA9, PA10, PA11, PA12) */
    hctrl->curtain.state = MOTOR_CLOSED;
    hctrl->curtain.driver_type = DRIVER_A4988;
    hctrl->curtain.current_phase = 0;
    hctrl->curtain.step_count = 0;
    hctrl->curtain.port = GPIOA;
    hctrl->curtain.pin_a = motor2_Pin;      // PA8  - STEP
    hctrl->curtain.pin_b = motor2A9_Pin;    // PA9  - DIR
    hctrl->curtain.pin_c = motor2A10_Pin;   // PA10 - MS1
    hctrl->curtain.pin_d = motor2A11_Pin;   // PA11 - MS2
    hctrl->curtain.pin_ms3 = motor2A12_Pin; // PA12 - MS3

    /* 释放ULN2003电机 (初始不通电) */
    Motor_Release(&hctrl->window);

    /* 初始化A4988 (设置细分模式) */
    Motor_A4988_Init(&hctrl->curtain);
}

/**
 * @brief  电机控制处理函数
 * @note   根据光照传感器的标志位控制电机
 */
void Motor_Process(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }

    /* 检查光照传感器标志位 */
    if (hctrl->hlsensor != NULL) {
        /* 处理窗户标志 */
        Window_Flag wflag = LightSensor_GetWindowFlag(hctrl->hlsensor);
        if (wflag == WINDOW_FLAG_OPEN && hctrl->window.state != MOTOR_OPENING) {
            Motor_OpenWindow(hctrl);
            LightSensor_ClearWindowFlag(hctrl->hlsensor);
        } else if (wflag == WINDOW_FLAG_CLOSE && hctrl->window.state != MOTOR_CLOSING) {
            Motor_CloseWindow(hctrl);
            LightSensor_ClearWindowFlag(hctrl->hlsensor);
        }

        /* 处理窗帘标志 */
        Curtain_Flag cflag = LightSensor_GetCurtainFlag(hctrl->hlsensor);
        if (cflag == CURTAIN_FLAG_OPEN && hctrl->curtain.state != MOTOR_OPENING) {
            Motor_OpenCurtain(hctrl);
            LightSensor_ClearCurtainFlag(hctrl->hlsensor);
        } else if (cflag == CURTAIN_FLAG_CLOSE && hctrl->curtain.state != MOTOR_CLOSING) {
            Motor_CloseCurtain(hctrl);
            LightSensor_ClearCurtainFlag(hctrl->hlsensor);
        }
    }

    /* 处理电机步进 */
    Motor_ProcessSingle(&hctrl->window);
    Motor_ProcessSingle(&hctrl->curtain);
}

/**
 * @brief  手动打开窗户
 */
void Motor_OpenWindow(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }
    hctrl->window.state = MOTOR_OPENING;
    hctrl->window.step_count = MOTOR_STEPS_ULN2003;
}

/**
 * @brief  手动关闭窗户
 */
void Motor_CloseWindow(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }
    hctrl->window.state = MOTOR_CLOSING;
    hctrl->window.step_count = MOTOR_STEPS_ULN2003;
}

/**
 * @brief  手动打开窗帘
 */
void Motor_OpenCurtain(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }
    /* 设置方向为正向 */
    Motor_A4988_SetDir(&hctrl->curtain, 1);
    hctrl->curtain.state = MOTOR_OPENING;
    hctrl->curtain.step_count = MOTOR_STEPS_A4988;
}

/**
 * @brief  手动关闭窗帘
 */
void Motor_CloseCurtain(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }
    /* 设置方向为反向 */
    Motor_A4988_SetDir(&hctrl->curtain, 0);
    hctrl->curtain.state = MOTOR_CLOSING;
    hctrl->curtain.step_count = MOTOR_STEPS_A4988;
}

/**
 * @brief  停止所有电机
 */
void Motor_StopAll(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }
    hctrl->window.step_count = 0;
    hctrl->window.state = MOTOR_IDLE;
    Motor_Release(&hctrl->window);

    hctrl->curtain.step_count = 0;
    hctrl->curtain.state = MOTOR_IDLE;
}

/**
 * @brief  获取窗户状态
 */
Motor_State Motor_GetWindowState(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return MOTOR_IDLE;
    }
    return hctrl->window.state;
}

/**
 * @brief  获取窗帘状态
 */
Motor_State Motor_GetCurtainState(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return MOTOR_IDLE;
    }
    return hctrl->curtain.state;
}

/* ==================== ULN2003 驱动实现 ==================== */

/**
 * @brief  设置ULN2003电机相位输出
 */
static void Motor_ULN2003_SetPhase(Motor_HandleTypeDef *hmotor, uint8_t phase)
{
    uint8_t seq = phase_table[phase & 0x07];

    /* ULN2003反向驱动: 低电平导通 */
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_a, (seq & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_b, (seq & 0x02) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_c, (seq & 0x04) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_d, (seq & 0x08) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief  ULN2003电机正转一步
 */
static void Motor_ULN2003_StepForward(Motor_HandleTypeDef *hmotor)
{
    hmotor->current_phase++;
    if (hmotor->current_phase >= MOTOR_PHASE_COUNT) {
        hmotor->current_phase = 0;
    }
    Motor_ULN2003_SetPhase(hmotor, hmotor->current_phase);
}

/**
 * @brief  ULN2003电机反转一步
 */
static void Motor_ULN2003_StepBackward(Motor_HandleTypeDef *hmotor)
{
    if (hmotor->current_phase == 0) {
        hmotor->current_phase = MOTOR_PHASE_COUNT - 1;
    } else {
        hmotor->current_phase--;
    }
    Motor_ULN2003_SetPhase(hmotor, hmotor->current_phase);
}

/**
 * @brief  释放ULN2003电机 (断电)
 */
static void Motor_ULN2003_Release(Motor_HandleTypeDef *hmotor)
{
    /* 所有引脚输出高电平 (ULN2003: 高电平=断开) */
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_a, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_b, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_c, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_d, GPIO_PIN_SET);
}

/* ==================== A4988 驱动实现 ==================== */

/**
 * @brief  初始化A4988驱动板
 * @note   设置细分模式为1/16步进
 *         EN引脚硬件接地，始终使能
 *
 *         细分模式表:
 *         MS1  MS2  MS3  ->  细分
 *         L    L    L    ->  全步进
 *         H    L    L    ->  1/2步进
 *         L    H    L    ->  1/4步进
 *         H    H    L    ->  1/8步进
 *         H    H    H    ->  1/16步进 (当前使用)
 */
static void Motor_A4988_Init(Motor_HandleTypeDef *hmotor)
{
    /* 设置细分模式: MS1=HIGH, MS2=HIGH, MS3=HIGH -> 1/16步进 */
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_c, GPIO_PIN_SET);     // MS1 = HIGH
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_d, GPIO_PIN_SET);     // MS2 = HIGH
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_ms3, GPIO_PIN_SET);   // MS3 = HIGH

    /* STEP和DIR初始状态 */
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_a, GPIO_PIN_RESET);   // STEP = LOW
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_b, GPIO_PIN_RESET);   // DIR = LOW (正向)
}

/**
 * @brief  设置A4988电机方向
 * @param  dir: 1=正向(HIGH), 0=反向(LOW)
 */
static void Motor_A4988_SetDir(Motor_HandleTypeDef *hmotor, uint8_t dir)
{
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_b, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  A4988执行一步
 * @note   产生一个STEP脉冲
 */
static void Motor_A4988_Step(Motor_HandleTypeDef *hmotor)
{
    /* 产生上升沿脉冲 */
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_a, GPIO_PIN_SET);
    delay_us(MOTOR_A4988_PULSE_US);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_a, GPIO_PIN_RESET);
}

/* ==================== 通用函数实现 ==================== */

/**
 * @brief  释放电机 (断电/禁用)
 * @note   A4988由于EN接地，无法软件禁用，此函数仅对ULN2003有效
 */
static void Motor_Release(Motor_HandleTypeDef *hmotor)
{
    if (hmotor->driver_type == DRIVER_ULN2003) {
        Motor_ULN2003_Release(hmotor);
    }
    /* A4988: EN接地，始终使能，无法软件禁用 */
}

/**
 * @brief  处理单个电机的步进
 */
static void Motor_ProcessSingle(Motor_HandleTypeDef *hmotor)
{
    if (hmotor->step_count <= 0) {
        return;
    }

    /* 根据驱动类型执行步进 */
    if (hmotor->driver_type == DRIVER_ULN2003) {
        /* ULN2003驱动 */
        if (hmotor->state == MOTOR_OPENING) {
            Motor_ULN2003_StepForward(hmotor);
        } else if (hmotor->state == MOTOR_CLOSING) {
            Motor_ULN2003_StepBackward(hmotor);
        }
        HAL_Delay(MOTOR_STEP_DELAY_MS);
    } else {
        /* A4988驱动 - 方向已在Open/Close函数中设置 */
        Motor_A4988_Step(hmotor);
        delay_us(MOTOR_A4988_DELAY_US);
    }

    hmotor->step_count--;

    /* 检查是否完成 */
    if (hmotor->step_count <= 0) {
        if (hmotor->state == MOTOR_OPENING) {
            hmotor->state = MOTOR_OPEN;
        } else if (hmotor->state == MOTOR_CLOSING) {
            hmotor->state = MOTOR_CLOSED;
        }
        Motor_Release(hmotor);
    }
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
