/**
 * @file    motor.c
 * @brief   步进电机驱动模块
 * @note    控制两个步进电机:
 *          - Motor1 (窗户): PA2, PA3, PA4, PA5
 *          - Motor2 (窗帘): PA8, PA9, PA10, PA11
 *          根据 gy_30 模块的标志位控制电机
 */

#include "motor.h"

/* 步进电机相位表 (4相8拍) */
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

static void Motor_SetPhase(Motor_HandleTypeDef *hmotor, uint8_t phase);
static void Motor_StepForward(Motor_HandleTypeDef *hmotor);
static void Motor_StepBackward(Motor_HandleTypeDef *hmotor);
static void Motor_Release(Motor_HandleTypeDef *hmotor);
static void Motor_ProcessSingle(Motor_HandleTypeDef *hmotor);

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

    /* 初始化窗户电机 (Motor1: PA2, PA3, PA4, PA5) */
    hctrl->window.state = MOTOR_CLOSED;
    hctrl->window.current_phase = 0;
    hctrl->window.step_count = 0;
    hctrl->window.port = GPIOA;
    hctrl->window.pin_a = motor1_Pin;       // PA2
    hctrl->window.pin_b = motor1A3_Pin;     // PA3
    hctrl->window.pin_c = motor1A4_Pin;     // PA4
    hctrl->window.pin_d = motor1A5_Pin;     // PA5

    /* 初始化窗帘电机 (Motor2: PA8, PA9, PA10, PA11) */
    hctrl->curtain.state = MOTOR_CLOSED;
    hctrl->curtain.current_phase = 0;
    hctrl->curtain.step_count = 0;
    hctrl->curtain.port = GPIOA;
    hctrl->curtain.pin_a = motor2_Pin;      // PA8
    hctrl->curtain.pin_b = motor2A9_Pin;    // PA9
    hctrl->curtain.pin_c = motor2A10_Pin;   // PA10
    hctrl->curtain.pin_d = motor2A11_Pin;   // PA11

    /* 释放电机 (初始不通电) */
    Motor_Release(&hctrl->window);
    Motor_Release(&hctrl->curtain);
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
    hctrl->window.step_count = MOTOR_STEPS_FULL;
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
    hctrl->window.step_count = MOTOR_STEPS_FULL;
}

/**
 * @brief  手动打开窗帘
 */
void Motor_OpenCurtain(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }
    hctrl->curtain.state = MOTOR_OPENING;
    hctrl->curtain.step_count = MOTOR_STEPS_FULL;
}

/**
 * @brief  手动关闭窗帘
 */
void Motor_CloseCurtain(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }
    hctrl->curtain.state = MOTOR_CLOSING;
    hctrl->curtain.step_count = MOTOR_STEPS_FULL;
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
    Motor_Release(&hctrl->curtain);
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

/* ==================== 私有函数实现 ==================== */

/**
 * @brief  设置电机相位输出
 */
static void Motor_SetPhase(Motor_HandleTypeDef *hmotor, uint8_t phase)
{
    uint8_t seq = phase_table[phase & 0x07];

    /* ULN2003反向驱动: 低电平导通 */
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_a, (seq & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_b, (seq & 0x02) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_c, (seq & 0x04) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_d, (seq & 0x08) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief  电机正转一步
 */
static void Motor_StepForward(Motor_HandleTypeDef *hmotor)
{
    hmotor->current_phase++;
    if (hmotor->current_phase >= MOTOR_PHASE_COUNT) {
        hmotor->current_phase = 0;
    }
    Motor_SetPhase(hmotor, hmotor->current_phase);
}

/**
 * @brief  电机反转一步
 */
static void Motor_StepBackward(Motor_HandleTypeDef *hmotor)
{
    if (hmotor->current_phase == 0) {
        hmotor->current_phase = MOTOR_PHASE_COUNT - 1;
    } else {
        hmotor->current_phase--;
    }
    Motor_SetPhase(hmotor, hmotor->current_phase);
}

/**
 * @brief  释放电机 (断电)
 */
static void Motor_Release(Motor_HandleTypeDef *hmotor)
{
    /* 所有引脚输出高电平 (ULN2003: 高电平=断开) */
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_a, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_b, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_c, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port, hmotor->pin_d, GPIO_PIN_SET);
}

/**
 * @brief  处理单个电机的步进
 */
static void Motor_ProcessSingle(Motor_HandleTypeDef *hmotor)
{
    if (hmotor->step_count <= 0) {
        return;
    }

    /* 执行一步 */
    if (hmotor->state == MOTOR_OPENING) {
        Motor_StepForward(hmotor);
    } else if (hmotor->state == MOTOR_CLOSING) {
        Motor_StepBackward(hmotor);
    }

    hmotor->step_count--;
    HAL_Delay(MOTOR_STEP_DELAY_MS);

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
