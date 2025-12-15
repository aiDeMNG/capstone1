/**
 * @file    motor.c
 * @brief   42步进电机驱动实现
 * @note    控制两个42步进电机:
 *          - Motor1 (窗户): PA2, PA3, PA4, PA5
 *          - Motor2 (窗帘): PA8, PA9, PA10, PA11
 *          根据BH1750光照传感器数值自动调节开度
 */

#include "motor.h"
#include <stdlib.h>

/* 步进电机相位表 (4相8拍驱动方式) */
/* 顺序: A, AB, B, BC, C, CD, D, DA */
static const uint8_t phase_table[MOTOR_PHASE_COUNT][4] = {
    {1, 0, 0, 0},   // Phase 0: A
    {1, 1, 0, 0},   // Phase 1: AB
    {0, 1, 0, 0},   // Phase 2: B
    {0, 1, 1, 0},   // Phase 3: BC
    {0, 0, 1, 0},   // Phase 4: C
    {0, 0, 1, 1},   // Phase 5: CD
    {0, 0, 0, 1},   // Phase 6: D
    {1, 0, 0, 1}    // Phase 7: DA
};

/* 私有函数声明 */
static void Motor_InitSingle(Motor_HandleTypeDef *hmotor, Motor_ID id,
                             GPIO_TypeDef *port_a, uint16_t pin_a,
                             GPIO_TypeDef *port_b, uint16_t pin_b,
                             GPIO_TypeDef *port_c, uint16_t pin_c,
                             GPIO_TypeDef *port_d, uint16_t pin_d);
static void Motor_SetPhase(Motor_HandleTypeDef *hmotor, uint8_t phase);
static void Motor_MoveToTarget(Motor_HandleTypeDef *hmotor);
static void Motor_AutoControl(MotorCtrl_HandleTypeDef *hctrl);

/**
 * @brief  初始化电机控制系统
 */
void Motor_Init(MotorCtrl_HandleTypeDef *hctrl, BH1750_HandleTypeDef *hbh1750)
{
    if (hctrl == NULL) {
        return;
    }

    /* 保存光照传感器句柄 */
    hctrl->hbh1750 = hbh1750;

    /* 设置默认模式为自动 */
    hctrl->mode = MOTOR_MODE_AUTO;
    hctrl->current_lux = 0;
    hctrl->last_update_tick = 0;

    /* 设置默认光照阈值 */
    hctrl->lux_window_open = LUX_WINDOW_OPEN_THRESHOLD;
    hctrl->lux_window_close = LUX_WINDOW_CLOSE_THRESHOLD;
    hctrl->lux_curtain_open = LUX_CURTAIN_OPEN_THRESHOLD;
    hctrl->lux_curtain_close = LUX_CURTAIN_CLOSE_THRESHOLD;

    /* 初始化窗户电机 (Motor1): PA2, PA3, PA4, PA5 */
    Motor_InitSingle(&hctrl->window_motor, MOTOR_WINDOW,
                     motor1_GPIO_Port, motor1_Pin,       // PA2 - A相
                     motor1A3_GPIO_Port, motor1A3_Pin,   // PA3 - B相
                     motor1A4_GPIO_Port, motor1A4_Pin,   // PA4 - C相
                     motor1A5_GPIO_Port, motor1A5_Pin);  // PA5 - D相

    /* 初始化窗帘电机 (Motor2): PA8, PA9, PA10, PA11 */
    Motor_InitSingle(&hctrl->curtain_motor, MOTOR_CURTAIN,
                     motor2_GPIO_Port, motor2_Pin,         // PA8 - A相
                     motor2A9_GPIO_Port, motor2A9_Pin,     // PA9 - B相
                     motor2A10_GPIO_Port, motor2A10_Pin,   // PA10 - C相
                     motor2A11_GPIO_Port, motor2A11_Pin);  // PA11 - D相

    /* 释放电机 (初始状态不通电) */
    Motor_Release(&hctrl->window_motor);
    Motor_Release(&hctrl->curtain_motor);
}

/**
 * @brief  初始化单个电机
 */
static void Motor_InitSingle(Motor_HandleTypeDef *hmotor, Motor_ID id,
                             GPIO_TypeDef *port_a, uint16_t pin_a,
                             GPIO_TypeDef *port_b, uint16_t pin_b,
                             GPIO_TypeDef *port_c, uint16_t pin_c,
                             GPIO_TypeDef *port_d, uint16_t pin_d)
{
    hmotor->id = id;
    hmotor->state = MOTOR_STATE_IDLE;
    hmotor->direction = MOTOR_DIR_CW;
    hmotor->current_position = 0;
    hmotor->target_position = 0;
    hmotor->current_phase = 0;
    hmotor->step_delay = MOTOR_STEP_DELAY_MS;

    /* 配置GPIO引脚 */
    hmotor->port_a = port_a;
    hmotor->pin_a = pin_a;
    hmotor->port_b = port_b;
    hmotor->pin_b = pin_b;
    hmotor->port_c = port_c;
    hmotor->pin_c = pin_c;
    hmotor->port_d = port_d;
    hmotor->pin_d = pin_d;
}

/**
 * @brief  设置电机相位输出
 */
static void Motor_SetPhase(Motor_HandleTypeDef *hmotor, uint8_t phase)
{
    if (phase >= MOTOR_PHASE_COUNT) {
        return;
    }

    /* 设置4个引脚状态 */
    HAL_GPIO_WritePin(hmotor->port_a, hmotor->pin_a,
                      phase_table[phase][0] ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port_b, hmotor->pin_b,
                      phase_table[phase][1] ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port_c, hmotor->pin_c,
                      phase_table[phase][2] ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port_d, hmotor->pin_d,
                      phase_table[phase][3] ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief  电机单步执行
 */
void Motor_Step(Motor_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return;
    }

    /* 根据方向更新相位 */
    if (hmotor->direction == MOTOR_DIR_CW) {
        hmotor->current_phase++;
        if (hmotor->current_phase >= MOTOR_PHASE_COUNT) {
            hmotor->current_phase = 0;
        }
        hmotor->current_position++;
    } else {
        if (hmotor->current_phase == 0) {
            hmotor->current_phase = MOTOR_PHASE_COUNT - 1;
        } else {
            hmotor->current_phase--;
        }
        hmotor->current_position--;
    }

    /* 限制位置范围 */
    if (hmotor->current_position > MOTOR_MAX_POSITION) {
        hmotor->current_position = MOTOR_MAX_POSITION;
    }
    if (hmotor->current_position < MOTOR_MIN_POSITION) {
        hmotor->current_position = MOTOR_MIN_POSITION;
    }

    /* 输出相位 */
    Motor_SetPhase(hmotor, hmotor->current_phase);
}

/**
 * @brief  释放电机 (断电)
 */
void Motor_Release(Motor_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return;
    }

    /* 所有引脚输出高电平 (ULN2003反向驱动时为断电状态) */
    HAL_GPIO_WritePin(hmotor->port_a, hmotor->pin_a, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port_b, hmotor->pin_b, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port_c, hmotor->pin_c, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->port_d, hmotor->pin_d, GPIO_PIN_SET);

    hmotor->state = MOTOR_STATE_IDLE;
}

/**
 * @brief  移动电机到目标位置
 */
static void Motor_MoveToTarget(Motor_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) {
        return;
    }

    /* 判断是否需要移动 */
    if (hmotor->current_position == hmotor->target_position) {
        /* 已到达目标位置 */
        if (hmotor->target_position >= MOTOR_MAX_POSITION) {
            hmotor->state = MOTOR_STATE_FULLY_OPEN;
        } else if (hmotor->target_position <= MOTOR_MIN_POSITION) {
            hmotor->state = MOTOR_STATE_FULLY_CLOSED;
        } else {
            hmotor->state = MOTOR_STATE_IDLE;
        }
        Motor_Release(hmotor);
        return;
    }

    /* 设置运行状态和方向 */
    hmotor->state = MOTOR_STATE_RUNNING;

    if (hmotor->target_position > hmotor->current_position) {
        hmotor->direction = MOTOR_DIR_CW;   // 正转打开
    } else {
        hmotor->direction = MOTOR_DIR_CCW;  // 反转关闭
    }

    /* 执行一步 */
    Motor_Step(hmotor);
}

/**
 * @brief  电机控制主处理函数
 */
void Motor_Process(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }

    uint32_t current_tick = HAL_GetTick();

    /* 读取光照传感器 (每500ms更新一次) */
    if (current_tick - hctrl->last_update_tick >= 500) {
        hctrl->last_update_tick = current_tick;

        if (hctrl->hbh1750 != NULL) {
            BH1750_ReadLight(hctrl->hbh1750, &hctrl->current_lux);
        }

        /* 自动模式下根据光照调整 */
        if (hctrl->mode == MOTOR_MODE_AUTO) {
            Motor_AutoControl(hctrl);
        }
    }

    /* 处理窗户电机移动 */
    if (hctrl->window_motor.state == MOTOR_STATE_RUNNING ||
        hctrl->window_motor.current_position != hctrl->window_motor.target_position) {
        Motor_MoveToTarget(&hctrl->window_motor);
        HAL_Delay(hctrl->window_motor.step_delay);
    }

    /* 处理窗帘电机移动 */
    if (hctrl->curtain_motor.state == MOTOR_STATE_RUNNING ||
        hctrl->curtain_motor.current_position != hctrl->curtain_motor.target_position) {
        Motor_MoveToTarget(&hctrl->curtain_motor);
        HAL_Delay(hctrl->curtain_motor.step_delay);
    }
}

/**
 * @brief  自动控制逻辑
 */
static void Motor_AutoControl(MotorCtrl_HandleTypeDef *hctrl)
{
    uint8_t window_target, curtain_target;

    /* 计算窗户目标开度 */
    window_target = Motor_CalculateTargetPosition(hctrl->current_lux,
                                                   hctrl->lux_window_open,
                                                   hctrl->lux_window_close);

    /* 计算窗帘目标开度 */
    curtain_target = Motor_CalculateTargetPosition(hctrl->current_lux,
                                                    hctrl->lux_curtain_open,
                                                    hctrl->lux_curtain_close);

    /* 设置窗户目标位置 */
    int32_t window_pos = PERCENT_TO_POSITION(window_target);
    if (abs(window_pos - hctrl->window_motor.current_position) >
        PERCENT_TO_POSITION(5)) {  // 5%死区
        hctrl->window_motor.target_position = window_pos;
    }

    /* 设置窗帘目标位置 */
    int32_t curtain_pos = PERCENT_TO_POSITION(curtain_target);
    if (abs(curtain_pos - hctrl->curtain_motor.current_position) >
        PERCENT_TO_POSITION(5)) {  // 5%死区
        hctrl->curtain_motor.target_position = curtain_pos;
    }
}

/**
 * @brief  根据光照值计算目标开度
 */
uint8_t Motor_CalculateTargetPosition(float lux, float open_threshold, float close_threshold)
{
    if (lux <= open_threshold) {
        /* 光照低于打开阈值 - 全开 */
        return 100;
    } else if (lux >= close_threshold) {
        /* 光照高于关闭阈值 - 全关 */
        return 0;
    } else {
        /* 线性插值计算开度 */
        /* 光照越强, 开度越小 */
        float range = close_threshold - open_threshold;
        float position = (close_threshold - lux) / range;
        return (uint8_t)(position * 100);
    }
}

/**
 * @brief  设置控制模式
 */
void Motor_SetMode(MotorCtrl_HandleTypeDef *hctrl, Motor_Mode mode)
{
    if (hctrl == NULL) {
        return;
    }
    hctrl->mode = mode;
}

/**
 * @brief  设置窗户开度
 */
void Motor_SetWindowPosition(MotorCtrl_HandleTypeDef *hctrl, uint8_t percent)
{
    if (hctrl == NULL) {
        return;
    }

    if (percent > 100) {
        percent = 100;
    }

    hctrl->window_motor.target_position = PERCENT_TO_POSITION(percent);
}

/**
 * @brief  设置窗帘开度
 */
void Motor_SetCurtainPosition(MotorCtrl_HandleTypeDef *hctrl, uint8_t percent)
{
    if (hctrl == NULL) {
        return;
    }

    if (percent > 100) {
        percent = 100;
    }

    hctrl->curtain_motor.target_position = PERCENT_TO_POSITION(percent);
}

/**
 * @brief  获取窗户当前开度
 */
uint8_t Motor_GetWindowPosition(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return 0;
    }
    return POSITION_TO_PERCENT(hctrl->window_motor.current_position);
}

/**
 * @brief  获取窗帘当前开度
 */
uint8_t Motor_GetCurtainPosition(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return 0;
    }
    return POSITION_TO_PERCENT(hctrl->curtain_motor.current_position);
}

/**
 * @brief  停止窗户电机
 */
void Motor_StopWindow(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }

    hctrl->window_motor.target_position = hctrl->window_motor.current_position;
    Motor_Release(&hctrl->window_motor);
}

/**
 * @brief  停止窗帘电机
 */
void Motor_StopCurtain(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }

    hctrl->curtain_motor.target_position = hctrl->curtain_motor.current_position;
    Motor_Release(&hctrl->curtain_motor);
}

/**
 * @brief  停止所有电机
 */
void Motor_StopAll(MotorCtrl_HandleTypeDef *hctrl)
{
    Motor_StopWindow(hctrl);
    Motor_StopCurtain(hctrl);
}

/**
 * @brief  设置光照阈值
 */
void Motor_SetLuxThreshold(MotorCtrl_HandleTypeDef *hctrl,
                           float window_open, float window_close,
                           float curtain_open, float curtain_close)
{
    if (hctrl == NULL) {
        return;
    }

    hctrl->lux_window_open = window_open;
    hctrl->lux_window_close = window_close;
    hctrl->lux_curtain_open = curtain_open;
    hctrl->lux_curtain_close = curtain_close;
}

/**
 * @brief  获取当前光照值
 */
float Motor_GetCurrentLux(MotorCtrl_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return 0;
    }
    return hctrl->current_lux;
}
