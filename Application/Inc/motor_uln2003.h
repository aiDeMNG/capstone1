/**
 * @file    motor_uln2003.h
 * @brief   ULN2003步进电机驱动模块
 * @note    控制ULN2003驱动板的步进电机 (28BYJ-48)
 *          使用4相8拍驱动方式
 *          支持两种控制模式：
 *          1. 相对模式（RELATIVE）：转动固定步数
 *          2. 位置模式（POSITION）：移动到绝对位置（带位置跟踪）
 */

#ifndef __MOTOR_ULN2003_H
#define __MOTOR_ULN2003_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ==================== ULN2003参数配置 ==================== */

#define MOTOR_ULN2003_PHASE_COUNT    8       // 相位数 (4相8拍)
#define MOTOR_ULN2003_STEP_DELAY_MS  2       // 步进延时 (ms)

/* 预设步数（用于相对模式） */
#define MOTOR_STEPS_FULL             2048    // 完整一圈
#define MOTOR_STEPS_HALF             1024    // 半圈
#define MOTOR_STEPS_QUARTER          512     // 1/4圈
#define MOTOR_STEPS_EIGHTH           256     // 1/8圈

/* ==================== 控制模式定义 ==================== */

typedef enum {
    MOTOR_MODE_RELATIVE = 0,    // 相对模式：转动固定步数
    MOTOR_MODE_POSITION         // 位置模式：移动到绝对位置
} Motor_ULN2003_Mode;

/* ==================== 电机状态定义 ==================== */

typedef enum {
    MOTOR_ULN2003_IDLE = 0,         // 空闲
    MOTOR_ULN2003_OPENING,          // 正在打开
    MOTOR_ULN2003_CLOSING,          // 正在关闭
    MOTOR_ULN2003_MOVING,           // 正在移动（位置模式）
    MOTOR_ULN2003_OPEN,             // 已全开
    MOTOR_ULN2003_CLOSED,           // 已全关
    MOTOR_ULN2003_AT_POSITION       // 在目标位置（位置模式）
} Motor_ULN2003_State;

/* ==================== ULN2003电机句柄 ==================== */

typedef struct {
    /* 基础配置 */
    GPIO_TypeDef *port;             // GPIO端口
    uint16_t pin_in1;               // IN1引脚
    uint16_t pin_in2;               // IN2引脚
    uint16_t pin_in3;               // IN3引脚
    uint16_t pin_in4;               // IN4引脚

    /* 控制模式 */
    Motor_ULN2003_Mode mode;        // 控制模式（相对/位置）
    Motor_ULN2003_State state;      // 电机状态

    /* 相位控制 */
    uint8_t current_phase;          // 当前相位 (0-7)

    /* 相对模式参数 */
    int32_t step_count;             // 剩余步数（相对模式）
    int32_t relative_steps;         // 单次移动步数（相对模式）

    /* 位置模式参数 */
    int32_t current_position;       // 当前位置 (0-max_position)
    int32_t target_position;        // 目标位置
    int32_t max_position;           // 最大位置（一圈 = 2048）
    uint8_t is_moving;              // 是否正在移动（位置模式）

} Motor_ULN2003_HandleTypeDef;

/* ==================== 基础函数 ==================== */

/**
 * @brief  初始化ULN2003电机
 * @param  hmotor: 电机句柄
 * @param  port: GPIO端口
 * @param  pin_in1: IN1引脚
 * @param  pin_in2: IN2引脚
 * @param  pin_in3: IN3引脚
 * @param  pin_in4: IN4引脚
 */
void Motor_ULN2003_Init(Motor_ULN2003_HandleTypeDef *hmotor,
                        GPIO_TypeDef *port,
                        uint16_t pin_in1,
                        uint16_t pin_in2,
                        uint16_t pin_in3,
                        uint16_t pin_in4);

/**
 * @brief  设置电机控制模式
 * @param  hmotor: 电机句柄
 * @param  mode: 控制模式（MOTOR_MODE_RELATIVE / MOTOR_MODE_POSITION）
 */
void Motor_ULN2003_SetMode(Motor_ULN2003_HandleTypeDef *hmotor, Motor_ULN2003_Mode mode);

/**
 * @brief  设置最大位置（仅位置模式有效）
 * @param  hmotor: 电机句柄
 * @param  max_pos: 最大位置（如：2048 = 一圈）
 */
void Motor_ULN2003_SetMaxPosition(Motor_ULN2003_HandleTypeDef *hmotor, int32_t max_pos);

/**
 * @brief  设置相对模式的单次移动步数
 * @param  hmotor: 电机句柄
 * @param  steps: 步数（如：512 = 1/4圈）
 */
void Motor_ULN2003_SetRelativeSteps(Motor_ULN2003_HandleTypeDef *hmotor, int32_t steps);

/**
 * @brief  电机处理函数 (主循环调用)
 * @param  hmotor: 电机句柄
 * @note   自动根据控制模式执行相应逻辑
 */
void Motor_ULN2003_Process(Motor_ULN2003_HandleTypeDef *hmotor);

/**
 * @brief  停止电机
 * @param  hmotor: 电机句柄
 */
void Motor_ULN2003_Stop(Motor_ULN2003_HandleTypeDef *hmotor);

/**
 * @brief  获取电机状态
 * @param  hmotor: 电机句柄
 * @return 电机状态
 */
Motor_ULN2003_State Motor_ULN2003_GetState(Motor_ULN2003_HandleTypeDef *hmotor);

/* ==================== 相对模式函数 ==================== */

/**
 * @brief  启动电机打开动作（相对模式）
 * @param  hmotor: 电机句柄
 * @note   使用 relative_steps 作为移动步数
 */
void Motor_ULN2003_Open(Motor_ULN2003_HandleTypeDef *hmotor);

/**
 * @brief  启动电机关闭动作（相对模式）
 * @param  hmotor: 电机句柄
 * @note   使用 relative_steps 作为移动步数
 */
void Motor_ULN2003_Close(Motor_ULN2003_HandleTypeDef *hmotor);

/**
 * @brief  启动电机半开动作（相对模式）
 * @param  hmotor: 电机句柄
 * @note   使用 relative_steps / 2 作为移动步数
 */
void Motor_ULN2003_Half(Motor_ULN2003_HandleTypeDef *hmotor);

/* ==================== 位置模式函数 ==================== */

/**
 * @brief  移动到指定位置（位置模式）
 * @param  hmotor: 电机句柄
 * @param  position: 目标位置 (0 - max_position)
 * @note   仅在位置模式下有效
 */
void Motor_ULN2003_MoveToPosition(Motor_ULN2003_HandleTypeDef *hmotor, int32_t position);

/**
 * @brief  移动到全开位置（位置模式）
 * @param  hmotor: 电机句柄
 * @note   等同于 MoveToPosition(hmotor, 0)
 */
void Motor_ULN2003_MoveToOpen(Motor_ULN2003_HandleTypeDef *hmotor);

/**
 * @brief  移动到半开位置（位置模式）
 * @param  hmotor: 电机句柄
 * @note   等同于 MoveToPosition(hmotor, max_position / 2)
 */
void Motor_ULN2003_MoveToHalf(Motor_ULN2003_HandleTypeDef *hmotor);

/**
 * @brief  移动到全关位置（位置模式）
 * @param  hmotor: 电机句柄
 * @note   等同于 MoveToPosition(hmotor, max_position)
 */
void Motor_ULN2003_MoveToClose(Motor_ULN2003_HandleTypeDef *hmotor);

/**
 * @brief  获取当前位置（位置模式）
 * @param  hmotor: 电机句柄
 * @return 当前位置
 */
int32_t Motor_ULN2003_GetPosition(Motor_ULN2003_HandleTypeDef *hmotor);

/**
 * @brief  复位位置为0（位置模式）
 * @param  hmotor: 电机句柄
 * @note   将当前位置设置为0，用于系统启动或校准
 */
void Motor_ULN2003_ResetPosition(Motor_ULN2003_HandleTypeDef *hmotor);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_ULN2003_H */
