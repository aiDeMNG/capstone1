/**
 * @file    motor_a4988.h
 * @brief   A4988步进电机驱动模块
 * @note    控制A4988驱动板的步进电机
 *          使用STEP/DIR控制方式
 *          支持细分模式配置 (当前使用1/16步进)
 */

#ifndef __MOTOR_A4988_H
#define __MOTOR_A4988_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdint.h>

    /* ==================== A4988参数配置 ==================== */

#define MOTOR_A4988_STEPS 3200   // 全开/全关所需步数 (1/16细分, 200*16=3200步/圈)
#define MOTOR_A4988_DELAY_US 500 // 步进延时 (us)
#define MOTOR_A4988_PULSE_US 10  // 脉冲宽度 (us)

/* 预设步数（用于相对模式） */
#define MOTOR_A4988_STEPS_FULL 3200   // 完整一圈 (1/16细分)
#define MOTOR_A4988_STEPS_HALF 1600   // 半圈
#define MOTOR_A4988_STEPS_QUARTER 800 // 1/4圈
#define MOTOR_A4988_STEPS_EIGHTH 400  // 1/8圈

    /* ==================== 控制模式定义 ==================== */

    typedef enum
    {
        MOTOR_A4988_MODE_RELATIVE = 0, // 相对模式：转动固定步数
        MOTOR_A4988_MODE_POSITION      // 位置模式：移动到绝对位置
    } Motor_A4988_Mode;

    /* ==================== 电机状态定义 ==================== */

    typedef enum
    {
        MOTOR_A4988_IDLE = 0,   // 空闲
        MOTOR_A4988_OPENING,    // 正在打开
        MOTOR_A4988_CLOSING,    // 正在关闭
        MOTOR_A4988_MOVING,     // 正在移动（位置模式）
        MOTOR_A4988_OPEN,       // 已全开
        MOTOR_A4988_CLOSED,     // 已全关
        MOTOR_A4988_AT_POSITION // 在目标位置（位置模式）
    } Motor_A4988_State;

    /* ==================== A4988电机句柄 ==================== */

    typedef struct
    {
        /* 基础配置 */
        GPIO_TypeDef *port; // GPIO端口
        uint16_t pin_step;  // STEP引脚
        uint16_t pin_dir;   // DIR引脚
        uint16_t pin_ms1;   // MS1引脚 (细分配置)
        uint16_t pin_ms2;   // MS2引脚 (细分配置)
        uint16_t pin_ms3;   // MS3引脚 (细分配置)

        /* 控制模式 */
        Motor_A4988_Mode mode;   // 控制模式（相对/位置）
        Motor_A4988_State state; // 电机状态

        /* 相对模式参数 */
        int32_t step_count;     // 剩余步数（相对模式）
        int32_t relative_steps; // 单次移动步数（相对模式）

        /* 位置模式参数 */
        int32_t current_position; // 当前位置 (0-max_position)
        int32_t target_position;  // 目标位置
        int32_t max_position;     // 最大位置（一圈 = 3200）
        uint8_t is_moving;        // 是否正在移动（位置模式）

    } Motor_A4988_HandleTypeDef;

    /* ==================== 函数声明 ==================== */

    /* ==================== 基础函数 ==================== */

    /**
     * @brief  初始化A4988电机
     * @param  hmotor: 电机句柄
     * @param  port: GPIO端口
     * @param  pin_step: STEP引脚
     * @param  pin_dir: DIR引脚
     * @param  pin_ms1: MS1引脚
     * @param  pin_ms2: MS2引脚
     * @param  pin_ms3: MS3引脚
     * @note   EN引脚硬件接地，始终使能
     *         自动配置为1/16步进模式 (MS1=H, MS2=H, MS3=H)
     */
    void Motor_A4988_Init(Motor_A4988_HandleTypeDef *hmotor,
                          GPIO_TypeDef *port,
                          uint16_t pin_step,
                          uint16_t pin_dir,
                          uint16_t pin_ms1,
                          uint16_t pin_ms2,
                          uint16_t pin_ms3);

    /**
     * @brief  设置电机控制模式
     * @param  hmotor: 电机句柄
     * @param  mode: 控制模式（MOTOR_A4988_MODE_RELATIVE / MOTOR_A4988_MODE_POSITION）
     */
    void Motor_A4988_SetMode(Motor_A4988_HandleTypeDef *hmotor, Motor_A4988_Mode mode);

    /**
     * @brief  设置最大位置（仅位置模式有效）
     * @param  hmotor: 电机句柄
     * @param  max_pos: 最大位置（如：3200 = 一圈）
     */
    void Motor_A4988_SetMaxPosition(Motor_A4988_HandleTypeDef *hmotor, int32_t max_pos);

    /**
     * @brief  设置相对模式的单次移动步数
     * @param  hmotor: 电机句柄
     * @param  steps: 步数（如：800 = 1/4圈）
     */
    void Motor_A4988_SetRelativeSteps(Motor_A4988_HandleTypeDef *hmotor, int32_t steps);

    /**
     * @brief  电机处理函数 (主循环调用)
     * @param  hmotor: 电机句柄
     * @note   自动根据控制模式执行相应逻辑
     */
    void Motor_A4988_Process(Motor_A4988_HandleTypeDef *hmotor);

    /**
     * @brief  停止电机
     * @param  hmotor: 电机句柄
     */
    void Motor_A4988_Stop(Motor_A4988_HandleTypeDef *hmotor);

    /**
     * @brief  获取电机状态
     * @param  hmotor: 电机句柄
     * @return 电机状态
     */
    Motor_A4988_State Motor_A4988_GetState(Motor_A4988_HandleTypeDef *hmotor);

    /* ==================== 相对模式函数 ==================== */

    /**
     * @brief  启动电机打开动作（相对模式）
     * @param  hmotor: 电机句柄
     * @note   使用 relative_steps 作为移动步数
     */
    void Motor_A4988_Open(Motor_A4988_HandleTypeDef *hmotor);

    /**
     * @brief  启动电机关闭动作（相对模式）
     * @param  hmotor: 电机句柄
     * @note   使用 relative_steps 作为移动步数
     */
    void Motor_A4988_Close(Motor_A4988_HandleTypeDef *hmotor);

    /**
     * @brief  启动电机半开动作（相对模式）
     * @param  hmotor: 电机句柄
     * @note   使用 relative_steps / 2 作为移动步数
     */
    void Motor_A4988_Half(Motor_A4988_HandleTypeDef *hmotor);

    /* ==================== 位置模式函数 ==================== */

    /**
     * @brief  移动到指定位置（位置模式）
     * @param  hmotor: 电机句柄
     * @param  position: 目标位置 (0 - max_position)
     * @note   仅在位置模式下有效
     */
    void Motor_A4988_MoveToPosition(Motor_A4988_HandleTypeDef *hmotor, int32_t position);

    /**
     * @brief  移动到全开位置（位置模式）
     * @param  hmotor: 电机句柄
     * @note   等同于 MoveToPosition(hmotor, 0)
     */
    void Motor_A4988_MoveToOpen(Motor_A4988_HandleTypeDef *hmotor);

    /**
     * @brief  移动到半开位置（位置模式）
     * @param  hmotor: 电机句柄
     * @note   等同于 MoveToPosition(hmotor, max_position / 2)
     */
    void Motor_A4988_MoveToHalf(Motor_A4988_HandleTypeDef *hmotor);

    /**
     * @brief  移动到全关位置（位置模式）
     * @param  hmotor: 电机句柄
     * @note   等同于 MoveToPosition(hmotor, max_position)
     */
    void Motor_A4988_MoveToClose(Motor_A4988_HandleTypeDef *hmotor);

    /**
     * @brief  获取当前位置（位置模式）
     * @param  hmotor: 电机句柄
     * @return 当前位置
     */
    int32_t Motor_A4988_GetPosition(Motor_A4988_HandleTypeDef *hmotor);

    /**
     * @brief  复位位置为0（位置模式）
     * @param  hmotor: 电机句柄
     * @note   将当前位置设置为0，用于系统启动或校准
     */
    void Motor_A4988_ResetPosition(Motor_A4988_HandleTypeDef *hmotor);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_A4988_H */
