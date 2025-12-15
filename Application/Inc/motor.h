/**
 * @file    motor.h
 * @brief   步进电机驱动模块
 * @note    控制两个步进电机:
 *          - Motor1 (窗户): PA2, PA3, PA4, PA5
 *          - Motor2 (窗帘): PA8, PA9, PA10, PA11
 *          根据 gy_30 模块的标志位控制电机
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "gy_30.h"
#include <stdint.h>

/* ==================== 电机参数配置 ==================== */

#define MOTOR_PHASE_COUNT       8       // 相位数 (4相8拍)
#define MOTOR_STEPS_FULL        2048    // 全开/全关所需步数
#define MOTOR_STEP_DELAY_MS     2       // 步进延时 (ms)

/* ==================== 电机状态定义 ==================== */

typedef enum {
    MOTOR_IDLE = 0,         // 空闲
    MOTOR_OPENING,          // 正在打开
    MOTOR_CLOSING,          // 正在关闭
    MOTOR_OPEN,             // 已全开
    MOTOR_CLOSED            // 已全关
} Motor_State;

/* ==================== 单个电机句柄 ==================== */

typedef struct {
    Motor_State state;              // 电机状态
    uint8_t current_phase;          // 当前相位 (0-7)
    int32_t step_count;             // 剩余步数
    GPIO_TypeDef *port;             // GPIO端口
    uint16_t pin_a;                 // A相引脚
    uint16_t pin_b;                 // B相引脚
    uint16_t pin_c;                 // C相引脚
    uint16_t pin_d;                 // D相引脚
} Motor_HandleTypeDef;

/* ==================== 电机控制系统句柄 ==================== */

typedef struct {
    Motor_HandleTypeDef window;             // 窗户电机
    Motor_HandleTypeDef curtain;            // 窗帘电机
    LightSensor_HandleTypeDef *hlsensor;    // 光照传感器句柄
} MotorCtrl_HandleTypeDef;

/* ==================== 函数声明 ==================== */

/**
 * @brief  初始化电机控制系统
 * @param  hctrl: 电机控制句柄
 * @param  hlsensor: 光照传感器句柄
 */
void Motor_Init(MotorCtrl_HandleTypeDef *hctrl, LightSensor_HandleTypeDef *hlsensor);

/**
 * @brief  电机控制处理函数 (主循环调用)
 * @param  hctrl: 电机控制句柄
 * @note   根据光照传感器的标志位控制电机
 */
void Motor_Process(MotorCtrl_HandleTypeDef *hctrl);

/**
 * @brief  手动打开窗户
 * @param  hctrl: 电机控制句柄
 */
void Motor_OpenWindow(MotorCtrl_HandleTypeDef *hctrl);

/**
 * @brief  手动关闭窗户
 * @param  hctrl: 电机控制句柄
 */
void Motor_CloseWindow(MotorCtrl_HandleTypeDef *hctrl);

/**
 * @brief  手动打开窗帘
 * @param  hctrl: 电机控制句柄
 */
void Motor_OpenCurtain(MotorCtrl_HandleTypeDef *hctrl);

/**
 * @brief  手动关闭窗帘
 * @param  hctrl: 电机控制句柄
 */
void Motor_CloseCurtain(MotorCtrl_HandleTypeDef *hctrl);

/**
 * @brief  停止所有电机
 * @param  hctrl: 电机控制句柄
 */
void Motor_StopAll(MotorCtrl_HandleTypeDef *hctrl);

/**
 * @brief  获取窗户状态
 * @param  hctrl: 电机控制句柄
 * @return 窗户状态
 */
Motor_State Motor_GetWindowState(MotorCtrl_HandleTypeDef *hctrl);

/**
 * @brief  获取窗帘状态
 * @param  hctrl: 电机控制句柄
 * @return 窗帘状态
 */
Motor_State Motor_GetCurtainState(MotorCtrl_HandleTypeDef *hctrl);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
