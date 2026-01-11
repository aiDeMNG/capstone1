/**
 * @file    servo_sg90.h
 * @brief   SG90舵机驱动模块
 * @note    使用PWM控制SG90舵机
 *          PA6 - TIM3_CH1 (PWM输出)
 *          舵机角度控制：
 *          - 全关：180度
 *          - 半开：135度
 *          - 全开：90度
 */

#ifndef __SERVO_SG90_H
#define __SERVO_SG90_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ==================== SG90参数配置 ==================== */

/* 舵机PWM参数 (基于TIM3配置: Prescaler=71, Period=19999, 50Hz PWM) */
#define SERVO_SG90_PWM_FREQ         50      // PWM频率 (Hz)
#define SERVO_SG90_PWM_PERIOD       19999   // PWM周期 (ARR值)

/* 舵机角度对应的CCR值 (脉宽映射)
 * SG90舵机：
 * - 0度   ≈ 0.5ms  → 0.5ms/20ms = 0.025 → CCR = 0.025 * 20000 = 500
 * - 90度  ≈ 1.5ms  → 1.5ms/20ms = 0.075 → CCR = 0.075 * 20000 = 1500
 * - 180度 ≈ 2.5ms  → 2.5ms/20ms = 0.125 → CCR = 0.125 * 20000 = 2500
 *
 * 窗户控制角度定义：
 * - 全关：180度 (CCR = 2500)
 * - 半开：135度 (CCR = 2000)
 * - 全开：90度  (CCR = 1500)
 */
#define SERVO_SG90_ANGLE_0_CCR      500     // 0度对应的CCR值
#define SERVO_SG90_ANGLE_90_CCR     1500    // 90度对应的CCR值
#define SERVO_SG90_ANGLE_135_CCR    2000    // 135度对应的CCR值
#define SERVO_SG90_ANGLE_180_CCR    2500    // 180度对应的CCR值

/* 窗户状态预设角度 */
#define SERVO_WINDOW_CLOSED_ANGLE   180     // 全关角度
#define SERVO_WINDOW_HALF_ANGLE     135     // 半开角度
#define SERVO_WINDOW_OPEN_ANGLE     90      // 全开角度

/* 窗户状态预设CCR值 */
#define SERVO_WINDOW_CLOSED_CCR     SERVO_SG90_ANGLE_180_CCR
#define SERVO_WINDOW_HALF_CCR       SERVO_SG90_ANGLE_135_CCR
#define SERVO_WINDOW_OPEN_CCR       SERVO_SG90_ANGLE_90_CCR

/* ==================== 舵机状态定义 ==================== */

typedef enum {
    SERVO_SG90_IDLE = 0,        // 空闲
    SERVO_SG90_OPENING,         // 正在打开
    SERVO_SG90_CLOSING,         // 正在关闭
    SERVO_SG90_MOVING,          // 正在移动
    SERVO_SG90_OPEN,            // 已全开
    SERVO_SG90_HALF,            // 半开
    SERVO_SG90_CLOSED           // 已全关
} Servo_SG90_State;

/* ==================== SG90舵机句柄 ==================== */

typedef struct {
    /* 硬件配置 */
    TIM_HandleTypeDef *htim;    // 定时器句柄
    uint32_t channel;           // 定时器通道

    /* 状态 */
    Servo_SG90_State state;     // 舵机状态
    uint16_t current_ccr;       // 当前CCR值
    uint16_t target_ccr;        // 目标CCR值
    uint8_t current_angle;      // 当前角度 (0-180)
    uint8_t target_angle;       // 目标角度 (0-180)

    /* 运动控制 */
    uint8_t is_moving;          // 是否正在移动
    uint32_t last_update_time;  // 上次更新时间 (ms)
    uint16_t move_delay_ms;     // 移动延迟 (ms)，用于平滑运动

} Servo_SG90_HandleTypeDef;

/* ==================== 基础函数 ==================== */

/**
 * @brief  初始化SG90舵机
 * @param  hservo: 舵机句柄
 * @param  htim: 定时器句柄
 * @param  channel: 定时器通道
 */
void Servo_SG90_Init(Servo_SG90_HandleTypeDef *hservo,
                     TIM_HandleTypeDef *htim,
                     uint32_t channel);

/**
 * @brief  设置舵机角度 (0-180度)
 * @param  hservo: 舵机句柄
 * @param  angle: 目标角度 (0-180)
 */
void Servo_SG90_SetAngle(Servo_SG90_HandleTypeDef *hservo, uint8_t angle);

/**
 * @brief  设置舵机CCR值 (直接控制脉宽)
 * @param  hservo: 舵机句柄
 * @param  ccr: CCR值 (1000-5000)
 */
void Servo_SG90_SetCCR(Servo_SG90_HandleTypeDef *hservo, uint16_t ccr);

/**
 * @brief  舵机处理函数 (主循环调用)
 * @param  hservo: 舵机句柄
 * @note   用于实现平滑运动（可选）
 */
void Servo_SG90_Process(Servo_SG90_HandleTypeDef *hservo);

/**
 * @brief  停止舵机 (停止PWM输出)
 * @param  hservo: 舵机句柄
 */
void Servo_SG90_Stop(Servo_SG90_HandleTypeDef *hservo);

/**
 * @brief  启动舵机PWM输出
 * @param  hservo: 舵机句柄
 */
void Servo_SG90_Start(Servo_SG90_HandleTypeDef *hservo);

/**
 * @brief  获取舵机状态
 * @param  hservo: 舵机句柄
 * @return 舵机状态
 */
Servo_SG90_State Servo_SG90_GetState(Servo_SG90_HandleTypeDef *hservo);

/**
 * @brief  获取当前角度
 * @param  hservo: 舵机句柄
 * @return 当前角度 (0-180)
 */
uint8_t Servo_SG90_GetAngle(Servo_SG90_HandleTypeDef *hservo);

/* ==================== 窗户控制函数 ==================== */

/**
 * @brief  打开窗户 (设置为全开角度90度)
 * @param  hservo: 舵机句柄
 */
void Servo_SG90_OpenWindow(Servo_SG90_HandleTypeDef *hservo);

/**
 * @brief  半开窗户 (设置为半开角度135度)
 * @param  hservo: 舵机句柄
 */
void Servo_SG90_HalfWindow(Servo_SG90_HandleTypeDef *hservo);

/**
 * @brief  关闭窗户 (设置为全关角度180度)
 * @param  hservo: 舵机句柄
 */
void Servo_SG90_CloseWindow(Servo_SG90_HandleTypeDef *hservo);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_SG90_H */
