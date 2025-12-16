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
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ==================== A4988参数配置 ==================== */

#define MOTOR_A4988_STEPS           3200    // 全开/全关所需步数 (1/16细分, 200*16=3200步/圈)
#define MOTOR_A4988_DELAY_US        500     // 步进延时 (us)
#define MOTOR_A4988_PULSE_US        10      // 脉冲宽度 (us)

/* ==================== 电机状态定义 ==================== */

typedef enum {
    MOTOR_A4988_IDLE = 0,         // 空闲
    MOTOR_A4988_OPENING,          // 正在打开
    MOTOR_A4988_CLOSING,          // 正在关闭
    MOTOR_A4988_OPEN,             // 已全开
    MOTOR_A4988_CLOSED            // 已全关
} Motor_A4988_State;

/* ==================== A4988电机句柄 ==================== */

typedef struct {
    Motor_A4988_State state;        // 电机状态
    int32_t step_count;             // 剩余步数
    GPIO_TypeDef *port;             // GPIO端口
    uint16_t pin_step;              // STEP引脚
    uint16_t pin_dir;               // DIR引脚
    uint16_t pin_ms1;               // MS1引脚 (细分配置)
    uint16_t pin_ms2;               // MS2引脚 (细分配置)
    uint16_t pin_ms3;               // MS3引脚 (细分配置)
} Motor_A4988_HandleTypeDef;

/* ==================== 函数声明 ==================== */

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
 * @brief  启动电机打开动作
 * @param  hmotor: 电机句柄
 */
void Motor_A4988_Open(Motor_A4988_HandleTypeDef *hmotor);

/**
 * @brief  启动电机关闭动作
 * @param  hmotor: 电机句柄
 */
void Motor_A4988_Close(Motor_A4988_HandleTypeDef *hmotor);

/**
 * @brief  停止电机
 * @param  hmotor: 电机句柄
 */
void Motor_A4988_Stop(Motor_A4988_HandleTypeDef *hmotor);

/**
 * @brief  电机处理函数 (主循环调用)
 * @param  hmotor: 电机句柄
 */
void Motor_A4988_Process(Motor_A4988_HandleTypeDef *hmotor);

/**
 * @brief  获取电机状态
 * @param  hmotor: 电机句柄
 * @return 电机状态
 */
Motor_A4988_State Motor_A4988_GetState(Motor_A4988_HandleTypeDef *hmotor);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_A4988_H */
