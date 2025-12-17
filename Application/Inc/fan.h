/**
 * @file    fan.h
 * @brief   风扇驱动模块
 * @note    控制PA0引脚驱动风扇
 *          支持两种控制模式:
 *          1. 恒速模式 - 低电平恒速转动
 *          2. PWM调速模式 - 通过PWM占空比调节转速
 */

#ifndef __FAN_H
#define __FAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ==================== 风扇控制模式定义 ==================== */

typedef enum {
    FAN_MODE_CONSTANT,      // 恒速模式 (GPIO低电平)
    FAN_MODE_PWM            // PWM调速模式
} Fan_Mode;

/* ==================== 风扇状态定义 ==================== */

typedef enum {
    FAN_STATE_STOPPED = 0,  // 停止
    FAN_STATE_RUNNING       // 运行中
} Fan_State;

/* ==================== 风扇句柄 ==================== */

typedef struct {
    Fan_Mode mode;              // 当前模式
    Fan_State state;            // 运行状态
    GPIO_TypeDef *gpio_port;    // GPIO端口
    uint16_t gpio_pin;          // GPIO引脚
    TIM_HandleTypeDef *htim;    // 定时器句柄 (PWM模式)
    uint32_t tim_channel;       // 定时器通道 (PWM模式)
    uint8_t duty_cycle;         // PWM占空比 (0-100)
} Fan_HandleTypeDef;

/* ==================== 函数声明 ==================== */

/**
 * @brief  初始化风扇
 * @param  hfan: 风扇句柄
 * @param  gpio_port: GPIO端口 (GPIOA)
 * @param  gpio_pin: GPIO引脚 (GPIO_PIN_0)
 * @param  htim: 定时器句柄 (用于PWM模式，可传入NULL如果只使用恒速模式)
 * @param  tim_channel: 定时器通道 (如TIM_CHANNEL_1)
 * @note   PA0可映射到TIM2_CH1用于PWM控制
 */
void Fan_Init(Fan_HandleTypeDef *hfan,
              GPIO_TypeDef *gpio_port,
              uint16_t gpio_pin,
              TIM_HandleTypeDef *htim,
              uint32_t tim_channel);

/**
 * @brief  启动风扇 - 恒速模式
 * @param  hfan: 风扇句柄
 * @note   将PA0设置为低电平，风扇恒速转动
 */
void Fan_StartConstant(Fan_HandleTypeDef *hfan);

/**
 * @brief  启动风扇 - PWM调速模式
 * @param  hfan: 风扇句柄
 * @param  duty_cycle: PWM占空比 (0-100)
 * @note   通过PWM控制风扇转速
 *         duty_cycle = 0: 停止
 *         duty_cycle = 100: 全速
 */
void Fan_StartPWM(Fan_HandleTypeDef *hfan, uint8_t duty_cycle);

/**
 * @brief  设置PWM占空比
 * @param  hfan: 风扇句柄
 * @param  duty_cycle: PWM占空比 (0-100)
 * @note   仅在PWM模式下有效
 */
void Fan_SetDuty(Fan_HandleTypeDef *hfan, uint8_t duty_cycle);

/**
 * @brief  停止风扇
 * @param  hfan: 风扇句柄
 */
void Fan_Stop(Fan_HandleTypeDef *hfan);

/**
 * @brief  获取风扇状态
 * @param  hfan: 风扇句柄
 * @return 风扇状态
 */
Fan_State Fan_GetState(Fan_HandleTypeDef *hfan);

/**
 * @brief  获取风扇模式
 * @param  hfan: 风扇句柄
 * @return 风扇模式
 */
Fan_Mode Fan_GetMode(Fan_HandleTypeDef *hfan);

/**
 * @brief  获取PWM占空比
 * @param  hfan: 风扇句柄
 * @return PWM占空比 (0-100)
 */
uint8_t Fan_GetDuty(Fan_HandleTypeDef *hfan);

#ifdef __cplusplus
}
#endif

#endif /* __FAN_H */
