/**
 * @file    motor.h
 * @brief   42步进电机驱动模块
 * @note    控制两个42步进电机:
 *          - Motor1: 控制窗户开度
 *          - Motor2: 控制窗帘开度
 *          根据光照传感器数值自动调节
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "bh1750.h"
#include <stdint.h>

/* 电机ID定义 */
typedef enum {
    MOTOR_WINDOW = 0,     // 窗户电机 (Motor1)
    MOTOR_CURTAIN = 1     // 窗帘电机 (Motor2)
} Motor_ID;

/* 电机方向定义 */
typedef enum {
    MOTOR_DIR_CW = 0,     // 顺时针 (正转/打开)
    MOTOR_DIR_CCW = 1     // 逆时针 (反转/关闭)
} Motor_Direction;

/* 电机状态定义 */
typedef enum {
    MOTOR_STATE_IDLE = 0,       // 空闲
    MOTOR_STATE_RUNNING,        // 运行中
    MOTOR_STATE_FULLY_OPEN,     // 完全打开
    MOTOR_STATE_FULLY_CLOSED    // 完全关闭
} Motor_State;

/* 电机控制模式 */
typedef enum {
    MOTOR_MODE_AUTO = 0,        // 自动模式 (根据光照)
    MOTOR_MODE_MANUAL           // 手动模式
} Motor_Mode;

/* 步进电机相位表 (4相8拍) */
#define MOTOR_PHASE_COUNT    8

/* 电机参数配置 */
#define MOTOR_STEPS_PER_REV     200     // 每圈步数 (1.8度步进角)
#define MOTOR_MAX_POSITION      2000    // 最大位置 (对应全开)
#define MOTOR_MIN_POSITION      0       // 最小位置 (对应全关)
#define MOTOR_STEP_DELAY_MS     2       // 步进延时 (ms), 控制速度

/* 光照阈值配置 (单位: lux) */
#define LUX_WINDOW_OPEN_THRESHOLD     500     // 窗户打开阈值
#define LUX_WINDOW_CLOSE_THRESHOLD    2000    // 窗户关闭阈值
#define LUX_CURTAIN_OPEN_THRESHOLD    300     // 窗帘打开阈值
#define LUX_CURTAIN_CLOSE_THRESHOLD   1500    // 窗帘关闭阈值
#define LUX_HYSTERESIS                50      // 滞后量 (防止频繁动作)

/* 开度百分比转换 */
#define POSITION_TO_PERCENT(pos)    ((pos) * 100 / MOTOR_MAX_POSITION)
#define PERCENT_TO_POSITION(pct)    ((pct) * MOTOR_MAX_POSITION / 100)

/* 单个电机句柄结构体 */
typedef struct {
    Motor_ID id;                    // 电机ID
    Motor_State state;              // 当前状态
    Motor_Direction direction;      // 当前方向
    int32_t current_position;       // 当前位置 (步数)
    int32_t target_position;        // 目标位置 (步数)
    uint8_t current_phase;          // 当前相位索引
    uint32_t step_delay;            // 步进延时 (ms)
    GPIO_TypeDef *port_a;           // 相A GPIO端口
    uint16_t pin_a;                 // 相A引脚
    GPIO_TypeDef *port_b;           // 相B GPIO端口
    uint16_t pin_b;                 // 相B引脚
    GPIO_TypeDef *port_c;           // 相C GPIO端口
    uint16_t pin_c;                 // 相C引脚
    GPIO_TypeDef *port_d;           // 相D GPIO端口
    uint16_t pin_d;                 // 相D引脚
} Motor_HandleTypeDef;

/* 电机控制系统句柄结构体 */
typedef struct {
    Motor_HandleTypeDef window_motor;       // 窗户电机
    Motor_HandleTypeDef curtain_motor;      // 窗帘电机
    BH1750_HandleTypeDef *hbh1750;          // 光照传感器句柄
    Motor_Mode mode;                        // 控制模式
    float current_lux;                      // 当前光照值
    float lux_window_open;                  // 窗户打开阈值
    float lux_window_close;                 // 窗户关闭阈值
    float lux_curtain_open;                 // 窗帘打开阈值
    float lux_curtain_close;                // 窗帘关闭阈值
    uint32_t last_update_tick;              // 上次更新时间
} MotorCtrl_HandleTypeDef;

/* ==================== 函数声明 ==================== */

/**
 * @brief  初始化电机控制系统
 * @param  hctrl: 电机控制句柄指针
 * @param  hbh1750: BH1750传感器句柄指针
 * @retval None
 */
void Motor_Init(MotorCtrl_HandleTypeDef *hctrl, BH1750_HandleTypeDef *hbh1750);

/**
 * @brief  电机控制主处理函数 (需在主循环中调用)
 * @param  hctrl: 电机控制句柄指针
 * @retval None
 */
void Motor_Process(MotorCtrl_HandleTypeDef *hctrl);

/**
 * @brief  设置控制模式
 * @param  hctrl: 电机控制句柄指针
 * @param  mode: 控制模式 (MOTOR_MODE_AUTO / MOTOR_MODE_MANUAL)
 * @retval None
 */
void Motor_SetMode(MotorCtrl_HandleTypeDef *hctrl, Motor_Mode mode);

/**
 * @brief  设置窗户开度
 * @param  hctrl: 电机控制句柄指针
 * @param  percent: 开度百分比 (0-100)
 * @retval None
 */
void Motor_SetWindowPosition(MotorCtrl_HandleTypeDef *hctrl, uint8_t percent);

/**
 * @brief  设置窗帘开度
 * @param  hctrl: 电机控制句柄指针
 * @param  percent: 开度百分比 (0-100)
 * @retval None
 */
void Motor_SetCurtainPosition(MotorCtrl_HandleTypeDef *hctrl, uint8_t percent);

/**
 * @brief  获取窗户当前开度
 * @param  hctrl: 电机控制句柄指针
 * @retval 开度百分比 (0-100)
 */
uint8_t Motor_GetWindowPosition(MotorCtrl_HandleTypeDef *hctrl);

/**
 * @brief  获取窗帘当前开度
 * @param  hctrl: 电机控制句柄指针
 * @retval 开度百分比 (0-100)
 */
uint8_t Motor_GetCurtainPosition(MotorCtrl_HandleTypeDef *hctrl);

/**
 * @brief  停止窗户电机
 * @param  hctrl: 电机控制句柄指针
 * @retval None
 */
void Motor_StopWindow(MotorCtrl_HandleTypeDef *hctrl);

/**
 * @brief  停止窗帘电机
 * @param  hctrl: 电机控制句柄指针
 * @retval None
 */
void Motor_StopCurtain(MotorCtrl_HandleTypeDef *hctrl);

/**
 * @brief  停止所有电机
 * @param  hctrl: 电机控制句柄指针
 * @retval None
 */
void Motor_StopAll(MotorCtrl_HandleTypeDef *hctrl);

/**
 * @brief  设置光照阈值
 * @param  hctrl: 电机控制句柄指针
 * @param  window_open: 窗户打开阈值 (lx)
 * @param  window_close: 窗户关闭阈值 (lx)
 * @param  curtain_open: 窗帘打开阈值 (lx)
 * @param  curtain_close: 窗帘关闭阈值 (lx)
 * @retval None
 */
void Motor_SetLuxThreshold(MotorCtrl_HandleTypeDef *hctrl,
                           float window_open, float window_close,
                           float curtain_open, float curtain_close);

/**
 * @brief  获取当前光照值
 * @param  hctrl: 电机控制句柄指针
 * @retval 光照值 (lx)
 */
float Motor_GetCurrentLux(MotorCtrl_HandleTypeDef *hctrl);

/**
 * @brief  根据光照值计算目标开度
 * @param  lux: 当前光照值
 * @param  open_threshold: 打开阈值
 * @param  close_threshold: 关闭阈值
 * @retval 目标开度百分比 (0-100)
 */
uint8_t Motor_CalculateTargetPosition(float lux, float open_threshold, float close_threshold);

/**
 * @brief  电机单步执行
 * @param  hmotor: 电机句柄指针
 * @retval None
 */
void Motor_Step(Motor_HandleTypeDef *hmotor);

/**
 * @brief  释放电机 (断电, 省电)
 * @param  hmotor: 电机句柄指针
 * @retval None
 */
void Motor_Release(Motor_HandleTypeDef *hmotor);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
