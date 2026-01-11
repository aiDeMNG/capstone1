/**
 * @file    manual_control.h
 * @brief   手动控制模块（上位机控制）
 * @note    通过HC05蓝牙接收上位机指令，手动控制窗户和窗帘
 *          指令格式:
 *          - "m0": 切换到自动模式
 *          - "m1": 切换到手动模式
 *          - "w0": 窗户0档（全关-180度）
 *          - "w1": 窗户1档（半开-135度）
 *          - "w2": 窗户2档（全开-90度）
 *          - "c0": 窗帘0档（全关-位置0）
 *          - "c1": 窗帘1档（半开-位置50%）
 *          - "c2": 窗帘2档（全开-位置100%）
 *          手动模式时优先级最高，抑制所有自动控制
 */

#ifndef __MANUAL_CONTROL_H
#define __MANUAL_CONTROL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdint.h>
#include "control_priority.h"
#include "servo_sg90.h"
#include "motor_a4988.h"
#include "HC05.h"

    /* ==================== 手动控制配置 ==================== */

#define MANUAL_CONTROL_TIMEOUT_MS 10000 // 手动控制超时时间 (10秒)
                                        // 超时后自动退出手动模式，恢复自动控制

    /* ==================== 控制模式 ==================== */

    typedef enum
    {
        CTRL_MODE_AUTO = 0,  // 自动模式
        CTRL_MODE_MANUAL = 1 // 手动模式
    } Control_Mode;

    /* ==================== 手动控制状态 ==================== */

    typedef enum
    {
        MANUAL_CTRL_IDLE = 0, // 空闲（自动控制模式）
        MANUAL_CTRL_ACTIVE    // 手动控制模式激活
    } Manual_Control_State;

    /* ==================== 手动控制句柄 ==================== */

    typedef struct
    {
        /* 执行器 */
        Servo_SG90_HandleTypeDef *hservo_window;   // 窗户舵机句柄
        Motor_A4988_HandleTypeDef *hmotor_curtain; // 窗帘电机句柄

        /* 控制状态 */
        Control_Mode mode;                 // 控制模式（自动/手动）
        Manual_Control_State state;        // 控制状态
        Control_Priority current_priority; // 当前优先级

        /* 超时控制 */
        uint32_t last_command_time; // 最后一次接收指令的时间戳 (ms)
        uint8_t timeout_enabled;    // 是否启用超时自动退出

        /* 指令接收缓冲区 */
        uint8_t cmd_buffer[32]; // 指令缓冲区
        uint16_t cmd_length;    // 当前指令长度

    } Manual_Control_HandleTypeDef;

    /* ==================== 函数声明 ==================== */

    /**
     * @brief  初始化手动控制模块
     * @param  hctrl: 控制句柄
     * @param  hservo_window: 窗户舵机句柄
     * @param  hmotor_curtain: 窗帘电机句柄
     * @param  timeout_enabled: 是否启用超时自动退出 (1=启用, 0=禁用)
     */
    void ManualControl_Init(Manual_Control_HandleTypeDef *hctrl,
                            Servo_SG90_HandleTypeDef *hservo_window,
                            Motor_A4988_HandleTypeDef *hmotor_curtain,
                            uint8_t timeout_enabled);

    /**
     * @brief  手动控制处理函数（主循环调用）
     * @param  hctrl: 控制句柄
     * @note   定期调用此函数检查HC05接收的指令并执行
     *         自动处理超时退出逻辑
     */
    void ManualControl_Process(Manual_Control_HandleTypeDef *hctrl);

    /**
     * @brief  解析并执行手动控制指令
     * @param  hctrl: 控制句柄
     * @param  command: 指令字符串 ("m0", "m1", "w0", "w1", "w2", "c0", "c1", "c2")
     * @return 1=指令有效并执行, 0=指令无效
     */
    uint8_t ManualControl_ExecuteCommand(Manual_Control_HandleTypeDef *hctrl, const char *command);

    /**
     * @brief  手动退出手动控制模式
     * @param  hctrl: 控制句柄
     * @note   恢复到自动控制模式
     */
    void ManualControl_Exit(Manual_Control_HandleTypeDef *hctrl);

    /**
     * @brief  获取控制模式
     * @param  hctrl: 控制句柄
     * @return 控制模式 (CTRL_MODE_AUTO / CTRL_MODE_MANUAL)
     */
    Control_Mode ManualControl_GetMode(Manual_Control_HandleTypeDef *hctrl);

    /**
     * @brief  获取控制状态
     * @param  hctrl: 控制句柄
     * @return 控制状态
     */
    Manual_Control_State ManualControl_GetState(Manual_Control_HandleTypeDef *hctrl);

    /**
     * @brief  获取当前优先级
     * @param  hctrl: 控制句柄
     * @return 当前优先级 (PRIORITY_MANUAL 或 PRIORITY_NONE)
     */
    Control_Priority ManualControl_GetPriority(Manual_Control_HandleTypeDef *hctrl);

    /**
     * @brief  检查是否在手动控制模式
     * @param  hctrl: 控制句柄
     * @return 1=手动模式, 0=自动模式
     */
    uint8_t ManualControl_IsManual(Manual_Control_HandleTypeDef *hctrl);

#ifdef __cplusplus
}
#endif

#endif /* __MANUAL_CONTROL_H */
