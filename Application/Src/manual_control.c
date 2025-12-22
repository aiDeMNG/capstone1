/**
 * @file    manual_control.c
 * @brief   手动控制模块（上位机控制）
 * @note    通过HC05蓝牙接收上位机指令，手动控制窗户和窗帘
 */

#include "manual_control.h"
#include <string.h>

/* ==================== 私有函数声明 ==================== */

static void ProcessWindowCommand(Manual_Control_HandleTypeDef *hctrl, uint8_t open);
static void ProcessCurtainCommand(Manual_Control_HandleTypeDef *hctrl, uint8_t open);
static void CheckTimeout(Manual_Control_HandleTypeDef *hctrl);

/* ==================== 公共函数实现 ==================== */

/**
 * @brief  初始化手动控制模块
 */
void ManualControl_Init(Manual_Control_HandleTypeDef *hctrl,
                        Motor_ULN2003_HandleTypeDef *hmotor_window,
                        Motor_ULN2003_HandleTypeDef *hmotor_curtain,
                        uint8_t timeout_enabled)
{
    if (hctrl == NULL) {
        return;
    }

    hctrl->hmotor_window = hmotor_window;
    hctrl->hmotor_curtain = hmotor_curtain;
    hctrl->state = MANUAL_CTRL_IDLE;
    hctrl->current_priority = PRIORITY_NONE;
    hctrl->last_command_time = 0;
    hctrl->timeout_enabled = timeout_enabled;
}

/**
 * @brief  手动控制处理函数
 */
void ManualControl_Process(Manual_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }

    /* 检查是否有HC05接收数据 */
    uint16_t available = HC05_Available();
    if (available > 0) {
        uint8_t buffer[32];
        uint16_t len = HC05_Read(buffer, sizeof(buffer) - 1);

        if (len > 0) {
            buffer[len] = '\0';  // 添加字符串结束符

            /* 解析并执行指令 */
            ManualControl_ExecuteCommand(hctrl, (const char*)buffer);
        }
    }

    /* 检查超时 */
    if (hctrl->state == MANUAL_CTRL_ACTIVE && hctrl->timeout_enabled) {
        CheckTimeout(hctrl);
    }
}

/**
 * @brief  解析并执行手动控制指令
 */
uint8_t ManualControl_ExecuteCommand(Manual_Control_HandleTypeDef *hctrl, const char* command)
{
    if (hctrl == NULL || command == NULL) {
        return 0;
    }

    /* 去除前后空白字符 */
    while (*command == ' ' || *command == '\r' || *command == '\n' || *command == '\t') {
        command++;
    }

    /* 解析指令 */
    if (strncmp(command, "w1", 2) == 0) {
        /* 窗户打开 */
        ProcessWindowCommand(hctrl, 1);
        hctrl->state = MANUAL_CTRL_ACTIVE;
        hctrl->current_priority = PRIORITY_MANUAL;
        hctrl->last_command_time = HAL_GetTick();
        HC05_SendString("Window Opening\r\n");
        return 1;
    }
    else if (strncmp(command, "w0", 2) == 0) {
        /* 窗户关闭 */
        ProcessWindowCommand(hctrl, 0);
        hctrl->state = MANUAL_CTRL_ACTIVE;
        hctrl->current_priority = PRIORITY_MANUAL;
        hctrl->last_command_time = HAL_GetTick();
        HC05_SendString("Window Closing\r\n");
        return 1;
    }
    else if (strncmp(command, "c1", 2) == 0) {
        /* 窗帘打开 */
        ProcessCurtainCommand(hctrl, 1);
        hctrl->state = MANUAL_CTRL_ACTIVE;
        hctrl->current_priority = PRIORITY_MANUAL;
        hctrl->last_command_time = HAL_GetTick();
        HC05_SendString("Curtain Opening\r\n");
        return 1;
    }
    else if (strncmp(command, "c0", 2) == 0) {
        /* 窗帘关闭 */
        ProcessCurtainCommand(hctrl, 0);
        hctrl->state = MANUAL_CTRL_ACTIVE;
        hctrl->current_priority = PRIORITY_MANUAL;
        hctrl->last_command_time = HAL_GetTick();
        HC05_SendString("Curtain Closing\r\n");
        return 1;
    }

    /* 未知指令 */
    HC05_SendString("Unknown Command\r\n");
    return 0;
}

/**
 * @brief  手动退出手动控制模式
 */
void ManualControl_Exit(Manual_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }

    hctrl->state = MANUAL_CTRL_IDLE;
    hctrl->current_priority = PRIORITY_NONE;
    HC05_SendString("Manual Control Exited\r\n");
}

/**
 * @brief  获取控制状态
 */
Manual_Control_State ManualControl_GetState(Manual_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return MANUAL_CTRL_IDLE;
    }
    return hctrl->state;
}

/**
 * @brief  获取当前优先级
 */
Control_Priority ManualControl_GetPriority(Manual_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return PRIORITY_NONE;
    }
    return hctrl->current_priority;
}

/**
 * @brief  检查是否在手动控制模式
 */
uint8_t ManualControl_IsActive(Manual_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return 0;
    }
    return (hctrl->state == MANUAL_CTRL_ACTIVE) ? 1 : 0;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief  处理窗户控制指令
 * @param  open: 1=打开, 0=关闭
 */
static void ProcessWindowCommand(Manual_Control_HandleTypeDef *hctrl, uint8_t open)
{
    if (hctrl == NULL || hctrl->hmotor_window == NULL) {
        return;
    }

    Motor_ULN2003_HandleTypeDef *motor = hctrl->hmotor_window;

    /* 根据电机模式选择控制方法 */
    if (motor->mode == MOTOR_MODE_POSITION) {
        /* 位置模式 */
        if (open) {
            Motor_ULN2003_MoveToOpen(motor);   // 移动到位置0（全开）
        } else {
            Motor_ULN2003_MoveToClose(motor);  // 移动到最大位置（全关）
        }
    } else {
        /* 相对模式 */
        if (open) {
            Motor_ULN2003_Open(motor);   // 执行打开动作
        } else {
            Motor_ULN2003_Close(motor);  // 执行关闭动作
        }
    }
}

/**
 * @brief  处理窗帘控制指令
 * @param  open: 1=打开, 0=关闭
 */
static void ProcessCurtainCommand(Manual_Control_HandleTypeDef *hctrl, uint8_t open)
{
    if (hctrl == NULL || hctrl->hmotor_curtain == NULL) {
        return;
    }

    Motor_ULN2003_HandleTypeDef *motor = hctrl->hmotor_curtain;

    /* 根据电机模式选择控制方法 */
    if (motor->mode == MOTOR_MODE_POSITION) {
        /* 位置模式 */
        if (open) {
            Motor_ULN2003_MoveToOpen(motor);   // 移动到位置0（全开）
        } else {
            Motor_ULN2003_MoveToClose(motor);  // 移动到最大位置（全关）
        }
    } else {
        /* 相对模式 */
        if (open) {
            Motor_ULN2003_Open(motor);   // 执行打开动作
        } else {
            Motor_ULN2003_Close(motor);  // 执行关闭动作
        }
    }
}

/**
 * @brief  检查超时并自动退出手动模式
 */
static void CheckTimeout(Manual_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }

    uint32_t current_time = HAL_GetTick();

    /* 检查是否超时 */
    if ((current_time - hctrl->last_command_time) >= MANUAL_CONTROL_TIMEOUT_MS) {
        /* 超时，自动退出手动模式 */
        hctrl->state = MANUAL_CTRL_IDLE;
        hctrl->current_priority = PRIORITY_NONE;
        HC05_SendString("Manual Control Timeout - Auto Mode Resumed\r\n");
    }
}
