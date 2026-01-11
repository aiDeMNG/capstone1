/**
 * @file    manual_control.c
 * @brief   手动控制模块（上位机控制）
 * @note    通过HC05蓝牙接收上位机指令，手动控制窗户和窗帘
 *          支持模式切换和三档控制
 */

#include "manual_control.h"
#include <string.h>

/* ==================== 私有函数声明 ==================== */

static void ProcessModeCommand(Manual_Control_HandleTypeDef *hctrl, char level);
static void ProcessWindowCommand(Manual_Control_HandleTypeDef *hctrl, char level);
static void ProcessCurtainCommand(Manual_Control_HandleTypeDef *hctrl, char level);
static void CheckTimeout(Manual_Control_HandleTypeDef *hctrl);
static void ReceiveAndParseCommands(Manual_Control_HandleTypeDef *hctrl);

/* ==================== 公共函数实现 ==================== */

/**
 * @brief  初始化手动控制模块
 */
void ManualControl_Init(Manual_Control_HandleTypeDef *hctrl,
                        Servo_SG90_HandleTypeDef *hservo_window,
                        Motor_A4988_HandleTypeDef *hmotor_curtain,
                        uint8_t timeout_enabled)
{
    if (hctrl == NULL) {
        return;
    }

    hctrl->hservo_window = hservo_window;
    hctrl->hmotor_curtain = hmotor_curtain;
    hctrl->mode = CTRL_MODE_AUTO;
    hctrl->state = MANUAL_CTRL_IDLE;
    hctrl->current_priority = PRIORITY_NONE;
    hctrl->last_command_time = 0;
    hctrl->timeout_enabled = timeout_enabled;
    hctrl->cmd_length = 0;
    memset(hctrl->cmd_buffer, 0, sizeof(hctrl->cmd_buffer));
}

/**
 * @brief  手动控制处理函数
 */
void ManualControl_Process(Manual_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }

    /* 接收并解析HC05指令 */
    ReceiveAndParseCommands(hctrl);

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
    if (hctrl == NULL || command == NULL || strlen(command) < 2) {
        return 0;
    }

    char type = command[0];   // 指令类型 (m/w/c)
    char level = command[1];  // 指令参数 (0/1/2)

    /* 更新最后指令时间（用于超时检测） */
    hctrl->last_command_time = HAL_GetTick();

    switch (type) {
        case 'm':  // 模式切换指令
            ProcessModeCommand(hctrl, level);
            return 1;

        case 'w':  // 窗户控制指令（自动切换到手动模式）
            hctrl->mode = CTRL_MODE_MANUAL;
            hctrl->state = MANUAL_CTRL_ACTIVE;
            hctrl->current_priority = PRIORITY_MANUAL;
            ProcessWindowCommand(hctrl, level);
            return 1;

        case 'c':  // 窗帘控制指令（自动切换到手动模式）
            hctrl->mode = CTRL_MODE_MANUAL;
            hctrl->state = MANUAL_CTRL_ACTIVE;
            hctrl->current_priority = PRIORITY_MANUAL;
            ProcessCurtainCommand(hctrl, level);
            return 1;

        default:
            HC05_SendString("Unknown command\r\n");
            return 0;
    }
}

/**
 * @brief  手动退出手动控制模式
 */
void ManualControl_Exit(Manual_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }

    hctrl->mode = CTRL_MODE_AUTO;
    hctrl->state = MANUAL_CTRL_IDLE;
    hctrl->current_priority = PRIORITY_NONE;
    HC05_SendString("Manual Control Exited - AUTO Mode\r\n");
}

/**
 * @brief  获取控制模式
 */
Control_Mode ManualControl_GetMode(Manual_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return CTRL_MODE_AUTO;
    }
    return hctrl->mode;
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
uint8_t ManualControl_IsManual(Manual_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return 0;
    }
    return (hctrl->mode == CTRL_MODE_MANUAL) ? 1 : 0;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief  处理模式切换指令
 * @param  level: '0'=自动模式, '1'=手动模式
 */
static void ProcessModeCommand(Manual_Control_HandleTypeDef *hctrl, char level)
{
    if (level == '0') {
        /* 切换到自动模式 */
        hctrl->mode = CTRL_MODE_AUTO;
        hctrl->state = MANUAL_CTRL_IDLE;
        hctrl->current_priority = PRIORITY_NONE;
        HC05_SendString("Mode: AUTO\r\n");
    } else if (level == '1') {
        /* 切换到手动模式 */
        hctrl->mode = CTRL_MODE_MANUAL;
        hctrl->state = MANUAL_CTRL_ACTIVE;
        hctrl->current_priority = PRIORITY_MANUAL;
        HC05_SendString("Mode: MANUAL\r\n");
    } else {
        HC05_SendString("Mode: Invalid parameter\r\n");
    }
}

/**
 * @brief  处理窗户控制指令
 * @param  level: '0'=全关(180度), '1'=半开(135度), '2'=全开(90度)
 */
static void ProcessWindowCommand(Manual_Control_HandleTypeDef *hctrl, char level)
{
    if (hctrl == NULL || hctrl->hservo_window == NULL) {
        return;
    }

    switch (level) {
        case '0':  // 窗户0档 - 全关（180度）
            Servo_SG90_CloseWindow(hctrl->hservo_window);
            HC05_SendString("Window: CLOSED\r\n");
            break;

        case '1':  // 窗户1档 - 半开（135度）
            Servo_SG90_HalfWindow(hctrl->hservo_window);
            HC05_SendString("Window: HALF\r\n");
            break;

        case '2':  // 窗户2档 - 全开（90度）
            Servo_SG90_OpenWindow(hctrl->hservo_window);
            HC05_SendString("Window: OPEN\r\n");
            break;

        default:
            HC05_SendString("Window: Invalid level\r\n");
            break;
    }
}

/**
 * @brief  处理窗帘控制指令
 * @param  level: '0'=全关(0%), '1'=半开(50%), '2'=全开(100%)
 */
static void ProcessCurtainCommand(Manual_Control_HandleTypeDef *hctrl, char level)
{
    if (hctrl == NULL || hctrl->hmotor_curtain == NULL) {
        return;
    }

    switch (level) {
        case '0':  // 窗帘0档 - 全关（位置0）
            Motor_A4988_MoveToClose(hctrl->hmotor_curtain);
            HC05_SendString("Curtain: CLOSED\r\n");
            break;

        case '1':  // 窗帘1档 - 半开（位置50%）
            Motor_A4988_MoveToHalf(hctrl->hmotor_curtain);
            HC05_SendString("Curtain: HALF\r\n");
            break;

        case '2':  // 窗帘2档 - 全开（位置100%）
            Motor_A4988_MoveToOpen(hctrl->hmotor_curtain);
            HC05_SendString("Curtain: OPEN\r\n");
            break;

        default:
            HC05_SendString("Curtain: Invalid level\r\n");
            break;
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
        /* 超时，自动退出手动模式，切换到自动模式 */
        hctrl->mode = CTRL_MODE_AUTO;
        hctrl->state = MANUAL_CTRL_IDLE;
        hctrl->current_priority = PRIORITY_NONE;
        HC05_SendString("Manual Control Timeout - AUTO Mode Resumed\r\n");
    }
}

/**
 * @brief  从HC05接收并解析指令
 * @note   按\r\n分割指令，支持连续多条指令
 */
static void ReceiveAndParseCommands(Manual_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL) {
        return;
    }

    while (HC05_Available() > 0) {
        uint8_t byte;
        HC05_Read(&byte, 1);

        /* 检查是否为换行符 */
        if (byte == '\n') {
            /* 去除\r（如果有） */
            if (hctrl->cmd_length > 0 && hctrl->cmd_buffer[hctrl->cmd_length - 1] == '\r') {
                hctrl->cmd_length--;
            }

            /* 添加字符串结束符 */
            hctrl->cmd_buffer[hctrl->cmd_length] = '\0';

            /* 执行指令 */
            if (hctrl->cmd_length > 0) {
                ManualControl_ExecuteCommand(hctrl, (const char*)hctrl->cmd_buffer);
            }

            /* 清空缓冲区 */
            hctrl->cmd_length = 0;
        }
        /* 检查是否为可打印字符 */
        else if (byte >= 32 && byte <= 126) {
            if (hctrl->cmd_length < sizeof(hctrl->cmd_buffer) - 1) {
                hctrl->cmd_buffer[hctrl->cmd_length++] = byte;
            }
        }
        /* 忽略其他字符（如\r, \t等） */
    }
}
