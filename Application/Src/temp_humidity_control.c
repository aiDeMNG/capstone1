/**
 * @file    temp_humidity_control.c
 * @brief   温湿度监测与自动控制模块
 * @note    监测DHT22温湿度传感器，根据温湿度自动调节窗户开度
 *          优先级: 温湿度控制 > 光照控制
 */

#include "temp_humidity_control.h"
#include <math.h>

/* ==================== 私有函数声明 ==================== */

static uint8_t CalculateWindowOpening(Temp_Humidity_Control_HandleTypeDef *hctrl);
static void AdjustWindow(Temp_Humidity_Control_HandleTypeDef *hctrl, uint8_t target_opening);
static void ReadSensors(Temp_Humidity_Control_HandleTypeDef *hctrl);

/* ==================== 公共函数实现 ==================== */

/**
 * @brief  初始化温湿度控制模块
 */
void TempHumidityControl_Init(Temp_Humidity_Control_HandleTypeDef *hctrl,
                              GPIO_TypeDef *dht_port,
                              uint16_t dht_pin,
                              Motor_ULN2003_HandleTypeDef *hmotor_window)
{
    if (hctrl == NULL)
    {
        return;
    }

    hctrl->dht_port = dht_port;
    hctrl->dht_pin = dht_pin;
    hctrl->hmotor_window = hmotor_window;
    hctrl->state = TEMP_HUM_CTRL_IDLE;
    hctrl->current_priority = PRIORITY_NONE;
    hctrl->temperature = 0.0f;
    hctrl->humidity = 0.0f;
    hctrl->temp_requires_open = 0;
    hctrl->humidity_requires_open = 0;
    hctrl->target_window_opening = 0;
    hctrl->window_controlled_by_temp_hum = 0;
}

/**
 * @brief  温湿度控制处理函数
 */
void TempHumidityControl_Process(Temp_Humidity_Control_HandleTypeDef *hctrl,
                                 Control_Priority external_priority)
{
    if (hctrl == NULL)
    {
        return;
    }

    /* 检查是否被更高优先级抑制（手动控制或空气质量控制） */
    if (external_priority >= PRIORITY_TEMP_HUMIDITY)
    {
        /* 如果正在调节，停止由温湿度控制启动的设备 */
        if (hctrl->state == TEMP_HUM_CTRL_REGULATING)
        {
            TempHumidityControl_Stop(hctrl);
        }
        hctrl->state = TEMP_HUM_CTRL_IDLE;
        hctrl->current_priority = PRIORITY_NONE;
        return;
    }

    /* 读取传感器数据 */
    hctrl->state = TEMP_HUM_CTRL_DETECTING;
    ReadSensors(hctrl);

    /* 计算所需窗户开度 */
    uint8_t required_opening = CalculateWindowOpening(hctrl);

    /* 判断是否需要控制 */
    if (required_opening > 0 || hctrl->temp_requires_open || hctrl->humidity_requires_open)
    {
        /* 需要控制，设置优先级 */
        hctrl->current_priority = PRIORITY_TEMP_HUMIDITY;
        hctrl->state = TEMP_HUM_CTRL_REGULATING;
        hctrl->target_window_opening = required_opening;

        /* 调节窗户 */
        AdjustWindow(hctrl, required_opening);
    }
    else
    {
        /* 温湿度适宜，不需要控制 */
        if (hctrl->state == TEMP_HUM_CTRL_REGULATING)
        {
            /* 如果之前在控制，停止控制 */
            TempHumidityControl_Stop(hctrl);
        }
        hctrl->state = TEMP_HUM_CTRL_IDLE;
        hctrl->current_priority = PRIORITY_NONE;
        hctrl->target_window_opening = 0;
    }
}

/**
 * @brief  手动停止温湿度控制
 */
void TempHumidityControl_Stop(Temp_Humidity_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL)
    {
        return;
    }

    hctrl->window_controlled_by_temp_hum = 0;
    hctrl->state = TEMP_HUM_CTRL_IDLE;
    hctrl->current_priority = PRIORITY_NONE;
}

/**
 * @brief  获取控制状态
 */
Temp_Humidity_Control_State TempHumidityControl_GetState(Temp_Humidity_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL)
    {
        return TEMP_HUM_CTRL_IDLE;
    }
    return hctrl->state;
}

/**
 * @brief  获取当前优先级
 */
Control_Priority TempHumidityControl_GetPriority(Temp_Humidity_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL)
    {
        return PRIORITY_NONE;
    }
    return hctrl->current_priority;
}

/**
 * @brief  获取当前温度
 */
float TempHumidityControl_GetTemperature(Temp_Humidity_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL)
    {
        return 0.0f;
    }
    return hctrl->temperature;
}

/**
 * @brief  获取当前湿度
 */
float TempHumidityControl_GetHumidity(Temp_Humidity_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL)
    {
        return 0.0f;
    }
    return hctrl->humidity;
}

/**
 * @brief  获取目标窗户开度
 */
uint8_t TempHumidityControl_GetTargetOpening(Temp_Humidity_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL)
    {
        return 0;
    }
    return hctrl->target_window_opening;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief  读取传感器数据
 */
static void ReadSensors(Temp_Humidity_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL)
    {
        return;
    }

    /* 读取DHT22传感器 */
    if (DHT22_update(hctrl->dht_port, hctrl->dht_pin) == DHTLIB_OK)
    {
        hctrl->temperature = (float)get_temperature();
        hctrl->humidity = (float)get_humidity();
    }
}

/**
 * @brief  计算所需窗户开度
 * @return 窗户开度 (0-100%)
 * @note   综合考虑温度和湿度需求
 *         决策优先级: 温度过低(保温) > 湿度控制 > 温度过高(散热)
 */
static uint8_t CalculateWindowOpening(Temp_Humidity_Control_HandleTypeDef *hctrl)
{
    if (hctrl == NULL)
    {
        return 0;
    }

    uint8_t temp_opening = 0;     // 温度需求的开度
    uint8_t humidity_opening = 0; // 湿度需求的开度

    /* ========== 温度控制逻辑 ========== */

    /* 温度过低 → 关窗保温 */
    if (hctrl->temperature < (TEMP_LOW_THRESHOLD - TEMP_HYSTERESIS))
    {
        temp_opening = WINDOW_OPEN_NONE; // 0%
        hctrl->temp_requires_open = 0;
    }
    /* 温度过高 → 开窗散热 */
    else if (hctrl->temperature > (TEMP_HIGH_THRESHOLD + TEMP_HYSTERESIS))
    {
        temp_opening = WINDOW_OPEN_FULL; // 100%
        hctrl->temp_requires_open = 1;
    }
    /* 温度适宜 → 不控制 */
    else
    {
        temp_opening = 0;
        hctrl->temp_requires_open = 0;
    }

    /* ========== 湿度控制逻辑 ========== */

    /* 湿度过高 → 开窗除湿 */
    if (hctrl->humidity > (HUMIDITY_HIGH_THRESHOLD + HUMIDITY_HYSTERESIS))
    {
        humidity_opening = WINDOW_OPEN_HALF; // 50%
        hctrl->humidity_requires_open = 1;
    }
    /* 湿度过低 → 关窗保湿 */
    else if (hctrl->humidity < (HUMIDITY_LOW_THRESHOLD - HUMIDITY_HYSTERESIS))
    {
        humidity_opening = WINDOW_OPEN_NONE; // 0%
        hctrl->humidity_requires_open = 0;
    }
    /* 湿度适宜 → 不控制 */
    else
    {
        humidity_opening = 0;
        hctrl->humidity_requires_open = 0;
    }

    /* ========== 综合决策：优先级链 ========== */
    uint8_t final_opening = 0;

    /* 优先级1: 温度过低 → 强制关窗保温（保温优先） */
    if (hctrl->temperature < (TEMP_LOW_THRESHOLD - TEMP_HYSTERESIS))
    {
        final_opening = WINDOW_OPEN_NONE; // 0%
    }
    /* 优先级2: 湿度需要控制 → 使用湿度需求的开度（湿度精细控制优先） */
    else if (hctrl->humidity_requires_open ||
             hctrl->humidity < (HUMIDITY_LOW_THRESHOLD - HUMIDITY_HYSTERESIS))
    {
        final_opening = humidity_opening;
    }
    /* 优先级3: 温度过高 → 使用温度需求的开度（温度散热） */
    else if (hctrl->temp_requires_open)
    {
        final_opening = temp_opening;
    }
    /* 温湿度都适宜 → 不需要控制 */
    else
    {
        final_opening = 0;
    }

    return final_opening;
}

/**
 * @brief  调节窗户到目标开度
 * @param  target_opening: 目标开度 (0-100%)
 * @note   根据窗户电机模式（相对/位置）选择合适的控制方法
 */
static void AdjustWindow(Temp_Humidity_Control_HandleTypeDef *hctrl, uint8_t target_opening)
{
    if (hctrl == NULL || hctrl->hmotor_window == NULL)
    {
        return;
    }

    Motor_ULN2003_HandleTypeDef *motor = hctrl->hmotor_window;

    /* 根据电机模式选择控制方法 */
    if (motor->mode == MOTOR_MODE_POSITION)
    {
        /* ========== 位置模式：精确控制窗户开度 ========== */

        int32_t target_pos = 0;

        if (target_opening == WINDOW_OPEN_NONE)
        {
            /* 全关 (最大位置) */
            target_pos = motor->max_position;
        }
        else if (target_opening >= WINDOW_OPEN_FULL)
        {
            /* 全开 (位置0) */
            target_pos = 0;
        }
        else
        {
            /* 半开或其他开度 */
            /* 将百分比转换为位置：开度越大，位置越接近0 */
            target_pos = motor->max_position - (motor->max_position * target_opening / 100);
        }

        /* 移动到目标位置 */
        Motor_ULN2003_MoveToPosition(motor, target_pos);
        hctrl->window_controlled_by_temp_hum = 1;
    }
    else
    {
        /* ========== 相对模式：简化控制（全开/半开/全关） ========== */

        if (target_opening == WINDOW_OPEN_NONE)
        {
            /* 全关 */
            if (motor->state != MOTOR_ULN2003_CLOSED && motor->state != MOTOR_ULN2003_CLOSING)
            {
                Motor_ULN2003_Close(motor);
                hctrl->window_controlled_by_temp_hum = 1;
            }
        }
        else if (target_opening >= WINDOW_OPEN_FULL)
        {
            /* 全开 */
            if (motor->state != MOTOR_ULN2003_OPEN && motor->state != MOTOR_ULN2003_OPENING)
            {
                Motor_ULN2003_Open(motor);
                hctrl->window_controlled_by_temp_hum = 1;
            }
        }
        else
        {
            /* 半开（或其他中间开度，统一处理为半开） */
            if (motor->state != MOTOR_ULN2003_OPEN && motor->state != MOTOR_ULN2003_OPENING)
            {
                Motor_ULN2003_Half(motor);
                hctrl->window_controlled_by_temp_hum = 1;
            }
        }
    }
}
