/**
 * @file    bh1750.h
 * @brief   BH1750 光强度传感器驱动 (STM32 HAL)
 * @note    基于 GY-30 模块 Arduino 示例代码移植
 *          I2C 地址: 0x23 (ADDR 引脚接地或悬空)
 *                   0x5C (ADDR 引脚接高电平)
 */

#ifndef __BH1750_H
#define __BH1750_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* BH1750 I2C 地址 (7位地址) */
#define BH1750_ADDR_LOW     0x23    // ADDR 引脚接地或悬空
#define BH1750_ADDR_HIGH    0x5C    // ADDR 引脚接高电平

/* BH1750 指令集 */
#define BH1750_POWER_DOWN   0x00    // 关闭电源
#define BH1750_POWER_ON     0x01    // 开启电源
#define BH1750_RESET        0x07    // 重置数据寄存器

/* 测量模式 */
#define BH1750_CONT_H_RES   0x10    // 连续高分辨率模式 (1 lx, 120ms)
#define BH1750_CONT_H_RES2  0x11    // 连续高分辨率模式2 (0.5 lx, 120ms)
#define BH1750_CONT_L_RES   0x13    // 连续低分辨率模式 (4 lx, 16ms)
#define BH1750_ONE_H_RES    0x20    // 单次高分辨率模式 (1 lx, 120ms)
#define BH1750_ONE_H_RES2   0x21    // 单次高分辨率模式2 (0.5 lx, 120ms)
#define BH1750_ONE_L_RES    0x23    // 单次低分辨率模式 (4 lx, 16ms)

/* BH1750 句柄结构体 */
typedef struct {
    I2C_HandleTypeDef *hi2c;    // I2C 句柄指针
    uint8_t addr;               // 设备地址 (7位)
    uint8_t mode;               // 测量模式
} BH1750_HandleTypeDef;

/* 函数声明 */

/**
 * @brief  初始化 BH1750 传感器
 * @param  hbh1750: BH1750 句柄指针
 * @param  hi2c: I2C 句柄指针
 * @param  addr: 设备地址 (BH1750_ADDR_LOW 或 BH1750_ADDR_HIGH)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BH1750_Init(BH1750_HandleTypeDef *hbh1750, I2C_HandleTypeDef *hi2c, uint8_t addr);

/**
 * @brief  设置测量模式
 * @param  hbh1750: BH1750 句柄指针
 * @param  mode: 测量模式 (BH1750_CONT_H_RES, BH1750_ONE_H_RES 等)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BH1750_SetMode(BH1750_HandleTypeDef *hbh1750, uint8_t mode);

/**
 * @brief  读取光强度值
 * @param  hbh1750: BH1750 句柄指针
 * @param  lux: 光强度值指针 (单位: lx)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BH1750_ReadLight(BH1750_HandleTypeDef *hbh1750, float *lux);

/**
 * @brief  读取光强度值 (整数版本)
 * @param  hbh1750: BH1750 句柄指针
 * @param  lux: 光强度值指针 (单位: lx)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BH1750_ReadLightInt(BH1750_HandleTypeDef *hbh1750, uint16_t *lux);

/**
 * @brief  关闭 BH1750 电源
 * @param  hbh1750: BH1750 句柄指针
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BH1750_PowerDown(BH1750_HandleTypeDef *hbh1750);

/**
 * @brief  开启 BH1750 电源
 * @param  hbh1750: BH1750 句柄指针
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BH1750_PowerOn(BH1750_HandleTypeDef *hbh1750);

/**
 * @brief  重置 BH1750 数据寄存器
 * @param  hbh1750: BH1750 句柄指针
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BH1750_Reset(BH1750_HandleTypeDef *hbh1750);

#ifdef __cplusplus
}
#endif

#endif /* __BH1750_H */
