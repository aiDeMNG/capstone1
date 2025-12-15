/**
 * @file    bh1750.c
 * @brief   BH1750 光强度传感器驱动 (STM32 HAL)
 * @note    基于 GY-30 模块 Arduino 示例代码移植
 */

#include "bh1750.h"

/* I2C 超时时间 (ms) */
#define BH1750_TIMEOUT  100

/**
 * @brief  发送命令到 BH1750
 * @param  hbh1750: BH1750 句柄指针
 * @param  cmd: 命令字节
 * @retval HAL_StatusTypeDef
 */
static HAL_StatusTypeDef BH1750_WriteCmd(BH1750_HandleTypeDef *hbh1750, uint8_t cmd)
{
    return HAL_I2C_Master_Transmit(hbh1750->hi2c,
                                   hbh1750->addr << 1,  // 7位地址左移1位
                                   &cmd,
                                   1,
                                   BH1750_TIMEOUT);
}

/**
 * @brief  从 BH1750 读取数据
 * @param  hbh1750: BH1750 句柄指针
 * @param  data: 数据缓冲区指针
 * @param  len: 数据长度
 * @retval HAL_StatusTypeDef
 */
static HAL_StatusTypeDef BH1750_ReadData(BH1750_HandleTypeDef *hbh1750, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Master_Receive(hbh1750->hi2c,
                                  hbh1750->addr << 1,  // 7位地址左移1位
                                  data,
                                  len,
                                  BH1750_TIMEOUT);
}

/**
 * @brief  初始化 BH1750 传感器
 */
HAL_StatusTypeDef BH1750_Init(BH1750_HandleTypeDef *hbh1750, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    HAL_StatusTypeDef status;

    /* 保存参数 */
    hbh1750->hi2c = hi2c;
    hbh1750->addr = addr;
    hbh1750->mode = BH1750_CONT_H_RES;  // 默认连续高分辨率模式

    /* 检查设备是否存在 */
    status = HAL_I2C_IsDeviceReady(hi2c, addr << 1, 3, BH1750_TIMEOUT);
    if (status != HAL_OK) {
        return status;
    }

    /* 开启电源 */
    status = BH1750_PowerOn(hbh1750);
    if (status != HAL_OK) {
        return status;
    }

    /* 设置默认测量模式 (连续高分辨率模式, 1 lx, 120ms) */
    status = BH1750_SetMode(hbh1750, BH1750_CONT_H_RES);

    return status;
}

/**
 * @brief  设置测量模式
 */
HAL_StatusTypeDef BH1750_SetMode(BH1750_HandleTypeDef *hbh1750, uint8_t mode)
{
    HAL_StatusTypeDef status;

    status = BH1750_WriteCmd(hbh1750, mode);
    if (status == HAL_OK) {
        hbh1750->mode = mode;
    }

    return status;
}

/**
 * @brief  读取光强度值 (浮点数)
 */
HAL_StatusTypeDef BH1750_ReadLight(BH1750_HandleTypeDef *hbh1750, float *lux)
{
    HAL_StatusTypeDef status;
    uint8_t buf[2];
    uint16_t raw;

    /* 读取2字节原始数据 */
    status = BH1750_ReadData(hbh1750, buf, 2);
    if (status != HAL_OK) {
        return status;
    }

    /* 计算光强度: lux = (高字节 << 8 | 低字节) / 1.2 */
    raw = (buf[0] << 8) | buf[1];
    *lux = (float)raw / 1.2f;

    return HAL_OK;
}

/**
 * @brief  读取光强度值 (整数)
 */
HAL_StatusTypeDef BH1750_ReadLightInt(BH1750_HandleTypeDef *hbh1750, uint16_t *lux)
{
    HAL_StatusTypeDef status;
    uint8_t buf[2];
    uint16_t raw;

    /* 读取2字节原始数据 */
    status = BH1750_ReadData(hbh1750, buf, 2);
    if (status != HAL_OK) {
        return status;
    }

    /* 计算光强度: lux = (高字节 << 8 | 低字节) / 1.2
     * 使用整数运算: lux = raw * 10 / 12 */
    raw = (buf[0] << 8) | buf[1];
    *lux = (raw * 10) / 12;

    return HAL_OK;
}

/**
 * @brief  关闭 BH1750 电源
 */
HAL_StatusTypeDef BH1750_PowerDown(BH1750_HandleTypeDef *hbh1750)
{
    return BH1750_WriteCmd(hbh1750, BH1750_POWER_DOWN);
}

/**
 * @brief  开启 BH1750 电源
 */
HAL_StatusTypeDef BH1750_PowerOn(BH1750_HandleTypeDef *hbh1750)
{
    return BH1750_WriteCmd(hbh1750, BH1750_POWER_ON);
}

/**
 * @brief  重置 BH1750 数据寄存器
 */
HAL_StatusTypeDef BH1750_Reset(BH1750_HandleTypeDef *hbh1750)
{
    return BH1750_WriteCmd(hbh1750, BH1750_RESET);
}
