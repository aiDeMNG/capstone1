#include "HC05.h"

static UART_HandleTypeDef *hc05_uart = NULL;
static uint8_t rx_buffer[HC05_BUFFER_SIZE];
static volatile uint16_t rx_write_index = 0;
static volatile uint16_t rx_read_index = 0;
static uint8_t rx_byte;

void HC05_Init(UART_HandleTypeDef *huart)
{
    hc05_uart = huart;
    rx_write_index = 0;
    rx_read_index = 0;

    HAL_UART_Receive_IT(hc05_uart, &rx_byte, 1);
}

void HC05_SendString(const char *str)
{
    if (hc05_uart != NULL && str != NULL)
    {
        HAL_UART_Transmit(hc05_uart, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
    }
}

void HC05_SendData(uint8_t *data, uint16_t len)
{
    if (hc05_uart != NULL && data != NULL && len > 0)
    {
        HAL_UART_Transmit(hc05_uart, data, len, HAL_MAX_DELAY);
    }
}

uint16_t HC05_Available(void)
{
    if (rx_write_index >= rx_read_index)
    {
        return rx_write_index - rx_read_index;
    }
    else
    {
        return HC05_BUFFER_SIZE - rx_read_index + rx_write_index;
    }
}

uint16_t HC05_Read(uint8_t *buffer, uint16_t max_len)
{
    uint16_t count = 0;

    while (count < max_len && HC05_Available() > 0)
    {
        buffer[count++] = rx_buffer[rx_read_index++];
        if (rx_read_index >= HC05_BUFFER_SIZE)
        {
            rx_read_index = 0;
        }
    }

    return count;
}

void HC05_ClearBuffer(void)
{
    rx_write_index = 0;
    rx_read_index = 0;
}

uint8_t HC05_SendATCommand(const char *cmd, char *response, uint16_t timeout_ms)
{
    if (hc05_uart == NULL || cmd == NULL)
    {
        return 0;
    }

    HC05_ClearBuffer();

    HC05_SendString(cmd);
    HC05_SendString("\r\n");

    uint32_t start_tick = HAL_GetTick();
    uint16_t response_len = 0;

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        uint16_t available = HC05_Available();
        if (available > 0 && response != NULL)
        {
            response_len += HC05_Read((uint8_t *)&response[response_len], available);
            response[response_len] = '\0';

            if (strstr(response, "OK") != NULL || strstr(response, "ERROR") != NULL)
            {
                return 1;
            }
        }
        HAL_Delay(10);
    }

    return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == hc05_uart)
    {
        rx_buffer[rx_write_index++] = rx_byte;

        if (rx_write_index >= HC05_BUFFER_SIZE)
        {
            rx_write_index = 0;
        }

        if (rx_write_index == rx_read_index)
        {
            rx_read_index++;
            if (rx_read_index >= HC05_BUFFER_SIZE)
            {
                rx_read_index = 0;
            }
        }

        HAL_UART_Receive_IT(hc05_uart, &rx_byte, 1);
    }
}
