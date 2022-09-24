#include <stdio.h>
#include <usart.h>
#include "utils.hpp"

extern "C" {

extern UART_HandleTypeDef huart1;

int _write(int file, char* ptr, int len)
{
    (void)file;

    if (!READ_BIT(huart1.Instance->CR1, USART_CR1_TE)) {
        return -1;
    }

    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        while (!READ_BIT(huart1.Instance->ISR, UART_FLAG_TXE)) {
        }
        huart1.Instance->TDR = *ptr++;
    }
    return len;
}

int _read(int file, char* ptr, int len)
{
    (void)file;
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        while (!READ_BIT(huart1.Instance->ISR, UART_FLAG_RXNE)) {
        }
        *ptr++ = huart1.Instance->RDR;
    }

    return len;
}
}
