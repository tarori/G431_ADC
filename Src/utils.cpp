#include <stdio.h>
#include "usart.h"
#include "usbd_cdc_if.h"
#include "utils.hpp"

void USB_Transmit_Data(uint8_t* ptr, int len)
{
    uint8_t ret;
    uint32_t timeout = 10000;
    uint32_t tick_start = uwTick;
    do {
        ret = CDC_Transmit_FS((uint8_t*)ptr, len);
    } while (ret != USBD_OK && uwTick - tick_start < timeout);

    if (uwTick - tick_start >= timeout) {
        printf("USB transmit timeout\n");
    }
}

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

    /*
    uint8_t ret;
    uint32_t timeout = 10;
    uint32_t tick_start = uwTick;
    do {
        ret = CDC_Transmit_FS((uint8_t*)ptr, len);
    } while (ret != USBD_OK && uwTick - tick_start < timeout);
    */
    return len;
}

volatile uint32_t cdc_buffer_left = 0;
volatile uint32_t cdc_buffer_index = 0;
uint8_t* cdc_buffer;

int _read(int file, char* ptr, int len)
{
    /*
    (void)file;
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        while (!READ_BIT(huart1.Instance->ISR, UART_FLAG_RXNE)) {
        }
        *ptr++ = huart1.Instance->RDR;
    }
    */

    cdc_buffer = (uint8_t*)ptr;
    cdc_buffer_index = 0;
    cdc_buffer_left = len;

    while (cdc_buffer_left > 0)
        ;

    return len;
}

void CDC_Receive_DATA(uint8_t* Buf, uint32_t Len)
{
    if (cdc_buffer_left == 0) {
        return;
    }

    for (uint32_t i = 0; i < Len && cdc_buffer_left > 0; ++i) {
        cdc_buffer[cdc_buffer_index] = Buf[i];
        cdc_buffer_index++;
        cdc_buffer_left--;
    }
}
}
