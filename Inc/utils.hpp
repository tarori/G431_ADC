#pragma once
#include <stm32g4xx.h>
#include <stdint.h>

void USB_Transmit_Data(uint8_t* ptr, int len);

#define FIELD_GET(mask, reg) (((reg) & (mask)) >> (__builtin_ffsll(mask) - 1))
#define __ALIGN_MASK(x, mask) ((x) & ~(mask))
#define ALIGN(x, a) __ALIGN_MASK(x, (typeof(x))(a)-1)

static inline void delay_us(uint32_t us)
{
    uint32_t old_val = SysTick->VAL;
    uint32_t end_val = us * (SystemCoreClock / 1000 / 1000);
    uint32_t elapsed = 0;
    while (elapsed < end_val) {
        uint32_t new_val = SysTick->VAL;
        if (new_val < old_val) {
            elapsed += old_val - new_val;
        } else {
            elapsed += old_val + SysTick->LOAD + 1 - new_val;
        }
        old_val = new_val;
    }
}

static inline void delay_ms(uint32_t ms)
{
    delay_us(1000 * ms);
}

static inline uint32_t dma_get_last_index(ADC_HandleTypeDef* hadc, uint32_t buf_size)
{
    return (2 * buf_size - 1 - hadc->DMA_Handle->Instance->CNDTR) % buf_size;
}

static inline bool HAL_Check(HAL_StatusTypeDef ret, const char* s)
{
    if (ret == HAL_OK) {
        return false;
    }
    printf("Error at %s: ", s);
    switch (ret) {
    case HAL_ERROR:
        printf("HAL_ERROR\n");
        break;
    case HAL_BUSY:
        printf("HAL_BUSY\n");
        break;
    case HAL_TIMEOUT:
        printf("HAL_TIMEOUT\n");
        break;
    default:
        printf("HAL_UNKNOWN\n");
    }
    Error_Handler();
    return true;
}

class ScopedLock
{
public:
    ScopedLock()
    {
        __disable_irq();
        if (lock_ref_count++) {
            printf("WARN: interrupt lock is already held\n");
        }
    }

    ~ScopedLock()
    {
        if (!(--lock_ref_count)) {
            __enable_irq();
        }
    }

private:
    static volatile uint32_t lock_ref_count;
};

inline volatile uint32_t ScopedLock::lock_ref_count = 0;
