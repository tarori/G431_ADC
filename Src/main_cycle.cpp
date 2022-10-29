#include "main_cycle.hpp"

#include <stdio.h>
#include <math.h>
#include "tim.h"
#include "utils.hpp"
#include "sram.hpp"

constexpr bool print_stat = false;

constexpr uint32_t data_buf_internal_len = 8192;
constexpr uint32_t data_buf_external_len = 4 * 65536;
uint16_t data_buf_internal[data_buf_internal_len];

SRAM sram(&hspi3, SRAM_CS_GPIO_Port, SRAM_CS_Pin);

void adc_measure_external();

void main_loop()
{
    delay_ms(1000);  // wait USB  connection
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    putc('\r', stdout);  // flush
    printf("Hello, I am working at %ld MHz\n", SystemCoreClock / 1000 / 1000);

    sram.init();

    delay_ms(1000);  // wait external ADC calibration > 500ms

    HAL_TIM_Base_Start(&htim2);  // to measure elapsed time
    HAL_TIM_Base_Start(&htim6);  // 1MHz event

    printf("ADC is ready\n");
    while (1) {
        while (1) {
            uint8_t c = getc(stdin);
            if (c == 's') {
                break;
            }
        }

        uint32_t start_clock = TIM2->CNT;
        {
            ScopedLock lock;
            adc_measure_external();
        }
        uint32_t clock_elapsed = TIM2->CNT - start_clock;

        if (!print_stat) {
            delay_ms(100);
            sram.start_read();
            for (uint32_t i = 0; i < data_buf_external_len; ++i) {
                while (!READ_BIT(hspi3.Instance->SR, SPI_FLAG_TXE)) {
                }
                hspi3.Instance->DR = 0;
                while (!READ_BIT(hspi3.Instance->SR, SPI_FLAG_RXNE)) {
                }
                int16_t code = hspi3.Instance->DR;
                printf("%d\n", code);
            }
            sram.end_read();
        } else {
            float clock_per_sample = 1.0f * clock_elapsed / data_buf_internal_len;
            float us_elapsed = clock_elapsed * 1.0e+6f / SystemCoreClock;
            float sample_rate_mega = data_buf_internal_len / us_elapsed;
            printf("%.2fus, %.3fMS/s, %.3fCLK\n", us_elapsed, sample_rate_mega, clock_per_sample);
        }
    }
    delay_ms(100);
}

static inline bool get_CNVST()
{
    return !(CNVST_IN_GPIO_Port->IDR & CNVST_IN_Pin);
}

static inline void set_CNVST(bool state)
{
    if (state) {
        CNVST_OUT_GPIO_Port->BSRR = CNVST_OUT_Pin;
    } else {
        CNVST_OUT_GPIO_Port->BRR = CNVST_OUT_Pin;
    }
}

__attribute__((long_call, section(".ccmram"))) void adc_measure_external()
{
    SPI_TypeDef* spi_adc = hspi1.Instance;
    SPI_TypeDef* spi_sram = hspi3.Instance;

    SET_BIT(spi_adc->CR1, SPI_CR1_SPE);
    while (READ_BIT(spi_adc->SR, SPI_FLAG_RXNE) == true) {
        [[maybe_unused]] uint16_t dummy = spi_adc->DR;
    }

    sram.start_write();

    uint32_t data_count = 0;
    uint32_t decim_count = 0;
    while (data_count < data_buf_external_len) {

        // Converting phase
        set_CNVST(true);
        while (get_CNVST() == true) {
        }
        set_CNVST(false);
        while (get_CNVST() == false) {
        }

        // Input Acquisition
        spi_adc->DR = 0;
        while (READ_BIT(spi_adc->SR, SPI_FLAG_RXNE) == false) {
        }
        uint16_t value = ~(spi_adc->DR);

        if (true) {  // No average
            spi_sram->DR = value;
            ++data_count;

        } else {
            static int32_t int1 = 0, int2 = 0, int3 = 0;
            int1 += static_cast<int16_t>(value);
            int2 += int1;
            int3 += int2;
            if (++decim_count % 32 == 0) {
                static int32_t int3_prev = 0, diff1_prev = 0, diff2_prev;
                int32_t diff1 = int3 - int3_prev;
                int3_prev = int3;
                int32_t diff2 = diff1 - diff1_prev;
                diff1_prev = diff1;
                int32_t diff3 = diff2 - diff2_prev;
                diff2_prev = diff2;
                spi_sram->DR = static_cast<uint16_t>(diff3 / 32 / 32 / 32);
                data_count++;
            }
        }
    }

    sram.end_write();
}

__attribute__((long_call, section(".ccmram"))) void adc_measure_external_test()
{
    SPI_TypeDef* spi_adc = hspi1.Instance;

    __HAL_SPI_ENABLE(&hspi1);

    while (READ_BIT(spi_adc->SR, SPI_FLAG_RXNE) == true) {
        [[maybe_unused]] uint16_t dummy = spi_adc->DR;
    }

    uint32_t data_count = 0;
    while (data_count < data_buf_internal_len) {

        // Converting phase
        set_CNVST(true);
        while (get_CNVST() == true) {
        }
        set_CNVST(false);
        while (get_CNVST() == false) {
        }

        // Input Acquisition
        spi_adc->DR = 0;
        while (READ_BIT(spi_adc->SR, SPI_FLAG_RXNE) == false) {
        }
        uint16_t value = ~(spi_adc->DR);
        data_buf_internal[data_count] = value;
        ++data_count;
    }
}

extern "C" {
extern PCD_HandleTypeDef hpcd_USB_FS;

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles USB low priority interrupt remap.
 */
void USB_LP_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

// For C-linkage
void SystemClock_Config(void)
{
}
}
