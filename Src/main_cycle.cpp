#include "main_cycle.hpp"

#include <stdio.h>
#include <math.h>
#include "tim.h"
#include "utils.hpp"
#include "sram.hpp"

constexpr bool print_stat = false;
constexpr bool internal_memory = false;

constexpr uint32_t data_buf_internal_len = 8192;
constexpr uint32_t data_buf_external_len = 8 * 65536;
uint16_t data_buf_internal[data_buf_internal_len];

SRAM sram(&hspi3, SRAM_CS_GPIO_Port, SRAM_CS_Pin);

void adc_measure();
void adc_measure_test();
uint8_t adc_spi_communicate(bool is_write, uint8_t tx_data);

extern UART_HandleTypeDef huart1;

void main_loop()
{
    delay_ms(1000);  // wait USB  connection
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    putc('\r', stdout);  // flush
    huart1.Init.BaudRate = 2000000;
    HAL_UART_Init(&huart1);
    printf("Hello, I am working at %ld MHz\n", SystemCoreClock / 1000 / 1000);

    sram.init();

    delay_ms(1000);  // wait external ADC calibration > 500ms

    HAL_TIM_Base_Start(&htim2);  // to measure elapsed time
    HAL_TIM_Base_Start(&htim6);  // 1MHz event

    printf("ADC reg: 0x%x\n", adc_spi_communicate(false, 0));
    printf("ADC is ready\n");

    /*
    while (1) {
        adc_measure();
    }
    */

    while (1) {
        /*
        while (1) {
            uint8_t c = getc(stdin);
            if (c == 's') {
                break;
            }
        }
        */

        uint32_t start_clock = TIM2->CNT;
        {
            ScopedLock lock;
            HAL_GPIO_WritePin(LED_OK_GPIO_Port, LED_OK_Pin, GPIO_PIN_SET);
            if (!internal_memory) {
                adc_measure();
            } else {
                adc_measure_test();
            }
            HAL_GPIO_WritePin(LED_OK_GPIO_Port, LED_OK_Pin, GPIO_PIN_RESET);
        }
        uint32_t clock_elapsed = TIM2->CNT - start_clock;

        if (!print_stat) {
            delay_ms(100);
            HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
            if (!internal_memory) {
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
                printf("\r");
                sram.end_read();
            } else {
                for (uint32_t i = 0; i < data_buf_internal_len; ++i) {
                    int16_t code = data_buf_internal[i];
                    printf("%d\n", code);
                }
                printf("\r");
            }
            HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
        } else {
            float clock_per_sample = 1.0f * clock_elapsed / data_buf_internal_len;
            float us_elapsed = clock_elapsed * 1.0e+6f / SystemCoreClock;
            float sample_rate_mega = data_buf_internal_len / us_elapsed;
            printf("%.2fus, %.3fMS/s, %.3fCLK\n", us_elapsed, sample_rate_mega, clock_per_sample);
        }
        delay_ms(100);

        delay_ms(10000);
    }
}

static inline bool get_CNVST()
{
    return (CNVST_IN_GPIO_Port->IDR & CNVST_IN_Pin);
}

static inline void set_CNVST(bool state)
{
    if (state) {
        CNVST_OUT_GPIO_Port->BSRR = CNVST_OUT_Pin;
    } else {
        CNVST_OUT_GPIO_Port->BRR = CNVST_OUT_Pin;
    }
}

__attribute__((long_call, section(".ccmram"))) void adc_measure()
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
        while (get_CNVST() != true) {
        }

        for (uint32_t i = 0; i < 14; ++i) {
            asm volatile("NOP");
        }

        set_CNVST(false);
        while (get_CNVST() != false) {
        }

        // Input Acquisition
        spi_adc->DR = 0;
        /*
        while (READ_BIT(spi_adc->SR, SPI_FLAG_RXNE) == false) {
        }
        */
        for (uint32_t i = 0; i < 10; ++i) {
            asm volatile("NOP");
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

__attribute__((long_call, section(".ccmram"))) void adc_measure_test()
{
    SPI_TypeDef* spi_adc = hspi1.Instance;
    set_CNVST(true);

    SET_BIT(spi_adc->CR1, SPI_CR1_SPE);
    while (READ_BIT(spi_adc->SR, SPI_FLAG_RXNE) == true) {
        [[maybe_unused]] uint16_t dummy = spi_adc->DR;
    }

    uint32_t data_count = 0;
    while (data_count < data_buf_internal_len) {

        // Converting phase
        set_CNVST(true);
        while (get_CNVST() != true) {
        }

        for (uint32_t i = 0; i < 15; ++i) {
            asm volatile("NOP");
        }

        set_CNVST(false);
        while (get_CNVST() != false) {
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

uint8_t adc_spi_communicate(bool is_write, uint8_t tx_reg)
{
    SPI_TypeDef* spi_adc = hspi1.Instance;
    SET_BIT(spi_adc->CR1, SPI_CR1_SPE);
    while (READ_BIT(spi_adc->SR, SPI_FLAG_RXNE) == true) {
        [[maybe_unused]] uint16_t dummy = spi_adc->DR;
    }

    uint16_t tx_data;
    if (is_write) {
        tx_data = 0b0001010011111111;
    } else {
        tx_data = 0b11010100 << 8;
        tx_data |= tx_reg;
    }

    set_CNVST(true);
    delay_us(10);
    set_CNVST(false);
    delay_us(10);
    while (READ_BIT(spi_adc->SR, SPI_FLAG_TXE) == false) {
    }
    spi_adc->DR = ~tx_data;
    while (READ_BIT(spi_adc->SR, SPI_FLAG_RXNE) == false) {
    }
    uint16_t rx_data = ~(spi_adc->DR);
    set_CNVST(true);
    delay_us(10);

    return rx_data & 0xFF;
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
