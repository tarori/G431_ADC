#include "main_cycle.hpp"

#include <stdio.h>
#include <math.h>
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "utils.hpp"
#include "sram.hpp"

constexpr bool print_stat = false;
constexpr bool is_external = true;

constexpr uint32_t data_buf_internal_len = 8192;
constexpr uint32_t data_buf_external_len = 4 * 65536;
constexpr uint32_t adc_dma_buf_len = 256;
uint16_t data_buf_internal[data_buf_internal_len];
uint16_t adc_dma_buf[adc_dma_buf_len];

constexpr uint32_t dac_dma_buf_len = 1000;
uint16_t dac_dma_buf[dac_dma_buf_len];

SRAM sram(&hspi3, SRAM_CS_GPIO_Port, SRAM_CS_Pin);

void adc_measure_internal();
void adc_measure_external();
void adc_calibration_external();
void setup_dac();

void main_loop()
{
    delay_ms(1000);  // wait USB  connection
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    putc('\r', stdout);  // flush
    printf("Hello, I am working at %ld MHz\n", SystemCoreClock / 1000 / 1000);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_DIFFERENTIAL_ENDED);
    HAL_ADC_Start(&hadc2);
    HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adc_dma_buf, adc_dma_buf_len / 2);
    MODIFY_REG(hadc1.DMA_Handle->Instance->CCR, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE, 0);

    setup_dac();
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dac_dma_buf, dac_dma_buf_len, DAC_ALIGN_12B_R);
    MODIFY_REG(hdac1.DMA_Handle1->Instance->CCR, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE, 0);

    sram.init();

    delay_ms(1000);  // wait external ADC calibration > 500ms

    HAL_TIM_Base_Start(&htim2);  // to measure elapsed time
    HAL_TIM_Base_Start(&htim6);  // 1MHz event for DAC
    HAL_TIM_Base_Start(&htim3);  // CNVST of ADC
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    printf("ADC is ready\n");
    while (1) {
        while (1) {
            uint8_t c = getc(stdin);
            if (c == 's') {
                break;
            }
            if (c == 'c') {
                adc_calibration_external();
            }
        }

        if (is_external) {
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

        } else {
            uint32_t start_clock = TIM2->CNT;
            {
                ScopedLock lock;
                adc_measure_internal();
            }
            uint32_t clock_elapsed = TIM2->CNT - start_clock;

            if (!print_stat) {
                for (uint32_t i = 0; i < data_buf_internal_len; ++i) {
                    int16_t code = data_buf_internal[i] - 2048;
                    printf("%d\n", code);
                }
            } else {
                float clock_per_sample = 1.0f * clock_elapsed / data_buf_internal_len;
                float us_elapsed = clock_elapsed * 1.0e+6f / SystemCoreClock;
                float sample_rate_mega = data_buf_internal_len / us_elapsed;
                printf("%.2fus, %.3fMS/s, %.3fCLK\n", us_elapsed, sample_rate_mega, clock_per_sample);
            }
        }

        delay_ms(100);
    }
}

__attribute__((long_call, section(".ccmram"))) void adc_measure_internal()
{
    uint32_t* data_ptr_32 = reinterpret_cast<uint32_t*>(data_buf_internal);
    uint32_t* dma_ptr_32 = reinterpret_cast<uint32_t*>(adc_dma_buf);
    uint32_t data_write_ptr = 0;
    uint32_t dma_current_ptr = dma_get_last_index(&hadc1, adc_dma_buf_len / 2);
    uint32_t dma_next_ptr = dma_current_ptr;
    while (data_write_ptr < data_buf_internal_len / 2) {
        while (dma_next_ptr == dma_current_ptr) {
            dma_current_ptr = dma_get_last_index(&hadc1, adc_dma_buf_len / 2);
        }
        data_ptr_32[data_write_ptr] = dma_ptr_32[dma_next_ptr];
        data_write_ptr++;
        dma_next_ptr = (dma_next_ptr + 1) % (adc_dma_buf_len / 2);
    }
}

static inline bool get_CNVST()
{
    // return TIM3->CNT < 60;
    return !(CNVST_IN_GPIO_Port->IDR & CNVST_IN_Pin);
}

__attribute__((long_call, section(".ccmram"))) void adc_measure_external()
{
    SPI_TypeDef* spi_adc = hspi1.Instance;
    SPI_TypeDef* spi_sram = hspi3.Instance;

    sram.start_write();

    while (get_CNVST() == true) {
    }
    while (get_CNVST() == false) {
    }
    uint32_t data_count = 0;
    uint32_t decim_count = 0;
    while (data_count < data_buf_external_len) {
        // Converting phase
        uint32_t spi_reg_next = READ_REG(spi_adc->CR1);
        spi_reg_next |= SPI_CR1_SPE;
        while (get_CNVST() == true) {
        }

        // Input Acquisition
        WRITE_REG(spi_adc->CR1, spi_reg_next);
        spi_reg_next &= ~SPI_CR1_SPE;
        WRITE_REG(spi_adc->CR1, spi_reg_next);
        while (READ_BIT(spi_adc->SR, SPI_FLAG_RXNE) == false) {
        }
        uint16_t value = ~(spi_adc->DR) & 0x3FFF;

        if (false) {  // No average
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
                if (decim_count > 32 * 10) {
                    data_count++;
                    spi_sram->DR = static_cast<uint16_t>(diff3 / 32 / 32 / 32);
                }
            }
        }

        while (get_CNVST() == false) {
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

    while (get_CNVST() == true) {
    }
    while (get_CNVST() == false) {
    }
    uint32_t data_count = 0;
    while (data_count < data_buf_internal_len) {
        // Converting phase
        while (get_CNVST() == true) {
        }

        // Input Acquisition
        spi_adc->DR = 0;

        while (READ_BIT(spi_adc->SR, SPI_FLAG_RXNE) == false) {
        }
        uint16_t value = spi_adc->DR;
        data_buf_internal[data_count] = value;
        ++data_count;
    }
}

void adc_calibration_external()
{
    printf("ADC self-calibration started\n");
    SPI_TypeDef* spi_adc = hspi1.Instance;
    uint32_t prev_width = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);  // CS Low
    delay_ms(1);

    // send 1024 clocks
    for (int i = 0; i < 1024 / 16; ++i) {
        SET_BIT(spi_adc->CR1, SPI_CR1_SPE);
        CLEAR_BIT(spi_adc->CR1, SPI_CR1_SPE);
        while (READ_BIT(spi_adc->SR, SPI_FLAG_RXNE) == false) {
        }
        [[maybe_unused]] uint16_t value = spi_adc->DR;
    }

    delay_ms(1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, prev_width);  // restore normal operation
    printf("ADC self-calibration ended\n");
}

void setup_dac()
{
    float fs = 1e+6;
    float freq = 10e+3;
    float PI = 3.14159265f;
    for (uint32_t i = 0; i < dac_dma_buf_len; ++i) {
        dac_dma_buf[i] = 2048 + 1023 * sinf(i * 2 * PI * freq / fs);
    }
}

extern "C" {
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_dac1_ch1;
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
 * @brief This function handles DMA1 channel1 global interrupt.
 */
void DMA1_Channel1_IRQHandler(void)
{
    printf("DMA1_Channel1_IRQHandler\n");
    HAL_DMA_IRQHandler(&hdma_adc1);
}

/**
 * @brief This function handles DMA1 channel2 global interrupt.
 */
void DMA1_Channel2_IRQHandler(void)
{
    printf("DMA1_Channel2_IRQHandler\n");
    HAL_DMA_IRQHandler(&hdma_dac1_ch1);
}

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
