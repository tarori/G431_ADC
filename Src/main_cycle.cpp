#include "main_cycle.hpp"

#include <stdio.h>
#include <math.h>
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "utils.hpp"
#include "sram.hpp"

constexpr bool print_stat = false;

constexpr uint32_t data_buf_internal_len = 8192;
constexpr uint32_t data_buf_external_len = 4096;
constexpr uint32_t adc_dma_buf_len = 256;
uint16_t data_buf_internal[data_buf_internal_len];
uint16_t adc_dma_buf[adc_dma_buf_len];

constexpr uint32_t dac_dma_buf_len = 1000;
uint16_t dac_dma_buf[dac_dma_buf_len];

SRAM sram(&hspi3, SRAM_CS_GPIO_Port, SRAM_CS_Pin);

void adc_measure_internal();
void adc_measure_external();
void setup_dac();

void main_loop()
{
    printf("Hello, I am working at %ld MHz\n", SystemCoreClock / 1000 / 1000);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_DIFFERENTIAL_ENDED);
    HAL_ADC_Start(&hadc2);
    HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adc_dma_buf, adc_dma_buf_len / 2);
    MODIFY_REG(hadc1.DMA_Handle->Instance->CCR, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE, 0);

    setup_dac();
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dac_dma_buf, dac_dma_buf_len, DAC_ALIGN_12B_R);
    MODIFY_REG(hdac1.DMA_Handle1->Instance->CCR, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE, 0);

    HAL_TIM_Base_Start(&htim2);  // to measure elapsed time
    HAL_TIM_Base_Start(&htim6);  // 1MHz event

    delay_ms(1000);

    while (1) {
        uint32_t start_clock = TIM2->CNT;
        {
            ScopedLock lock;
            adc_measure_internal();
        }
        uint32_t clock_elapsed = TIM2->CNT - start_clock;

        if (!print_stat) {
            for (uint32_t i = 0; i < data_buf_internal_len; ++i) {
                float voltage = 3.3f / 2047 * (data_buf_internal[i] - 2048);
                printf("%.5f V\n", voltage);
            }
        } else {
            float clock_per_sample = 1.0f * clock_elapsed / data_buf_internal_len;
            float us_elapsed = clock_elapsed * 1.0e+6f / SystemCoreClock;
            float sample_rate_mega = data_buf_internal_len / us_elapsed;
            printf("%.2fus, %.3fMS/s, %.3fCLK\n", us_elapsed, sample_rate_mega, clock_per_sample);
        }

        delay_ms(10000);
    }
}

void adc_measure_internal()
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
        dma_next_ptr = (dma_next_ptr + 1) % (adc_dma_buf_len / 2);
        data_write_ptr++;
    }
}

void adc_measure_external()
{
}

void setup_dac()
{
    for (uint32_t i = 0; i < dac_dma_buf_len; ++i) {
        dac_dma_buf[i] = 2048 + 2047 * sinf(i * 2 * 3.14159f / dac_dma_buf_len);
    }
}
