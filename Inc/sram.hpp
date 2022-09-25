#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stm32g4xx.h>
#include "gpio.h"
#include "spi.h"
#include "utils.hpp"

/* SPI 23LC512 */
class SRAM
{
public:
    SRAM(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin) : hspi(hspi), cs_port(cs_port), cs_pin(cs_pin)
    {
    }

    void init()
    {
        __HAL_SPI_ENABLE(hspi);
        cs_high();
        delay_1clk();
    }

    void start_write()
    {
        cs_low();
        uint16_t ret = read_write_command(0x02);  // write command
        if (ret != 0x6364) {
            printf("SPI: ID mismatch: 0x%x\n", ret);
        }
        send_dummy_4byte();
        cs_high();
        delay_command();
        MODIFY_REG(hspi->Instance->CR1, SPI_CR1_BR, SPI_BAUDRATEPRESCALER_4);
        cs_low();
    }

    void start_read()
    {
        cs_low();
        uint16_t ret = read_write_command(0x03);  // read command
        if (ret != 0x6364) {
            printf("SPI: ID mismatch: 0x%x\n", ret);
        }
        send_dummy_4byte();
        cs_high();
        delay_command();
        cs_low();
    }

    void end_write()
    {
        send_dummy_4byte();
        cs_high();
        MODIFY_REG(hspi->Instance->CR1, SPI_CR1_BR, SPI_BAUDRATEPRESCALER_16);
    }

    void end_read()
    {
        send_dummy_4byte();
        cs_high();
    }

    uint16_t read_write_command(uint16_t command)
    {
        uint16_t ret;
        while (!READ_BIT(hspi->Instance->SR, SPI_FLAG_TXE)) {
        }
        hspi->Instance->DR = command;
        while (!READ_BIT(hspi->Instance->SR, SPI_FLAG_RXNE)) {
        }
        ret = hspi->Instance->DR;
        return ret;
    }

    void sram_test()
    {
        constexpr int sram_test_size = 65535;
        this->start_write();
        for (int i = 0; i < sram_test_size; ++i) {
            while (!READ_BIT(hspi->Instance->SR, SPI_FLAG_TXE)) {
            }
            hspi->Instance->DR = i;
        }
        this->end_write();

        delay_ms(100);

        this->start_read();
        for (int i = 0; i < sram_test_size; ++i) {
            while (!READ_BIT(hspi->Instance->SR, SPI_FLAG_TXE)) {
            }
            hspi->Instance->DR = 0;
            while (!READ_BIT(hspi->Instance->SR, SPI_FLAG_RXNE)) {
            }
            uint16_t data = hspi->Instance->DR;
            if (i % 10000 == 0) {
                printf("%d\n", data);
            }
        }
        this->end_read();
    }

private:
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;

    void delay_1clk()
    {
        delay_us(1);
    }

    void delay_command()
    {
        delay_ms(20);
    }

    // for ESP32 SPI DMA errata
    void send_dummy_4byte()
    {
        read_write_command(0x0000);
        read_write_command(0x0000);
    }

    void cs_low()
    {
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    }

    void cs_high()
    {
        while (READ_BIT(hspi->Instance->SR, SPI_FLAG_BSY)) {
        }
        while (READ_BIT(hspi->Instance->SR, SPI_FLAG_RXNE)) {
            [[maybe_unused]] volatile uint16_t dummy = hspi->Instance->DR;
        }
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    }
};
