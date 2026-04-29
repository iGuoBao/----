#include "pmw3901.h"
#include "Delay.h"

extern SPI_HandleTypeDef hspi2;
uint16_t test;

static uint8_t PMW3901_SwitchToMode3(void)
{
    if (hspi2.Init.CLKPolarity == SPI_POLARITY_HIGH && hspi2.Init.CLKPhase == SPI_PHASE_2EDGE)
    {
        return 0u;
    }

    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;

    return (HAL_SPI_Init(&hspi2) == HAL_OK) ? 0u : 1u;
}

void SPI_PMW_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = PMW_SPI_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PMW_SPI_CS_PORT, &GPIO_InitStruct);
    
    SPI_PMW_CS_HIGH();
    delay_20ms(1);

    if (PMW3901_SwitchToMode3() != 0u)
    {
        return;
    }
    SPI_PMW_SendByte(0x3A, 0x5A); // Reset command
    SPI_PMW_ReadByte(0x02);
    SPI_PMW_ReadByte(0x03);
    SPI_PMW_ReadByte(0x04);
    SPI_PMW_ReadByte(0x05);
    SPI_PMW_ReadByte(0x06);

    // Performance Optimization Registers
    SPI_PMW_SendByte(0x7F, 0x00);
    SPI_PMW_SendByte(0x61, 0xAD);
    SPI_PMW_SendByte(0x7F, 0x03);
    SPI_PMW_SendByte(0x40, 0x00);
    SPI_PMW_SendByte(0x7F, 0x05);
    SPI_PMW_SendByte(0x41, 0xB3);
    SPI_PMW_SendByte(0x43, 0xF1);
    SPI_PMW_SendByte(0x45, 0x14);
    SPI_PMW_SendByte(0x5B, 0x32);
    SPI_PMW_SendByte(0x5F, 0x34);
    SPI_PMW_SendByte(0x7B, 0x08);
    SPI_PMW_SendByte(0x7F, 0x06);
    SPI_PMW_SendByte(0x44, 0x1B);
    SPI_PMW_SendByte(0x40, 0xBF);
    SPI_PMW_SendByte(0x4E, 0x3F);

    SPI_PMW_SendByte(0x7F, 0x08);
    SPI_PMW_SendByte(0x65, 0x20);
    SPI_PMW_SendByte(0x6A, 0x18);
    SPI_PMW_SendByte(0x7F, 0x09);
    SPI_PMW_SendByte(0x4F, 0xAF);
    SPI_PMW_SendByte(0x5F, 0x40);
    SPI_PMW_SendByte(0x48, 0x80);
    SPI_PMW_SendByte(0x49, 0x80);
    SPI_PMW_SendByte(0x57, 0x77);
    SPI_PMW_SendByte(0x60, 0x78);
    SPI_PMW_SendByte(0x61, 0x78);
    SPI_PMW_SendByte(0x62, 0x08);
    SPI_PMW_SendByte(0x63, 0x50);
    SPI_PMW_SendByte(0x7F, 0x0A);
    SPI_PMW_SendByte(0x45, 0x60);
    SPI_PMW_SendByte(0x7F, 0x00);
    SPI_PMW_SendByte(0x4D, 0x11);
    SPI_PMW_SendByte(0x55, 0x80);
    SPI_PMW_SendByte(0x74, 0x1F);
    SPI_PMW_SendByte(0x75, 0x1F);
    SPI_PMW_SendByte(0x4A, 0x78);
    SPI_PMW_SendByte(0x4B, 0x78);
    SPI_PMW_SendByte(0x44, 0x08);
    SPI_PMW_SendByte(0x45, 0x50);
    SPI_PMW_SendByte(0x64, 0xFF);
    SPI_PMW_SendByte(0x65, 0x1F);
    SPI_PMW_SendByte(0x7F, 0x14);
    SPI_PMW_SendByte(0x65, 0x67);
    SPI_PMW_SendByte(0x66, 0x08);
    SPI_PMW_SendByte(0x63, 0x70);
    SPI_PMW_SendByte(0x7F, 0x15);
    SPI_PMW_SendByte(0x48, 0x48);
    SPI_PMW_SendByte(0x7F, 0x07);
    SPI_PMW_SendByte(0x41, 0x0D);
    SPI_PMW_SendByte(0x43, 0x14);
    SPI_PMW_SendByte(0x4B, 0x0E);
    SPI_PMW_SendByte(0x45, 0x0F);
    SPI_PMW_SendByte(0x44, 0x42);
    SPI_PMW_SendByte(0x4C, 0x80);
    SPI_PMW_SendByte(0x7F, 0x10);
    SPI_PMW_SendByte(0x5B, 0x02);
    SPI_PMW_SendByte(0x7F, 0x07);
    SPI_PMW_SendByte(0x40, 0x41);
    SPI_PMW_SendByte(0x70, 0x00);

    delay_20ms(1);

    SPI_PMW_SendByte(0x32, 0x44);
    SPI_PMW_SendByte(0x7F, 0x07);
    SPI_PMW_SendByte(0x40, 0x40);
    SPI_PMW_SendByte(0x7F, 0x06);
    SPI_PMW_SendByte(0x62, 0xF0);
    SPI_PMW_SendByte(0x63, 0x00);
    SPI_PMW_SendByte(0x7F, 0x0D);
    SPI_PMW_SendByte(0x48, 0xC0);
    SPI_PMW_SendByte(0x6F, 0xD5);
    SPI_PMW_SendByte(0x7F, 0x00);
    SPI_PMW_SendByte(0x5B, 0xA0);
    SPI_PMW_SendByte(0x4E, 0xA8);
    SPI_PMW_SendByte(0x5A, 0x50);
    SPI_PMW_SendByte(0x40, 0x80);
    SPI_PMW_SendByte(0x7F, 0x00);
    SPI_PMW_SendByte(0x5A, 0x10);
    SPI_PMW_SendByte(0x54, 0x00);
}

uint8_t SPI_PMW_SendByte(uint8_t location, uint8_t byte)
{
    location |= 0x80u;
    
    SPI_PMW_CS_LOW();
    
    uint8_t rx_data;
    if (HAL_SPI_TransmitReceive(&hspi2, &location, &rx_data, 1, 10) != HAL_OK) {
        SPI_PMW_CS_HIGH();
        return 1;
    }
    
    if (HAL_SPI_TransmitReceive(&hspi2, &byte, &rx_data, 1, 10) != HAL_OK) {
        SPI_PMW_CS_HIGH();
        return 2;
    }

    // Datasheet requires a short write-to-write interval.
    Delay_us(20);
    
    SPI_PMW_CS_HIGH(); 
    return 0; 
}

uint8_t SPI_PMW_ReadByte(uint8_t byte)
{
    byte &= ~0x80u;
    
    SPI_PMW_CS_LOW();
    
    uint8_t rx_data;
    uint8_t tx_dummy = 0x00;
    
    if (HAL_SPI_TransmitReceive(&hspi2, &byte, &rx_data, 1, 10) != HAL_OK)
    {
        SPI_PMW_CS_HIGH();
        return 0xFFu;
    }

    // tSRAD: wait before reading register data.
    Delay_us(35);
    
    if (HAL_SPI_TransmitReceive(&hspi2, &tx_dummy, &rx_data, 1, 10) != HAL_OK)
    {
        SPI_PMW_CS_HIGH();
        return 0xFFu;
    }

    Delay_us(1);
    
    SPI_PMW_CS_HIGH();
    
    return rx_data; 
}

PMW3901_Delta_t PMW3901_ReadDelta(void)
{
    PMW3901_Delta_t delta;
    
    // Read Motion register (0x02) first to latch data into Delta_X and Delta_Y registers 
    SPI_PMW_ReadByte(0x02);
    
    uint8_t xl = SPI_PMW_ReadByte(0x03);
    uint8_t xh = SPI_PMW_ReadByte(0x04);
    uint8_t yl = SPI_PMW_ReadByte(0x05);
    uint8_t yh = SPI_PMW_ReadByte(0x06);
    
    delta.x = (int16_t)((xh << 8) | xl);
    delta.y = (int16_t)((yh << 8) | yl);
    
    return delta;
}
