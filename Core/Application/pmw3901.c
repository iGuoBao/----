#include "pmw3901.h"

extern SPI_HandleTypeDef hspi2;
uint16_t test;

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
    HAL_Delay(10); // Wait for Power-up

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
    
    SPI_PMW_CS_HIGH(); 
    return 0; 
}

uint8_t SPI_PMW_ReadByte(uint8_t byte)
{
    byte &= ~0x80u;
    
    SPI_PMW_CS_LOW();
    
    uint8_t rx_data;
    uint8_t tx_dummy = 0x00;
    
    HAL_SPI_TransmitReceive(&hspi2, &byte, &rx_data, 1, 10);
    
    HAL_SPI_TransmitReceive(&hspi2, &tx_dummy, &rx_data, 1, 10);
    
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
