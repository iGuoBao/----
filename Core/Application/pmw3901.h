#ifndef __SPI_PMW_H
#define __SPI_PMW_H

#include "main.h"
#include <stdio.h>

#define PMW_SPI_CS_PORT GPIOC
#define PMW_SPI_CS_PIN  GPIO_PIN_4

#define SPI_PMW_CS_LOW()   HAL_GPIO_WritePin(PMW_SPI_CS_PORT, PMW_SPI_CS_PIN, GPIO_PIN_RESET)
#define SPI_PMW_CS_HIGH()  HAL_GPIO_WritePin(PMW_SPI_CS_PORT, PMW_SPI_CS_PIN, GPIO_PIN_SET)

void SPI_PMW_Init(void);
uint8_t SPI_PMW_ReadByte(uint8_t byte);
uint8_t SPI_PMW_SendByte(uint8_t location, uint8_t byte);

typedef struct {
    int16_t x;
    int16_t y;
} PMW3901_Delta_t;

PMW3901_Delta_t PMW3901_ReadDelta(void);

#endif /* __SPI_PMW_H */
