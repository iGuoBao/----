# SPI Device Addition Feasibility Analysis

## Current Configuration
- The existing SPI bus for the wireless NRF module is **SPI2** (`hspi2`).
- The NRF24L01 Chip Select (CS) pin is connected to **PC6** (GPIOC, GPIO_PIN_6).

## Proposed Configuration
- New SPI device: **PMW3901**.
- Shared Bus: **SPI2**.
- Proposed PMW3901 CS pin: **PC4**.

## Feasibility
**High Feasibility.** It is completely standard to share the MOSI, MISO, and SCK signals of an SPI bus (SPI2) among multiple slave devices, as long as each has a distinct, independently controlled CS line. PC4 is an available GPIO that can be configured as a push-pull output to provide the CS signal for PMW3901.
We must ensure that:
1. NRF and PMW3901 are not selected at the same time (their respective CS lines must never be low simultaneously).
2. The SPI configuration (Baud Rate, CPOL, CPHA, etc.) matches the requirement for PMW3901, or is dynamically reconfigured before transactions if PMW3901 requires a different SPI mode than NRF24L01. 
*(PMW3901 often requires SPI Mode 3 `CPOL=1, CPHA=1`, while NRF24L01 works on SPI Mode 0 `CPOL=0, CPHA=0`. They can share the bus, but you might need to handle SPI mode switching when accessing different devices. However, STM32 HAL `hspi2` can be re-initialized if needed, or if both tolerate the same settings, it simplifies matters.)*
3. The PMW3901 driver (which currently uses Standard Peripheral Library) needs to be ported to STM32 HAL library, consistent with the rest of the project.

No hardware conflicts exist on PC4, and standard SPI operation inherently supports this arrangement.