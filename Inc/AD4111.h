
#ifndef STM32_AD4111_AD4111_H
#define STM32_AD4111_AD4111_H


#include "defines.h"
#include <stdbool.h>
#include <memory.h>
#include <stm32f1xx_hal.h>
#include <stm32f103xb.h>
#include "main.h"
#include "us_delay.h"
#define AD4111_SETUP_DELAY     5
#define AD4111_USED_CHANNELS   4
typedef struct{
    uint32_t data;
    uint8_t channel;
}AD4111_Reading_t;

void AD4111_Setup(SPI_HandleTypeDef* spi);
bool AD4111_CheckRegisters(SPI_HandleTypeDef* spi);
bool AD4111_CheckID(SPI_HandleTypeDef* spi);
void AD4111_WaitUntilRdy(void);
void AD4111_ResetDevice(SPI_HandleTypeDef* spi);
uint16_t AD4111_Read16BitRegister(SPI_HandleTypeDef* spi, uint8_t address);
AD4111_Reading_t AD4111_ReadData(SPI_HandleTypeDef *spi);
double AD4111_ConvertDoubleSided(uint32_t raw);
#endif //STM32_AD4111_AD4111_H
