
#include "AD4111.h"

static uint8_t tx[16] = {0};
static uint8_t rx[16];
static str[16];
static double current = 0;

static void reset_buffer(void) {
    memset(tx, 0, 16);
    memset(rx, 0, 16);
}

static uint32_t deserialize_24bit(uint8_t *buffer) {
    uint32_t val = 0;
    val |= (uint32_t) buffer[0] << 16;
    val |= (uint32_t) buffer[1] << 8;
    val |= (uint32_t) buffer[2];
    return val;
}

static uint8_t get_channel_number(uint8_t status){
    return (uint8_t)(status & 0x0F);
}

double AD4111_ConvertDoubleSided(uint32_t raw) {
    //  raw = 2^23 * ((I * R/ Vref) + 1)
    return ((double) raw / 8388608 - 1) * 2.5 / 50;
}

void AD4111_ResetDevice(SPI_HandleTypeDef* spi){
    reset_buffer();
    for (int i=0; i<8; i++){
        tx[i] = 0xFF;
    }
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, tx, 8, 100);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
}

bool AD4111_CheckID(SPI_HandleTypeDef* spi){
    uint8_t spi_tx[3] = {0};
    uint8_t spi_rx[3] = {0};
    spi_tx[0] = 0x07;
    spi_tx[0] |= 1 << 6;
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    us_Delay(50);
    HAL_SPI_TransmitReceive(spi, spi_tx, spi_rx, 3, 100);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
    return (spi_rx[1] == 0x30) && (spi_rx[2] == 0xDE);
}

uint16_t AD4111_Read16BitRegister(SPI_HandleTypeDef* spi, uint8_t address){
    reset_buffer();
    tx[0] = address;
    tx[0] |= 1 << 6;
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(spi, tx, rx, 3, 100);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
    uint16_t reg_value = rx[2];
    reg_value |= rx[1] << 8;
    return reg_value;
}

bool AD4111_CheckRegisters(SPI_HandleTypeDef* spi){
    if (AD4111_Read16BitRegister(spi, AD4111_ADCMODE_ADDR) != AD4111_ADCMODE_VAL){
        return false;
    }
    if (AD4111_Read16BitRegister(spi, AD4111_SETUPCON1_ADDR) != AD4111_SETUPCON1_VAL){
        return false;
    }
    if (AD4111_Read16BitRegister(spi, AD4111_CH0_ADDR) != AD4111_CH0_VAL){
        return false;
    }
    if (AD4111_Read16BitRegister(spi, AD4111_CH1_ADDR) != AD4111_CH1_VAL){
        return false;
    }
    if (AD4111_Read16BitRegister(spi, AD4111_CH2_ADDR) != AD4111_CH2_VAL){
        return false;
    }
    if (AD4111_Read16BitRegister(spi, AD4111_CH3_ADDR) != AD4111_CH3_VAL){
        return false;
    }
    return true;
}

void AD4111_Setup(SPI_HandleTypeDef *spi) {
    tx[0] = AD4111_ADCMODE_ADDR;
    tx[1] = AD4111_ADCMODE_VAL >> 8;
    tx[2] = (uint8_t) AD4111_ADCMODE_VAL;
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, tx, 3, 100);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
    HAL_Delay(AD4111_SETUP_DELAY);

    tx[0] = AD4111_IFMODE_ADDR;
    tx[1] = AD4111_IFMODE_VAL >> 8;
    tx[2] = (uint8_t) AD4111_IFMODE_VAL;
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, tx, 3, 100);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
    HAL_Delay(AD4111_SETUP_DELAY);

    tx[0] = AD4111_SETUPCON1_ADDR;
    tx[1] = AD4111_SETUPCON1_VAL >> 8;
    tx[2] = (uint8_t) AD4111_SETUPCON1_VAL;
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, tx, 3, 100);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
    HAL_Delay(AD4111_SETUP_DELAY);

    tx[0] = AD4111_CH0_ADDR;
    tx[1] = AD4111_CH0_VAL >> 8;
    tx[2] = (uint8_t) AD4111_CH0_VAL;
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, tx, 3, 100);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
    HAL_Delay(AD4111_SETUP_DELAY);

    tx[0] = AD4111_CH1_ADDR;
    tx[1] = AD4111_CH1_VAL >> 8;
    tx[2] = (uint8_t) AD4111_CH1_VAL;
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, tx, 3, 100);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

    tx[0] = AD4111_CH2_ADDR;
    tx[1] = AD4111_CH2_VAL >> 8;
    tx[2] = (uint8_t) AD4111_CH2_VAL;
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, tx, 3, 100);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
    HAL_Delay(AD4111_SETUP_DELAY);

    tx[0] = AD4111_CH3_ADDR;
    tx[1] = AD4111_CH3_VAL >> 8;
    tx[2] = (uint8_t) AD4111_CH3_VAL;
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, tx, 3, 100);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
    HAL_Delay(AD4111_SETUP_DELAY);
};

AD4111_Reading_t AD4111_ReadData(SPI_HandleTypeDef *spi) {
    AD4111_Reading_t reading;
    reset_buffer();
    tx[0] = 0x4;
    tx[0] |= 1 << 6;

    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    us_Delay(10);
    HAL_SPI_TransmitReceive(spi, tx, rx, 5, 100);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

    reading.data = deserialize_24bit(rx + 1);
    reading.channel = get_channel_number(rx[4]);  // status register
    return reading;
};

void AD4111_WaitUntilRdy(void) {
    int counter = 0;
    do {
        us_Delay(10);
        counter++;
        if (counter > 100) {
            break;
        }
    } while (HAL_GPIO_ReadPin(RDY_PORT, RDY_PIN) == GPIO_PIN_SET);
}