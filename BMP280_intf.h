#include "BMP280_config.h"
#include "BMP280.h"
#include <stdint.h>

#ifndef BMP280_INTF_H
#define BMP280_INTF_H

int8_t BMP280_GetDataFromRegisters(BMP280* instance, uint8_t reg, uint8_t*data, uint8_t count);
int8_t BMP280_SetDataToRegister(BMP280* instance, uint8_t reg, uint8_t data);

#endif