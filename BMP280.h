#include <stdint.h>
#include <stdbool.h>
#include "BMP280_config.h"



#ifndef BMP280_H_
#define BMP280_H_



#define BMP280_CONTROL_MEASURE_REGISTER 0xF4
#define BMP280_CONFIG_REGISTER 0xF5

#define BMP280_TEMPERATURE_REGISTER 0xFA
#define BMP280_PRESSURE_REGISTER 0xF7
#define BMP280_TRIMMING_REGISTER 0x88

#define BMP280_STATUS_REGISTER 0xF3
#define BMP280_RESET_REGISTER 0xE0

#if BMP280_USE_I2C == BMP280_USE_SPI
#error "Choose one interface"
#endif

typedef enum {
    skippedValue = 0b0,
    oversampling_1x = 0b001,
    oversampling_2x = 0b010,
    oversampling_4x = 0b011,
    oversampling_8x = 0b100,
    oversampling_16x = 0b101,

} BMP280_Oversampling;

typedef enum {
    filter_off = 0,
    filter_2x = 0b001,
    filter_4x = 0b010,
    filter_8x = 0b011,
    filter_16x = 0b100
} BMP280_IIRFilter;

typedef enum {
    BMP280_dataReady = 0,
    BMP280_imUpdate = 0b1,
    BMP280_measuring = 0b1000
} BMP280_Status;

typedef enum {
    BMP280_standby_0_5ms = 0b000,
    BMP280_standby_62_5ms = 0b001,
    BMP280_standby_125ms = 0b010,
    BMP280_standby_250ms = 0b011,
    BMP280_standby_500ms = 0b100,
    BMP280_standby_1s = 0b100,
    BMP280_standby_2s = 0b110,
    BMP280_standby_4s = 0b111
} BMP280_StandbyTime;

#define BMP280_SKIPPED_VALUE 0b0
#define BMP280_OVERSAMPLING_1X 0b001
#define BMP280_OVERSAMPLING_2X 0b010
#define BMP280_OVERSAMPLING_4X 0b011
#define BMP280_OVERSAMPLING_8X 0b100
#define BMP280_OVERSAMPLING_16X 0b101

#define BMP280_IIR_FILTER_OFF 0b0
#define BMP280_IIR_FILTER_2X 0b001
#define BMP280_IIR_FILTER_4X 0b010
#define BMP280_IIR_FILTER_8X 0b011
#define BMP280_IIR_FILTER_16X 0b100

#define BMP280_SLEEP_MODE 0b0
#define BMP280_FORCED_MODE 0b01
#define BMP280_NORMAL_MODE 0b11



#if BMP280_USE_I2C
typedef enum {
    BMP280_I2C_PrimaryAdress = 0x76,
    BMP280_I2C_SecondaryAdress = 0x77
} BMP280_I2C_AdressSelection;

typedef enum {
    BMP280_I2C_StandartSpeed = 0x01UL,
    BMP280_I2C_FastSpeed = 0x02UL,
    BMP280_I2C_FastPlusSpeed = 0x03UL,
    BMP280_I2C_HighSpeed = 0x04UL
} BMP280_I2C_SpeedLevel;

#define BMP280_PRIMARY_ADRESS 0x76
#define BMP280_SECONDARY_ADRESS 0x77
#endif

#define BMP280_INTERFACE_CONFIG_ERROR -1
#define BMP280_DATA_LOST_ERROR -2
#define BMP280_DATA_NOT_READY -3
#define BMP280_OTHER_ERROR -4

typedef struct {
    //settings
    BMP280_Oversampling temperatureOversampling;
    BMP280_Oversampling pressureOversampling;
    BMP280_IIRFilter IIRFilter;
    BMP280_StandbyTime standbyTime;

    //interface
    void* handler;

#if BMP280_USE_I2C
    BMP280_I2C_AdressSelection adress;
    BMP280_I2C_SpeedLevel I2C_speed;
#endif

#if BMP280_USE_SPI
    bool use_3wire_spi;
    int32_t SPI_speed;	
#endif
	
    //raw data
    uint8_t rawTemperature[3];
    uint8_t rawPressure[3];

    //processed data
    float temperature;
    float pressure;

    //trimming
    uint16_t trimmingTemp1;
    int16_t trimmingTemp2, trimmingTemp3;

    uint16_t trimmingPres1;
    int16_t trimmingOtherPres[8];

} BMP280;

//Data processing
void BMP280_CalculatePressure(BMP280* instance);
void BMP280_CalculateTemperature(BMP280* instance);

//Control
void BMP280_SetSleepMode(BMP280* instance);
void BMP280_SetForcedMode(BMP280* instance);
void BMP280_SetNormalMode(BMP280* instance);

BMP280_Status BMP280_GetStatus(BMP280* instance);
void BMP280_Reset(BMP280* instance);

//Raw data funcs
int8_t BMP280_GetAllRawData(BMP280* instance);
int8_t BMP280_GetRawPressure(BMP280* instance);
int8_t BMP280_GetRawTemperature(BMP280* instance);

//Get Processed data
void BMP280_GetPressure(BMP280* instance);
void BMP280_GetTemperature(BMP280* instance);
void BMP280_GetAllData(BMP280* instance);

//Initialization funcs
int8_t BMP280_Initialize(BMP280* instance, uint8_t startingMode);
void BMP280_Deinitialize(BMP280* instance);



#endif
