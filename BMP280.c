#include "BMP280.h"
#include "BMP280_intf.h"
#include "BMP280_config.h"


#if BMP280_USE_SINGLE_PRECISION_ARIPH
#define real float
#else
#define real double
#endif

void BMP280_CalculateTemperature(BMP280* instance) {
    real var1, var2;

    int32_t rawTemp32;
    rawTemp32 = ((int32_t)instance->rawTemperature[0] << 12) + ((int32_t)instance->rawTemperature[1] << 4) + ((int32_t)instance->rawTemperature[2] >> 4);
    var1 = (((real)rawTemp32)/16384.0 - ((real)instance->trimmingTemp1)/1024.0)*((real)instance->trimmingTemp2);
    var2 = ((((real)rawTemp32)/131072.0 - ((real)instance->trimmingTemp1)/8192.0)*(((real)rawTemp32)/131072.0-((real)instance->trimmingTemp1)/8192.0))*((real)instance->trimmingTemp3);

    real temp = (var1 + var2)/5120.0;
    instance->temperature = temp;
}

void BMP280_CalculatePressure(BMP280* instance) {
    real var1, var2, p;

    int32_t rawPressure32 = ((int32_t)instance->rawPressure[0] << 12) + ((int32_t)instance->rawPressure[1] << 4) + ((int32_t)instance->rawPressure[2] >> 4);
    var1 = (instance->temperature*2060.0) - 64000.0;
    var2 = var1*var1*((real)instance->trimmingOtherPres[4])/32786.0;
    var2 = var2 + var1 * ((real)instance->trimmingOtherPres[3])*2.0;
    var2 = (var2/4.0)+(((real)instance->trimmingOtherPres[2]) * 65536.0);
    var1 = (((real)instance->trimmingOtherPres[1]) * var1 * var1/524288.0 + ((real)instance->trimmingOtherPres[0])*var1)/524288.0;
    var1 = (1.0 + var1/32768.0)*((real)instance->trimmingPres1);
    p = 1048576.0 - (real)rawPressure32;
    p = (p - (var2/4096.0))*6250.0/var1;
    var1 = ((real)instance->trimmingOtherPres[7])* p * p / 2147483648;
    var2 = p*((real)instance->trimmingOtherPres[6])/32768.0;
    p = p + (var1 + var2 + ((real)instance->trimmingOtherPres[5]))/16.0;
    instance->pressure = p;
}


int8_t BMP280_GetRawPressure(BMP280* instance) {
		#if BMP280_CHECK_STATUS_BEFORE_READ 
		BMP280_Status a = BMP280_GetStatus(instance);
		if(a != BMP280_dataReady){
			return BMP280_DATA_NOT_READY;
		}
		#endif
    BMP280_GetDataFromRegisters(instance, BMP280_PRESSURE_REGISTER, instance->rawPressure, 3);
		return 0;
}
int8_t BMP280_GetRawTemperature(BMP280* instance) {
    BMP280_Status a = BMP280_GetStatus(instance);
		if(a != BMP280_dataReady){
			return BMP280_DATA_NOT_READY;
		}
    BMP280_GetDataFromRegisters(instance, BMP280_PRESSURE_REGISTER, instance->rawPressure, 3);
		return 0;
}

int8_t BMP280_GetAllRawData(BMP280* instance) {
    uint8_t data[6];
    BMP280_Status a = BMP280_GetStatus(instance);
		if(a != BMP280_dataReady){
			return BMP280_DATA_NOT_READY;
		}
    BMP280_GetDataFromRegisters(instance, BMP280_PRESSURE_REGISTER, data, 6);

    for(int i = 0; i < 3; i++) {
        instance->rawPressure[i] = data[i];
        instance->rawTemperature[i] = data[i+3];
    }

}

int8_t BMP280_Initialize(BMP280* instance, uint8_t startingMode) {
    uint8_t controlMeasureValue = (instance->temperatureOversampling << 5) | (instance->pressureOversampling << 2) | startingMode;
    uint8_t configValue = (instance->standbyTime << 5) | (instance->IIRFilter << 2);
    uint8_t trimmingParams[24];
#if BMP280_USE_SPI
    configValue |= instance->use_3wire_spi;
#endif
	
		int8_t a = BMP280_SetDataToRegister(instance, BMP280_CONFIG_REGISTER, configValue);
		while(a != 0){
			if(a == BMP280_INTERFACE_CONFIG_ERROR) return BMP280_INTERFACE_CONFIG_ERROR;
			else a = BMP280_SetDataToRegister(instance, BMP280_CONFIG_REGISTER, configValue);
		}
		a = BMP280_SetDataToRegister(instance, BMP280_CONTROL_MEASURE_REGISTER, controlMeasureValue);
		while(a != 0){
			if(a == BMP280_INTERFACE_CONFIG_ERROR) return BMP280_INTERFACE_CONFIG_ERROR;
			else a = BMP280_SetDataToRegister(instance, BMP280_CONTROL_MEASURE_REGISTER, controlMeasureValue);
		}
		a = BMP280_GetDataFromRegisters(instance, BMP280_TRIMMING_REGISTER, trimmingParams, 24);
		while(a != 0){
			if(a == BMP280_INTERFACE_CONFIG_ERROR) return BMP280_INTERFACE_CONFIG_ERROR;
			else a = BMP280_GetDataFromRegisters(instance, BMP280_TRIMMING_REGISTER, trimmingParams, 24);
		}
    instance->trimmingTemp1 = (trimmingParams[1] << 8) | trimmingParams[0];
    instance->trimmingTemp2 = (trimmingParams[3] << 8) | trimmingParams[2];
    instance->trimmingTemp3 = (trimmingParams[5] << 8) | trimmingParams[4];

    instance->trimmingPres1 = (trimmingParams[7] << 8) | trimmingParams[6];
    for(int i = 8; i < 24; i += 2) {
        instance->trimmingOtherPres[(i-8)/2] = (trimmingParams[i+1] << 8) | trimmingParams[i];
    }
		
		return 0;

}
void BMP280_Deinitialize(BMP280* instance) {
    //uint8_t controlMeasureValue = BMP280_SLEEP_MODE;
    //BMP280_SetDataToRegister(instance, BMP280_CONTROL_MEASURE_REGISTER, controlMeasureValue);
    BMP280_Reset(instance);

    instance->trimmingTemp1 = 0;
    instance->trimmingTemp2 = 0;
    instance->trimmingTemp3 = 0;
    instance->trimmingPres1 = 0;

    for(int i = 0; i < 8; i++) {
        instance->trimmingOtherPres[i] = 0;
    }
}
void BMP280_SetSleepMode(BMP280* instance) {
    uint8_t controlMeasureValue = (instance->temperatureOversampling << 5) | (instance->pressureOversampling << 2) | BMP280_SLEEP_MODE;
    BMP280_SetDataToRegister(instance, BMP280_CONTROL_MEASURE_REGISTER, controlMeasureValue);
}
void BMP280_SetForcedMode(BMP280* instance) {
    uint8_t controlMeasureValue = (instance->temperatureOversampling << 5) | (instance->pressureOversampling << 2) | BMP280_FORCED_MODE;
    BMP280_SetDataToRegister(instance, BMP280_CONTROL_MEASURE_REGISTER, controlMeasureValue);
}
void BMP280_SetNormalMode(BMP280* instance) {
    uint8_t controlMeasureValue = (instance->temperatureOversampling << 5) | (instance->pressureOversampling << 2) | BMP280_NORMAL_MODE;
    BMP280_SetDataToRegister(instance, BMP280_CONTROL_MEASURE_REGISTER, controlMeasureValue);
}

BMP280_Status BMP280_GetStatus(BMP280* instance) {
    uint8_t data;
    BMP280_GetDataFromRegisters(instance, 0xF3, &data, 1);

    switch (data&0b1001) {
    case 0b1:
        return BMP280_imUpdate;
    case 0b1000:
        return BMP280_measuring;
    default:
        return BMP280_dataReady;
    }
}
void BMP280_Reset(BMP280* instance) {
    BMP280_SetDataToRegister(instance, BMP280_RESET_REGISTER, 0xB6);
}

void BMP280_GetPressure(BMP280* instance) {
    BMP280_GetRawPressure(instance);
    BMP280_CalculatePressure(instance);
}
void BMP280_GetTemperature(BMP280* instance) {
    BMP280_GetRawTemperature(instance);
    BMP280_CalculateTemperature(instance);
}
void BMP280_GetAllData(BMP280* instance) {
    BMP280_GetAllRawData(instance);
    BMP280_CalculateTemperature(instance);
    BMP280_CalculatePressure(instance);
}