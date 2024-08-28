
#include "BMP280_config.h"
#include "BMP280_intf.h"
#include "BMP280.h"


#if BMP280_USE_I2C
#include "Driver_I2C.h"

static volatile uint32_t BMP280_I2C_Event;
void BMP280_I2C_Callback (uint32_t event) {
  BMP280_I2C_Event |= event;
}

#endif

#if BMP280_USE_SPI
#include "Driver_SPI.h"

static volatile uint32_t BMP280_SPI_Event;
void BMP280_SPI_Callback(uint32_t event)
{
	BMP280_SPI_Event |= event;
}

#endif


int8_t BMP280_GetDataFromRegisters(BMP280* instance, uint8_t reg, uint8_t*data, uint8_t count) {
#if BMP280_USE_I2C

    ARM_DRIVER_I2C *handler = (ARM_DRIVER_I2C*)instance->handler;

    handler->Initialize(BMP280_I2C_Callback);
    handler->PowerControl(ARM_POWER_FULL);
    handler->Control(ARM_I2C_BUS_SPEED, instance->I2C_speed);
    handler->Control(ARM_I2C_BUS_CLEAR, 0);

		BMP280_I2C_Event = 0U;
	
    handler->MasterTransmit(instance->adress, &reg, 1, 1);
    while ((BMP280_I2C_Event & ARM_I2C_EVENT_TRANSFER_DONE) == 0U);
		if ((BMP280_I2C_Event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) != 0U){
			handler->PowerControl(ARM_POWER_OFF);
			handler->Uninitialize();
			return BMP280_DATA_LOST_ERROR;
		}
		BMP280_I2C_Event = 0U;
	
    handler->MasterReceive(instance->adress, data, count, 0);
    while ((BMP280_I2C_Event & ARM_I2C_EVENT_TRANSFER_DONE) == 0U);
		if ((BMP280_I2C_Event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) != 0U){
			handler->PowerControl(ARM_POWER_OFF);
			handler->Uninitialize();
			return BMP280_DATA_LOST_ERROR;
		}
		BMP280_I2C_Event = 0U;

    handler->PowerControl(ARM_POWER_OFF);
    handler->Uninitialize();
#endif


#if BMP280_USE_SPI
    ARM_DRIVER_SPI *handler = (ARM_DRIVER_SPI*)instance->handler;
    handler->Initialize(BMP280_SPI_Callback);
    handler->PowerControl(ARM_POWER_FULL);
		
		int32_t speed = instance->SPI_speed;
		if(instance->SPI_speed == 0){
			speed = BMP280_DEFAULT_SPI_SPEED;
		}
		
    uint8_t a = handler->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_MSB_LSB | ARM_SPI_SS_MASTER_HW_OUTPUT | ARM_SPI_DATA_BITS(8), instance->SPI_speed);
		if(a != 0){
			return BMP280_INTERFACE_CONFIG_ERROR;
		}
		handler->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
		
		
		handler->Send(&reg, 1);
		while((BMP280_SPI_Event & ARM_SPI_EVENT_TRANSFER_COMPLETE) == 0U);
		if((BMP280_SPI_Event & ARM_SPI_EVENT_DATA_LOST) == 0b10){
			handler->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
			handler->PowerControl(ARM_POWER_OFF);
			handler->Uninitialize();
			BMP280_SPI_Event = 0;
			return BMP280_DATA_LOST_ERROR;
		}
		BMP280_SPI_Event = 0;
		
		handler->Receive(data, count);
		while((BMP280_SPI_Event & ARM_SPI_EVENT_TRANSFER_COMPLETE) == 0U);
		if((BMP280_SPI_Event & ARM_SPI_EVENT_DATA_LOST) == 0b10){
			handler->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
			handler->PowerControl(ARM_POWER_OFF);
			handler->Uninitialize();
			BMP280_SPI_Event = 0;
			return BMP280_DATA_LOST_ERROR;
		}
		BMP280_SPI_Event = 0;

		handler->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
		handler->PowerControl(ARM_POWER_OFF);
		handler->Uninitialize();
    
    
#endif
    return 0;
}

int8_t BMP280_SetDataToRegister(BMP280* instance, uint8_t reg, uint8_t data) {
#if BMP280_USE_I2C
    uint8_t pack[2] = {reg, data};
    ARM_DRIVER_I2C *handler = (ARM_DRIVER_I2C*)instance->handler;
    handler->Initialize(BMP280_I2C_Callback);
    handler->PowerControl(ARM_POWER_FULL);
		
    handler->Control(ARM_I2C_BUS_SPEED, instance->I2C_speed);
    handler->Control(ARM_I2C_BUS_CLEAR, 0);

		BMP280_I2C_Event = 0U;
		
    handler->MasterTransmit(instance->adress, pack, 2, 0);
		while ((BMP280_I2C_Event & ARM_I2C_EVENT_TRANSFER_DONE) == 0U);
		if ((BMP280_I2C_Event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) != 0U){
			handler->PowerControl(ARM_POWER_OFF);
			handler->Uninitialize();
			return BMP280_DATA_LOST_ERROR;
		}
		BMP280_I2C_Event = 0U;

    handler->PowerControl(ARM_POWER_OFF);
    handler->Uninitialize();
#endif
#if BMP280_USE_SPI
    ARM_DRIVER_SPI *handler = (ARM_DRIVER_SPI*)instance->handler;
    handler->Initialize(BMP280_SPI_Callback);
    handler->PowerControl(ARM_POWER_FULL);
		
		int32_t speed = instance->SPI_speed;
		
		if(instance->SPI_speed == 0){
			speed = BMP280_DEFAULT_SPI_SPEED;
		}
		
		uint8_t a = handler->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_MSB_LSB | ARM_SPI_SS_MASTER_HW_OUTPUT | ARM_SPI_DATA_BITS(8), speed);
		if(a != 0){
			return BMP280_INTERFACE_CONFIG_ERROR;
		}
		handler->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
		
		uint8_t reg2 = reg & 0x7F;
		uint8_t pack[2] = {reg2, data};
			
		handler->Send(pack, 2);
		while((BMP280_SPI_Event & ARM_SPI_EVENT_TRANSFER_COMPLETE) == 0U);
		if((BMP280_SPI_Event & ARM_SPI_EVENT_DATA_LOST) == 0b10){
			handler->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
			handler->PowerControl(ARM_POWER_OFF);
			handler->Uninitialize();
			BMP280_SPI_Event = 0;
			return BMP280_DATA_LOST_ERROR;
		}
		BMP280_SPI_Event = 0;
		
		handler->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
		handler->PowerControl(ARM_POWER_OFF);
		handler->Uninitialize();
		
#endif
return 0;
}
