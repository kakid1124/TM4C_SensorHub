#include <stdint.h>

extern "C" {
	
void I2C1_Init(void);
void I2C3_Init(void);
void I2C3_Init_8MHz(void);
	
uint32_t I2C1_Write_2Bytes(uint8_t device_address, uint8_t device_register, uint16_t device_data);
uint16_t I2C1_Read_2Bytes(uint8_t device_address, uint8_t device_register);
	
void I2C3_Write_Byte(uint8_t device_address, uint8_t device_register, uint8_t data);
uint8_t I2C3_Read_Byte(uint8_t device_address, uint8_t device_register);
void I2C3_Read_Bytes(uint8_t device_address, uint8_t device_register, uint8_t length, uint8_t * destination);

}
