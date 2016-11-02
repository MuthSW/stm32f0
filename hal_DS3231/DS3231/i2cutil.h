#ifndef __I2C_H
#define __I2C_H

void I2C_WriteBuffer(I2C_HandleTypeDef hi, uint8_t DEV_ADDR, uint8_t sizebuf);
void I2C_ReadBuffer(I2C_HandleTypeDef hi, uint8_t DEV_ADDR, uint8_t sizebuf);

uint8_t I2C_Write_Byte(I2C_HandleTypeDef hi,uint8_t DEV_ADDR,uint8_t addr, uint8_t data);
uint8_t I2C_Read_Byte(I2C_HandleTypeDef hi,uint8_t DEV_ADDR,uint8_t addr);

#endif /* __I2C_H */

