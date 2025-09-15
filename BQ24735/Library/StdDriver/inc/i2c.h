/**
 ******************************************************************************
 *
 * @author     Bacancy Systems LLP 
 * @date       13, September, 2021
 * @File_name: i2c.h
 ******************************************************************************
 */
 
#ifndef _I2C_H_
#define _I2C_H_

#include "Function_define_MS51_16K.h"
/****************************************************************************
 * MACROS
 ***************************************************************************/
#define I2C0    0
#define I2C1    1

#define      I2C_R    1
#define      I2C_W    0
#define      I2C_NACK  1
#define      ONE_BYTE  8
/****************************************************************************
 * FUNCTIONS PROTOTYPES
 ****************************************************************************/
void I2C_Error(void);
void Init_I2C(void);
void I2C_Write_Process(UINT8 u8DAT);
void I2C_start(void);
void I2C_stop(void);
unsigned char I2C_read(unsigned char ack_mode);
void I2C_write(unsigned char value);
void I2C_Read_Process(UINT8 u8DAT);
unsigned char BQ24735_read(unsigned char address);
void BQ24735_write(unsigned char address, unsigned int value);
#endif