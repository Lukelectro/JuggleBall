#ifndef __I2CADXL
#define __I2CADXL
#include "stm32f030xx.h"
#define I2C_ADR 0xA6 // adxl 345 alternate adres/sdo low (0x53+rw dus 0xA6/7)
int i2c_read_byte(int);
void i2c_read_n_bytes(int, int, int* );
void i2c_write_byte(int, int);
#endif
