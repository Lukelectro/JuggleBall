#include "adxl_i2c.h"
// spiekbriefje: I2C_CR2 |= (NBYTES 23 downto 16)|(SLADR) ;// BIT25=AutoEnd, Bit14 = Manualy generate a STOP, bit13 = generate a START BIT10=R/!W
// Slave adres on 7:1 with bit 0 don't care (For 7 bit adr. slaves such as MPU6050/adxl345)

int i2c_read_byte(int addr) { 
    I2C1_CR2 = (BIT13)|(1<<16)|(I2C_ADR); // Write 1 byte to I2C_ADR and sent start
    I2C1_TXDR = addr;
    while (!(I2C1_ISR & BIT0)); // wait for TX empty before changing CR2 and sending next byte

    I2C1_CR2 = BIT10 | BIT13 | (1<<16) | I2C_ADR | BIT25; // read (BIT10) one byte (1<<16) from I2C_ADR, generate start (BIT13), and generate stop when done (BIT25)
    
    while (!(I2C1_ISR & BIT2)); // wait till data in receive buffer 
    return I2C1_RXDR;
    
}

void i2c_read_n_bytes(int addr, int n, int* buff) { 
    I2C1_CR2 = (BIT13)|(1<<16)|(I2C_ADR); // Write 1 byte to I2C_ADR and sent start
    I2C1_TXDR = addr;
    while (!(I2C1_ISR & BIT0)); // wait for TX empty before changing CR2 and sending next byte

    I2C1_CR2 = BIT10 | BIT13 | ((n&0xFF)<<16) | I2C_ADR | BIT25; // read (BIT10) n byte (<<16) from I2C_ADR, generate start (BIT13), and generate stop when done (BIT25)
    
    
    for(int i=0;i<n;i++){
    	while (!(I2C1_ISR & BIT2)); // wait till data in receive buffer 
    	buff[i]=I2C1_RXDR;
    }
}

void i2c_write_byte(int addr, int data) { 
    I2C1_CR2 = (BIT13 | (2<<16) | I2C_ADR | BIT25);
  
    I2C1_TXDR = addr;
    while (!(I2C1_ISR & BIT0));

    I2C1_TXDR = data;
    while (!(I2C1_ISR & BIT0));
}

