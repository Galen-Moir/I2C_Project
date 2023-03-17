#ifndef i2c_H
#define i2c_H


#define I2C_M_START I2C1->C1 |= I2C_C1_MST_MASK
#define I2C_M_STOP   I2C1->C1 &= ~I2C_C1_MST_MASK
#define I2C_M_RSTART I2C1->C1 |= I2C_C1_RSTA_MASK

#define I2C_TRAN I2C1->C1 |= I2C_C1_TX_MASK
#define I2C_REC I2C1->C1 &= ~I2C_C1_TX_MASK

#define BUSY_ACK    while(I2C1->S & 0x01)
#define TRANS_COMP while(!(I2C1->S & 0x80))
	
/*
#define I2C_WAIT			while((I2C1->S & I2C_S_IICIF_MASK)==0) {} \
                                 I2C1->S |= I2C_S_IICIF_MASK;
*/

#define I2C_WAIT			if ((I2C1->C1 & I2C_C1_TX_MASK)==0x10){while( ((I2C1->S & I2C_S_IICIF_MASK)==0) || ((I2C1->S & I2C_S_RXAK_MASK)==0x01) ){} }\
                      if ((I2C1->C1 & I2C_C1_TX_MASK)==0x00){while( ((I2C1->S & I2C_S_IICIF_MASK)==0) ) {} }\
											I2C1->S |= I2C_S_IICIF_MASK;
	

#define NACK        I2C1->C1 |= I2C_C1_TXAK_MASK
#define ACK           I2C1->C1 &= ~I2C_C1_TXAK_MASK

	
void i2c_init(void);

void i2c_start(void);
void i2c_read_setup(uint8_t dev, uint8_t address);
uint8_t i2c_repeated_read(uint8_t);

uint8_t i2c_read_byte(uint8_t addr);
uint8_t i2c_read_byte_intReg(uint8_t dev, uint8_t address);
void i2c_write_byte_intReg(uint8_t dev, uint8_t address, uint8_t data);
void i2c_write_byte(uint8_t dev, uint8_t data);

#endif
