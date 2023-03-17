#ifndef I2C_LCD_H
#define I2C_LCD_H


#define LCD_ADDR   (0x27)
#define LCD_ADDR_W (0x4E)
#define LCD_ADDR_R (0x4F)

#define QMC5883L_DEVICE_ADDRESS   (0x0D)
#define QMC5883L_READ_ADDR        (0x1B)
#define QMC5883L_WRITE_ADDR       (0x1A)
#define IMU_ADDR_W (0xD0)
#define IMU_ADDR_R (0xD1)
#define LCD_ADDR   (0x27)
#define LCD_ADDR_W (0x4E)
#define LCD_ADDR_R (0x4F)


#define LcdTimeDelay (0.05)

typedef unsigned          char uint8_t;
#define MASK(x) (1UL << (x))

#define SCL_POS  (10) 
#define SDA_POS  (11)

void i2c_prestart_LCD(void);
void init_I2C_LCD(uint8_t addr);
void wait_read_BF(void);
void LCD_Write_4bit_CMD(uint8_t addr, uint8_t c);
void LCD_Write_8bit_CMD(uint8_t addr,uint8_t c);
void LCD_Write_8bit(uint8_t addr, uint8_t c);
void Set_Cursor(uint8_t addr, uint8_t column, uint8_t row);
void Clear_LCD(uint8_t addr);
void lcd_putchar(uint8_t addr, char c);
void Print_I2C_LCD_String(uint8_t addr,char *string);
void Print_I2C_LCD_IO_READ(uint8_t addr);
void Monitor_LCD(unsigned int x);

#endif
