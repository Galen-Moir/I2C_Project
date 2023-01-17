#ifndef IO_expander_H
#define IO_expander_H

#define IOEX_ADDR   (0x26)
#define IOEX_ADDR_W (0x4C)
#define IOEX_ADDR_R (0x4D)

#define INT_PIN_POS (1)
#define GREEN_POS (0x01)
#define YELLOW_POS (0x02)
#define RED_POS (0x04)
#define LIGHT_SENSOR_INT_POS (0x80)
#define BUTTON_INT_POS (0x40)

typedef unsigned          char uint8_t;

extern volatile uint8_t IOdata;

uint8_t count (uint8_t x);
uint8_t IO_READ_1Byte(uint8_t addr,volatile uint8_t x);
void INT_IO_init (void);
void PORTA_IRQHandler(void);

#endif
