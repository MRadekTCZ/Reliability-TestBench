#ifndef TI_SPI_H_
#define TI_SPI_H_

#include "driverlib.h"
#include "device.h"
#include <stdint.h>
#include "UCC5870/hvp045a_io.h"

#define  SPIFSI_CLK_SPEED    4000000U

void configureSPI(uint32_t base);
void configureSPI_GPIO(void);
void GD_Init_LED_blink(uint16_t imax);

#endif