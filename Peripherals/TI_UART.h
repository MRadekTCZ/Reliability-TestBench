#ifndef TI_UART_H_
#define TI_UART_H_

#include "driverlib.h"
#include "device.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TI_UART_SCIB_BASE       SCIB_BASE
#define TI_UART_SCIB_BAUDRATE   115200U

void TI_UART_SCIB_GPIO_Init(void);
void TI_UART_Init(uint32_t base, uint32_t baudrate);

void TI_UART_Send(uint16_t frame_amount, uint16_t *data);
bool TI_UART_Read(uint16_t frame_amount, uint16_t *data);

#ifdef __cplusplus
}
#endif

#endif /* TI_UART_H_ */