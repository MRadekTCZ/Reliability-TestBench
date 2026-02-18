#ifndef TI_CAN_H_
#define TI_CAN_H_

#include "driverlib.h"
#include "device.h"
#include <stdint.h>
#define CANB_BITRATE_HZ     500000UL
#define CAN_TX_DLC          8U
#define CAN_RX_DLC_DONTCARE 0U   // per TI example: "don't care" for RX mailbox



void CANB_MSG_INIT(uint8_t start_tx, uint8_t end_tx, uint8_t start_rx, uint8_t end_rx);
uint64_t CAN16x8_to_u64(uint16_t can[8]);
void u64_to_CAN16x8(uint64_t value, uint16_t can[8]);
uint64_t int16x4_to_u64(int16_t in[4]);
void u64_to_4x_int16(uint64_t value, int16_t out[4]);
uint64_t float4_to_u64_k(float a, float b, float c, float d, float k);
uint64_t pack4_to_u64(int32_t a, int32_t b, int32_t c, int32_t d);
void u64_to_4x_u32(uint64_t value,uint32_t *w0,uint32_t *w1,uint32_t *w2,uint32_t *w3);
#endif
