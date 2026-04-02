#ifndef TI_CAN_H_
#define TI_CAN_H_

#include "driverlib.h"
#include "device.h"
#include <stdint.h>
#define CANB_BITRATE_HZ     500000UL
#define CAN_TX_DLC          8U
#define CAN_RX_DLC_DONTCARE 0U   // per TI example: "don't care" for RX mailbox

typedef enum
{
    STATUS_ON = 0,
    STATUS_OFF,
    CAN_CONTROL,
    CURRENT_MEASURED,
    TEMPERATURE_NTC,
    ATC_ACTIVE,
    LOW_TEMP_LIMIT_ACTIVE,
    DRIVE_CYCLE_ON,
    VGTH_MONITOR,
    BACKTOBACK,
    DIRECT_SWITCH_CONTROL,
    PULSE_SWITCH_ON,
    T1_ON,
    T2_ON,
    T3_ON,
    MODULATION

}Status;

void CANB_MSG_INIT(uint8_t start_tx, uint8_t end_tx, uint8_t start_rx, uint8_t end_rx);
uint64_t CAN16x8_to_u64(uint16_t can[8]);
void u64_to_CAN16x8(uint64_t value, uint16_t can[8]);
uint64_t int16x4_to_u64(int16_t in[4]);
void u64_to_4x_int16(uint64_t value, int16_t out[4]);
uint64_t float3k_to_u64(float a, float b, float c, float k);
void u64_to_float3k(uint64_t packed, float *a, float *b, float *c);
uint64_t pack4_to_u64(int32_t a, int32_t b, int32_t c, int32_t d);
void u64_to_4x_u32(uint64_t value,uint32_t *w0,uint32_t *w1,uint32_t *w2,uint32_t *w3);
uint16_t getStatus(uint64_t config, uint16_t bitIndex);
#endif
