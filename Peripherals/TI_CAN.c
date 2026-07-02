#include "driverlib.h"
#include "device.h"
#include "TI_CAN.h"
#include <stdint.h>
#include <math.h>
void CANB_MSG_INIT(uint8_t tx_mailbox_offset,
                   uint16_t tx_can_id_offset,
                   uint8_t number_tx,
                   uint8_t rx_mailbox_offset,
                   uint16_t rx_can_id_offset,
                   uint8_t number_rx)
{
    uint8_t i = 0;
    uint8_t mailbox = 0;
    uint16_t can_id = 0;

    // TX message objects
    for(i = 0; i < number_tx; i++)
    {
        mailbox = tx_mailbox_offset + i;
        can_id  = tx_can_id_offset + i;

        CAN_setupMessageObject(CANB_BASE,
                               mailbox,              // C2000 message object / mailbox number
                               can_id,               // CAN ID on the bus
                               CAN_MSG_FRAME_STD,
                               CAN_MSG_OBJ_TYPE_TX,
                               0,
                               CAN_MSG_OBJ_TX_INT_ENABLE,
                               CAN_TX_DLC);
    }

    // RX message objects
    for(i = 0; i < number_rx; i++)
    {
        mailbox = rx_mailbox_offset + i;
        can_id  = rx_can_id_offset + i;

        CAN_setupMessageObject(CANB_BASE,
                               mailbox,              // C2000 message object / mailbox number
                               can_id,               // CAN ID on the bus
                               CAN_MSG_FRAME_STD,
                               CAN_MSG_OBJ_TYPE_RX,
                               0,
                               CAN_MSG_OBJ_NO_FLAGS,
                               CAN_RX_DLC_DONTCARE);
    }
}


uint64_t CAN16x8_to_u64(uint16_t can[8])
{
    uint64_t value = 0;
    int i;

    for(i = 0; i < 8; i++)
    {
        value |= ((uint64_t)(can[i] & 0x00FF)) << (8 * i);
    }

    return value;
}

void u64_to_CAN16x8(uint64_t value, uint16_t can[8])
{
    int i;

    for(i = 0; i < 8; i++)
    {
        can[i] = (uint16_t)((value >> (8 * i)) & 0x00FF);
    }
}

uint64_t int16x4_to_u64(int16_t in[4])
{
    uint64_t value = 0;
    int i;

    for(i = 0; i < 4; i++)
    {
        value |= ((uint64_t)((uint16_t)in[i])) << (16 * i);
    }

    return value;
}

void u64_to_4x_int16(uint64_t value, int16_t out[4])
{
    int i;

    for(i = 0; i < 4; i++)
    {
        out[i] = (int16_t)((value >> (16 * i)) & 0xFFFF);
    }
}


uint64_t float3k_to_u64(float a, float b, float c, float k)
{
    uint64_t value = 0;
    float v;
    int16_t s16;
    uint16_t u16k;

    // ---- a scaled by k -> int16 ----
    v = roundf(a * k);
    if (v > 32767.0f)       v = 32767.0f;
    else if (v < -32768.0f) v = -32768.0f;
    s16 = (int16_t)v;
    value |= ((uint64_t)(uint16_t)s16) << 0;

    // ---- b scaled by k -> int16 ----
    v = roundf(b * k);
    if (v > 32767.0f)       v = 32767.0f;
    else if (v < -32768.0f) v = -32768.0f;
    s16 = (int16_t)v;
    value |= ((uint64_t)(uint16_t)s16) << 16;

    // ---- c scaled by k -> int16 ----
    v = roundf(c * k);
    if (v > 32767.0f)       v = 32767.0f;
    else if (v < -32768.0f) v = -32768.0f;
    s16 = (int16_t)v;
    value |= ((uint64_t)(uint16_t)s16) << 32;

    // ---- k itself (NOT scaled) -> uint16 ----
    v = roundf(k);
    if (v > 65535.0f)       v = 65535.0f;
    else if (v < 0.0f)      v = 0.0f;        // clamp negatives to 0 for uint16
    u16k = (uint16_t)v;
    value |= ((uint64_t)u16k) << 48;

    return value;
}

#include <stdint.h>

void u64_to_float3k(uint64_t packed, float *a, float *b, float *c)
{
    int16_t sa, sb, sc;
    uint16_t uk;
    float k;

    // Unpack
    sa = (int16_t)((packed >> 0)  & 0xFFFFu);
    sb = (int16_t)((packed >> 16) & 0xFFFFu);
    sc = (int16_t)((packed >> 32) & 0xFFFFu);
    uk = (uint16_t)((packed >> 48) & 0xFFFFu);

    // Avoid divide-by-zero
    if (uk == 0u) {
        if (a) *a = 0.0f;
        if (b) *b = 0.0f;
        if (c) *c = 0.0f;
        return;
    }

    k = (float)uk;

    // Reconstruct original floats
    if (a) *a = (float)sa / k;
    if (b) *b = (float)sb / k;
    if (c) *c = (float)sc / k;
}


uint64_t pack4_to_u64(int32_t a, int32_t b, int32_t c, int32_t d)
{
    uint64_t value = 0;

    value |= ((uint64_t)((uint16_t)a)) << 0;
    value |= ((uint64_t)((uint16_t)b)) << 16;
    value |= ((uint64_t)((uint16_t)c)) << 32;
    value |= ((uint64_t)((uint16_t)d)) << 48;

    return value;
}

void u64_to_4x_u32(uint64_t value,
                   uint32_t *w0,
                   uint32_t *w1,
                   uint32_t *w2,
                   uint32_t *w3)
{
    *w0 = (uint32_t)( value        & 0xFFFFULL);
    *w1 = (uint32_t)((value >> 16) & 0xFFFFULL);
    *w2 = (uint32_t)((value >> 32) & 0xFFFFULL);
    *w3 = (uint32_t)((value >> 48) & 0xFFFFULL);
}

uint16_t getStatus(uint64_t config, uint16_t bitIndex)
{
    return (uint16_t)((config >> bitIndex) & 1ULL);
}