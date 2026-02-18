#include "driverlib.h"
#include "device.h"
#include "TI_CAN.h"
#include <stdint.h>

void CANB_MSG_INIT(uint8_t start_tx, uint8_t end_tx, uint8_t start_rx, uint8_t end_rx){
    
    uint8_t msg_indx = 0;
    for(msg_indx = start_tx; msg_indx <= end_tx; msg_indx++){
    CAN_setupMessageObject(CANB_BASE, msg_indx, msg_indx,
                       CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                       CAN_MSG_OBJ_TX_INT_ENABLE, CAN_TX_DLC);
    }
    msg_indx = 0;   
    // RX message object (interrupt on reception)
    for(msg_indx = start_rx; msg_indx <= end_rx; msg_indx++){
    CAN_setupMessageObject(CANB_BASE, msg_indx, msg_indx,
                       CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                       CAN_MSG_OBJ_NO_FLAGS, CAN_RX_DLC_DONTCARE);
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

#include <stdint.h>
#include <math.h>

uint64_t float4_to_u64_k(float a, float b, float c, float d, float k)
{
    uint64_t value = 0;
    float v;
    int16_t s;

    // ---- a ----
    v = roundf(a * k);
    if (v > 32767.0f)      v = 32767.0f;
    else if (v < -32768.0f) v = -32768.0f;
    s = (int16_t)v;
    value |= ((uint64_t)(uint16_t)s) << 0;

    // ---- b ----
    v = roundf(b * k);
    if (v > 32767.0f)      v = 32767.0f;
    else if (v < -32768.0f) v = -32768.0f;
    s = (int16_t)v;
    value |= ((uint64_t)(uint16_t)s) << 16;

    // ---- c ----
    v = roundf(c * k);
    if (v > 32767.0f)      v = 32767.0f;
    else if (v < -32768.0f) v = -32768.0f;
    s = (int16_t)v;
    value |= ((uint64_t)(uint16_t)s) << 32;

    // ---- d ----
    v = roundf(d * k);
    if (v > 32767.0f)      v = 32767.0f;
    else if (v < -32768.0f) v = -32768.0f;
    s = (int16_t)v;
    value |= ((uint64_t)(uint16_t)s) << 48;

    return value;
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