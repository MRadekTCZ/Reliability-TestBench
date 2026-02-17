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
