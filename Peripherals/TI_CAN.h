#ifndef TI_CAN_H_
#define TI_CAN_H_

#include "driverlib.h"
#include "device.h"
#include <stdint.h>
#define CANB_BITRATE_HZ     500000UL
#define CAN_TX_DLC          8U
#define CAN_RX_DLC_DONTCARE 0U   // per TI example: "don't care" for RX mailbox

#define TX1_MSG_OBJ_ID      2U
#define TX2_MSG_OBJ_ID      3U
#define RX_MSG_OBJ_ID       1U

#define TX1_MSG_ID          2
#define TX2_MSG_ID          3
#define RX_MSG_ID           1


void CANB_MSG_INIT(uint8_t start_tx, uint8_t end_tx, uint8_t start_rx, uint8_t end_rx);
uint64_t CAN16x8_to_u64(uint16_t can[8]);
void u64_to_CAN16x8(uint64_t value, uint16_t can[8]);
#endif
