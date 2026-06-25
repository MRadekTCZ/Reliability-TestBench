#include "driverlib.h"
#include "device.h"
#include "Peripherals/TI_TIMER.h"
#include "Peripherals/TI_CAN.h"
#include "SourceCode/ClarkPark_Math.h"
#include "SourceCode/ATC.h"
#include <stdint.h>

//Model parameter defines
#define Tamb 22.0f
const float kpc = (1.0f/one_by_sqrt2/sqrt3);
const float inv_kpc = 1/(1.0f/one_by_sqrt2/sqrt3);
//GPIO
#define BLUE_LED    31
#define RED_LED    34
//Clocks
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
// Interrupt counters
uint32_t timerCOM = 2000; // Hz of timer 0 - adjustable in debugger
uint32_t timerALGO = 100;
uint32_t timerCOM_cnt = 0;
uint32_t timerALGO_cnt = 0;

//Automated test
uint8_t automated_test = 0;
uint32_t automated_test_counter = 0;

// Electric variables
threephase Current_est, Current_meas, U_ref, U_read;
float Udc_base = 70.0f;
float Udc_meas;
float gate_strenght = 1.0f;
float set_freq_kHz = 20.0f;
float Ug_set = 15.0f;
//Aging params
//ATC algo
AgingParam Tj, Rdson, Uth;
float Uth_base = 0.0f;
float Rdson_base = 0.0f;

//Temperature monitor
float Tj_est, Tj_NTC_based, T_NTC;
float Tj_ref, Tj_NoATC, fsw_khz;
float Tj_max_drivecycle, Tj_avg_drivecycle, Tj_start;
float PowerT_est;
//CAN
#define CAN_TX_OFFSET 25
#define CAN_MSG_TX_AMOUNT 3
#define CAN_RX_OFFSET 10
#define CAN_MSG_RX_AMOUNT 11
uint16_t can_msg_tx[CAN_MSG_TX_AMOUNT][8];
uint64_t can_data_tx[CAN_MSG_TX_AMOUNT];
uint16_t can_msg_rx[CAN_MSG_RX_AMOUNT][8];
uint64_t can_data_rx[CAN_MSG_RX_AMOUNT];
uint64_t config = 0x10005;
void main(void){
    // Device init (clock, PLL, watchdog config etc.)
    Device_init();

    // GPIO init (unlocks pins, sets default states)
    Device_initGPIO();

    // LED pin
    GPIO_setPadConfig(BLUE_LED, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(BLUE_LED, GPIO_DIR_MODE_OUT);
    GPIO_writePin(BLUE_LED, 0);
    GPIO_setPadConfig(RED_LED, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(RED_LED, GPIO_DIR_MODE_OUT);
    GPIO_writePin(RED_LED, 0);

    // CANB pins (your board define must match the board)
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXB);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXB);

    // Interrupt module + vector table (driverlib replacement for PIE init)
    Interrupt_initModule();
    Interrupt_initVectorTable();

    Interrupt_register(INT_TIMER0, &cpu_timer0_isr);
    Interrupt_register(INT_TIMER1, &cpu_timer1_isr);

    // Peripheral init
    TI_TIMER_InitHz(CPUTIMER0_BASE, timerCOM, DEVICE_SYSCLK_FREQ);
    TI_TIMER_InitHz(CPUTIMER1_BASE, timerALGO, DEVICE_SYSCLK_FREQ);
    // CAN init (example)
    CAN_initModule(CANB_BASE);
    CAN_setBitRate(CANB_BASE, DEVICE_SYSCLK_FREQ, 500000, 16);
    // Configure TX message object
    // TX message objects (enable TX interrupt if you want to track completion later)
    CANB_MSG_INIT(CAN_TX_OFFSET,CAN_TX_OFFSET+CAN_MSG_TX_AMOUNT-1,CAN_RX_OFFSET,CAN_RX_OFFSET+CAN_MSG_RX_AMOUNT-1);
    CAN_enableAutoBusOn(CANB_BASE);
    CAN_setAutoBusOnTime(CANB_BASE, 200000U);
    CAN_startModule(CANB_BASE);
    // Enable peripheral interrupt sources (some are already enabled inside your TI_PWM/TI_TIMER init)
    // For timer: TI_TIMER_InitHz enables timer interrupt generation, but you still must enable the CPU interrupt line:
    Interrupt_enable(INT_TIMER0);
    Interrupt_enable(INT_TIMER1);

    // Enable global interrupts
    Interrupt_enableMaster();
    ERTM; // optional, enables real-time debug events


    //variable init
    U_ref.dq.d = 0.0f;
    U_ref.dq.q = 0.0f;
    U_ref.theta = 0.0f;
    U_ref.omega = 500.0f;
 
    //CAN DATA
    for(;;)
    {
    }
}

__interrupt void cpu_timer0_isr(void)
{
    timerCOM_cnt++;
    if(timerCOM_cnt < 500)
    {
        GPIO_writePin(RED_LED, 1);
    }
    else if(timerCOM_cnt < 1000)
    {
        GPIO_writePin(RED_LED, 0);
    }
    else if(timerCOM_cnt == 1000) timerCOM_cnt = 0;

    if(automated_test){
        if(automated_test_counter < 10000){U_ref.dq.d = 20.0; U_ref.omega = 2000.0f;}
        else if(automated_test_counter < 30000){U_ref.dq.d = 0.0; U_ref.omega = 0.0f;}
        else if(automated_test_counter < 40000){U_ref.dq.d = 10.0; U_ref.omega = 300.0f;}
        else if(automated_test_counter < 60000){U_ref.dq.d = 0.0; U_ref.omega = 0.0f;}
        else if(automated_test_counter < 70000){U_ref.dq.d = 2.5; U_ref.omega = 30.0f;}
        else if(automated_test_counter < 90000){U_ref.dq.d = 0.0; U_ref.omega = 0.0f;}
        else if(automated_test_counter < 100000){U_ref.dq.d = 2.5; U_ref.omega = 10.0f;}
        else if(automated_test_counter < 120000){U_ref.dq.d = 0.0; U_ref.omega = 0.0f;}
        else {automated_test_counter = 0; automated_test = 0;}
        automated_test_counter++;
    }
    

    // Clear timer overflow flag (important)
    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);
    static uint16_t can_tx_msg_cnt = 0;
    static uint16_t can_rx_msg_cnt = 0;  

    //CAN - shift transmited data into CAN registers

    //Read and Send all CAN data to transmitter - cyclic FIFO
        switch(can_tx_msg_cnt)
    {
        case 0:
        can_data_tx[0] = config; break;
        case 1:
        can_data_tx[1] = float3k_to_u64(U_ref.dq.d, U_ref.dq.q, U_ref.omega, 50.0f); break;
        case 2:
        can_data_tx[2] = float3k_to_u64(set_freq_kHz, gate_strenght, Ug_set, 10.0f); break;
        default: break;
    }
    // Send
    u64_to_CAN16x8(can_data_tx[can_tx_msg_cnt], can_msg_tx[can_tx_msg_cnt]);
    CAN_sendMessage(CANB_BASE, CAN_TX_OFFSET+can_tx_msg_cnt, 8, can_msg_tx[can_tx_msg_cnt]);
    DEVICE_DELAY_US(2);
    // Read
    CAN_readMessage(CANB_BASE, CAN_RX_OFFSET+can_rx_msg_cnt, can_msg_rx[can_rx_msg_cnt]);
    can_data_rx[can_rx_msg_cnt] = CAN16x8_to_u64(can_msg_rx[can_rx_msg_cnt]);
    switch(can_rx_msg_cnt)
    {
        case 0:
        u64_to_float3k(can_data_rx[0], &Current_meas.ph.a, &Current_meas.ph.b, &Current_meas.ph.c); break;
        case 1:
        u64_to_float3k(can_data_rx[1], &Current_meas.dq.d, &Current_meas.dq.q, &Current_meas.Im); break;
        case 2:
        u64_to_float3k(can_data_rx[2], &T_NTC, &Tj_NTC_based, &Tj_est); break;
        case 3:
        u64_to_float3k(can_data_rx[3], &PowerT_est, &Tj_NoATC, &fsw_khz); break;
        case 4:
        u64_to_float3k(can_data_rx[4], &Tj_max_drivecycle, &Tj_start, &Tj_ref); break;
        case 8:
        u64_to_float3k(can_data_rx[8], &Uth.up1, &Uth.up2, &Uth.up3); break;
        default: break;
    }
    
    can_tx_msg_cnt++;
    can_rx_msg_cnt++;
    if(can_tx_msg_cnt >= (CAN_MSG_TX_AMOUNT)) can_tx_msg_cnt = 0;
    if(can_rx_msg_cnt >= (CAN_MSG_RX_AMOUNT)) can_rx_msg_cnt = 0;
    
    DEVICE_DELAY_US(5);
    //CAN - Decode read CAN data


    // Ack PIE group for TIMER0 (usually group 1)
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
      
}

__interrupt void cpu_timer1_isr(void)
{
    timerALGO_cnt++;
    // High importancy TIMER - ATC algo will be here
    if(timerALGO_cnt < 70)
    {
        GPIO_writePin(BLUE_LED, 0);
    }
    else if(timerALGO_cnt < 150)
    {
        GPIO_writePin(BLUE_LED, 1);
    }
    else if(timerALGO_cnt == 150) timerALGO_cnt = 0;

    CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}