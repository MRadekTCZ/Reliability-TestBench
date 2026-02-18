#include "driverlib.h"
#include "device.h"
#include "Peripherals/TI_PWM.h"
#include "Peripherals/TI_TIMER.h"
#include "Peripherals/TI_CAN.h"
#include "SourceCode/SVPWM.h"
#include "SourceCode/ATC.h"
#include <stdint.h>

//Model parameter defines
#define Ls 0.001f
#define Rs 0.0036f
#define OnebyLs 1000.0f
#define eps_damp_coeff 24.1f
#define Tamb 22.0f;
const float kpc = (1.0f/one_by_sqrt2/sqrt3);
const float inv_kpc = 1/(1.0f/one_by_sqrt2/sqrt3);
//GPIO
#define BLUE_LED    31
#define GREEN_LED    25
#define RED_LED    34
//Clocks
// Interrupt declarations
__interrupt void epwm1_isr(void);
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
// Interrupt counters
uint32_t gPWMHz = PWM_FREQ_HZ;
uint32_t timerCOM = 1000; // Hz of timer 0 - adjustable in debugger
uint32_t timerALGO = 100;
uint32_t timerCOM_cnt = 0;
uint32_t timerALGO_cnt = 0;
//ATC - PWM timer
float gPWMratio = 1.0f;
float Ts = 0.0001f;
//ATC algo
AgingParam Tj, Rdson, Uth;
float Uth_base = 3.9f;
float Rdson_base = 0.08f;
// Electric variables
threephase Current_est, Current_meas, U_ref, U_read;
SVPWM svpwm;
float Udc_base = 25.0f;
float Udc_meas;
uint32_t Ud_read_int, Uq_read_int, omega_read_int;

//CAN
#define CAN_TX_OFFSET 10
#define CAN_MSG_TX_AMOUNT 10
#define CAN_RX_OFFSET 25
#define CAN_MSG_RX_AMOUNT 1
uint16_t can_msg_tx[CAN_MSG_TX_AMOUNT][8];
uint64_t can_data_tx[CAN_MSG_TX_AMOUNT];
uint16_t can_msg_rx[CAN_MSG_RX_AMOUNT][8];
uint64_t can_data_rx[CAN_MSG_RX_AMOUNT];
uint32_t config = 0x5A5055AA;
void main(void)
{
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

    // Register ISRs
    Interrupt_register(INT_EPWM1, &epwm1_isr);
    Interrupt_register(INT_TIMER0, &cpu_timer0_isr);
    Interrupt_register(INT_TIMER1, &cpu_timer1_isr);

    // Peripheral init
    TI_PWM_Init_123(PWM_FREQ_HZ, DEADTIME_NS, TBCLK_HZ);
    TI_TIMER_InitHz(CPUTIMER0_BASE, timerCOM, DEVICE_SYSCLK_FREQ);
    TI_TIMER_InitHz(CPUTIMER1_BASE, timerALGO, DEVICE_SYSCLK_FREQ);
    // CAN init (example)
    CAN_initModule(CANB_BASE);
    CAN_setBitRate(CANB_BASE, DEVICE_SYSCLK_FREQ, 500000, 16);
    // Configure TX message object
    // TX message objects (enable TX interrupt if you want to track completion later)
    CANB_MSG_INIT(CAN_TX_OFFSET,CAN_TX_OFFSET+CAN_MSG_TX_AMOUNT-1,CAN_RX_OFFSET,CAN_RX_OFFSET+CAN_MSG_RX_AMOUNT-1);
    CAN_startModule(CANB_BASE);
    // Enable peripheral interrupt sources (some are already enabled inside your TI_PWM/TI_TIMER init)
    // For timer: TI_TIMER_InitHz enables timer interrupt generation, but you still must enable the CPU interrupt line:
    Interrupt_enable(INT_TIMER0);
    Interrupt_enable(INT_TIMER1);
    // For EPWM1: TI_PWM_Init_123 enabled EPWM interrupt generation; enable CPU interrupt line:
    Interrupt_enable(INT_EPWM1);
    // Enable global interrupts
    Interrupt_enableMaster();
    ERTM; // optional, enables real-time debug events


    //variable init
    U_ref.dq.d = 0.3f*Udc_base;
    U_ref.dq.q = 0.1f*Udc_base;
    U_ref.scale = 1.0f;
    U_ref.theta = 0.0f;
    //Aging placeholder
    Tj.up1 = Tamb; Tj.up2 = Tamb; Tj.up3 = Tamb; Tj.down1 = Tamb; Tj.down2 = Tamb; Tj.down3 = Tamb;
    Rdson.up1 = Rdson_base; Rdson.up2 = Rdson_base; Rdson.up3 = Rdson_base; Rdson.down1 = Rdson_base; Rdson.down2 = Rdson_base; Rdson.down3 = Rdson_base;
    Uth.up1 = Uth_base; Uth.up2 = Uth_base; Uth.up3 = Uth_base; Uth.down1 = Uth_base; Uth.down2 = Uth_base; Uth.down3 = Uth_base;
    //CAN DATA
    can_data_tx[0] = 777;
    can_data_tx[1] = 123456;
    for(;;)
    {
    }
}

__interrupt void epwm1_isr(void)
{
    // Update PWM
    U_ref.theta = U_ref.theta + ANGLE_dT*0.1f;
    if(U_ref.theta >= 2*PI ) U_ref.theta = U_ref.theta - 2*PI;
    svpwm = svPWM(U_ref.dq.d, U_ref.dq.q, U_ref.theta, Udc_meas);
    DQ_to_AlfaBeta(&U_ref);
    AlfaBeta_to_ABC(&U_ref, kpc);

    gPWMHz = (uint32_t)(PWM_FREQ_HZ*gPWMratio);
    Ts = 1/gPWMHz;
    TI_PWM_SetFreqHz_123(gPWMHz, TBCLK_HZ, svpwm.d1d4, svpwm.d2d5, svpwm.d3d6);

    CurrentObserver(&U_ref, &Current_est, Ts, Rs, OnebyLs, eps_damp_coeff, kpc);
    DQ_to_AlfaBeta(&Current_est);
    AlfaBeta_to_ABC(&Current_est, kpc);
    DQ_RMS(&Current_est);

    //Test only
    Current_meas = Current_est;//Test only
    Udc_meas = Udc_base;

    // Clear ePWM interrupt flag
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);

    // Ack PIE group for ePWM (usually group 3)
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
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


    // Clear timer overflow flag (important)
    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);


    //CAN - shift transmited data into CAN registers
    can_data_tx[0] = float4_to_u64_k(Current_meas.ph.a, Current_meas.ph.b, Current_meas.ph.c, 0.0f, 100.0f);
    can_data_tx[1] = float4_to_u64_k(Current_meas.dq.d, Current_meas.dq.q, Current_meas.RMS, Udc_meas, 100.0f);
    can_data_tx[2] = float4_to_u64_k(Tj.up1, Tj.up2, Tj.up3, 0.0f, 10.0f);
    can_data_tx[3] = float4_to_u64_k(Tj.down1, Tj.down2, Tj.down3, 0.0f, 10.0f);
    can_data_tx[4] = float4_to_u64_k(Current_est.ph.a, Current_est.ph.b, Current_est.ph.c, 0.0f, 10.0f);
    can_data_tx[5] = float4_to_u64_k(Current_est.dq.d, Current_est.dq.q, Current_est.RMS, Udc_base, 10.0f);
    can_data_tx[6] = float4_to_u64_k(Rdson.up1, Rdson.up2, Rdson.up3, 0.0f, 10.0f);
    can_data_tx[7] = float4_to_u64_k(Rdson.down1, Rdson.down2, Rdson.down3, 0.0f, 10.0f);
    can_data_tx[8] = float4_to_u64_k(Uth.up1, Uth.up2, Uth.up3, 0.0f, 100.0f);
    can_data_tx[9] = float4_to_u64_k(Uth.down1, Uth.down2, Uth.down3, 0.0f, 100.0f);;
    //Read and Send all CAN data to transmitter - cyclic FIFO
    static uint16_t can_tx_msg_cnt = 0;
    static uint16_t can_rx_msg_cnt = 0;    
    CAN_readMessage(CANB_BASE, CAN_RX_OFFSET+can_rx_msg_cnt, can_msg_rx[can_rx_msg_cnt]);
    can_data_rx[can_rx_msg_cnt] = CAN16x8_to_u64(can_msg_rx[can_rx_msg_cnt]);
    DEVICE_DELAY_US(10);
    u64_to_CAN16x8(can_data_tx[can_tx_msg_cnt], can_msg_tx[can_tx_msg_cnt]);
    CAN_sendMessage(CANB_BASE, CAN_TX_OFFSET+can_tx_msg_cnt, 8, can_msg_tx[can_tx_msg_cnt]);

    u64_to_4x_u32(can_data_rx[0], &config, &Ud_read_int, &Uq_read_int, &omega_read_int);
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
}
