#include "driverlib.h"
#include "device.h"
#include "Peripherals/TI_PWM.h"
#include "Peripherals/TI_TIMER.h"
#include "Peripherals/TI_CAN.h"
#include "Peripherals/TI_SPI.h"
#include "SourceCode/SVPWM.h"
#include "SourceCode/ATC.h"
#include "UCC5870/ucc5870.h"
#include "UCC5870/hvp045a_io.h"
#include "UCC5870/ucc5870_regs.h"
#include <stdint.h>

//Model parameter defines
#define Ls 0.001f
#define Rs 0.0036f
#define OnebyLs 1000.0f
#define eps_damp_coeff 24.1f
#define Tamb 22.0f
const float part_coeff = (1.0f/one_by_sqrt2/sqrt3);
const float inv_part_coeff = 1.0f/(1.0f/one_by_sqrt2/sqrt3);
float kpc, inv_kpc;
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
uint32_t timerCOM = 2000; // Hz of timer 0 - adjustable in debugger
uint32_t timerALGO = 100;
uint32_t timerCOM_cnt = 0;
uint32_t timerALGO_cnt = 0;

//ATC - PWM timer
float gPWMratio = 1.0f;
float Ts = 0.0001f;

//ATC algo
AgingParam Tj, Rdson, Uth;
float Uth_base = 3.9f;
float Rdson_base = 82.0f; //mOhm
float Rdson_real = 0.082;
// Electric variables
threephase Current_est, Current_meas, U_ref, U_read;
PWM svpwm, spwm;
float Udc_base = 1.0f;
float Udc_meas;
float Udc_ref = 1.0f;
uint32_t Ud_read_int, Uq_read_int, omega_read_int;

//CAN
#define CAN_TX_OFFSET 10
#define CAN_MSG_TX_AMOUNT 11
#define CAN_RX_OFFSET 25
#define CAN_MSG_RX_AMOUNT 2
uint16_t can_msg_tx[CAN_MSG_TX_AMOUNT][8];
uint64_t can_data_tx[CAN_MSG_TX_AMOUNT];
uint16_t can_msg_rx[CAN_MSG_RX_AMOUNT][8];
uint64_t can_data_rx[CAN_MSG_RX_AMOUNT];
uint64_t config = 0x106; //106 if with Uth monitor
uint8_t CAN_on = 1;

// CPU usage assesment
#define MAX_IDLE_2kHz 16544
#define CPU_SCALE 0.00604487f
uint32_t idle_cnt;
uint32_t free_computing_time;
float uCPU; //0 - 100%


//Gate driver UCC5870
GD_UCC gd[6];
uint16_t GD1_AI1, GD2_AI1;

void main(void)
{
    //variable init
    U_ref.dq.d = 0.3f*Udc_base;
    U_ref.dq.q = 0.1f*Udc_base;
    U_ref.scale = 1.0f;
    U_ref.theta = 0.0f;
    U_ref.omega = 50.7;

    //Aging placeholder
    Tj.up1 = Tamb; Tj.up2 = Tamb; Tj.up3 = Tamb; Tj.down1 = Tamb; Tj.down2 = Tamb; Tj.down3 = Tamb;
    Rdson.up1 = Rdson_base; Rdson.up2 = Rdson_base; Rdson.up3 = Rdson_base; Rdson.down1 = Rdson_base; Rdson.down2 = Rdson_base; Rdson.down3 = Rdson_base;
    Uth.up1 = Uth_base; Uth.up2 = Uth_base; Uth.up3 = Uth_base; Uth.down1 = Uth_base; Uth.down2 = Uth_base; Uth.down3 = Uth_base;


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

    //SPI and Gate Driver UCC5870 init
    ERTM; // optional, enables real-time debug events
    configureSPI_GPIO();
    configureSPI(GD_SPI_BASE);
    Init_UCC5870_Regs();
    Init_UCC5870();

    if(getStatus(config, VGTH_MONITOR))
    {
    DEVICE_DELAY_US(5000);
    int addr = 0;
    for (addr = 1; addr <= 6; addr++){
        writeRegUCC5870(addr, CONTROL2, VGTH_MEAS);
    }
    DEVICE_DELAY_US(10000);
    addr = 0;
    for(addr = 1; addr <=6; addr++){
        gd[addr-1].Uth = UCC5870_ADC_READ(readRegUCC5870(addr, ADCDATA8))*VGTH_SCALE;
        DEVICE_DELAY_US(10);
    }
    Uth.up1 = gd[1].Uth;
    Uth.down1 = gd[0].Uth;
    Uth.up2 = gd[3].Uth;
    Uth.down2 = gd[2].Uth;
    Uth.up3 = gd[5].Uth;
    Uth.down3 = gd[4].Uth;
    }
    
    GD_Init_LED_blink(7);

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
    CAN_enableAutoBusOn(CANB_BASE);
    CAN_setAutoBusOnTime(CANB_BASE, 200000U);
    CAN_startModule(CANB_BASE);

    // Enable peripheral interrupt sources (some are already enabled inside your TI_PWM/TI_TIMER init)
    // For timer: TI_TIMER_InitHz enables timer interrupt generation, but you still must enable the CPU interrupt line:
    Interrupt_enable(INT_TIMER0);
    Interrupt_enable(INT_TIMER1);
    // For EPWM1: TI_PWM_Init_123 enabled EPWM interrupt generation; enable CPU interrupt line:
    Interrupt_enable(INT_EPWM1);
    // Enable global interrupts
    Interrupt_enableMaster();
    

    for(;;)
    {
        idle_cnt++;
    }
}
// Main PWM control
__interrupt void epwm1_isr(void)
{

    //Safety turning off
    if((getStatus(config, STATUS_OFF)||(!getStatus(config, STATUS_ON)))&&CAN_on)
    {
        U_ref.omega = 0.0f;
        U_ref.dq.d = 0.0f;
        U_ref.dq.q = 0.0f;
    }
    //Interpreting Config for control purposes
    if(getStatus(config, MODULATION)) kpc = part_coeff;
    else kpc = 1.0f;
    // Update PWM
    U_ref.theta = U_ref.theta + U_ref.omega*Ts;
    if(U_ref.theta >= 2.0f*PI ) U_ref.theta = U_ref.theta - 2.0f*PI;
    SPWM(U_ref.dq.d, U_ref.dq.q, U_ref.theta, Udc_meas, &spwm);
    DQ_to_AlfaBeta(&U_ref);
    AlfaBeta_to_ABC(&U_ref, kpc);

    gPWMHz = (uint32_t)(PWM_FREQ_HZ*gPWMratio);
    Ts = 1.0f/gPWMHz;
    TI_PWM_SetFreqHz_123(gPWMHz, TBCLK_HZ, svpwm.d1d4, svpwm.d2d5, svpwm.d3d6);

    CurrentObserver(&U_ref, &Current_est, Ts, Rs+Rdson_real*2, OnebyLs, kpc);
    AlfaBeta_to_DQ(&Current_est);
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
    // low importancy TIMER - External communication is there
    timerCOM_cnt++;
    if(timerCOM_cnt == 500)
    {
        
        GPIO_writePin(RED_LED, 1);
            uint16_t gd_addr = 0;
            
        for(gd_addr = 1; gd_addr <=6; gd_addr++){
            gd[gd_addr-1].AI[1] = UCC5870_ADC_READ(readRegUCC5870(gd_addr, ADCDATA1));
            DEVICE_DELAY_US(10);
            gd[gd_addr-1].temperature = UCC5870_TEMPERATURE_READ(readRegUCC5870(gd_addr, ADCDATA7));
            DEVICE_DELAY_US(10);
            gd[gd_addr-1].DATA_SPI = readRegUCC5870(gd_addr, SPITEST);
            DEVICE_DELAY_US(10);
            
        }
    }
    else if(timerCOM_cnt == 1000)
    {
        GPIO_writePin(RED_LED, 0);
        timerCOM_cnt = 0;
    }

    

    // Clear timer overflow flag (important)
    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);
    
    //Read and Send all CAN data to transmitter - cyclic FIFO
    static uint16_t can_tx_msg_cnt = 0;
    static uint16_t can_rx_msg_cnt = 0;  
    //CAN - shift transmited data into CAN registers
    switch(can_tx_msg_cnt)
    {
        case 0:
        can_data_tx[0] = float3k_to_u64(Current_meas.ph.a, Current_meas.ph.b, Current_meas.ph.c, 100.0f); break;
        case 1:
        can_data_tx[1] = float3k_to_u64(Current_meas.dq.d, Current_meas.dq.q, Current_meas.RMS,100.0f); break;
        case 2:
        can_data_tx[2] = float3k_to_u64(Tj.up1, Tj.up2, Tj.up3,10.0f); break;
        case 3: 
        can_data_tx[3] = float3k_to_u64(Tj.down1, Tj.down2, Tj.down3, 10.0f); break;
        case 4: 
        can_data_tx[4] = float3k_to_u64(Current_est.ph.a, Current_est.ph.b, Current_est.ph.c,100.0f); break;
        case 5:
        can_data_tx[5] = float3k_to_u64(Current_est.dq.d, Current_est.dq.q, Current_est.RMS, 100.0f); break;
        case 6:
        can_data_tx[6] = float3k_to_u64(Rdson.up1, Rdson.up2, Rdson.up3, 10.0f); break;
        case 7:
        can_data_tx[7] = float3k_to_u64(Rdson.down1, Rdson.down2, Rdson.down3, 10.0f); break;
        case 8:
        can_data_tx[8] = float3k_to_u64(Uth.up1, Uth.up2, Uth.up3,100.0f); break;
        case 9:
        can_data_tx[9] = float3k_to_u64(Uth.down1, Uth.down2, Uth.down3, 100.0f); break;
        case 10:
        can_data_tx[10] = float3k_to_u64(Udc_meas, Udc_ref, 0.0f, 10.0f); break;
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
        config = can_data_rx[0]; break;
        case 1:
        if(getStatus(config, CAN_CONTROL))
        {
        u64_to_float3k(can_data_rx[1], &U_ref.dq.d, &U_ref.dq.q, &U_ref.omega); break;
        }
        default: break;
    }
    
    can_tx_msg_cnt++;
    can_rx_msg_cnt++;
    if(can_tx_msg_cnt >= (CAN_MSG_TX_AMOUNT)) can_tx_msg_cnt = 0;
    if(can_rx_msg_cnt >= (CAN_MSG_RX_AMOUNT)) can_rx_msg_cnt = 0;
    //CAN - Decode read CAN data
    
    // Ack PIE group for TIMER0 (usually group 1)
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    //Calcuating margin for computing time
    free_computing_time = idle_cnt;
    idle_cnt = 0;
    uCPU = 100.0f - free_computing_time * CPU_SCALE;


      
}

__interrupt void cpu_timer1_isr(void)
{
    
    timerALGO_cnt++;
    // High importancy TIMER - ATC algo will be here
    if(timerALGO_cnt == 70)
    {
        GPIO_writePin(BLUE_LED, 0);
    }
    else if(timerALGO_cnt == 150)
    {
        GPIO_writePin(BLUE_LED, 1);
        timerALGO_cnt = 0;
    }
    
    // Clear Timer1 interrupt source
    CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);

    // Ack PIE group (Timer1 is also in group 1 on most C2000 setups)
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    
}
