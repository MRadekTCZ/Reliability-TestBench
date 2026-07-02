#include "driverlib.h"
#include "device.h"
#include "Peripherals/TI_PWM.h"
#include "Peripherals/TI_TIMER.h"
#include "Peripherals/TI_CAN.h"
#include "Peripherals/TI_SPI.h"
#include "Peripherals/TI_NTC.h"

#include "SourceCode/SVPWM.h"
#include "SourceCode/ATC.h"
#include "UCC5870/ucc5870.h"
#include "UCC5870/hvp045a_io.h"
#include "UCC5870/ucc5870_regs.h"
#include "SourceCode/DriveCycle.h"
#include <stdint.h>

//Model parameter defines
#ifdef GCMX080A
#define Ls 0.00043f
#define Rs 0.06f
#define CableR 0.07f
#define OnebyLs 2325.581f
float Rdson_base = 100.0f; //mOhm //+ measured offset (cables)
float Rdson_real = 0.1f;
#endif
#ifdef GCMX020A
#define Ls 0.001f
#define Rs 0.0036f
#define CableR 0.1f
#define OnebyLs 1000.0f
float Rdson_base = 19.5f;
float Rdson_real = 0.0195f;
#endif
#define Tamb 25.0f
#define B2B_baseU 0.16667f
const float park_coeff = (1.0f/one_by_sqrt2/sqrt3);
const float inv_park_coeff = 1.0f/(1.0f/one_by_sqrt2/sqrt3);
float kpc;
//GPIO
#define BLUE_LED    31
#define GREEN_LED    25
#define RED_LED    34
#define ACCELERATED_DRIVECYCLE 300000

uint64_t config = 0x10005; //open loop, control from debugger, freq and slope changable from dubugger
//uint64_t config = 0x10105; //open loop, control from debugger, freq and slope changable from dubugger + Vth measurement
#ifdef _FLASH
uint8_t CAN_on = 1;
#else
uint8_t CAN_on = 0;
#endif

//Clocks
uint32_t  drive_cycle_time_ms = 0;
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
#define PWM_MARGIN 0.5f
//ATC - PWM timer
float Ts = 0.0001f;
uint32_t fsw_ATC;
float fsw_ATC_ratio = 1.0f;
float fsw_force_set_kHz = 20.0f;
float gate_strenght = 1.0; //[A] 5-30
uint16_t CFG8_write = 0x0;
#define VIRTUAL_HEATSINK_COEFF 5.0f
//ATC algo
AgingParam Tj, Rdson, Uth;
float Uth_base = 3.9f;



ThermalModel th_model, th_virtual_heatsink, th_bare_bond_wire;
ThermalState th_state_noATC, th_state_ref, th_state_ATC, th_state_real_NTC;
GateDriveParams gd_param_noATC, gd_param_ATC;
float temp_offset = 0.0f; 
float NTC_read_from_GD;
float Tj_NTCbased;
float Tj_est = 0.0, Tj_ref = 0.0, Tj_noATC = 0.0, PowerT_est = 0.0, PowerT_ref = 0.0, PowerT_noATC = 0.0, PowerT_NTCbased = 0.0;
float b2b_emul_scale;
float NMOSFET  = 1.0f;
float Tj_start, Tj_max_drivecycle, Tj_estimated_NTC_tracker, Tj_delta_max, Tj_avg_drivecycle, Tj_delta_drivecycle;
PI_Controller atc_pi;

//ATC limits
const float Inom = 10.0f;
const float ATC_active_range = 40.0f;
//ATC drive cycle

MA_State ma_omega, ma_Ud, ma_Uq;
// Electric variables
threephase Current_est, Current_meas, Current_b2b;
threephase U_ref, U_set, U_force, U_back, U_delta, HDDT, HDDT_filtered;
PWM spwm1, spwm2;
float Ud_deadtime_comp, Uq_deadtime_comp;
//SET PROPER UDC VOLTAGE!
float Udc_meas = 150.0f, Udc_base = 200.0f;
#define UDC_DRIVE_CYCLE 200.0f

//CAN
#define CAN_ID_TX_OFFSET 10
#define CAN_MSG_TX_OFFSET 10
#define CAN_MSG_TX_AMOUNT 11
#define CAN_ID_RX_OFFSET 25
#define CAN_MSG_RX_OFFSET 25
#define CAN_MSG_RX_AMOUNT 3
uint16_t can_msg_tx[CAN_MSG_TX_AMOUNT][8];
uint64_t can_data_tx[CAN_MSG_TX_AMOUNT];
uint16_t can_msg_rx[CAN_MSG_RX_AMOUNT][8];
uint64_t can_data_rx[CAN_MSG_RX_AMOUNT];


// CPU usage assesment
#define MAX_IDLE_2kHz 16544
#define CPU_SCALE 0.00604487f
uint32_t idle_cnt;
uint32_t free_computing_time;
float CPU_computing_time;
float uCPU; //0 - 100%


//Gate driver UCC5870
GD_UCC gd[6];
uint16_t GD1_AI1, GD2_AI1;
int16_t CFG8_check;
void main(void)
{
    //variable init
    U_set.dq.d = 0.0f;
    U_set.dq.q = 0.0f;
    U_set.theta = 0.0f;
    U_set.omega = 314.7f*3.0f;
    U_ref = U_set;
    //Aging placeholder
    Tj.up1 = Tamb; Tj.up2 = Tamb; Tj.up3 = Tamb; Tj.down1 = Tamb; Tj.down2 = Tamb; Tj.down3 = Tamb;
    Rdson.up1 = Rdson_base; Rdson.up2 = Rdson_base; Rdson.up3 = Rdson_base; Rdson.down1 = Rdson_base; Rdson.down2 = Rdson_base; Rdson.down3 = Rdson_base;
    Uth.up1 = Uth_base; Uth.up2 = Uth_base; Uth.up3 = Uth_base; Uth.down1 = Uth_base; Uth.down2 = Uth_base; Uth.down3 = Uth_base;

    //ATC algo inits
    float ATC_Ts = 1.0f/timerALGO;
    ThermalModelInit(&th_model);
    ThermalModelInit(&th_virtual_heatsink);
    // Base plate temperature directly from NTC -GCMX080A
    th_bare_bond_wire.Rth1 = 0.22f;
    th_bare_bond_wire.Cth1 = 0.0023f;
    th_bare_bond_wire.Rth2 = 0.41f;
    th_bare_bond_wire.Cth2 = 0.0123f;
    th_bare_bond_wire.Rth3 = 0.63f;
    th_bare_bond_wire.Cth3 = 0.0789f;
    th_bare_bond_wire.Rth4 = 0.00001f;
    th_bare_bond_wire.Cth4 = 0.00001f;

    VirtualHeatsink_ThermalModelInit(&th_virtual_heatsink, VIRTUAL_HEATSINK_COEFF);
    Thermal_Init(&th_state_ref, &th_virtual_heatsink, temp_offset,  ATC_Ts);
    Thermal_Init(&th_state_noATC, &th_model, temp_offset,  ATC_Ts);
    Thermal_Init(&th_state_ATC, &th_model, temp_offset,  ATC_Ts);
    Thermal_Init(&th_state_real_NTC, &th_bare_bond_wire, temp_offset,  ATC_Ts);

    GateDriveParams_init(&gd_param_noATC);
    GateDriveParams_init(&gd_param_ATC);
    //It was divided by 3 before
    b2b_emul_scale = 600.0f/10.0f/UDC_DRIVE_CYCLE/5*Udc_meas/Udc_base;
    
    //              kP      kI      Ts      min        max      Init value
    PI_Init(&atc_pi, 0.01f,   0.10f,   ATC_Ts,  0.6f,    1.2f,    1.0f ); 

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
    DEVICE_DELAY_US(10000);
    if(getStatus(config, VGTH_MONITOR))
    {
    DEVICE_DELAY_US(5000);
    int addr = 0;
    for (addr = 1; addr <= 6; addr++){
        writeRegUCC5870(addr, CONTROL2, VGTH_MEAS);
    }
    DEVICE_DELAY_US(1000000);
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
    
    CFG8_check = readRegUCC5870(1, CFG8);
    DEVICE_DELAY_US(10);
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
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC); // Disable PWM Clock to Sync PWM123 and PWM456 later
    TI_PWM_Init_123(PWM_FREQ_HZ, DEADTIME_CYCLES, TBCLK_HZ);
    TI_PWM_Init_456(PWM_FREQ_HZ, DEADTIME_CYCLES, TBCLK_HZ);
    TI_TIMER_InitHz(CPUTIMER0_BASE, timerCOM, DEVICE_SYSCLK_FREQ);
    TI_TIMER_InitHz(CPUTIMER1_BASE, timerALGO, DEVICE_SYSCLK_FREQ);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC); // PWM123 and PWM456 Sync
    // CAN init (example)
    CAN_initModule(CANB_BASE);
    CAN_setBitRate(CANB_BASE, DEVICE_SYSCLK_FREQ, 500000, 16);
    // Configure TX message object
    // TX message objects (enable TX interrupt if you want to track completion later)
    CANB_MSG_INIT(CAN_MSG_TX_OFFSET,CAN_ID_TX_OFFSET,CAN_MSG_TX_AMOUNT,CAN_MSG_RX_OFFSET,CAN_ID_RX_OFFSET,CAN_MSG_RX_AMOUNT);
    CAN_enableAutoBusOn(CANB_BASE);
    CAN_setAutoBusOnTime(CANB_BASE, 200000U);
    CAN_startModule(CANB_BASE);

    // Enable peripheral interrupt sources (some are already enabled inside your TI_PWM/TI_TIMER init)
    // For timer: TI_TIMER_InitHz enables timer interrupt generation, but you still must enable the CPU interrupt line:
    Interrupt_enable(INT_TIMER0);
    Interrupt_enable(INT_TIMER1);
    // For EPWM1: TI_PWM_Init_123 enabled EPWM interrupt generation; enable CPU interrupt line:
    Interrupt_enable(INT_EPWM1);
    // For EPWM2: TI_PWM_Init_456 enabled EPWM interrupt generation; enable CPU interrupt line:
    //Interrupt_enable(INT_EPWM4);
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
    //Test only
    Current_meas = Current_est;//Test only

    //U ref (set from CAN)
    U_ref.theta = U_ref.theta + U_ref.omega*Ts;
    if(U_ref.theta >= 2.0f*PI ) U_ref.theta = U_ref.theta - 2.0f*PI; 
    //U set (set from debugger)
    U_set.theta = U_set.theta + U_set.omega*Ts;
    if(U_set.theta >= 2.0f*PI ) U_set.theta = U_set.theta - 2.0f*PI; 

    if(CAN_on)
    {
        U_set = U_ref;
    }

    DQ_to_AlfaBeta(&U_set);
    AlfaBeta_to_ABC(&U_set, kpc);
    DQ_RMS(&Current_est);
    DQ_Im(&Current_est);

    //HDDT Voltage delta
    HDDT_filtered.theta = HDDT_filtered.theta + HDDT_filtered.omega*Ts;
    if(HDDT_filtered.theta >= 2.0f*PI ) HDDT_filtered.theta = HDDT_filtered.theta - 2.0f*PI;

    if(getStatus(config, DRIVE_CYCLE_ON))
    {
        U_delta = HDDT_filtered;
        U_set = HDDT_filtered;
    }
    else U_delta = U_set;

    //B2B voltages
    U_force.dq.d = B2B_baseU*Udc_meas + U_delta.dq.d * 0.5f;
    U_force.dq.q = B2B_baseU*Udc_meas + U_delta.dq.q * 0.5f;
    U_back.dq.d = B2B_baseU*Udc_meas - U_delta.dq.d * 0.5f;
    U_back.dq.q = B2B_baseU*Udc_meas - U_delta.dq.q * 0.5f;
    U_force.omega = U_delta.omega;
    U_force.theta = U_delta.theta;
    U_back.omega = U_delta.omega;
    U_back.theta = U_delta.theta;



    if(getStatus(config, BACKTOBACK))
    {
        //Current Observer
        CurrentObserver(&U_delta, &Current_est, Ts, (Rs+Rdson_real+CableR), OnebyLs, kpc);
        // Force PWM - Inverter 1 (EPWM 1,2,3)
        SPWM(U_force.dq.d, U_force.dq.q, U_force.theta, Udc_meas, &spwm1);
        // Back EMF - Inverter 2 (EPWM 4,5,6)
        SPWM(U_back.dq.d, U_back.dq.q, U_back.theta, Udc_meas, &spwm2);
    }
    else
    {
    
        //Current Observer
        CurrentObserver(&U_set, &Current_est, Ts, Rs+Rdson_real+CableR, OnebyLs, kpc);
        Ud_deadtime_comp = DeadTimeVoltageCompensation(gPWMHz, DEADTIME_NS, Udc_meas);
        Uq_deadtime_comp = DeadTimeVoltageCompensation(gPWMHz, DEADTIME_NS, Udc_meas);
        if(U_set.dq.d < PWM_MARGIN) Ud_deadtime_comp = 0.0f;
        if(U_set.dq.q < PWM_MARGIN) Uq_deadtime_comp = 0.0f;
        // Open loop Uref
        SVPWM(U_set.dq.d+Ud_deadtime_comp, U_set.dq.q+Uq_deadtime_comp, U_set.theta, Udc_meas, &spwm1);
        // Second PWM (Alternative)
        SVPWM(U_set.dq.d+Ud_deadtime_comp, U_set.dq.q+Uq_deadtime_comp, U_set.theta, Udc_meas, &spwm2);

        if(getStatus(config, MODULATION))
        {
            kpc = park_coeff;
        }
        else
        {
            kpc = 1.0f;          
        }
        
    }


    //Force Test Output
    //Diagnostic of inverter 1 - EPWM1,2,3
    
    if(getStatus(config, DIRECT_SWITCH_CONTROL))
    {
        if(getStatus(config, AUTOMATED_TEST))
        {
            //Place Holder for Automated Test procedure
        }
        else 
        {
            EPWM_setActionQualifierContSWForceAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A,EPWM_AQ_SW_OUTPUT_LOW);
            EPWM_setActionQualifierContSWForceAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A,EPWM_AQ_SW_OUTPUT_LOW);
            EPWM_setActionQualifierContSWForceAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A,EPWM_AQ_SW_OUTPUT_LOW);
            if(getStatus(config, T1_ON))
            {
            EPWM_setActionQualifierContSWForceAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A,EPWM_AQ_SW_OUTPUT_HIGH);
            }
            else EPWM_setActionQualifierContSWForceAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A,EPWM_AQ_SW_OUTPUT_LOW);
            if(getStatus(config, T2_ON))
            {
            EPWM_setActionQualifierContSWForceAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A,EPWM_AQ_SW_OUTPUT_HIGH);
            }
            else EPWM_setActionQualifierContSWForceAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A,EPWM_AQ_SW_OUTPUT_LOW);
            if(getStatus(config, T3_ON))
            {
            EPWM_setActionQualifierContSWForceAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A,EPWM_AQ_SW_OUTPUT_HIGH);
            }
            else EPWM_setActionQualifierContSWForceAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A,EPWM_AQ_SW_OUTPUT_LOW); 
        }
        
               
        
    }
    else if(!getStatus(config, DRIVE_CYCLE_ON)) EPWM_DISABLE_FORCE_ALL();

    //Safety - NTC turn off
    if(CAN_on)
    {
    if(NTC_read_from_GD >= 155.0f)
    {
        spwm1.d1d4 = 0.0f;
        spwm1.d2d5 = 0.0f;
        spwm1.d3d6 = 0.0f;
        spwm2.d1d4 = 0.0f;
        spwm2.d2d5 = 0.0f;
        spwm2.d3d6 = 0.0f;
    }
    }

    //ATC PWM
    TI_PWM_SetFreqHz_123(gPWMHz, TBCLK_HZ, spwm1.d1d4, spwm1.d2d5, spwm1.d3d6);
    //No ATC PWM
    TI_PWM_SetFreqHz_456(PWM_FREQ_HZ, TBCLK_HZ, spwm2.d1d4, spwm2.d2d5, spwm2.d3d6);
    
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
            //gd[gd_addr-1].AI[1] = UCC5870_ADC_READ(readRegUCC5870(gd_addr, ADCDATA1));
            //DEVICE_DELAY_US(10);
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
        can_data_tx[1] = float3k_to_u64(Current_meas.dq.d, Current_meas.dq.q, Current_meas.Im,100.0f); break;
        case 2:
        can_data_tx[2] = float3k_to_u64(NTC_read_from_GD, Tj_NTCbased, Tj_est, 50.0f); break; //Real temperature
        case 3: 
        can_data_tx[3] = float3k_to_u64(PowerT_est, Tj_noATC, (float)gPWMHz*0.001f, 50.0f); break; //Reference temperatures + GD temperature
        case 4: 
        can_data_tx[4] = float3k_to_u64(Tj_max_drivecycle, Tj_start, Tj_ref,50.0f); break;
        case 5:
        can_data_tx[5] = float3k_to_u64(Current_est.dq.d, Current_est.dq.q, Current_est.Im, 100.0f); break;
        case 6:
        can_data_tx[6] = float3k_to_u64(Rdson.up1, Rdson.up2, Rdson.up3, 10.0f); break;
        case 7:
        can_data_tx[7] = float3k_to_u64(Rdson.down1, Rdson.down2, Rdson.down3, 10.0f); break;
        case 8:
        can_data_tx[8] = float3k_to_u64(Uth.up1, Uth.up2, Uth.up3,100.0f); break;
        case 9:
        can_data_tx[9] = float3k_to_u64(Uth.down1, Uth.down2, Uth.down3, 100.0f); break;
        case 10:
        can_data_tx[10] = float3k_to_u64(Udc_meas, uCPU, 0.0f, 10.0f); break;
        default: break;
    }
    // Send 
    u64_to_CAN16x8(can_data_tx[can_tx_msg_cnt], can_msg_tx[can_tx_msg_cnt]);
    CAN_sendMessage(CANB_BASE, CAN_MSG_TX_OFFSET +can_tx_msg_cnt, 8, can_msg_tx[can_tx_msg_cnt]);
    DEVICE_DELAY_US(2);
    // Read
    CAN_readMessage(CANB_BASE, CAN_MSG_RX_OFFSET +can_rx_msg_cnt, can_msg_rx[can_rx_msg_cnt]);
    can_data_rx[can_rx_msg_cnt] = CAN16x8_to_u64(can_msg_rx[can_rx_msg_cnt]);

    if(CAN_on)
    {
    switch(can_rx_msg_cnt)
    {
        case 0:  
            config = can_data_rx[0]; break;
        case 1:
            u64_to_float3k(can_data_rx[1], &U_ref.dq.d, &U_ref.dq.q, &U_ref.omega); break;
        case 2:
            u64_to_float3k(can_data_rx[2], &fsw_force_set_kHz, &gate_strenght, &gd_param_ATC.Ug_on); break;
        default: break;
    }
    }
    
    can_tx_msg_cnt++;
    can_rx_msg_cnt++;
    if(can_tx_msg_cnt >= (CAN_MSG_TX_AMOUNT)) can_tx_msg_cnt = 0;
    if(can_rx_msg_cnt >= (CAN_MSG_RX_AMOUNT)) can_rx_msg_cnt = 0;
    //CAN - Decode read CAN data
    //Safety turning off
    if((getStatus(config, STATUS_OFF)||(!getStatus(config, STATUS_ON))))
    {
        U_ref.omega = 0.0f;
        U_ref.dq.d = 0.0f;
        U_ref.dq.q = 0.0f;
    }




    // Ack PIE group for TIMER0 (usually group 1)
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    //Calcuating margin for computing time
    free_computing_time = idle_cnt;
    idle_cnt = 0;
    uCPU = 100.0f - free_computing_time * CPU_SCALE;
    CPU_computing_time = CPU_computing_time + 0.0005f;

      
}

__interrupt void cpu_timer1_isr(void)
{
    
    //NTC read
    gd[0].AI[1] = UCC5870_ADC_READ(readRegUCC5870(2, ADCDATA1));
    if(gd[0].AI[1] > 0.01f) NTC_read_from_GD = NTC_conversion(gd[0].AI[1], NTC_VCC); //NTC_VCC = 15.0 v - diode voltage drop
    //NTC_read_from_GD = NTC_conversion(gd[5].AI[1], 15.0f - 0.5f);
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

    if(getStatus(config, DRIVE_CYCLE_ON))
    {
        if(drive_cycle_time_ms == 0) {
        EPWM_DISABLE_FORCE_ALL();
        Tj_max_drivecycle = 30.0;
        }
        if(drive_cycle_time_ms == 21000) Tj_start = NTC_read_from_GD;
        drive_cycle_time_ms = drive_cycle_time_ms + 10;
        #ifdef ACCELERATED_DRIVECYCLE
        if(drive_cycle_time_ms == ACCELERATED_DRIVECYCLE) drive_cycle_time_ms = drive_cycle_time_ms + 1800000;
        #endif
        if(drive_cycle_time_ms < 2701000)
        {   
            Tj_avg_drivecycle = Tj_avg_drivecycle + NTC_read_from_GD * DRIVE_CYCLE_PERIOD;
            if(NTC_read_from_GD > Tj_max_drivecycle) Tj_max_drivecycle = NTC_read_from_GD;
            if ((drive_cycle_time_ms % 100) == 0)
            {
                HDDT.omega = (float)DRIVE_CYCLE_omega_LUT[drive_cycle_time_ms/1000] * TWO_PI;
                HDDT.dq.d = (float)DRIVE_CYCLE_Ud_LUT[drive_cycle_time_ms/1000] * b2b_emul_scale ;
                HDDT.dq.q = (float)DRIVE_CYCLE_Uq_LUT[drive_cycle_time_ms/1000] * b2b_emul_scale;
                HDDT_filtered.omega =  moving_average(HDDT.omega, &ma_omega);
                HDDT_filtered.dq.d =  moving_average(HDDT.dq.d, &ma_Ud);
                HDDT_filtered.dq.q =  moving_average(HDDT.dq.q, &ma_Uq);
                //FIltering temporary removed - to have higher peaks
                //HDDT_filtered.omega =  HDDT.omega;
                //HDDT_filtered.dq.d =  HDDT.dq.d;
                //HDDT_filtered.dq.q =  HDDT.dq.q;
            }
        }
       
        else if(drive_cycle_time_ms < 3000000)
        //stop for 13 second for measurement procedures - Uth, Rdson
        {
            HDDT_filtered.omega =  0.0f;
            HDDT_filtered.dq.d =  0.0f;
            HDDT_filtered.dq.q =  0.0f;
            EPWM_FORCE_OFF_ALL();
            if(drive_cycle_time_ms == 2710000) Tj_delta_drivecycle = Tj_max_drivecycle - Tj_start;
            // SPACE FOR AUTOMATED TEST

        }  
        else drive_cycle_time_ms = 0;
    }
    //Temperature Estimation - reference case
    DQ_Im(&Current_est);
    PowerT_ref = LossCalc_linear(&gd_param_noATC, Udc_meas, Current_est.Im, PWM_FREQ_HZ, Tj_noATC) * NMOSFET;
    Tj_ref = Thermal_Step(&th_state_ref, PowerT_ref) + Tamb;
    //Temperature Estimation - ATC
    PowerT_est = LossCalc_linear(&gd_param_ATC, Udc_meas, Current_est.Im, gPWMHz, Tj_est) * NMOSFET;
    Tj_est = Thermal_Step(&th_state_ATC, PowerT_est) + Tamb;
    //Temperature Estimation - comparison case
    PowerT_noATC = LossCalc_linear(&gd_param_noATC, Udc_meas, Current_est.Im, PWM_FREQ_HZ, Tj_noATC) * NMOSFET;
    Tj_noATC = Thermal_Step(&th_state_noATC, PowerT_noATC) + Tamb;
    
    //Temperature Estimation - Estimated better with help of NTC
    PowerT_NTCbased = LossCalc_linear(&gd_param_ATC, Udc_meas, Current_est.Im, gPWMHz, Tj_NTCbased);
    Tj_NTCbased = Thermal_Step(&th_state_real_NTC, PowerT_NTCbased) + NTC_read_from_GD;

    //PLACE TO IMPLEMENT ATC ALGORITHM - FINAL
    fsw_ATC_ratio = ATC(&atc_pi, Tj_ref, Tj_est, Current_est.Im, Inom,ATC_active_range);



    if(getStatus(config, FREQ_FORCE_SET))
    {
        if(gate_strenght < 5.0f) {
        CFG8_write = 0x026A;
        // 1.0, 3.0 and 6.0 are inner gate driver resistances
        //gd_param_ATC.Rg_on = 3.9f + 6.0f;
        }
        else if (gate_strenght < 10.0f) {
        CFG8_write = 0x0269;
        //gd_param_ATC.Rg_on = 3.9f + 3.0f;
        }
        else {CFG8_write = 0x0268;
        //gd_param_ATC.Rg_on = 3.9f + 1.0f;
        }
        int addr = 0;
        for (addr = 1; addr <= 6; addr++){
            writeRegUCC5870(addr, CFG8, CFG8_write);         
        }
        CFG8_check = readRegUCC5870(1, CFG8);
        DEVICE_DELAY_US(10);
    gPWMHz = (uint32_t)(PWM_FREQ_HZ*fsw_force_set_kHz*SCALEkHz);
    }
    else gPWMHz = PWM_FREQ_HZ;
    if(getStatus(config, ATC_ACTIVE))
    {     
        gPWMHz = (uint32_t)(PWM_FREQ_HZ*fsw_ATC_ratio);
    }
    if(gPWMHz >= 4000) Ts = 1.0f/gPWMHz;
    else 
    {   
        gPWMHz = 4000;
        Ts = 1/4000;
    }
    // Clear Timer1 interrupt source
    CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);

    // Ack PIE group (Timer1 is also in group 1 on most C2000 setups)
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    
}
