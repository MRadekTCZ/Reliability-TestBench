#include "driverlib.h"
#include "device.h"

#include "Peripherals/TI_TIMER.h"
#include "Peripherals/TI_CAN.h"

#include "SourceCode/ClarkPark_Math.h"
#include "SourceCode/ATC.h"

#include <stdint.h>

//
// Model parameter defines
//
#define Tamb 22.0f

const float kpc = (1.0f / one_by_sqrt2 / sqrt3);
const float inv_kpc = 1.0f / (1.0f / one_by_sqrt2 / sqrt3);

//
// GPIOs for LAUNCHXL-F28P65X user LEDs
// LED4 = GPIO12, red
// LED5 = GPIO13, green
// Active-low: 0 = ON, 1 = OFF
//
#define RED_LED_GPIO        12U
#define GREEN_LED_GPIO      13U

#define RED_LED_CFG         GPIO_12_GPIO12
#define GREEN_LED_CFG       GPIO_13_GPIO13


//
// CAN module used on F28P65x LaunchPad J14
//
#define CAN_MASTER_BASE     CANA_BASE

//
// Interrupt prototypes
//
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);

//
// Interrupt counters
//
uint32_t timerCOM = 2000;       // Hz of timer 0 - adjustable in debugger
uint32_t timerALGO = 1;

uint32_t timerCOM_cnt = 0;
uint32_t timerALGO_cnt = 0;

//
// Automated test
//
uint8_t automated_test = 0;
uint32_t automated_test_counter = 0;

//
// Electric variables
//
threephase Current_est, Current_meas, U_ref, U_read;

float Udc_base = 70.0f;
float Udc_meas;

float gate_strenght = 1.0f;
float set_freq_kHz = 20.0f;
float Ug_set = 15.0f;

//
// Aging params / ATC algo
//
AgingParam Tj, Rdson, Uth;

float Uth_base = 0.0f;
float Rdson_base = 0.0f;

//
// Temperature monitor
//
float Tj_est, Tj_NTC_based, T_NTC;
float Tj_ref, Tj_NoATC, fsw_khz;
float Tj_max_drivecycle, Tj_avg_drivecycle, Tj_start;
float PowerT_est;

//
// CAN message objects
//
#define CAN_ID_TX_OFFSET      25U
#define CAN_MSG_TX_OFFSET     25U
#define CAN_MSG_TX_AMOUNT     3U

#define CAN_ID_RX_OFFSET      10U
#define CAN_MSG_RX_OFFSET     10U
#define CAN_MSG_RX_AMOUNT     11U
#define ID2_DEVICE_OFFSET 0x100U
uint16_t CAN_ID = 0x1; //0x1 - for device 1 with ATC (offset 0x000), 0x2 for device 2 without ATC (offset 0x100)
uint16_t CAN_ID_OFFSET = 0x0;
uint16_t previous_can_id;
uint16_t can_msg_tx[CAN_MSG_TX_AMOUNT][8];
uint64_t can_data_tx[CAN_MSG_TX_AMOUNT];

uint16_t can_msg_rx[CAN_MSG_RX_AMOUNT][8];
uint64_t can_data_rx[CAN_MSG_RX_AMOUNT];

uint64_t config = 0x10005;

void main(void)
{
    //
    // Device init: clock, PLL, watchdog config, etc.
    //
    Device_init();

    //
    // GPIO init: unlock pins, enable internal pullups.
    //
    Device_initGPIO();

    //
    // LED GPIO pin mux
    //
    GPIO_setPinConfig(RED_LED_CFG);
    GPIO_setPinConfig(GREEN_LED_CFG);

    //
    // LED GPIO configuration
    //
    GPIO_setPadConfig(GREEN_LED_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(GREEN_LED_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(GREEN_LED_GPIO, GPIO_QUAL_SYNC);

    GPIO_setPadConfig(RED_LED_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(RED_LED_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(RED_LED_GPIO, GPIO_QUAL_SYNC);

    //
    // LEDs OFF initially.
    // Active-low: 1 = OFF, 0 = ON
    //
    GPIO_writePin(GREEN_LED_GPIO, 1);
    GPIO_writePin(RED_LED_GPIO, 1);

    //
    // CANA pins for LAUNCHXL-F28P65X CAN connector J14.
    // In your device.h with _LAUNCHXL_F28P65X:
    // DEVICE_GPIO_CFG_CANTXA should be GPIO_4_CANA_TX
    // DEVICE_GPIO_CFG_CANRXA should be GPIO_5_CANA_RX
    //
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXA);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXA);

    //
    // Interrupt module + vector table
    //
    Interrupt_initModule();
    Interrupt_initVectorTable();

    Interrupt_register(INT_TIMER0, &cpu_timer0_isr);
    Interrupt_register(INT_TIMER1, &cpu_timer1_isr);

    //
    // Timer init
    //
    TI_TIMER_InitHz(CPUTIMER0_BASE, timerCOM, DEVICE_SYSCLK_FREQ);
    TI_TIMER_InitHz(CPUTIMER1_BASE, timerALGO, DEVICE_SYSCLK_FREQ);

    //
    // CANA init
    //
    CAN_initModule(CAN_MASTER_BASE);
    CAN_setBitRate(CAN_MASTER_BASE, DEVICE_SYSCLK_FREQ, 500000U, 16U);
    if(CAN_ID == 0x1) CAN_ID_OFFSET = 0x0;
    else if(CAN_ID == 0x2) CAN_ID_OFFSET = 0x100;
    previous_can_id = CAN_ID;
    //
    // Configure CANA message objects.
    // This requires updated TI_CAN.c/.h with CANA_MSG_INIT().
    //
    CANA_MSG_INIT(CAN_MSG_TX_OFFSET,                         // TX mailbox offset
                CAN_ID_TX_OFFSET + CAN_ID_OFFSET,   // TX CAN ID offset
                CAN_MSG_TX_AMOUNT,                         // number of TX mailboxes
                CAN_MSG_RX_OFFSET,                         // RX mailbox offset
                CAN_ID_RX_OFFSET + CAN_ID_OFFSET,   // RX CAN ID offset
                CAN_MSG_RX_AMOUNT);                        // number of RX mailboxes

    CAN_enableAutoBusOn(CAN_MASTER_BASE);
    CAN_setAutoBusOnTime(CAN_MASTER_BASE, 200000U);
    CAN_startModule(CAN_MASTER_BASE);

    //
    // Enable timer interrupt lines
    //
    Interrupt_enable(INT_TIMER0);
    Interrupt_enable(INT_TIMER1);

    //
    // Enable global interrupts
    //
    EINT;
    ERTM;

    //
    // Variable init
    //
    U_ref.dq.d = 0.0f;
    U_ref.dq.q = 0.0f;
    U_ref.theta = 0.0f;
    U_ref.omega = 500.0f;

    for(;;)
    {
        //
        // Main loop intentionally empty.
        // Communication is handled cyclically in timer0 ISR.
        //
    }
}

__interrupt void cpu_timer0_isr(void)
{
    static uint16_t can_tx_msg_cnt = 0U;
    static uint16_t can_rx_msg_cnt = 0U;

    timerCOM_cnt++;

    //
    // Automated test
    //
    if(automated_test)
    {
        if(automated_test_counter < 10000U)
        {
            U_ref.dq.d = 20.0f;
            U_ref.omega = 2000.0f;
        }
        else if(automated_test_counter < 30000U)
        {
            U_ref.dq.d = 0.0f;
            U_ref.omega = 0.0f;
        }
        else if(automated_test_counter < 40000U)
        {
            U_ref.dq.d = 10.0f;
            U_ref.omega = 300.0f;
        }
        else if(automated_test_counter < 60000U)
        {
            U_ref.dq.d = 0.0f;
            U_ref.omega = 0.0f;
        }
        else if(automated_test_counter < 70000U)
        {
            U_ref.dq.d = 2.5f;
            U_ref.omega = 30.0f;
        }
        else if(automated_test_counter < 90000U)
        {
            U_ref.dq.d = 0.0f;
            U_ref.omega = 0.0f;
        }
        else if(automated_test_counter < 100000U)
        {
            U_ref.dq.d = 2.5f;
            U_ref.omega = 10.0f;
        }
        else if(automated_test_counter < 120000U)
        {
            U_ref.dq.d = 0.0f;
            U_ref.omega = 0.0f;
        }
        else
        {
            automated_test_counter = 0U;
            automated_test = 0U;
        }

        automated_test_counter++;
    }

    //
    // Clear timer overflow flag
    //
    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);

    //
    // Prepare cyclic CAN TX data
    //
    GPIO_writePin(GREEN_LED_GPIO, (timerCOM_cnt/ 100) % 2);
    switch(can_tx_msg_cnt)
    {
        case 0U:
            can_data_tx[0] = config;
            break;

        case 1U:
            can_data_tx[1] = float3k_to_u64(U_ref.dq.d,
                                            U_ref.dq.q,
                                            U_ref.omega,
                                            50.0f);
            break;

        case 2U:
            can_data_tx[2] = float3k_to_u64(set_freq_kHz,
                                            gate_strenght,
                                            Ug_set,
                                            10.0f);
            break;

        default:
            break;
    }

    //
    // Send one CAN message cyclically
    //
    u64_to_CAN16x8(can_data_tx[can_tx_msg_cnt],
                   can_msg_tx[can_tx_msg_cnt]);

    CAN_sendMessage(CAN_MASTER_BASE,
                    CAN_MSG_TX_OFFSET + can_tx_msg_cnt,
                    8U,
                    can_msg_tx[can_tx_msg_cnt]);

    DEVICE_DELAY_US(2);

    //
    // Read one CAN message object cyclically
    //
    CAN_readMessage(CAN_MASTER_BASE,
                    CAN_MSG_RX_OFFSET + can_rx_msg_cnt,
                    can_msg_rx[can_rx_msg_cnt]);

    can_data_rx[can_rx_msg_cnt] =
        CAN16x8_to_u64(can_msg_rx[can_rx_msg_cnt]);
    if(can_data_rx[0]!=0)
    {
        GPIO_writePin(RED_LED_GPIO, (timerCOM_cnt/ 100) % 2);
    }
    //
    // Decode received CAN data
    //
    switch(can_rx_msg_cnt)
    {
        case 0U:
            u64_to_float3k(can_data_rx[0],
                           &Current_meas.ph.a,
                           &Current_meas.ph.b,
                           &Current_meas.ph.c);
            break;

        case 1U:
            u64_to_float3k(can_data_rx[1],
                           &Current_meas.dq.d,
                           &Current_meas.dq.q,
                           &Current_meas.Im);
            break;

        case 2U:
            u64_to_float3k(can_data_rx[2],
                           &T_NTC,
                           &Tj_NTC_based,
                           &Tj_est);
            break;

        case 3U:
            u64_to_float3k(can_data_rx[3],
                           &PowerT_est,
                           &Tj_NoATC,
                           &fsw_khz);
            break;

        case 4U:
            u64_to_float3k(can_data_rx[4],
                           &Tj_max_drivecycle,
                           &Tj_start,
                           &Tj_ref);
            break;

        case 8U:
            u64_to_float3k(can_data_rx[8],
                           &Uth.up1,
                           &Uth.up2,
                           &Uth.up3);
            break;

        default:
            break;
    }

    //
    // Increment cyclic CAN counters
    //
    can_tx_msg_cnt++;
    can_rx_msg_cnt++;

    if(can_tx_msg_cnt >= CAN_MSG_TX_AMOUNT)
    {
        can_tx_msg_cnt = 0U;
    }

    if(can_rx_msg_cnt >= CAN_MSG_RX_AMOUNT)
    {
        can_rx_msg_cnt = 0U;
    }

    DEVICE_DELAY_US(5);

    //
    // Ack PIE group for TIMER0
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void cpu_timer1_isr(void)
{
    timerALGO_cnt++;
    if(previous_can_id != CAN_ID)
    {
        if(CAN_ID == 0x1) CAN_ID_OFFSET = 0x0;
        else if(CAN_ID == 0x2) CAN_ID_OFFSET = 0x100;
        CANA_MSG_INIT(CAN_MSG_TX_OFFSET,                         // TX mailbox offset
                CAN_ID_TX_OFFSET + CAN_ID_OFFSET,   // TX CAN ID offset
                CAN_MSG_TX_AMOUNT,                         // number of TX mailboxes
                CAN_MSG_RX_OFFSET,                         // RX mailbox offset
                CAN_ID_RX_OFFSET + CAN_ID_OFFSET,   // RX CAN ID offset
                CAN_MSG_RX_AMOUNT);                        // number of RX mailboxes
        previous_can_id = CAN_ID;
    }
    CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}