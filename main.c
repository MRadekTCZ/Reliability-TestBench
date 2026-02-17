#include "driverlib.h"
#include "device.h"
#include "Peripherals/TI_PWM.h"
#include "Peripherals/TI_TIMER.h"
#include "Peripherals/TI_CAN.h"
#include <stdint.h>
//GPIO
#define BLINKY_LED_GPIO    31
//Clocks
#define PWM_FREQ_HZ     10000UL
#define DEADTIME_NS     10UL
#define TBCLK_HZ        100000000UL
#define SYSCLK_HZ       200000000UL

// Interrupt declarations
__interrupt void epwm1_isr(void);
__interrupt void cpu_timer0_isr(void);
__interrupt void canb0_isr(void);

// Globals
uint32_t gPwmHz = PWM_FREQ_HZ;
volatile float pwm1a = 0.50f;
volatile float pwm2a = 0.50f;
volatile float pwm3a = 0.50f;

volatile uint32_t gTimerHz = 2; // Hz of timer 0 - adjustable in debugger

int i = 0;
int can_itrrp_cnt = 0;
//CAN

U64decode candata0x01, candata0x02, candata0x03;
void main(void)
{
    // Device init (clock, PLL, watchdog config etc.)
    Device_init();

    // GPIO init (unlocks pins, sets default states)
    Device_initGPIO();

    // LED pin
    GPIO_setPadConfig(BLINKY_LED_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(BLINKY_LED_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_writePin(BLINKY_LED_GPIO, 0);

    // CANB pins (your board define must match the board)
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXB);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXB);

    // Interrupt module + vector table (driverlib replacement for PIE init)
    Interrupt_initModule();
    Interrupt_initVectorTable();

    // Register ISRs
    Interrupt_register(INT_EPWM1, &epwm1_isr);
    Interrupt_register(INT_TIMER0, &cpu_timer0_isr);

    // NOTE: CANB interrupt vector name depends on device/driverlib.
    // Common ones are INT_CANB0 or INT_CANB.
    // If INT_CANB0 doesn't compile, search driverlib "interrupt.h" for "CANB".
    Interrupt_register(INT_CANB0, &canb0_isr);

    // Peripheral init
    TI_PWM_Init_123(PWM_FREQ_HZ, DEADTIME_NS, TBCLK_HZ);
    TI_TIMER_InitHz(gTimerHz, DEVICE_SYSCLK_FREQ);

    // CAN init (example)
    CAN_initModule(CANB_BASE);
    CAN_setBitRate(CANB_BASE, DEVICE_SYSCLK_FREQ, 500000, 16);
    // Configure TX message object
    // TX message objects (enable TX interrupt if you want to track completion later)
    CANB_MSG_INIT(2,3,1,1);
    //CAN_setupMessageObject(CANB_BASE, TX1_MSG_OBJ_ID, TX1_MSG_ID, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_TX_INT_ENABLE, CAN_TX_DLC); 
    //CAN_setupMessageObject(CANB_BASE, TX2_MSG_OBJ_ID, TX2_MSG_ID, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_TX_INT_ENABLE, CAN_TX_DLC); 
    //CAN_setupMessageObject(CANB_BASE, RX_MSG_OBJ_ID, RX_MSG_ID, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE, CAN_RX_DLC_DONTCARE);
    CAN_startModule(CANB_BASE);
    // Enable peripheral interrupt sources (some are already enabled inside your TI_PWM/TI_TIMER init)
    // For timer: TI_TIMER_InitHz enables timer interrupt generation, but you still must enable the CPU interrupt line:
    Interrupt_enable(INT_TIMER0);

    // For EPWM1: TI_PWM_Init_123 enabled EPWM interrupt generation; enable CPU interrupt line:
    Interrupt_enable(INT_EPWM1);

    // For CANB: you must also enable CAN module interrupts in CAN registers, then enable CPU interrupt line
    // (exact CAN interrupt enabling depends on whether you're using msg obj interrupts, error/status, etc.)
    CAN_enableInterrupt(CANB_BASE, CAN_INT_IE0 | CAN_INT_ERROR | CAN_INT_STATUS);
    Interrupt_enable(INT_CANB0);
    CAN_enableGlobalInterrupt(CANB_BASE, CAN_GLOBAL_INT_CANINT0);
    // Enable global interrupts
    Interrupt_enableMaster();
    ERTM; // optional, enables real-time debug events
    //variable init
    candata0x02.u64 = 7;
    candata0x03.u64 = 17;
    for(;;)
    {
    }
}

__interrupt void epwm1_isr(void)
{
    // Update PWM
    TI_PWM_SetFreqHz_123(gPwmHz, TBCLK_HZ, pwm1a, pwm2a, pwm3a);

    // Clear ePWM interrupt flag
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);

    // Ack PIE group for ePWM (usually group 3)
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

__interrupt void cpu_timer0_isr(void)
{
    static uint16_t ledState = 0;

    ledState ^= 1U;
    GPIO_writePin(BLINKY_LED_GPIO, ledState);

    i++;

    // Clear timer overflow flag (important)
    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);

    // --- CANB transmit (non-blocking / timeout so ISR never hangs) ---
    // --- CANB transmit (driverlib-only status polling) ---
    //CAN_readMessage(CANB_BASE, RX_MSG_OBJ_ID, candata0x01.b);
    DEVICE_DELAY_US(5);
    CAN_sendMessage(CANB_BASE, TX1_MSG_ID, 8, candata0x02.b);
    DEVICE_DELAY_US(5);
    CAN_sendMessage(CANB_BASE, TX2_MSG_ID, 8, candata0x03.b);
    DEVICE_DELAY_US(5);
    // Ack PIE group for TIMER0 (usually group 1)
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
      
}

__interrupt void canb0_isr(void)
{
    uint32_t cause = CAN_getInterruptCause(CANB_BASE);
    can_itrrp_cnt++;
    CAN_readMessage(CANB_BASE, RX_MSG_OBJ_ID, candata0x01.b);  
    CAN_clearInterruptStatus(CANB_BASE, cause);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
