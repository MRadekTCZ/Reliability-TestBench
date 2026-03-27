#include "TI_PWM.h"
#include <stdint.h>

static uint16_t gTbprd = 0;

static inline uint16_t clampCmpA_fromDuty(float duty, uint16_t tbprd)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 0.95f) duty = 0.95f;  // margin for deadtime
    return (uint16_t)(duty * (float)tbprd);
}

static uint16_t calcTbprd_updown(uint32_t tbclk_hz, uint32_t pwm_hz)
{
    // Fpwm = TBCLK / (2*TBPRD) (up-down mode)
    if (pwm_hz == 0U) return 0U;
    return (uint16_t)(tbclk_hz / (2UL * pwm_hz));
}

static void initEPwmGpio_1_2_3(void)
{
    // ePWM1A/B -> GPIO0/1
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPinConfig(GPIO_1_EPWM1B);

    // ePWM2A/B -> GPIO2/3
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_2_EPWM2A);
    GPIO_setPinConfig(GPIO_3_EPWM2B);

    // ePWM3A/B -> GPIO4/5
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_4_EPWM3A);
    GPIO_setPinConfig(GPIO_5_EPWM3B);
}
static void initEPwmGpio_4_5_6(void)
{
    // ePWM1A/B -> GPIO0/1
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_6_EPWM4A);
    GPIO_setPinConfig(GPIO_7_EPWM4B);

    // ePWM2A/B -> GPIO2/3
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_8_EPWM5A);
    GPIO_setPinConfig(GPIO_9_EPWM5B);

    // ePWM3A/B -> GPIO4/5
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_10_EPWM6A);
    GPIO_setPinConfig(GPIO_11_EPWM6B);
}
static void initEPwmModule(uint32_t base, uint16_t tbprd, uint16_t dbCount)
{
    // Time-Base: up-down symmetric PWM
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(base);
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(base, tbprd);
    EPWM_setTimeBaseCounter(base, 0U);

    // Compare A shadow load on CTR=ZERO
    EPWM_setCounterCompareShadowLoadMode(base,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    // AQ for EPWMA: set on up CMPA, clear on down CMPA
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    // Dead-band: generate complementary B from A with deadtime
    EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, true);

    // Use EPWMA as input to deadband (A drives both edges)
    EPWM_setRisingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);

    // Polarity: Active high complementary (matches your DB_ACTV_HIC usage intent)
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

    // Output mode: full enable (both RED and FED)
    EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, true);
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_A, false);
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_B, false);

    EPWM_setRisingEdgeDelayCount(base, dbCount);
    EPWM_setFallingEdgeDelayCount(base, dbCount);

    // Ensure B output is driven by deadband (most devices do this automatically once DB is enabled)
    // No extra call needed in standard driverlib flow.
}

void TI_PWM_Init_123(uint32_t pwm_hz, uint32_t deadtime_cycles, uint32_t tbclk_hz)
{
    const uint16_t tbprd   = calcTbprd_updown(tbclk_hz, pwm_hz);
    const uint16_t dbCount = (uint16_t)deadtime_cycles;
    gTbprd = tbprd;

    // GPIO mux
    initEPwmGpio_1_2_3();

    // Stop TBCLK while configuring (driverlib helper)
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // Configure ePWM1/2/3
    initEPwmModule(EPWM1_BASE, tbprd, dbCount);
    initEPwmModule(EPWM2_BASE, tbprd, dbCount);
    initEPwmModule(EPWM3_BASE, tbprd, dbCount);

    // ePWM1 interrupt on CTR=ZERO, every event (ISR in main)
    EPWM_setInterruptSource(EPWM1_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM1_BASE);
    EPWM_setInterruptEventCount(EPWM1_BASE, 1U);

    // Re-enable TBCLK
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}
void TI_PWM_Init_456(uint32_t pwm_hz, uint32_t deadtime_cycles, uint32_t tbclk_hz)
{
    const uint16_t tbprd   = calcTbprd_updown(tbclk_hz, pwm_hz);
    const uint16_t dbCount = (uint16_t)deadtime_cycles;
    gTbprd = tbprd;

    // GPIO mux
    initEPwmGpio_4_5_6();

    // Stop TBCLK while configuring (driverlib helper)
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // Configure ePWM1/2/3
    initEPwmModule(EPWM4_BASE, tbprd, dbCount);
    initEPwmModule(EPWM5_BASE, tbprd, dbCount);
    initEPwmModule(EPWM6_BASE, tbprd, dbCount);

    // ePWM1 interrupt on CTR=ZERO, every event (ISR in main)
    EPWM_setInterruptSource(EPWM4_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM4_BASE);
    EPWM_setInterruptEventCount(EPWM4_BASE, 1U);

    // Re-enable TBCLK
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}
void TI_PWM_SetDuty_123(float duty1, float duty2, float duty3)
{
    const uint16_t tbprd = (uint16_t)EPWM_getTimeBasePeriod(EPWM1_BASE);

    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A,
                                clampCmpA_fromDuty(duty1, tbprd));

    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A,
                                clampCmpA_fromDuty(duty2, tbprd));

    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A,
                                clampCmpA_fromDuty(duty3, tbprd));
}
void TI_PWM_SetDuty_456(float duty1, float duty2, float duty3)
{
    const uint16_t tbprd = (uint16_t)EPWM_getTimeBasePeriod(EPWM4_BASE);

    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A,
                                clampCmpA_fromDuty(duty1, tbprd));

    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_A,
                                clampCmpA_fromDuty(duty2, tbprd));

    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A,
                                clampCmpA_fromDuty(duty3, tbprd));
}

uint16_t TI_PWM_GetTbprd(void)
{
    return gTbprd;
}


void TI_PWM_SetFreqHz_123(uint32_t pwm_hz, uint32_t tbclk_hz, float duty1, float duty2, float duty3)
{
    if (pwm_hz < 100U)     pwm_hz = 100U;
    if (pwm_hz > 100000U)  pwm_hz = 100000U;

    uint16_t tbprd = calcTbprd_updown(tbclk_hz, pwm_hz);
    if (tbprd < 20U) tbprd = 20U;

    // TBPRD shadow load enabled by default on most devices; ensure period updates are safe.
    // Update period
    EPWM_setTimeBasePeriod(EPWM1_BASE, tbprd);
    EPWM_setTimeBasePeriod(EPWM2_BASE, tbprd);
    EPWM_setTimeBasePeriod(EPWM3_BASE, tbprd);

    // Clamp duty and update compares against new period
    if (duty1 < 0.0f) duty1 = 0.0f; if (duty1 > 0.95f) duty1 = 0.95f;
    if (duty2 < 0.0f) duty2 = 0.0f; if (duty2 > 0.95f) duty2 = 0.95f;
    if (duty3 < 0.0f) duty3 = 0.0f; if (duty3 > 0.95f) duty3 = 0.95f;

    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A,
                                (uint16_t)(duty1 * (float)tbprd));
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A,
                                (uint16_t)(duty2 * (float)tbprd));
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A,
                                (uint16_t)(duty3 * (float)tbprd));

    gTbprd = tbprd;
}

void TI_PWM_SetFreqHz_456(uint32_t pwm_hz, uint32_t tbclk_hz, float duty1, float duty2, float duty3)
{
    if (pwm_hz < 100U)     pwm_hz = 100U;
    if (pwm_hz > 100000U)  pwm_hz = 100000U;

    uint16_t tbprd = calcTbprd_updown(tbclk_hz, pwm_hz);
    if (tbprd < 20U) tbprd = 20U;

    // TBPRD shadow load enabled by default on most devices; ensure period updates are safe.
    // Update period
    EPWM_setTimeBasePeriod(EPWM4_BASE, tbprd);
    EPWM_setTimeBasePeriod(EPWM5_BASE, tbprd);
    EPWM_setTimeBasePeriod(EPWM6_BASE, tbprd);

    // Clamp duty and update compares against new period
    if (duty1 < 0.0f) duty1 = 0.0f; if (duty1 > 0.95f) duty1 = 0.95f;
    if (duty2 < 0.0f) duty2 = 0.0f; if (duty2 > 0.95f) duty2 = 0.95f;
    if (duty3 < 0.0f) duty3 = 0.0f; if (duty3 > 0.95f) duty3 = 0.95f;

    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A,
                                (uint16_t)(duty1 * (float)tbprd));
    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_A,
                                (uint16_t)(duty2 * (float)tbprd));
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A,
                                (uint16_t)(duty3 * (float)tbprd));

    gTbprd = tbprd;
}
