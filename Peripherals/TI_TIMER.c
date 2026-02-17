#include "TI_TIMER.h"
#include <stdint.h>

static uint32_t calcTimerPeriod(uint32_t freq_hz, uint32_t cpuclk_hz)
{
    if (freq_hz == 0U) return 0xFFFFFFFFUL;

    // Timer period = CPUCLK / freq  (period register is "period - 1")
    return (cpuclk_hz / freq_hz) - 1UL;
}

void TI_TIMER_InitHz(uint32_t freq_hz, uint32_t cpuclk_hz)
{
    const uint32_t period = calcTimerPeriod(freq_hz, cpuclk_hz);

    // Stop timer
    CPUTimer_stopTimer(CPUTIMER0_BASE);

    // Set period and prescaler (0 => divide by 1)
    CPUTimer_setPeriod(CPUTIMER0_BASE, period);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0U);

    // Reload counter
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);

    // Enable interrupt generation from the timer
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);

    // Start timer
    CPUTimer_startTimer(CPUTIMER0_BASE);
}

void TI_TIMER_SetFreqHz(uint32_t freq_hz, uint32_t cpuclk_hz)
{
    const uint32_t period = calcTimerPeriod(freq_hz, cpuclk_hz);

    CPUTimer_stopTimer(CPUTIMER0_BASE);                 // stop
    CPUTimer_setPeriod(CPUTIMER0_BASE, period);         // update period
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);        // reload
    CPUTimer_startTimer(CPUTIMER0_BASE);                // start
}
