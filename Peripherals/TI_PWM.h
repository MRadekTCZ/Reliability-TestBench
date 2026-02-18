#ifndef TI_PWM_H_
#define TI_PWM_H_

#include "driverlib.h"
#include "device.h"
#include <stdint.h>
#define PWM_FREQ_HZ     10000UL
#define DEADTIME_NS     10UL
#define TBCLK_HZ        100000000UL
#define SYSCLK_HZ       200000000UL
#ifdef __cplusplus
extern "C" {
#endif

// Init ePWM1/2/3 A&B on GPIO0..5 with:
// - Up-down symmetric PWM
// - Complementary B + deadtime (DBRED/DBFED)
// - ePWM1 interrupt enabled (vector/ISR handled in main)
void TI_PWM_Init_123(uint32_t pwm_hz, uint32_t deadtime_cycles, uint32_t tbclk_hz);

// Update duties online (0.0 .. ~0.95 recommended)
void TI_PWM_SetDuty_123(float duty1, float duty2, float duty3);

// Optional helper (debug)
uint16_t TI_PWM_GetTbprd(void);

void TI_PWM_SetFreqHz_123(uint32_t pwm_hz, uint32_t tbclk_hz,
                          float duty1, float duty2, float duty3);

#ifdef __cplusplus
}
#endif

#endif /* TI_PWM_H_ */
