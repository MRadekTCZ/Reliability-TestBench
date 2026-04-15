#include "TI_NTC.h"
#include <math.h>

#define RFIX      22000.0f     // external fixed divider resistor
#define R25       5000.0f      // NTC nominal resistance at 25C
#define INV_R25   (1.0f / R25)

#define T0_INV    0.0033540164f   // 1 / 298.15 K
#define K0        273.15f

#define BETA_25_50_INV  (1.0f / 3380.0f)
#define BETA_25_80_INV  (1.0f / 3440.0f)

float NTC_conversion(float NTC_ADC_voltage, float vcc)
{
    float v_ntc = NTC_ADC_voltage;

    // Protect against invalid ADC readings
    if (v_ntc <= 0.0f || v_ntc >= vcc) {
        return NAN;
    }

    // Assumes: NTC to VCC, RFIX to GND
    float R_ntc = RFIX * v_ntc / (vcc - v_ntc);

    // First estimate using beta25/50
    float T1_raw = 1.0f / (T0_INV + BETA_25_50_INV * logf(R_ntc * INV_R25)) - K0;

    // Blend between beta25/50 and beta25/80 from 50C to 80C
    float a = (T1_raw - 50.0f) / 30.0f;
    if (a < 0.0f) a = 0.0f;
    if (a > 1.0f) a = 1.0f;

    float beta_inv = BETA_25_50_INV * (1.0f - a) + BETA_25_80_INV * a;

    float temperature = 1.0f / (T0_INV + beta_inv * logf(R_ntc * INV_R25)) - K0;

    return temperature;
}