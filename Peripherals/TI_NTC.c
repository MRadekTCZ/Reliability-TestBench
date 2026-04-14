#include "TI_NTC.h"
//Calibration function of NTC in GCMX
#define RFIX 22000.0f
#define kRFIX 0.0000454545f // 1 / RFIX
#define kT0 0.0033540164f
#define K0 273.15f
//#define Beta2580 3440.0f
#define Beta80 0.0002906976744f
#define Beta50 0.0002958579882f

float NTC_conversion(float NTC_ADC_voltage, float vcc)
{
    float temperature;
    float v_ntc = NTC_ADC_voltage;
    float R_ntc;
    R_ntc = RFIX * v_ntc/(vcc - v_ntc);

    // --- First rough temperature using Beta50 (closest to room temp)
    float T1_raw = 1.0f/(kT0 + Beta50 * logf(R_ntc*kRFIX)) - K0;
    // Linearize Beta based on temperature
    float lineBeta1 = (T1_raw - 50.0f) / 30.0f;   

    lineBeta1 = fminf(1.0f, fmaxf(0.0f, lineBeta1));

    float Beta1 = Beta50*(1.0f-lineBeta1) + Beta80*lineBeta1;

    // --- Final temperature values
    temperature = 1.0f/(kT0 + Beta1 * logf(R_ntc*kRFIX)) - K0;
    return temperature;
}