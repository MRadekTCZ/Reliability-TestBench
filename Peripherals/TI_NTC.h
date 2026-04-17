#ifndef TI_NTC_H_
#define TI_NTC_H_
#include <math.h>
#define NTC_VCC (15.0f-0.45f)
float NTC_conversion(float NTC_ADC_voltage, float vcc);

#endif