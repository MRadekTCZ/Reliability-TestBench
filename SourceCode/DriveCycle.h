#ifndef DRIVECYCLE
#define DRIVECYCLE
#define DRIVE_CYCLE_LENGTH 2701
#define DRIVE_CYCLE_PERIOD (1.0f/270100.0f)
#define DRIVE_CYCLE_SPEEDUP10 10
#define DRIVE_CYCLE_SPEEDUP100 100
#include "device.h"
extern const uint8_t DRIVE_CYCLE_omega_LUT[DRIVE_CYCLE_LENGTH];
extern const uint8_t DRIVE_CYCLE_Ud_LUT[DRIVE_CYCLE_LENGTH];
extern const uint8_t DRIVE_CYCLE_Uq_LUT[DRIVE_CYCLE_LENGTH];
#endif