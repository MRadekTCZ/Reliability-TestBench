/*
 * SVPWM.h
 *
 *  Created on: Nov 25, 2024
 *      Author: brzycki
 */

#ifndef DEVICE_SVPWM_H_
#define DEVICE_SVPWM_H_

#include "ClarkPark_Math.h"
#define ANGLE_dT 0.0314159f

typedef struct PWM_vectors {
    float d1d4;
    float d2d5;
    float d3d6;
    float t0;
    float t1;
    float t2;
    float mod_index;
}PWM;

void SVPWM(float Ud, float Uq, float angle, float U_dc, PWM *duty_cycles);
void SPWM(float Ud, float Uq, float angle, float U_dc, PWM *duty_cycles);


#endif /* DEVICE_SVPWM_H_ */
