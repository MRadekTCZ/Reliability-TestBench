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

typedef struct SVPWM_vectors {
    float d1d4;
    float d2d5;
    float d3d6;
    float t0;
    float t1;
    float t2;
    float mod_index;
}SVPWM;

SVPWM svPWM(float Ud, float Uq, float theta, float U_dc);



#endif /* DEVICE_SVPWM_H_ */
