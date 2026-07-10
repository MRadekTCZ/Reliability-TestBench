/*
 * SVPWM.c
 *
 *  Created on: Nov 25, 2024
 *      Author: brzycki
 */

#include "SVPWM.h"
void SVPWM(float Ud, float Uq, float angle, float U_dc, PWM *duty_cycles)
{
float theta = 0;
float u_alfa = 0;
float u_beta = 0;
int sektor; // actual sector of PLL theta (1 of 6);
float u_alfa_1 = 0; //part of alfa vector
float u_alfa_2 = 0; //part of beta vector
float u_beta_1 = 0;
float u_beta_2 = 0;
float PWM_vector_1[3] = { 0,0,0 };
float PWM_vector_2[3] = { 0,0,0 };
float PWM_vector_0[3] = { 0,0,0 };
const short int OUT[6][3] = { {1,0,0},{1,1,0},{0,1,0},{0,1,1},{0,0,1},{1,0,1} }; //space vectors
//We need only 3 transistor states, but I want to start indexing from 1 to be consinstant with theory in paper. Compiler will book space in memory for even number of bytes anyway.
unsigned short int T1[3 + 1];
unsigned short int T2[3 + 1];

float time1_vector, time2_vector, time0_vector;
float Uvmax;
float Udq;
float modulation_index;


Uvmax =  U_dc*one_by_sqrt3;
Udq = sqrtf(Ud*Ud+Uq*Uq);
if(Udq>=Uvmax)
{
Ud = Uvmax/Udq *Ud;
Uq = Uvmax/Udq *Uq;
}
modulation_index = Udq/Uvmax;
//Ud = Ud * sqrt3 * one_by_sqrt2;
//Uq = Uq * sqrt3 * one_by_sqrt2;

u_alfa = Ud * cosf(angle) - Uq * sinf(angle);
u_beta = Ud * sinf(angle) + Uq * cosf(angle);
theta=atan2f(-u_beta,-u_alfa) + PI;
sektor = floorf(theta * one_by_pi_by_3);
if (sektor < 0) sektor = 0;
if (sektor > 5) sektor = 5;
switch (sektor)
{
case 0:
	u_alfa_1 = sqrt2_by_sqrt3 * U_dc;
	u_beta_1 = 0.0;
	u_alfa_2 = one_by_sqrt6 * U_dc;
	u_beta_2 = one_by_sqrt2 * U_dc;
	break;
case 1:
	u_alfa_1 = one_by_sqrt6 * U_dc;
	u_beta_1 = one_by_sqrt2 * U_dc;
	u_alfa_2 = -one_by_sqrt6 * U_dc;
	u_beta_2 = one_by_sqrt2 * U_dc;
	break;
case 2:
	u_alfa_1 = -one_by_sqrt6 * U_dc;
	u_beta_1 = one_by_sqrt2 * U_dc;
	u_alfa_2 = -sqrt2_by_sqrt3 * U_dc;
	u_beta_2 = 0.0;
	break;
case 3:
	u_alfa_1 = -sqrt2_by_sqrt3 * U_dc;
	u_beta_1 = 0.0;
	u_alfa_2 = -one_by_sqrt6 * U_dc;
	u_beta_2 = -one_by_sqrt2 * U_dc;
	break;
case 4:
	u_alfa_1 = -one_by_sqrt6 * U_dc;
	u_beta_1 = -one_by_sqrt2 * U_dc;
	u_alfa_2 = one_by_sqrt6 * U_dc;
	u_beta_2 = -one_by_sqrt2 * U_dc;
	break;
case 5:
	u_alfa_1 = one_by_sqrt6 * U_dc;
	u_beta_1 = -one_by_sqrt2 * U_dc;
	u_alfa_2 = sqrt2_by_sqrt3 * U_dc;
	u_beta_2 = 0.0;
	break;
}


time1_vector = (u_alfa * u_beta_2 - u_beta * u_alfa_2) / (u_alfa_1 * u_beta_2 - u_beta_1 * u_alfa_2);
time2_vector = ((-u_alfa * u_beta_1 + u_beta * u_alfa_1) /(u_alfa_1 * u_beta_2 - u_beta_1 * u_alfa_2));
time0_vector = 1 -  time1_vector -time2_vector;


;
//First sector part
T1[1] = OUT[sektor][0];
T1[2] = OUT[sektor][1];
T1[3] = OUT[sektor][2];

PWM_vector_1[0] = T1[1] * time1_vector;
PWM_vector_1[1] = T1[2] * time1_vector;
PWM_vector_1[2] = T1[3] * time1_vector;

//Second sector part	
if (sektor == 5)
{
	T2[1] = OUT[0][0];
	T2[2] = OUT[0][1];
	T2[3] = OUT[0][2];
}
else
{
	T2[1] = OUT[sektor + 1][0];
	T2[2] = OUT[sektor + 1][1];
	T2[3] = OUT[sektor + 1][2];

}
PWM_vector_2[0] = T2[1] * time2_vector;
PWM_vector_2[1] = T2[2] * time2_vector;
PWM_vector_2[2] = T2[3] * time2_vector;



PWM_vector_0[0] = 0.5f * time0_vector;
PWM_vector_0[1] = 0.5f * time0_vector;
PWM_vector_0[2] = 0.5f * time0_vector;

//Sum of time vectors
if(modulation_index >= 0.01f){
duty_cycles->d1d4 = PWM_vector_1[0] + PWM_vector_2[0] + PWM_vector_0[0];
duty_cycles->d2d5 = PWM_vector_1[1] + PWM_vector_2[1] + PWM_vector_0[1];
duty_cycles->d3d6 = PWM_vector_1[2] + PWM_vector_2[2] + PWM_vector_0[2];
}
else
{
duty_cycles->d1d4 = 0;
duty_cycles->d2d5 = 0;
duty_cycles->d3d6 = 0;
}
duty_cycles->t0 = time0_vector;
duty_cycles->t1 = time1_vector;
duty_cycles->t2 = time2_vector;
duty_cycles->mod_index = modulation_index;
}



void SPWM(float Ud, float Uq, float angle, float U_dc, PWM *duty_cycles)
{
float u_alfa = 0;
float u_beta = 0;
float Uvmax;
float Udq;
float modulation_index;
float Ua,Ub,Uc;

Uvmax =  U_dc*0.5;
Udq = sqrtf(Ud*Ud+Uq*Uq);
if(Udq>=Uvmax)
{
Ud = Uvmax/Udq *Ud;
Uq = Uvmax/Udq *Uq;
}
modulation_index = Udq/Uvmax;
//Ud = Ud * sqrt3 * one_by_sqrt2;
//Uq = Uq * sqrt3 * one_by_sqrt2;
u_alfa = Ud * cosf(angle) - Uq * sinf(angle);
u_beta = Ud * sinf(angle) + Uq * cosf(angle);

/* Inverse Clarke: alpha beta -> abc
       assuming zero-sequence = 0 */
    Ua = u_alfa;
    Ub = -0.5f * u_alfa + 0.86602540378f * u_beta;   /* +sqrt(3)/2 * u_beta */
    Uc = -0.5f * u_alfa - 0.86602540378f * u_beta;   /* -sqrt(3)/2 * u_beta */

	if(modulation_index >= 0.01f){
    /* SPWM duty cycles */
    duty_cycles->d1d4 = 0.5f + (Ua / U_dc);
    duty_cycles->d2d5 = 0.5f + (Ub / U_dc);
    duty_cycles->d3d6 = 0.5f + (Uc / U_dc);
	}
	else
	{
	duty_cycles->d1d4 = 0.0f;
	duty_cycles->d2d5 = 0.0f;
	duty_cycles->d3d6 = 0.0f;
	}
    /* Clamp to [0, 1] */
    if (duty_cycles->d1d4 < 0.0f) duty_cycles->d1d4 = 0.0f;
    if (duty_cycles->d1d4 > 1.0f) duty_cycles->d1d4 = 1.0f;

    if (duty_cycles->d2d5 < 0.0f) duty_cycles->d2d5 = 0.0f;
    if (duty_cycles->d2d5 > 1.0f) duty_cycles->d2d5 = 1.0f;

    if (duty_cycles->d3d6 < 0.0f) duty_cycles->d3d6 = 0.0f;
    if (duty_cycles->d3d6 > 1.0f) duty_cycles->d3d6 = 1.0f;

    duty_cycles->mod_index = modulation_index;
}
