#include "ATC.h"

float LossCalc_precise(const GateDriveParams *g, float v, float Im, float fsw){
    float i;
    i = fabsf(Im)*TWO_BY_PI;
    float Pcond, Eon, Eoff, Psw, Ptotal;

    Pcond = g->Ron*Im*Im*ONE_BY_4PI*(PI);

    Eon = v * i * 0.5f * (g->Rg_on * (g->Cgs+g->Cgd) * logf(1.0f / (1.0f - (g->Vplat / g->Ug_on)))- (g->Rg_on * (g->Cgs+g->Cgd) * logf(1.0f / (1.0f - (g->Vth / (g->Ug_on))))) + (g->Rg_on * g->Cgd * (v - g->Ron * i) / ((g->Ug_on) - g->Vplat)));
    Eoff = v * i * 0.5f * ((g->Rg_off * g->Cgd * (v - g->Ron * i) / g->Vplat) + (g->Rg_off * (g->Cgs+g->Cgd) * logf(g->Vplat/g->Vth)));
    Psw = fsw*(Eon+Eoff);
    //Ptotal = (Pcond+Psw*SPWMdutyfactor);
    Ptotal = (Pcond+Psw);
    return Ptotal;
}

float LossCalc_linear(const GateDriveParams *g, float v, float Im, float fsw){
    float Pcond, Eon, Eoff, Psw, Ptotal;
    float idc = fabsf(Im) * TWO_BY_PI;

    Pcond = g->Ron*Im*Im/4.0f; //*2 for back2back with high inductance, current flows during both sinewaves
    Eon = ((g->Rg_on-5.0f)*0.0075f+0.02f)*idc*v/600.0f/1000.0f    *2; // *2 - just for test


    Eoff = 0.0f;
    Psw = fsw*(Eon+Eoff);
    //Ptotal = (Pcond+Psw*SPWMdutyfactor);
    Ptotal = (Pcond+Psw);
    return Ptotal;
}


void GateDriveParams_init(GateDriveParams *g)
{
    #ifdef GCMX020A
    g->Ron     = 0.0195f;
    g->Rg_on   = 10.0f;
    g->Rg_off  = 2.5f;
    g->Ug_on   = 18.0f;
    g->Ug_off  = -5.0f;
    g->Ug      = 23.0f;
    g->Qg      = 235e-9f;
    g->Cgd     = 0.22e-9f;
    g->Cgs 		= 6.18e-9f;
    g->Vplat   = 4.32;
    g->Vth     = 3.6f;
    #else
    g->Ron     = 0.00165f;
    g->Rg_on   = 10.5f;
    g->Rg_off  = 4.2f;
    g->Ug_on   = 18.0f;
    g->Ug_off  = -5.0f;
    g->Ug      = 23.0f;
    g->Qg      = 1.48e-6f;
    g->Cgd     = 0.25e-9f;
    g->Cgs 		= 40.15e-9f;
    g->Vplat   = 6.24;
    g->Vth     = 3.9f;
    #endif
}


void Thermal_Init(ThermalState *state,
                  const ThermalModel *model,
                  float Tambient,
                  float Ts)
{
    float tau1 = model->Rth1 * model->Cth1;
    float tau2 = model->Rth2 * model->Cth2;
    float tau3 = model->Rth3 * model->Cth3;
    float tau4 = model->Rth4 * model->Cth4;

    state->y1 = 0.0f;
    state->y2 = 0.0f;
    state->y3 = 0.0f;
    state->y4 = 0.0f;

    state->p.a1 = tau1 / (tau1 + Ts);
    state->p.b1 = (model->Rth1 * Ts) / (tau1 + Ts);

    state->p.a2 = tau2 / (tau2 + Ts);
    state->p.b2 = (model->Rth2 * Ts) / (tau2 + Ts);

    state->p.a3 = tau3 / (tau3 + Ts);
    state->p.b3 = (model->Rth3 * Ts) / (tau3 + Ts);

    state->p.a4 = tau4 / (tau4 + Ts);
    state->p.b4 = (model->Rth4 * Ts) / (tau4 + Ts);

    state->p.Tambient = Tambient;
}


float Thermal_Step(ThermalState *state, float Power)
{
    state->y1 = state->p.a1 * state->y1 + state->p.b1 * Power;
    state->y2 = state->p.a2 * state->y2 + state->p.b2 * Power;
    state->y3 = state->p.a3 * state->y3 + state->p.b3 * Power;
    state->y4 = state->p.a4 * state->y4 + state->p.b4 * Power;

    return state->p.Tambient
         + state->y1
         + state->y2
         + state->y3
         + state->y4;
}

void ThermalModelInit(ThermalModel *thm)
{
    #ifdef GCMX020A
    thm->Rth1 = 0.17f;
    thm->Cth1 = 0.476f;
    thm->Rth2 = 0.119f;
    thm->Cth2 = 2.84f;
    thm->Rth3 = 0.709f;
    thm->Cth3 = 5.26f;
    thm->Rth4 = 3.4f;
    thm->Cth4 = 30.0f;
    #else
    thm->Rth1 = 0.0119f;
    thm->Cth1 = 0.458f;
    thm->Rth2 = 0.0422f;
    thm->Cth2 = 1.0f;
    thm->Rth3 = 0.0377f;
    thm->Cth3 = 8.17f;
    thm->Rth4 = 0.0149f;
    thm->Cth4 = 85.5f;
    #endif
}

void VirtualHeatsink_ThermalModelInit(ThermalModel *thm, float vhs_coeff)
{
    thm->Rth1 = thm->Rth1;
    thm->Cth1 = thm->Cth1*vhs_coeff;
    thm->Rth2 = thm->Rth2;
    thm->Cth2 = thm->Cth2*vhs_coeff;
    thm->Rth3 = thm->Rth3;
    thm->Cth3 = thm->Cth3*vhs_coeff;
    thm->Rth4 = thm->Rth4;
    thm->Cth4 = thm->Cth4;
}

void PI_Init(PI_Controller *pi,
             float kp,
             float ki,
             float ts,
             float u_min,
             float u_max,
             float initial_output)
{
    if (pi == 0) return;

    pi->kp = kp;
    pi->ki = ki;
    pi->ts = ts;
    pi->u_min = u_min;
    pi->u_max = u_max;

    pi->integrator = initial_output;
    pi->output = initial_output;
}

/* PI compute function */
float PI_Update(PI_Controller *pi, float error)
{
    if (pi == 0) return 0.0f;

    float p = pi->kp * error;

    /* Save previous integrator for anti-windup rollback */
    float integrator_prev = pi->integrator;

    /* Integrate */
    pi->integrator += pi->ki * pi->ts * error;

    /* Unsaturated output */
    float u = p + pi->integrator;

    /* Saturation + simple anti-windup */
    if (u > pi->u_max) {
        u = pi->u_max;

        if (error > 0.0f) {
            pi->integrator = integrator_prev;
        }
    }
    else if (u < pi->u_min) {
        u = pi->u_min;

        if (error < 0.0f) {
            pi->integrator = integrator_prev;
        }
    }

    pi->output = u;
    return u;
}

float current_coupling(float i, float inom)
{
float gain;
float gain_max = 5.0f, gain_min = 1.0;
gain = inom/i;
if (gain >= gain_max) gain = gain_max;
else if (gain <= gain_min) gain = gain_min;

return gain;
}

float ATC(PI_Controller *pid, float Tjref, float Tj, float I_pu, float I_nom, float ATC_activateRange)
{
float error = (Tjref - Tj)*current_coupling(I_pu, I_nom);
float u = PI_Update(pid, error);
if(Tj < ATC_activateRange) 
{error = 0.0f;
u = 1.0f;
}
else if(I_pu < (0.2f * I_nom))
{error = 0.0f;
u = 1.0f;
}
return  u;
}