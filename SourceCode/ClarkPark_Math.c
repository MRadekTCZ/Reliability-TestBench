#include "ClarkPark_Math.h"

void DQ_to_AlfaBeta(threephase *x)
{
    x->ab.alfa = x->dq.d * cosf(x->theta) - x->dq.q * sinf(x->theta);
    x->ab.beta = x->dq.d * sinf(x->theta) + x->dq.q * cosf(x->theta);
}

void AlfaBeta_to_ABC(threephase *x, float kpc)
{
    x->ph.a = kpc * x->ab.alfa;

    x->ph.b = kpc * 0.5f * (-x->ab.alfa + sqrt3 * x->ab.beta);

    x->ph.c = kpc * 0.5f * (-x->ab.alfa - sqrt3 * x->ab.beta);
}

void AlfaBeta_to_DQ(threephase *x)
{
    float c = cosf(x->theta);
    float s = sinf(x->theta);

    x->dq.d =  x->ab.alfa * c + x->ab.beta * s;
    x->dq.q = -x->ab.alfa * s + x->ab.beta * c;
}


void ABC_to_AlfaBeta(threephase *x, float inv_kpc)
{

    float a = x->ph.a * inv_kpc;
    float b = x->ph.b * inv_kpc;
    float c = x->ph.c * inv_kpc;

    x->ab.alfa = a;
    x->ab.beta = (a + 2.0f * b) * one_by_sqrt3;
}

void CurrentObserver(threephase *Voltage, threephase *current_est, float Ts, float Res, float Ls_inv, float kpc)
{
float Ls_val;
    float den;
    float a;
    float b;
    float v_alfa, v_beta;

    /*
       Previous voltage samples for trapezoidal integration.
       Static = remembered between function calls.
    */
    static float v_alfa_prev = 0.0f;
    static float v_beta_prev = 0.0f;
    static int initialized = 0;

    /* Use same angle/frame as voltage */
    current_est->theta = Voltage->theta;
    current_est->omega = Voltage->omega;

    /* Convert voltage dq -> alphabeta */
    DQ_to_AlfaBeta(Voltage);

    v_alfa = Voltage->ab.alfa;
    v_beta = Voltage->ab.beta;

    /*
       Reconstruct L from Ls_inv because function input cannot be changed.
       Ls_inv = 1 / Ls
    */
    Ls_val = 1.0f / Ls_inv;

    /*
       Tustin / trapezoidal discretization of:

           L di/dt + R i = v

       i[n] = a*i[n-1] + b*(v[n] + v[n-1])

       a = (2L - R Ts) / (2L + R Ts)
       b = Ts / (2L + R Ts)
    */
    den = 2.0f * Ls_val + Res * Ts;

    a = (2.0f * Ls_val - Res * Ts) / den;
    b = Ts / den;

    /*
       Avoid startup half-step error.
       On first call, set previous voltage equal to present voltage.
    */
    if (!initialized)
    {
        v_alfa_prev = v_alfa;
        v_beta_prev = v_beta;
        initialized = 1;
    }

    current_est->ab.alfa = a * current_est->ab.alfa +
                           b * (v_alfa + v_alfa_prev);

    current_est->ab.beta = a * current_est->ab.beta +
                           b * (v_beta + v_beta_prev);

    v_alfa_prev = v_alfa;
    v_beta_prev = v_beta;

    /* Convert estimated current to abc and dq */
    AlfaBeta_to_ABC(current_est, kpc);
    AlfaBeta_to_DQ(current_est);

}

void DQ_RMS(threephase *current){
    float id = current->dq.d;
    float iq = current->dq.q;
    current->RMS = sqrtf(id*id + iq*iq)*one_by_sqrt2;
}

void DQ_Im(threephase *current){
    float id = current->dq.d;
    float iq = current->dq.q;
    current->Im = sqrtf(id*id + iq*iq);
}

float moving_average(float new_sample, MA_State *s) {
    s->sum -= s->buffer[s->index];
    s->buffer[s->index] = new_sample;
    s->sum += new_sample;

    s->index++;
    if (s->index >= N_AVG) s->index = 0;
    if (s->filled < N_AVG) s->filled++;

    return s->sum / s->filled;
}