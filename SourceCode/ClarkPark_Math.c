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
    float kL;
    float kR;
    float v_alfa, v_beta;

    /* Use same angle/frame as voltage */
    current_est->theta = Voltage->theta;
    current_est->omega = Voltage->omega;

    /* Convert differential voltage dq -> alphabeta */
    DQ_to_AlfaBeta(Voltage);

    v_alfa = Voltage->ab.alfa;
    v_beta = Voltage->ab.beta;

    /* Discrete coefficients */
    kL = Ts * Ls_inv;      /* Ts / L */
    kR = Res * kL;         /* Ts * R / L */

    /* Incremental RL observer in alphabeta */
    current_est->ab.alfa += kL * v_alfa - kR * current_est->ab.alfa;
    current_est->ab.beta += kL * v_beta - kR * current_est->ab.beta;

    /* Convert estimated current to abc */
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