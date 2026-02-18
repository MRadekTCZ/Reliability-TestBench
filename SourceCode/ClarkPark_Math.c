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

void CurrentObserver(threephase *Voltage,threephase *current_est, float Ts, float Res, float Ls_inv, float eps_dump, float kpc)
{
    float static ia_est = 0.0f;
    float static ib_est = 0.0f;
    float static ia_prev = 0.0f;
    float static ib_prev = 0.0f;
    float k1 = Ts*Ls_inv;

    DQ_to_AlfaBeta(Voltage);
    ia_est = ia_prev + k1 * (Voltage->ab.alfa - Res*eps_dump*ia_prev);
    ib_est = ib_prev + k1 * ( Voltage->ab.beta - Res*eps_dump*ib_prev);
    ia_prev = ia_est; 
    ib_prev = ib_est;
    current_est->ab.alfa = ia_est;
    current_est->ab.beta = ib_est;
    AlfaBeta_to_ABC(current_est, kpc);
    
}

void DQ_RMS(threephase *current){
    float id = current->dq.d;
    float iq = current->dq.q;
    current->RMS = sqrtf(id*id + iq*iq)*one_by_sqrt2;
}