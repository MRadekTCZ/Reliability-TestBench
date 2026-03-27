#ifndef ClarkPark_Math_H_
#define ClarkPark_Math_H_

#include <math.h>
#define PI 3.141592653f//;
#define one_by_sqrt6 0.40824829f
#define one_by_sqrt3 0.57735f
#define one_by_sqrt2 0.70710678f
#define sqrt2_by_sqrt3 0.816496581f
#define sqrt3 1.732050f
#define pi_by_3 1.04719755f
#define one_by_pi_by_3 0.9549296596425f



struct Phase {
    float a;
    float b;
    float c;
};
struct DQ {
    float d;
    float q;
};
struct AlfaBeta {
    float alfa;
    float beta;
};

struct ThreePhase {
    struct Phase ph;
    struct DQ dq;
    struct AlfaBeta ab;
    float RMS;
    float Im;
    float omega;
    float theta;
    float scale;
};
typedef struct ThreePhase threephase;

void CurrentObserver(threephase *Voltage,threephase *current_est, float Ts, float Res, float Ls_inv, float kpc);

void DQ_to_AlfaBeta(threephase *x);
void AlfaBeta_to_ABC(threephase *x, float kpc);
void AlfaBeta_to_DQ(threephase *x);
void ABC_to_AlfaBeta(threephase *x, float inv_kpc);
void DQ_RMS(threephase *current);
void DQ_Im(threephase *current);

#endif

