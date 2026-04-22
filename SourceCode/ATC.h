#ifndef ATC_ALGO_H_
#define ATC_ALGO_H_

#include <math.h>
#define ONE_BY_3PI 0.1061032954f
#define ONE_BY_4PI 0.0795774715f
#define PI 3.141592653f//;
#define ONE_BY_SQRT2 0.707106781f
#define PI_BY_2 1.570796327f
#define SVPWMdutyfactor 0.66667f
#define SPWMdutyfactor 0.5f
#define SVPWMcoeff 0.866f
#define TWO_BY_PI 0.636619f
#define SQRT2 1.41421356237f
#define Afactor 1.0f
#define ALFA_LOSS_LINEAR_COEFF 0.0075f
#define ALFA_LOSS_LINEAR_OFFSET 5.0f
#define GCMX020A




typedef struct FosterThermalModel{
    float Rth1;
    float Cth1;
    float Rth2;
    float Cth2;
    float Rth3;
    float Cth3;
    float Rth4;
    float Cth4;
}ThermalModel;

typedef struct{
	float Ron;
	float Rg_on;
	float Rg_off;
	float Ug_on;
	float Ug_off;
	float Ug;
	float Qg;
	float Cgd;
	float Cgs;
	float Vplat;
	float Vth;
}GateDriveParams;

typedef struct AgingParameter {
    float up1;
    float down1;
    float up2;
    float down2;
    float up3;
    float down3;
}AgingParam;

typedef struct
{
    float a1;
    float b1;
    float a2;
    float b2;
    float a3;
    float b3;
    float a4;
    float b4;
    float Tambient;
} ThermalParams;

typedef struct
{
    float y1;
    float y2;
    float y3;
    float y4;
    ThermalParams p;
} ThermalState;

/* Generic PI controller */
typedef struct {
    /* Tunable parameters */
    float kp;
    float ki;
    float ts;

    /* Output limits */
    float u_min;
    float u_max;

    /* State */
    float integrator;
    float output;
} PI_Controller;

float LossCalc_precise(const GateDriveParams *g, float v, float Im, float fsw);
float LossCalc_linear(const GateDriveParams *g, float V, float Im, float fsw);
void GateDriveParams_init(GateDriveParams *g);
float Tj_estimation(void);

float Thermal_Step(ThermalState *state, float Power);

void Thermal_Init(ThermalState *state,
                  const ThermalModel *model,
                  float Tambient,
                  float Ts);

void ThermalModelInit(ThermalModel *thm);  
void VirtualHeatsink_ThermalModelInit(ThermalModel *thm, float vhs_coeff);



void PI_Init(PI_Controller *pi,float kp, float ki, float ts, float u_min, float u_max, float initial_output);
float PI_Update(PI_Controller *pi, float error);
float current_coupling(float i, float inom);
float ATC(PI_Controller *pid, float Tjref, float Tj, float I_pu, float I_nom, float ATC_activateRange);

float DeadTimeVoltageCompensation(unsigned int fsw, unsigned int deadtime);


#endif
