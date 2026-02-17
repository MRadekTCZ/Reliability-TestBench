#include <math.h>
#ifndef ClarkPark_Math_H_
#define ClarkPark_Math_H_
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
};

