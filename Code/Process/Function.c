#include"Function.h"
#include<math.h>

double Curve_Mult(double x) {
    double M = 1.2;
    double N = 2;

    double t = x - 1.0;
    double base = 1.0 - pow(t * t, M);
    return pow(base, N);
}
