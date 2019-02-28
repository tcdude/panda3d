#include "equation-solver.h"

#define _USE_MATH_DEFINES
#include <cmath>

int solve_quadratic(PN_stdfloat x[2], PN_stdfloat a, PN_stdfloat b, PN_stdfloat c) {
    if (fabs(a) < 1e-14) {
        if (fabs(b) < 1e-14) {
            if (c == 0)
                return -1;
            return 0;
        }
        x[0] = -c/b;
        return 1;
    }
    PN_stdfloat dscr = b*b-4*a*c;
    if (dscr > 0) {
        dscr = sqrt(dscr);
        x[0] = (-b+dscr)/(2*a);
        x[1] = (-b-dscr)/(2*a);
        return 2;
    } else if (dscr == 0) {
        x[0] = -b/(2*a);
        return 1;
    } else
        return 0;
}

int solve_cubic(PN_stdfloat x[3], PN_stdfloat a, PN_stdfloat b, PN_stdfloat c, PN_stdfloat d) {
    if (fabs(a) < 1e-14)
        return solve_quadratic(x, b, c, d);
    return solve_cubic_normed(x, b/a, c/a, d/a);
}

int solve_cubic_normed(PN_stdfloat *x, PN_stdfloat a, PN_stdfloat b, PN_stdfloat c) {
    PN_stdfloat a2 = a*a;
    PN_stdfloat q  = (a2 - 3*b)/9;
    PN_stdfloat r  = (a*(2*a2-9*b) + 27*c)/54;
    PN_stdfloat r2 = r*r;
    PN_stdfloat q3 = q*q*q;
    PN_stdfloat A, B;
    if (r2 < q3) {
        PN_stdfloat t = r/sqrt(q3);
        if (t < -1) t = -1;
        if (t > 1) t = 1;
        t = acos(t);
        a /= 3; q = -2*sqrt(q);
        x[0] = q*cos(t/3)-a;
        x[1] = q*cos((t+2*M_PI)/3)-a;
        x[2] = q*cos((t-2*M_PI)/3)-a;
        return 3;
    } else {
        A = -pow(fabs(r)+sqrt(r2-q3), 1/3.);
        if (r < 0) A = -A;
        B = A == 0 ? 0 : q/A;
        a /= 3;
        x[0] = (A+B)-a;
        x[1] = -0.5*(A+B)-a;
        x[2] = 0.5*sqrt(3.)*(A-B);
        if (fabs(x[2]) < 1e-14)
            return 2;
        return 1;
    }
}