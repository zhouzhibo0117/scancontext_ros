//
// Created by zhou on 9/30/19.
//

#include "util/math_util.h"

double GetTheta(double x, double y) {
    double theta = atan2(y, x);
    if (theta <= 0) {
        theta += 2 * M_PI;
    }
    return theta;
}
