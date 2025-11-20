#pragma once
#include "transform.h"
#include "jacobian_inverse.h"

class Robot {
    public:
    vec<6> q_rad;
    transform tf;










    private:
    virtual mat<12, 6> jcobian(double q0, double q1, double q2, double q3, double q4, double q5) = 0; // [rad]
};




