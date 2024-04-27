#ifndef MOTOR_INFO_HPP
#define MOTOR_INFO_HPP

#include <iostream>

/**
 * The reduction ratio of LIFT, PAN, Steering, respectively.
*/
const double N_LIFT = 158.0;
const double N_PAN = 3969.0/256.0;
const double N_STEERING = 70.0/40.0;

/**
 * Count per turn Info
*/

const double LIFT_QCPT   = 4*4096.0;
const double PAN_QCPT    = 4*1024.0;
const double DXL_QCPT    = 4096.0;

/**
 * inc to Degree
*/
const double inc2deg_LIFT   = 360.0 / (LIFT_QCPT*N_LIFT);
const double inc2deg_PAN    = 360.0 / (PAN_QCPT*N_PAN);
const double inc2deg_DXL    = 360.0 / (DXL_QCPT*N_STEERING);

// deg to inc
const double deg2target_LIFT    = LIFT_QCPT/360.0 * N_LIFT;

#endif