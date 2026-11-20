// PID controller turns tilt error into a motor output (no hardware)

#include "pid.h"
#include "imu.c"

#define Ke 0.1
#define Kd 0.1

// goal theta (we want it at 90 degrees)
#define theta_goal 90


float pid_loop(float theta, float thetadot) {
    theta_d = 0;
    err = theta_goal - theta;
    u = Ke * err + Kd * thetadot;
    return u;
}