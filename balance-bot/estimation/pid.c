// PID controller turns tilt error into a motor output (no hardware)

#include "pid.h"

#define Ke 0.1f
#define Kd 0.1f

// goal theta (we want it at 90 degrees)
#define theta_goal 90.0f


float pid_loop(float theta, float thetadot) {
    float err = theta_goal - theta;
    float u = Ke * err + Kd * thetadot;
    return u;
}