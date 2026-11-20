// PID controller turns tilt error into a motor output (no hardware)

#include "pid.h"

#define Ke 0.04f
#define Kd 0.0f //damping

// goal theta (we want it at 90 degrees)
#define theta_goal 90.0f


float pid_loop(float theta, float thetadot) {
    //when robot is laying flat do not make motors spin at full speed
    float err = theta_goal - theta;
    float u = Ke * err + Kd * thetadot;
    return u;
}