// PID controller turns tilt error into a motor output (no hardware)

#include "pid.h"

#define Kp 0.8f
#define Kd 0.05f //damping

// goal theta (we want it at 90 degrees)
// #define theta_goal 0.0f
#define theta_goal (-3.55f)


float pid_loop(float theta, float thetadot) {
    //when robot is laying flat do not make motors spin at full speed
    float err = theta_goal - theta;
    // float u = Kp * err + Kd * thetadot;
    float u = Kp * err - Kd * thetadot;
    return u;
}