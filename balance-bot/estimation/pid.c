// PID controller turns tilt error into a motor output (no hardware)

#include "pid.h"


// #define Kp 2.5f
// #define Kd 0.12f
// #define Kv -5.0f
// #define Kx -5.0f

#define Kp 2.5f
#define Kd 0.12f
#define Kv -5.0f
#define Kx -5.0f

// best?
// #define Kp 3.0f
// #define Kd 0.12f
// #define Kv -5.0f
// #define Kx -5.0f


// goal theta (we want it at 90 degrees)
// #define theta_goal 0.0f
// #define theta_goal (-3.0f)
// neg is towards not screen side
#define theta_goal (2.0f)

// #define theta_goal (2.0f)

// float pid_loop(float theta, float thetadot) {
// x is how far the wheels have turned since power-on
// v is how fast the wheels are rolling (wheel turns per second)
float pid_loop(float theta, float thetadot, float v, float x) {
    //when robot is laying flat do not make motors spin at full speed
    float err = theta_goal - theta;
    // float u = Kp * err + Kd * thetadot;
    // float u = Kp * err - Kd * thetadot;
    float u = Kp * err - Kd * thetadot - Kv * v - Kx * x;
    return u;
}

