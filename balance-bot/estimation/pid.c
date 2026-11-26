// PID controller turns tilt error into a motor output (no hardware)

#include "pid.h"

//Original
// Kp =1.2
// Kd = .15

//Pretty Good
//1.5
//.20

// #define Kp 2.0f
// #define Kd 0.1f //damping
// #define Kv -5.0f
// #define Kx 0.0f

// should work with no push?
// #define Kp 2.0f
// #define Kd 0.08f //damping
// #define Kv -2.9f
// #define Kx -2.9f

// best?
#define Kp 3.0f
#define Kd 0.12f
#define Kv -4.0f
#define Kx -4.0f


// goal theta (we want it at 90 degrees)
// #define theta_goal 0.0f
// #define theta_goal (-3.0f)
// neg is towards not screen side
#define theta_goal (-3.98f)

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

