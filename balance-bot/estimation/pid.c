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


#define Kp 2.0f
#define Kd 0.08f //damping
#define Kv 2.9f
#define Kx 2.9f

// goal theta (we want it at 90 degrees)
// #define theta_goal 0.0f
// #define theta_goal (-3.0f)
#define theta_goal (-1.7f)

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

// float pid_loop(float theta, float thetadot) {
//     //when robot is laying flat do not make motors spin at full speed
//     float err = theta_goal - theta;
//     // float u = Kp * err + Kd * thetadot;
//     float u = Kp * err - Kd * thetadot;
//     return u;
// }

// #define Kp 1.2f
// #define Kd 0.15f //damping
// #define Ki 0.5f
// #define time_diff 0.005f //time between loops 
// #define I_MAX 10.0f //maximum error value

// // goal theta (we want it at 90 degrees)
// // #define theta_goal 0.0f
// #define theta_goal (-3.55f)


// float pid_loop(float theta, float thetadot) {
//     //when robot is laying flat do not make motors spin at full speed
//     float err = theta_goal - theta;
//     // the angle error over time
//     static float total_err = 0.0f;

//     // only add to error when its less than 20 degrees
//     if (fabsf(err) < 20.0f) {
//         // turning this num degrees in this many seconds
//         total_err += err * time_diff;
//         total_err = fminf(fmaxf(total_err, -I_MAX), I_MAX);
//     }

//     // float u = Kp * err + Kd * thetadot;
//     // float u = Kp * err - Kd * thetadot;
//     float u = Kp * err + Ki * total_err - Kd * thetadot;
//     return u;
// }
