// I2C accelerometer app
//
// Read from I2C accelerometer on the Microbit

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "nrf_delay.h"
#include "nrf_twi_mngr.h"

#include "microbit_v2.h"
#include "imu.h"
#include "app_timer.h"
#include "pid.h"
#include "motor.h"
#include "logger.h"

// #define UMAX 20
#define UMAX 20

// Global variables
NRF_TWI_MNGR_DEF(twi_mngr_instance, 1, 0);
APP_TIMER_DEF(acc_timer);

static void acc_timer_callback(void* _unused) {
  lsm303agr_measurement_t acc = lsm303agr_read_accelerometer();
  lsm303agr_measurement_t gyro = lsm303agr_read_gyroscope();
  float tilt = lsm303agr_tilt_angle(acc);

  // printf("Acceleration (g): x=%.3f y=%.3f z=%.3f\n", acc.x_axis, acc.y_axis, acc.z_axis);

  // printf("Tilt angle (phi): %.2f deg\n", tilt);

  float pitch = lsm303agr_pitch_angle(acc);

  // printf("Pitch: %.2f deg  Roll: %.2f deg\n", pitch, lsm303agr_roll_angle(acc));

  //rotation rate in degress per sec
  // printf("Gyroscope (deg/s): x=%.3f y=%.3f z=%.3f\n\n", gyro.x_axis, gyro.y_axis, gyro.z_axis);

  //PID stuff incoming
  //theta thetadot
  // float u = pid_loop(tilt, gyro.x_axis);

  // static float pitch_f = 0.0f;
  // pitch_f = 0.8f * pitch_f + 0.2f * pitch;
  // float u = pid_loop(pitch_f, gyro.x_axis);

  // static float theta = 0.0f;
  // static bool theta_init = false;

  // if (!theta_init) { theta = pitch; theta_init = true; }
  // theta = 0.98f * (theta - gyro.x_axis * 0.005f) + 0.02f * pitch;

  // float u = pid_loop(theta, gyro.x_axis);

  static float theta = 0.0f;
  static bool theta_init = false;
  
  if (!theta_init) {
    // we need to reset the theta in every lo
    theta = pitch; 
    theta_init = true;
  }
  theta = 0.98f * (theta - gyro.y_axis * 0.005f) + 0.02f * pitch;

  static float u_avg = 0.0f;
  // float u = pid_loop(theta, -gyro.y_axis);
  float u = pid_loop(theta - 0.1f * u_avg, -gyro.y_axis);

  // printf("u value: u=%.3f \n\n", u);

  // if (fabsf(theta) > 20.0f) {
  //   u = 0;
  //   theta = pitch;
  // }

  if (fabsf(theta) > 20.0f) {
    u = 0;
    if (fabsf(pitch) > 20.0f) {
      theta = pitch;
    }
  }
  
  if (fabsf(u) < 1.0f) u = 0;

  //testing motor
  // set_motor(0.5f, 1.0f);
  u_avg = 0.99f * u_avg + 0.01f * u;
  set_motor(u, UMAX);
  log_sample(theta, pitch, gyro, u);
}

void run_IMU(void){
  // Initialize I2C peripheral and driver
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;

  // For external I2C communication (QUIIC):
  // EDGE_P19 (a.k.a. I2C_QWIIC_SCL) and EDGE_P20 (a.k.a. I2C_QWIIC_SDA)
  i2c_config.scl = EDGE_P19;
  i2c_config.sda = EDGE_P20;
  // i2c_config.frequency = NRF_DRV_TWI_FREQ_100K;
  i2c_config.frequency = NRF_DRV_TWI_FREQ_400K;
  i2c_config.interrupt_priority = 0;
  nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);

  // Initialize the LSM303AGR accelerometer sensor
  lsm303agr_init(&twi_mngr_instance);
}


// Read IMU -> Compute the angle -> Run PID -> motor_set()

int main(void) {
  printf("Board started!\n");
  run_IMU();
  motor_init();


  app_timer_init();
  app_timer_create(&acc_timer, APP_TIMER_MODE_REPEATED, acc_timer_callback);

  //100 times a second
  app_timer_start(acc_timer, APP_TIMER_TICKS(5), NULL);

  // Loop forever
  while (1) {
    // Don't put any code in here. Instead put periodic code in a callback using a timer.
    nrf_delay_ms(1000);
  }
}