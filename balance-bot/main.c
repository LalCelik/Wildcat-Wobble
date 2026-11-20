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

#define UMAX 1

// Global variables
NRF_TWI_MNGR_DEF(twi_mngr_instance, 1, 0);
APP_TIMER_DEF(acc_timer);

static void acc_timer_callback(void* _unused) {
  lsm303agr_measurement_t acc = lsm303agr_read_accelerometer();
  lsm303agr_measurement_t gyro = lsm303agr_read_gyroscope();
  float tilt = lsm303agr_tilt_angle(acc);

  printf("Acceleration (g): x=%.3f y=%.3f z=%.3f\n", acc.x_axis, acc.y_axis, acc.z_axis);
  printf("Tilt angle (phi): %.2f deg\n", tilt);
  printf("Pitch: %.2f deg  Roll: %.2f deg\n", lsm303agr_pitch_angle(acc), lsm303agr_roll_angle(acc));

  //rotation rate in degress per sec
  printf("Gyroscope (deg/s): x=%.3f y=%.3f z=%.3f\n\n", gyro.x_axis, gyro.y_axis, gyro.z_axis);

  //PID stuff incoming
  //theta thetadot
  float u = pid_loop(tilt, gyro.x_axis);
  printf("u value: u=%.3f \n\n", u);

  // only start tilting when we pick up robot
  if(fabs(titl - 90.0f) > 40.0f) {
    u = 0;
  }
  set_motor(u, UMAX);

}

void run_IMU(void){
  // Initialize I2C peripheral and driver
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;

  // For external I2C communication (QUIIC):
  // EDGE_P19 (a.k.a. I2C_QWIIC_SCL) and EDGE_P20 (a.k.a. I2C_QWIIC_SDA)
  i2c_config.scl = EDGE_P19;
  i2c_config.sda = EDGE_P20;
  i2c_config.frequency = NRF_DRV_TWI_FREQ_100K;
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

  //4 times a second 
  app_timer_start(acc_timer, APP_TIMER_TICKS(10), NULL);

  // Loop forever
  while (1) {
    // Don't put any code in here. Instead put periodic code in a callback using a timer.
    nrf_delay_ms(1000);
  }
}