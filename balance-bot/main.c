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
#include "encoder.h"
#include "light.h"
// #include "sound.h"

// tilt angle in degrees (filtered)
float theta = 0.0f;

//#include "logger.h"

// #define UMAX 20

#define UMAX 34

// Global variables
NRF_TWI_MNGR_DEF(twi_mngr_instance, 1, 0);
APP_TIMER_DEF(acc_timer);
APP_TIMER_DEF(light_timer);

static void light_timer_callback(void* _unused) {
  float lux = light_read_lux();
  printf("Light: %.2f lux\n", lux);
}

static void acc_timer_callback(void* _unused) {
  lsm303agr_measurement_t acc = lsm303agr_read_accelerometer();
  lsm303agr_measurement_t gyro = lsm303agr_read_gyroscope();

  // printf("Acceleration (g): x=%.3f y=%.3f z=%.3f\n", acc.x_axis, acc.y_axis, acc.z_axis);

  // printf("Tilt angle (phi): %.2f deg\n", tilt);

  float pitch = lsm303agr_pitch_angle(acc);

  // printf("Pitch: %.2f deg  Roll: %.2f deg\n", pitch, lsm303agr_roll_angle(acc));

  //rotation rate in degress per sec
  // printf("Gyroscope (deg/s): x=%.3f y=%.3f z=%.3f\n\n", gyro.x_axis, gyro.y_axis, gyro.z_axis);


  static bool theta_init = false;
  
  if (!theta_init) {
    // we need to reset the theta in every lo
    theta = pitch; 
    theta_init = true;
  }

  
  theta = 0.98f * (theta - gyro.y_axis * 0.005f) + 0.02f * pitch;

  static float u_avg = 0.0f;

  static int dir_count = 0;
  static int kick_left = 0;
  static float kick_dir = 0.0f;
  float kick_shift = 0.0f;
  if (kick_left > 0) {
    // kick_shift = kick_dir * 3.0f;
    kick_shift = kick_dir * 0.0f;
    kick_left--;
  }

  //how far wheels turned since they were powered on
  float x = encoder_position();
  static float x_prev = 0.0f;
  static float v = 0.0f;

  if (nrf_gpio_pin_read(BTN_A) == 0) {
    encoder_reset();
    x = 0.0f;
    x_prev = 0.0f;
    v = 0.0f;
    theta = pitch;
  }

  if (nrf_gpio_pin_read(BTN_B) == 0) {
    NVIC_SystemReset();
  }
  
  //speed
  v = 0.8f * v + 0.2f * (x - x_prev) / 0.005f;
  x_prev = x;
  float u = pid_loop(theta - kick_shift, -gyro.y_axis, v, x);


  if (fabsf(theta) > 30.0f) {
    u = 0;
    if (fabsf(pitch) > 30.0f) {
      theta = pitch;
    }
  }
  
  if (fabsf(u) < 1.0f) u = 0;

  //testing motor
  // set_motor(0.5f, 1.0f);
  u_avg = 0.99f * u_avg + 0.01f * u;

  if (u > 0.0f) {
    dir_count = (dir_count > 0) ? dir_count + 1 : 1;
  } else if (u < 0.0f) {
    dir_count = (dir_count < 0) ? dir_count - 1 : -1;
  } else {
    dir_count = 0;
  }
  if (kick_left == 0 && (dir_count >= 100 || dir_count <= -100)) {
    kick_dir = (dir_count > 0) ? 1.0f : -1.0f;
    kick_left = 10;
    dir_count = 0;
  }

  set_motor(u, UMAX);
  // tilt_sound();

  // print 5 times a second instead of every 5 ms loop (printing every loop slows the controller)
  static int print_count = 0;
  if (++print_count >= 40) {
    print_count = 0;
    printf("theta=%.2f pitch=%.2f gyro_y=%.2f x=%.3f v=%.3f u=%.2f\n", theta, pitch, gyro.y_axis, x, v, u);
    encoder_print_raw(); // DEBUG
  }
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
  light_init(&twi_mngr_instance);
  motor_init();
  encoder_init();
  nrf_gpio_cfg_input(BTN_A, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(BTN_B, NRF_GPIO_PIN_NOPULL);


  app_timer_init();
  app_timer_create(&acc_timer, APP_TIMER_MODE_REPEATED, acc_timer_callback);
  app_timer_create(&light_timer, APP_TIMER_MODE_REPEATED, light_timer_callback);

  //100 times a second
  app_timer_start(acc_timer, APP_TIMER_TICKS(5), NULL);
  app_timer_start(light_timer, APP_TIMER_TICKS(500), NULL);

  // Loop forever
  while (1) {
    // Don't put any code in here. Instead put periodic code in a callback using a timer.
    nrf_delay_ms(1000);
  }
}