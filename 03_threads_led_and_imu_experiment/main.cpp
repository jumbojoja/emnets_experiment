#include "board.h"
#include "clk.h"
#include "msg.h"
#include "periph/gpio.h"
#include "periph_conf.h"
#include "shell.h"
#include "thread.h"
#include "timex.h"
#include "ztimer.h"
#include <cmath>
#include <errno.h>
#include <log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
// #include "xtimer.h"
#include "ledcontroller.hh"
#include "mpu6050.h"
#define THREAD_STACKSIZE (THREAD_STACKSIZE_IDLE)
static char stack_for_led_thread[THREAD_STACKSIZE];
static char stack_for_imu_thread[THREAD_STACKSIZE];

static kernel_pid_t _led_pid;
#define LED_MSG_TYPE_ISR (0x3456)
#define LED_MSG_TYPE_RED (0x3001)
#define LED_MSG_TYPE_GREEN (0x3002)
#define LED_MSG_TYPE_YELLO (0x3003)
#define LED_MSG_TYPE_NONE (0x3004)
#define LED_MSG_TYPE_BLUE (0x3005)
#define LED_MSG_TYPE_WHITE (0x3006)
#define LED_GPIO_R GPIO26
#define LED_GPIO_G GPIO25
#define LED_GPIO_B GPIO27
struct MPU6050Data {
  float ax, ay, az;
  float gx, gy, gz;
};
enum MoveState { Stationary, Tilted, Rotating, MovingX, MovingY, MovingZ };

void delay_ms(uint32_t sleep_ms) {
  ztimer_sleep(ZTIMER_USEC, sleep_ms * US_PER_MS);
  return;
}
/**
 * LED control thread function.
 * Then, it enters an infinite loop where it waits for messages to control the
 * LED.
 * @param arg Unused argument.
 * @return NULL.
 */
void *_led_thread(void *arg) {
  (void)arg;
  LEDController led(LED_GPIO_R, LED_GPIO_G, LED_GPIO_B);
  led.change_led_color(4);
  while (1) {
    // Input your codes
    // Wait for a message to control the LED
    // Display different light colors based on the motion state of the device.
    printf("[LED_THREAD] WAIT\n");
    msg_t msg;
    // Wait for the message from OTHER thread
    msg_receive(&msg);
    if (msg.type == LED_MSG_TYPE_RED) {
      // TURN ON LIGHT
      led.change_led_color(1);
      // printf("[LED_THREAD]: LED TURN OFF!!\n");
    } else if (msg.type == LED_MSG_TYPE_GREEN) {
      led.change_led_color(2);
    } else if (msg.type == LED_MSG_TYPE_YELLO) {
      led.change_led_color(3);
    } else if (msg.type == LED_MSG_TYPE_NONE) {
      led.change_led_color(4);
    } else if (msg.type == LED_MSG_TYPE_BLUE) {
      led.change_led_color(5);
    } else if (msg.type == LED_MSG_TYPE_WHITE) {
      led.change_led_color(6);
    }
  }
  return NULL;
}

#define g_acc (9.8)
MoveState detectMovement(MPU6050Data &data) {
  // Input your code
  // Please use your imagination or search online
  // to determine the motion state of the device
  // based on the data obtained from the MPU6050 sensor.
  if ((data.ax > -0.5 || data.ax < 1.5) && (data.ay > 5.0 || data.ay < -5.0) &&
      (data.az < 8.0) && (data.gx > -0.9 || data.gx < 0.0) &&
      (data.gy > -1.5 || data.gy < -0.3) && (data.gz > -0.3 || data.gz < 0.5)) {
    return Tilted;
  } else if ((data.ax > 5.0 || data.ax < -5.0) &&
             (data.ay > -0.5 || data.ay < 1.5) && (data.az < 8.0) &&
             (data.gx > -0.9 || data.gx < 0.0) &&
             (data.gy > -1.5 || data.gy < -0.3) &&
             (data.gz > -0.3 || data.gz < 0.5)) {
    return Tilted;
  } else if ((data.ax > 0.0 || data.ax < 1.0) &&
             (data.ay > -0.4 || data.ay < 0.4) &&
             (data.az > 9.0 || data.az < 10.0) &&
             ((data.gx > 120.0 || data.gx < -120.0) ||
              (data.gy > 120.0 || data.gy < -120.0) ||
              (data.gz > 120.0 || data.gz < -120.0))) {
    return Rotating;
  } else if ((data.ax > 2.5 || data.ax < -1.5) &&
             (data.ay > -0.4 || data.ay < 0.4) &&
             (data.az > 9.0 || data.az < 10.0) &&
             (data.gx > -0.9 || data.gx < 0.0) &&
             (data.gy > -1.5 || data.gy < -0.3) &&
             (data.gz > -0.3 || data.gz < 0.5)) {
    return MovingX;
  } else if ((data.ax > 0.0 || data.ax < 1.0) &&
             (data.ay > -0.4 || data.ay < 0.4) &&
             (data.az > 11.0 || data.az < 7.0) &&
             (data.gx > -0.9 || data.gx < 0.0) &&
             (data.gy > -1.5 || data.gy < -0.3) &&
             (data.gz > -0.3 || data.gz < 0.5)) {
    return MovingZ;
  } else if ((data.ax > 0.0 || data.ax < 1.0) &&
             (data.ay > 1.5 || data.ay < -1.6) &&
             (data.az > 9.0 || data.az < 10.0) &&
             (data.gx > -0.9 || data.gx < 0.0) &&
             (data.gy > -1.5 || data.gy < -0.3) &&
             (data.gz > -0.3 || data.gz < 0.5)) {
    return MovingY;
  }

  return Stationary;
}

void *_imu_thread(void *arg) {
  (void)arg;
  // Input your code
  // 1. initial mpu6050 sensor
  // 2. Acquire sensor data every 100ms
  // 3. Determine the motion state
  // 4. notify the LED thread to display the light color through a message.
  // Initialize MPU6050 sensor

  MPU6050 mpu;
  // get mpu6050 device id
  uint8_t device_id = mpu.getDeviceID();
  // get mpu6050 device id
  printf("[IMU_THREAD] DEVICE_ID:0x%x\n", device_id);
  mpu.initialize();
  // Configure gyroscope and accelerometer full scale ranges
  uint8_t gyro_fs = mpu.getFullScaleGyroRange();
  uint8_t accel_fs_g = mpu.getFullScaleAccelRange();
  uint16_t accel_fs_real = 1;
  float gyro_fs_convert = 1.0;

  // Convert gyroscope full scale range to conversion factor
  if (gyro_fs == MPU6050_GYRO_FS_250)
    gyro_fs_convert = 131.0;
  else if (gyro_fs == MPU6050_GYRO_FS_500)
    gyro_fs_convert = 65.5;
  else if (gyro_fs == MPU6050_GYRO_FS_1000)
    gyro_fs_convert = 32.8;
  else if (gyro_fs == MPU6050_GYRO_FS_2000)
    gyro_fs_convert = 16.4;
  else
    printf("[IMU_THREAD] Unknown GYRO_FS: 0x%x\n", gyro_fs);

  // Convert accelerometer full scale range to real value
  if (accel_fs_g == MPU6050_ACCEL_FS_2)
    accel_fs_real = g_acc * 2;
  else if (accel_fs_g == MPU6050_ACCEL_FS_4)
    accel_fs_real = g_acc * 4;
  else if (accel_fs_g == MPU6050_ACCEL_FS_8)
    accel_fs_real = g_acc * 8;
  else if (accel_fs_g == MPU6050_ACCEL_FS_16)
    accel_fs_real = g_acc * 16;
  else
    printf("[IMU_THREAD] Unknown ACCEL_FS: 0x%x\n", accel_fs_g);

  // Calculate accelerometer conversion factor
  float accel_fs_convert = 32768.0 / accel_fs_real;

  // Initialize variables
  int16_t ax, ay, az, gx, gy, gz;
  MoveState state = Stationary;
  delay_ms(1000);
  // Main loop
  while (1) {
    // Read sensor data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    delay_ms(200);
    MPU6050Data data;
    msg_t msg;
    // Convert raw sensor data to real values
    data.ax = ax / accel_fs_convert;
    data.ay = ay / accel_fs_convert;
    data.az = az / accel_fs_convert;
    data.gx = gx / gyro_fs_convert;
    data.gy = gy / gyro_fs_convert;
    data.gz = gz / gyro_fs_convert;
    // detectMovement
    state = detectMovement(data);
    if (state == MovingX) {
      msg.type = LED_MSG_TYPE_RED;
    } else if (state == MovingY) {
      msg.type = LED_MSG_TYPE_GREEN;
    } else if (state == MovingZ) {
      msg.type = LED_MSG_TYPE_YELLO;
    } else if (state == Stationary) {
      msg.type = LED_MSG_TYPE_NONE;
    } else if (state == Tilted) {
      msg.type = LED_MSG_TYPE_BLUE;
    } else if (state == Rotating) {
      msg.type = LED_MSG_TYPE_WHITE;
    }
    if (msg_send(&msg, _led_pid) <= 0) {
      printf("[SLEEP_THREAD]: possibly lost interrupt.\n");
    } else {
      printf("[SLEEP_THREAD]: Successfully set interrupt.\n");
    }
    // Print sensor data and balance angle
    printf("----------------------------------------\n");
    printf("[IMU_THREAD] (X,Y,Z):(%.02f,%.02f,%.02f)(m/s^2), "
           "(XG,YG,ZG):(%.02f,%.02f,%.02f)(°/s)\n",
           data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
  }

  // uint16_t sleep_ms = 1000;
  // uint8_t color = 0;
  // while(1){
  //     // sleep 1000 ms
  //     ztimer_sleep(ZTIMER_USEC, sleep_ms * US_PER_MS);
  //     printf("[SLEEP_THREAD]: SLEEP FINISH\n");
  //     msg_t msg;
  //     color++;
  //     if (color % 6 == 1)
  //     {
  //         msg.type = LED_MSG_TYPE_RED;
  //     }
  //     else if (color % 6 == 2)
  //     {
  //         msg.type = LED_MSG_TYPE_GREEN;
  //     }
  //     else if (color % 6 == 3)
  //     {
  //         msg.type = LED_MSG_TYPE_YELLO;
  //     }
  //     else if (color % 6 == 4)
  //     {
  //         msg.type = LED_MSG_TYPE_NONE;
  //     }
  //     else if (color % 6 == 5)
  //     {
  //         msg.type = LED_MSG_TYPE_BLUE;
  //     }
  //     else if (color % 6 == 0)
  //     {
  //         msg.type = LED_MSG_TYPE_WHITE;
  //     }
  //     // send the message to the led thread(_led_pid)
  //     if (msg_send(&msg, _led_pid) <= 0){
  //         printf("[SLEEP_THREAD]: possibly lost interrupt.\n");
  //     }
  //     else{
  //         printf("[SLEEP_THREAD]: Successfully set interrupt.\n");
  //     }
  // }
  return NULL;
}
static const shell_command_t shell_commands[] = {{NULL, NULL, NULL}};

int main(void) {
  _led_pid = thread_create(stack_for_led_thread, sizeof(stack_for_led_thread),
                           THREAD_PRIORITY_MAIN - 2, THREAD_CREATE_STACKTEST,
                           _led_thread, NULL, "led_controller_thread");
  if (_led_pid <= KERNEL_PID_UNDEF) {
    printf("[MAIN] Creation of receiver thread failed\n");
    return 1;
  }
  thread_create(stack_for_imu_thread, sizeof(stack_for_imu_thread),
                THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, _imu_thread,
                NULL, "imu_read_thread");
  printf("[Main] Initialization successful - starting the shell now\n");
  while (1) {
  }
  return 0;
}
