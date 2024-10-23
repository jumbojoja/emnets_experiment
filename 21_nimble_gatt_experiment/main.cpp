/*
 * Copyright (C) 2018 Freie Universität Berlin
 *               2018 Codecoup
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       BLE peripheral example using NimBLE
 *
 * Have a more detailed look at the api here:
 * https://mynewt.apache.org/latest/tutorials/ble/bleprph/bleprph.html
 *
 * More examples (not ready to run on RIOT) can be found here:
 * https://github.com/apache/mynewt-nimble/tree/master/apps
 *
 * Test this application e.g. with Nordics "nRF Connect"-App
 * iOS: https://itunes.apple.com/us/app/nrf-connect/id1054362403
 * Android:
 * https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Andrzej Kaczmarek <andrzej.kaczmarek@codecoup.pl>
 * @author      Hendrik van Essen <hendrik.ve@fu-berlin.de>
 *
 * @}
 */
#include "board.h"
#include "clk.h"
#include "msg.h"
#include "periph/gpio.h"
#include "periph_conf.h"
#include "thread.h"
#include "timex.h"
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
extern "C" {
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "log.h"
#include "nimble_autoadv.h"
#include "nimble_riot.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
}
#include "ledcontroller.hh"
#include "mpu6050.h"

using namespace std;
void setup();
int predict(float *imu_data, int data_len, float threashold, int class_num);
#define LED_GPIO_R GPIO26
#define LED_GPIO_G GPIO25
#define LED_GPIO_B GPIO27
#define g_acc (9.8)
#define class_num (6)
#define SAMPLES_PER_GESTURE (10)
#define THREAD_STACKSIZE (THREAD_STACKSIZE_IDLE)
#define LED_MSG_TYPE_ISR (0x3456)
#define LED_MSG_TYPE_RED (0x3001)
#define LED_MSG_TYPE_GREEN (0x3002)
#define LED_MSG_TYPE_YELLO (0x3003)
#define LED_MSG_TYPE_NONE (0x3004)
#define LED_MSG_TYPE_BLUE (0x3005)
#define LED_MSG_TYPE_WHITE (0x3006)
static char stack_for_led_thread[THREAD_STACKSIZE];
static char stack_for_motion_thread[THREAD_STACKSIZE];
string all_motions[class_num] = {"Stationary", "Tilted",  "Rotating",
                                 "MovingX",    "MovingY", "MovingZ"};
int p = 0;
float threshold = 0.6;

#define DEMO_BUFFER_SIZE 100
// static char rm_demo_write_data[DEMO_BUFFER_SIZE] = "This characteristic is
// read- and writeable!";
#define STR_ANSWER_BUFFER_SIZE 100
// static char str_answer[STR_ANSWER_BUFFER_SIZE];

static kernel_pid_t _led_pid;
static kernel_pid_t _motion_pid;
struct MPU6050Data {
  float ax, ay, az; // acceler_x_axis, acceler_y_axis, acceler_z_axis
  float gx, gy, gz; // gyroscope_x_axis, gyroscope_y_axis, gyroscope_z_axis
};
void delay_ms(uint32_t sleep_ms) {
  ztimer_sleep(ZTIMER_USEC, sleep_ms * US_PER_MS);
  return;
}
// static int r_led_state = 0;
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

float gyro_fs_convert = 1.0;
float accel_fs_convert;
static int collect_interval_ms = 20;
static int predict_interval_ms = 200;

void get_imu_data(MPU6050 mpu, float *imu_data) {
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < SAMPLES_PER_GESTURE; ++i) {
    // i += 1;
    /* code */
    delay_ms(collect_interval_ms);
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    imu_data[i * 6 + 0] = ax / accel_fs_convert;
    imu_data[i * 6 + 1] = ay / accel_fs_convert;
    imu_data[i * 6 + 2] = az / accel_fs_convert;
    imu_data[i * 6 + 3] = gx / gyro_fs_convert;
    imu_data[i * 6 + 4] = gy / gyro_fs_convert;
    imu_data[i * 6 + 5] = gz / gyro_fs_convert;
  }
}
void *_motion_thread(void *arg) {
  (void)arg;
  // Initialize MPU6050 sensor
  MPU6050 mpu;
  // get mpu6050 device id
  uint8_t device_id = mpu.getDeviceID();
  printf("[IMU_THREAD] DEVICE_ID:0x%x\n", device_id);
  mpu.initialize();
  // Configure gyroscope and accelerometer full scale ranges
  uint8_t gyro_fs = mpu.getFullScaleGyroRange();
  uint8_t accel_fs_g = mpu.getFullScaleAccelRange();
  uint16_t accel_fs_real = 1;

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
  accel_fs_convert = 32768.0 / accel_fs_real;
  float imu_data[SAMPLES_PER_GESTURE * 6] = {0};
  int data_len = SAMPLES_PER_GESTURE * 6;
  delay_ms(200);
  // Main loop
  int ret = 0;
  string motions[class_num] = {"Stationary", "Tilted",  "Rotating",
                               "MovingX",    "MovingY", "MovingZ"};
  while (1) {
    delay_ms(predict_interval_ms);
    // Read sensor data
    get_imu_data(mpu, imu_data);
    ret = predict(imu_data, data_len, threshold, class_num);
    p = ret;
    // tell the led thread to do some operations
    // input your code
    msg_t msg;
    if (ret == 0) {
      msg.type = LED_MSG_TYPE_NONE;
    } else if (ret == 1) {
      msg.type = LED_MSG_TYPE_BLUE;
    } else if (ret == 2) {
      msg.type = LED_MSG_TYPE_WHITE;
    } else if (ret == 3) {
      msg.type = LED_MSG_TYPE_RED;
    } else if (ret == 4) {
      msg.type = LED_MSG_TYPE_GREEN;
    } else if (ret == 5) {
      msg.type = LED_MSG_TYPE_YELLO;
    }
    if (msg_send(&msg, _led_pid) <= 0) {
      printf("[SLEEP_THREAD]: possibly lost interrupt.\n");
    } else {
      printf("[SLEEP_THREAD]: Successfully set interrupt.\n");
    }
    // Print result
    // save motion for gatt server
    printf("Predict: %d, %s\n", ret, motions[ret].c_str());
  }
  return NULL;
}

// input your code, 自定义想要的UUID
/* UUID = 1bce38b3-d137-48ff-a13e-033e14c7a335 */
static const ble_uuid128_t gatt_svr_svc_rw_demo_uuid = {
    {128},
    {0x15, 0xa3, 0xc7, 0x14, 0x3e, 0x03, 0x3e, 0xa1, 0xff, 0x48, 0x37, 0xd1,
     0xb3, 0x38, 0xce, 0x1b}};
static const ble_uuid128_t gatt_svr_chr_rw_demo_writethreshold_uuid = {
    {128},
    {0x62, 0x17, 0x99, 0x7e, 0x50, 0x27, 0x38, 0xba, 0x3b, 0x4f, 0x70, 0x30,
     0x86, 0x83, 0x11, 0x11}};
static const ble_uuid128_t gatt_svr_chr_rw_demo_writedelay_uuid = {
    {128},
    {0x62, 0x17, 0x99, 0x7e, 0x50, 0x27, 0x38, 0xba, 0x3b, 0x4f, 0x70, 0x30,
     0x86, 0x83, 0x22, 0x22}};
static const ble_uuid128_t gatt_svr_chr_rw_demo_writeled_uuid = {
    {128},
    {0x62, 0x17, 0x33, 0x33, 0x50, 0x27, 0x38, 0xba, 0x3b, 0x4f, 0x70, 0x30,
     0x86, 0x83, 0x00, 0x03}};
static const ble_uuid128_t gatt_svr_chr_rw_demo_readmove_uuid = {
    {128},
    {0x62, 0x17, 0x99, 0x7e, 0x50, 0x27, 0x38, 0xba, 0x3b, 0x4f, 0x70, 0x30,
     0x86, 0x83, 0x44, 0x44}};

static int gatt_svr_chr_access_rw_demo(uint16_t conn_handle,
                                       uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt,
                                       void *arg);
#define DEMO_BUFFER_SIZE 100
static char rm_demo_write_data[DEMO_BUFFER_SIZE] =
    "This characteristic is read- and writeable!";
#define STR_ANSWER_BUFFER_SIZE 100
static char str_answer[STR_ANSWER_BUFFER_SIZE];

/* define several bluetooth services for our device */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /*
     * access_cb defines a callback for read and write access events on
     * given characteristics
     */
    {/* Service: Read/Write Demo */
     .type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = (ble_uuid_t *)&gatt_svr_svc_rw_demo_uuid.u,
     .characteristics =
         (struct ble_gatt_chr_def[]){
             {
                 /* Characteristic: Write Threshold */
                 .uuid =
                     (ble_uuid_t *)&gatt_svr_chr_rw_demo_writethreshold_uuid.u,
                 .access_cb = gatt_svr_chr_access_rw_demo,
                 .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                 // .flags = BLE_GATT_CHR_F_READ,
                 // .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP,
             },
             {
                 .uuid = (ble_uuid_t *)&gatt_svr_chr_rw_demo_writedelay_uuid.u,
                 .access_cb = gatt_svr_chr_access_rw_demo,
                 .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                 // .flags = BLE_GATT_CHR_F_READ,
             },
             {
                 .uuid = (ble_uuid_t *)&gatt_svr_chr_rw_demo_writeled_uuid.u,
                 .access_cb = gatt_svr_chr_access_rw_demo,
                 .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                 // .flags = BLE_GATT_CHR_F_READ,
             },
             {
                 .uuid = (ble_uuid_t *)&gatt_svr_chr_rw_demo_readmove_uuid.u,
                 .access_cb = gatt_svr_chr_access_rw_demo,
                 .flags = BLE_GATT_CHR_F_READ,
             },
             {
                 0, /* No more characteristics in this service */
             },
         }},
    {
        0, /* No more services */
    },
};

static int gatt_svr_chr_access_rw_demo(uint16_t conn_handle,
                                       uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt,
                                       void *arg) {
  (void)conn_handle;
  (void)attr_handle;
  (void)arg;
  // input your code
  int rc = 0;
  ble_uuid_t *readmove_uuid =
      (ble_uuid_t *)&gatt_svr_chr_rw_demo_readmove_uuid.u;
  ble_uuid_t *writethreshold_uuid =
      (ble_uuid_t *)&gatt_svr_chr_rw_demo_writethreshold_uuid.u;
  ble_uuid_t *writedelay_uuid =
      (ble_uuid_t *)&gatt_svr_chr_rw_demo_writedelay_uuid.u;
  ble_uuid_t *writeled_uuid =
      (ble_uuid_t *)&gatt_svr_chr_rw_demo_writeled_uuid.u;

  if (ble_uuid_cmp(ctxt->chr->uuid, readmove_uuid) == 0) {
    printf("access to characteristic 'readmove'\n");
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
      snprintf(str_answer, STR_ANSWER_BUFFER_SIZE, "MOVEMENT: %s",
               all_motions[p].c_str());
      printf("%s\n", str_answer);
      rc = os_mbuf_append(ctxt->om, &str_answer, strlen(str_answer));
      return rc;
    }
    return 0;
  } else if (ble_uuid_cmp(ctxt->chr->uuid, writethreshold_uuid) == 0) {
    printf("access to characteristic 'writethreshold'\n");
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
      printf("read from characteristic\n");
      snprintf(str_answer, STR_ANSWER_BUFFER_SIZE, "THRESHOLD: %.2f",
               threshold);
      printf("%s\n", str_answer);
      rc = os_mbuf_append(ctxt->om, &str_answer, strlen(str_answer));
      break;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
      printf("write to characteristic\n");
      uint16_t om_len;
      om_len = OS_MBUF_PKTLEN(ctxt->om);
      rc = ble_hs_mbuf_to_flat(ctxt->om, &rm_demo_write_data,
                               sizeof(rm_demo_write_data), &om_len);
      rm_demo_write_data[om_len] = '\0';
      if (rm_demo_write_data[0] == 0x00) {
        threshold = 0.6;
      } else if (rm_demo_write_data[0] == 0x01) {
        threshold = (threshold > 0.1) ? threshold - 0.1 : threshold;
      } else if (rm_demo_write_data[0] == 0x02) {
        threshold = (threshold < 0.9) ? threshold + 0.1 : threshold;
      }
      break;
    default:
      printf("unhandled operation!\n");
      rc = 1;
      break;
    }
    return rc;
  } else if (ble_uuid_cmp(ctxt->chr->uuid, writedelay_uuid) == 0) {
    printf("access to characteristic 'writedelay'\n");
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
      printf("read from characteristic\n");
      snprintf(str_answer, STR_ANSWER_BUFFER_SIZE, "DELAY: %dms",
               collect_interval_ms);
      printf("%s\n", str_answer);
      rc = os_mbuf_append(ctxt->om, &str_answer, strlen(str_answer));
      break;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
      printf("write to characteristic\n");
      uint16_t om_len;
      om_len = OS_MBUF_PKTLEN(ctxt->om);
      rc = ble_hs_mbuf_to_flat(ctxt->om, &rm_demo_write_data,
                               sizeof(rm_demo_write_data), &om_len);
      rm_demo_write_data[om_len] = '\0';
      if (rm_demo_write_data[0] == 0x00) {
        collect_interval_ms = 20;
      } else if (rm_demo_write_data[0] == 0x01) {
        collect_interval_ms = (collect_interval_ms > 15)
                                  ? collect_interval_ms - 1
                                  : collect_interval_ms;
      } else if (rm_demo_write_data[0] == 0x02) {
        collect_interval_ms = (collect_interval_ms < 1000)
                                  ? collect_interval_ms + 10
                                  : collect_interval_ms;
      }
      break;
    default:
      printf("unhandled operation!\n");
      rc = 1;
      break;
    }
    return rc;
  } else if (ble_uuid_cmp(ctxt->chr->uuid, writeled_uuid) == 0) {
    printf("access to characteristic 'writedelay'\n");
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
      printf("read from characteristic\n");
      snprintf(str_answer, STR_ANSWER_BUFFER_SIZE, "LED: %d", p);
      printf("%s\n", str_answer);
      rc = os_mbuf_append(ctxt->om, &str_answer, strlen(str_answer));
      break;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
      printf("write to characteristic\n");
      uint16_t om_len;
      om_len = OS_MBUF_PKTLEN(ctxt->om);
      rc = ble_hs_mbuf_to_flat(ctxt->om, &rm_demo_write_data,
                               sizeof(rm_demo_write_data), &om_len);
      rm_demo_write_data[om_len] = '\0';
      msg_t msg;
      if (rm_demo_write_data[0] == 0x00) {
        msg.type = LED_MSG_TYPE_NONE;
      } else if (rm_demo_write_data[0] == 0x01) {
        msg.type = LED_MSG_TYPE_RED;
      } else if (rm_demo_write_data[0] == 0x02) {
        msg.type = LED_MSG_TYPE_YELLO;
      } else if (rm_demo_write_data[0] == 0x03) {
        msg.type = LED_MSG_TYPE_BLUE;
      } else if (rm_demo_write_data[0] == 0x04) {
        msg.type = LED_MSG_TYPE_GREEN;
      } else if (rm_demo_write_data[0] == 0x05) {
        msg.type = LED_MSG_TYPE_WHITE;
      }
      msg_send(&msg, _led_pid);
      break;
    default:
      printf("unhandled operation!\n");
      rc = 1;
      break;
    }
    return rc;
  }
  (void)ctxt;
  return 1;
}

int main(int argc, char *argv[]) {

  (void)argc;
  (void)argv;
  setup();
  // create led thread
  _led_pid = thread_create(stack_for_led_thread, sizeof(stack_for_led_thread),
                           THREAD_PRIORITY_MAIN - 2, THREAD_CREATE_STACKTEST,
                           _led_thread, NULL, "led_controller_thread");
  if (_led_pid <= KERNEL_PID_UNDEF) {
    printf("[MAIN] Creation of receiver thread failed\n");
    return 1;
  } else {
    printf("[MAIN] LED_PID: %d\n", _led_pid);
  }
  // create motion thread
  _motion_pid =
      thread_create(stack_for_motion_thread, sizeof(stack_for_motion_thread),
                    THREAD_PRIORITY_MAIN - 2, THREAD_CREATE_STACKTEST,
                    _motion_thread, NULL, "motion_predict_thread");
  if (_motion_pid <= KERNEL_PID_UNDEF) {
    printf("[MAIN] Creation of receiver thread failed\n");
    return 1;
  } else {
    printf("[MAIN] MOTION_PID: %d\n", _motion_pid);
  }

  int rc = 0;
  (void)rc;

  /* verify and add our custom services */
  rc = ble_gatts_count_cfg(gatt_svr_svcs);
  assert(rc == 0);
  rc = ble_gatts_add_svcs(gatt_svr_svcs);
  assert(rc == 0);

  /* set the device name */
  ble_svc_gap_device_name_set(CONFIG_NIMBLE_AUTOADV_DEVICE_NAME);
  /* reload the GATT server to link our added services */
  ble_gatts_start();

  // 获取蓝牙设备的默认 MAC 地址
  uint8_t own_addr_type;
  uint8_t own_addr[6];
  ble_hs_id_infer_auto(0, &own_addr_type);
  ble_hs_id_copy_addr(own_addr_type, own_addr, NULL);

  // 打印 MAC 地址
  LOG_INFO("Default MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n", own_addr[5],
           own_addr[4], own_addr[3], own_addr[2], own_addr[1], own_addr[0]);
  /* start to advertise this node */
  nimble_autoadv_start(NULL);

  return 0;
}
