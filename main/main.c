#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "bno085.h"
#include "sdkconfig.h"

#define BNO085_I2C_PORT     I2C_NUM_0
#define BNO085_SDA_GPIO     ((gpio_num_t)CONFIG_APP_BNO085_I2C_SDA_GPIO)
#define BNO085_SCL_GPIO     ((gpio_num_t)CONFIG_APP_BNO085_I2C_SCL_GPIO)
#define BNO085_INT_GPIO     ((gpio_num_t)CONFIG_APP_BNO085_INT_GPIO)
#define BNO085_RST_GPIO     ((gpio_num_t)CONFIG_APP_BNO085_RST_GPIO)
#define BNO085_I2C_ADDR     CONFIG_APP_BNO085_I2C_ADDR
#define BNO085_I2C_FREQ_HZ  CONFIG_APP_BNO085_I2C_FREQ_HZ

static const char *TAG = "app_main";

/* Convert quaternion to Euler angles (in radians) */
static void quaternion_to_euler(float i, float j, float k, float real,
                                float *roll, float *pitch, float *yaw)
{
    /* Quaternion rotation matrix to Euler angles conversion */
    float q0 = real, q1 = i, q2 = j, q3 = k;

    /* Roll (rotation around X axis) */
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

    /* Pitch (rotation around Y axis) */
    float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    sin_pitch = (sin_pitch > 1.0f) ? 1.0f : (sin_pitch < -1.0f) ? -1.0f : sin_pitch;
    *pitch = asinf(sin_pitch);

    /* Yaw (rotation around Z axis) */
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

#ifdef CONFIG_APP_BNO085_OUTPUT_CSV
static bool csv_header_printed = false;

static void print_csv_header(void)
{
    printf("timestamp_ms");
#ifdef CONFIG_APP_BNO085_ENABLE_ROTATION_VECTOR
    #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
    printf(",rv_i,rv_j,rv_k,rv_real,rv_acc");
    #else
    printf(",rv_roll,rv_pitch,rv_yaw,rv_acc");
    #endif
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GAME_ROTATION_VECTOR
    #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
    printf(",grv_i,grv_j,grv_k,grv_real,grv_acc");
    #else
    printf(",grv_roll,grv_pitch,grv_yaw,grv_acc");
    #endif
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GEOMAGNETIC_ROTATION_VECTOR
    #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
    printf(",gmrv_i,gmrv_j,gmrv_k,gmrv_real,gmrv_acc");
    #else
    printf(",gmrv_roll,gmrv_pitch,gmrv_yaw,gmrv_acc");
    #endif
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_AR_VR_STABILIZED_RV
    #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
    printf(",arvr_i,arvr_j,arvr_k,arvr_real,arvr_acc");
    #else
    printf(",arvr_roll,arvr_pitch,arvr_yaw,arvr_acc");
    #endif
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYRO_INTEGRATED_RV
    #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
    printf(",girv_i,girv_j,girv_k,girv_real,girv_acc");
    #else
    printf(",girv_roll,girv_pitch,girv_yaw,girv_acc");
    #endif
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_ACCELEROMETER
    printf(",ac_x,ac_y,ac_z");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_LINEAR_ACCELERATION
    printf(",la_x,la_y,la_z");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GRAVITY
    printf(",gr_x,gr_y,gr_z");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE
    printf(",gy_x,gy_y,gy_z");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE_UNCALIBRATED
    printf(",gyu_x,gyu_y,gyu_z,gyu_bx,gyu_by,gyu_bz");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD
    printf(",mg_x,mg_y,mg_z");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD_UNCALIBRATED
    printf(",mgu_x,mgu_y,mgu_z,mgu_bx,mgu_by,mgu_bz");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GAME_ROTATION_VECTOR
    printf(",grv_i,grv_j,grv_k,grv_real,grv_acc");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GEOMAGNETIC_ROTATION_VECTOR
    printf(",gmrv_i,gmrv_j,gmrv_k,gmrv_real,gmrv_acc");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_AR_VR_STABILIZED_RV
    printf(",arvr_i,arvr_j,arvr_k,arvr_real,arvr_acc");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYRO_INTEGRATED_RV
    printf(",girv_i,girv_j,girv_k,girv_real,girv_acc");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_RAW_ACCELEROMETER
    printf(",rac_x,rac_y,rac_z");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_RAW_GYROSCOPE
    printf(",rgy_x,rgy_y,rgy_z");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_RAW_MAGNETOMETER
    printf(",rmg_x,rmg_y,rmg_z");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_PRESSURE
    printf(",pr");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_HUMIDITY
    printf(",hm");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_TEMPERATURE
    printf(",tm");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_AMBIENT_LIGHT
    printf(",al");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_STEP_COUNTER
    printf(",sc");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_TAP_DETECTOR
    printf(",tap_type,tap_dir");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_STABILITY_CLASSIFIER
    printf(",stab_class");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_PERSONAL_ACTIVITY_CLASSIFIER
    printf(",pact_class");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_STEP_DETECTOR
    printf(",step_ev");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_SIGNIFICANT_MOTION
    printf(",sig_motion");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_SHAKE_DETECTOR
    printf(",shake");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_FLIP_DETECTOR
    printf(",flip");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_PICKUP_DETECTOR
    printf(",pickup");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_TILT_DETECTOR
    printf(",tilt");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_POCKET_DETECTOR
    printf(",pocket");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_CIRCLE_DETECTOR
    printf(",circle");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_SLEEP_DETECTOR
    printf(",sleep");
#endif
    printf("\n");
    fflush(stdout);
}
#endif

static void sensor_callback(bno085_handle_t handle, const bno085_sensor_value_t *value, void *user_context)
{
    (void) handle;
    (void) user_context;

    if (!value) return;

#ifdef CONFIG_APP_BNO085_OUTPUT_CSV
    if (!csv_header_printed) {
        print_csv_header();
        csv_header_printed = true;
    }

    printf("%llu", value->timestamp_us / 1000);

    switch (value->sensor_id) {
#ifdef CONFIG_APP_BNO085_ENABLE_ROTATION_VECTOR
        case BNO085_SENSOR_ROTATION_VECTOR: {
            #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
            printf(",%.6f,%.6f,%.6f,%.6f,%.6f",
                   value->data.quaternion.i, value->data.quaternion.j,
                   value->data.quaternion.k, value->data.quaternion.real,
                   value->data.quaternion.accuracy_rad);
            #else
            float roll, pitch, yaw;
            quaternion_to_euler(value->data.quaternion.i, value->data.quaternion.j,
                              value->data.quaternion.k, value->data.quaternion.real,
                              &roll, &pitch, &yaw);
            float accuracy_deg = value->data.quaternion.accuracy_rad * 57.2958f;
            printf(",%.2f,%.2f,%.2f,%.1f",
                   roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f, accuracy_deg);
            #endif
            break;
        }
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_ACCELEROMETER
        case BNO085_SENSOR_ACCELEROMETER:
            printf(",%.4f,%.4f,%.4f",
                   value->data.accelerometer.x, value->data.accelerometer.y, value->data.accelerometer.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_LINEAR_ACCELERATION
        case BNO085_SENSOR_LINEAR_ACCELERATION:
            printf(",%.4f,%.4f,%.4f",
                   value->data.linear_acceleration.x, value->data.linear_acceleration.y, value->data.linear_acceleration.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GRAVITY
        case BNO085_SENSOR_GRAVITY:
            printf(",%.4f,%.4f,%.4f",
                   value->data.gravity.x, value->data.gravity.y, value->data.gravity.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE
        case BNO085_SENSOR_GYROSCOPE_CALIBRATED:
            printf(",%.6f,%.6f,%.6f",
                   value->data.gyroscope_calibrated.x, value->data.gyroscope_calibrated.y, value->data.gyroscope_calibrated.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE_UNCALIBRATED
        case BNO085_SENSOR_GYROSCOPE_UNCALIBRATED:
            printf(",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                   value->data.gyroscope_uncalibrated.x, value->data.gyroscope_uncalibrated.y, value->data.gyroscope_uncalibrated.z,
                   value->data.gyroscope_uncalibrated.bias_x, value->data.gyroscope_uncalibrated.bias_y, value->data.gyroscope_uncalibrated.bias_z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD
        case BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED:
            printf(",%.4f,%.4f,%.4f",
                   value->data.magnetic_field_calibrated.x, value->data.magnetic_field_calibrated.y, value->data.magnetic_field_calibrated.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD_UNCALIBRATED
        case BNO085_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
            printf(",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                   value->data.magnetic_field_uncalibrated.x, value->data.magnetic_field_uncalibrated.y, value->data.magnetic_field_uncalibrated.z,
                   value->data.magnetic_field_uncalibrated.bias_x, value->data.magnetic_field_uncalibrated.bias_y, value->data.magnetic_field_uncalibrated.bias_z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GAME_ROTATION_VECTOR
        case BNO085_SENSOR_GAME_ROTATION_VECTOR: {
            #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
            printf(",%.6f,%.6f,%.6f,%.6f,%.6f",
                   value->data.game_rotation_vector.i, value->data.game_rotation_vector.j,
                   value->data.game_rotation_vector.k, value->data.game_rotation_vector.real,
                   value->data.game_rotation_vector.accuracy_rad);
            #else
            float roll, pitch, yaw;
            quaternion_to_euler(value->data.game_rotation_vector.i, value->data.game_rotation_vector.j,
                              value->data.game_rotation_vector.k, value->data.game_rotation_vector.real,
                              &roll, &pitch, &yaw);
            float accuracy_deg = value->data.game_rotation_vector.accuracy_rad * 57.2958f;
            printf(",%.2f,%.2f,%.2f,%.1f",
                   roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f, accuracy_deg);
            #endif
            break;
        }
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GEOMAGNETIC_ROTATION_VECTOR
        case BNO085_SENSOR_GEOMAGNETIC_ROTATION_VECTOR: {
            #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
            printf(",%.6f,%.6f,%.6f,%.6f,%.6f",
                   value->data.geomagnetic_rotation_vector.i, value->data.geomagnetic_rotation_vector.j,
                   value->data.geomagnetic_rotation_vector.k, value->data.geomagnetic_rotation_vector.real,
                   value->data.geomagnetic_rotation_vector.accuracy_rad);
            #else
            float roll, pitch, yaw;
            quaternion_to_euler(value->data.geomagnetic_rotation_vector.i, value->data.geomagnetic_rotation_vector.j,
                              value->data.geomagnetic_rotation_vector.k, value->data.geomagnetic_rotation_vector.real,
                              &roll, &pitch, &yaw);
            float accuracy_deg = value->data.geomagnetic_rotation_vector.accuracy_rad * 57.2958f;
            printf(",%.2f,%.2f,%.2f,%.1f",
                   roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f, accuracy_deg);
            #endif
            break;
        }
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_AR_VR_STABILIZED_RV
        case BNO085_SENSOR_AR_VR_STABILIZED_RV: {
            #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
            printf(",%.6f,%.6f,%.6f,%.6f,%.6f",
                   value->data.ar_vr_stabilized_rv.i, value->data.ar_vr_stabilized_rv.j,
                   value->data.ar_vr_stabilized_rv.k, value->data.ar_vr_stabilized_rv.real,
                   value->data.ar_vr_stabilized_rv.accuracy_rad);
            #else
            float roll, pitch, yaw;
            quaternion_to_euler(value->data.ar_vr_stabilized_rv.i, value->data.ar_vr_stabilized_rv.j,
                              value->data.ar_vr_stabilized_rv.k, value->data.ar_vr_stabilized_rv.real,
                              &roll, &pitch, &yaw);
            float accuracy_deg = value->data.ar_vr_stabilized_rv.accuracy_rad * 57.2958f;
            printf(",%.2f,%.2f,%.2f,%.1f",
                   roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f, accuracy_deg);
            #endif
            break;
        }
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYRO_INTEGRATED_RV
        case BNO085_SENSOR_GYRO_INTEGRATED_RV: {
            #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
            printf(",%.6f,%.6f,%.6f,%.6f,%.6f",
                   value->data.gyro_integrated_rv.i, value->data.gyro_integrated_rv.j,
                   value->data.gyro_integrated_rv.k, value->data.gyro_integrated_rv.real,
                   value->data.gyro_integrated_rv.accuracy_rad);
            #else
            float roll, pitch, yaw;
            quaternion_to_euler(value->data.gyro_integrated_rv.i, value->data.gyro_integrated_rv.j,
                              value->data.gyro_integrated_rv.k, value->data.gyro_integrated_rv.real,
                              &roll, &pitch, &yaw);
            float accuracy_deg = value->data.gyro_integrated_rv.accuracy_rad * 57.2958f;
            printf(",%.2f,%.2f,%.2f,%.1f",
                   roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f, accuracy_deg);
            #endif
            break;
        }
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_RAW_ACCELEROMETER
        case BNO085_SENSOR_RAW_ACCELEROMETER:
            printf(",%d,%d,%d", value->data.raw_accelerometer.x, value->data.raw_accelerometer.y, value->data.raw_accelerometer.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_RAW_GYROSCOPE
        case BNO085_SENSOR_RAW_GYROSCOPE:
            printf(",%d,%d,%d", value->data.raw_gyroscope.x, value->data.raw_gyroscope.y, value->data.raw_gyroscope.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_RAW_MAGNETOMETER
        case BNO085_SENSOR_RAW_MAGNETOMETER:
            printf(",%d,%d,%d", value->data.raw_magnetometer.x, value->data.raw_magnetometer.y, value->data.raw_magnetometer.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_PRESSURE
        case BNO085_SENSOR_PRESSURE:
            printf(",%.2f", value->data.pressure.value);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_HUMIDITY
        case BNO085_SENSOR_HUMIDITY:
            printf(",%.2f", value->data.humidity.value);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_TEMPERATURE
        case BNO085_SENSOR_TEMPERATURE:
            printf(",%.2f", value->data.temperature.value);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_AMBIENT_LIGHT
        case BNO085_SENSOR_AMBIENT_LIGHT:
            printf(",%.2f", value->data.ambient_light.value);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_STEP_COUNTER
        case BNO085_SENSOR_STEP_COUNTER:
            printf(",%u", value->data.step_counter.count);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_TAP_DETECTOR
        case BNO085_SENSOR_TAP_DETECTOR:
            printf(",%u,%u", value->data.tap_detector.tap_type, value->data.tap_detector.direction);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_STABILITY_CLASSIFIER
        case BNO085_SENSOR_STABILITY_CLASSIFIER:
            printf(",%u", value->data.stability_classifier.activity);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_PERSONAL_ACTIVITY_CLASSIFIER
        case BNO085_SENSOR_PERSONAL_ACTIVITY_CLASSIFIER:
            printf(",%u", value->data.personal_activity_classifier.activity);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_STEP_DETECTOR
        case BNO085_SENSOR_STEP_DETECTOR:
            printf(",%u", value->data.step_detector.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_SIGNIFICANT_MOTION
        case BNO085_SENSOR_SIGNIFICANT_MOTION:
            printf(",%u", value->data.significant_motion.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_SHAKE_DETECTOR
        case BNO085_SENSOR_SHAKE_DETECTOR:
            printf(",%u", value->data.shake_detector.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_FLIP_DETECTOR
        case BNO085_SENSOR_FLIP_DETECTOR:
            printf(",%u", value->data.flip_detector.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_PICKUP_DETECTOR
        case BNO085_SENSOR_PICKUP_DETECTOR:
            printf(",%u", value->data.pickup_detector.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_TILT_DETECTOR
        case BNO085_SENSOR_TILT_DETECTOR:
            printf(",%u", value->data.tilt_detector.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_POCKET_DETECTOR
        case BNO085_SENSOR_POCKET_DETECTOR:
            printf(",%u", value->data.pocket_detector.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_CIRCLE_DETECTOR
        case BNO085_SENSOR_CIRCLE_DETECTOR:
            printf(",%u", value->data.circle_detector.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_SLEEP_DETECTOR
        case BNO085_SENSOR_SLEEP_DETECTOR:
            printf(",%u", value->data.sleep_detector.event);
            break;
#endif
    }
    printf("\n");
    fflush(stdout);

#else
    /* Verbose (human-readable) output */
    switch (value->sensor_id) {
#ifdef CONFIG_APP_BNO085_ENABLE_ROTATION_VECTOR
        case BNO085_SENSOR_ROTATION_VECTOR: {
            #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
            ESP_LOGI(TAG, "Rotation Vector: i=%.6f, j=%.6f, k=%.6f, real=%.6f, accuracy=%.1f°",
                     value->data.quaternion.i, value->data.quaternion.j,
                     value->data.quaternion.k, value->data.quaternion.real,
                     value->data.quaternion.accuracy_rad * 57.2958f);
            #else
            float roll, pitch, yaw;
            quaternion_to_euler(value->data.quaternion.i, value->data.quaternion.j,
                              value->data.quaternion.k, value->data.quaternion.real,
                              &roll, &pitch, &yaw);
            float accuracy_deg = value->data.quaternion.accuracy_rad * 57.2958f;
            ESP_LOGI(TAG, "Rotation (Euler): roll=%.1f°, pitch=%.1f°, yaw=%.1f°, accuracy=%.1f°",
                     roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f, accuracy_deg);
            #endif
            break;
        }
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_ACCELEROMETER
        case BNO085_SENSOR_ACCELEROMETER:
            ESP_LOGI(TAG, "Accel: x=%.2f, y=%.2f, z=%.2f m/s² (with gravity)",
                     value->data.accelerometer.x, value->data.accelerometer.y, value->data.accelerometer.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_LINEAR_ACCELERATION
        case BNO085_SENSOR_LINEAR_ACCELERATION:
            ESP_LOGI(TAG, "Linear Accel: x=%.2f, y=%.2f, z=%.2f m/s² (gravity removed)",
                     value->data.linear_acceleration.x, value->data.linear_acceleration.y, value->data.linear_acceleration.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GRAVITY
        case BNO085_SENSOR_GRAVITY:
            ESP_LOGI(TAG, "Gravity: x=%.2f, y=%.2f, z=%.2f m/s²",
                     value->data.gravity.x, value->data.gravity.y, value->data.gravity.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE
        case BNO085_SENSOR_GYROSCOPE_CALIBRATED:
            ESP_LOGI(TAG, "Gyro (calibrated): x=%.4f, y=%.4f, z=%.4f rad/s",
                     value->data.gyroscope_calibrated.x, value->data.gyroscope_calibrated.y, value->data.gyroscope_calibrated.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE_UNCALIBRATED
        case BNO085_SENSOR_GYROSCOPE_UNCALIBRATED:
            ESP_LOGI(TAG, "Gyro (uncal): x=%.4f, y=%.4f, z=%.4f rad/s, bias x=%.4f, y=%.4f, z=%.4f",
                     value->data.gyroscope_uncalibrated.x, value->data.gyroscope_uncalibrated.y, value->data.gyroscope_uncalibrated.z,
                     value->data.gyroscope_uncalibrated.bias_x, value->data.gyroscope_uncalibrated.bias_y, value->data.gyroscope_uncalibrated.bias_z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD
        case BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED:
            ESP_LOGI(TAG, "Mag (calibrated): x=%.1f, y=%.1f, z=%.1f µT",
                     value->data.magnetic_field_calibrated.x, value->data.magnetic_field_calibrated.y, value->data.magnetic_field_calibrated.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD_UNCALIBRATED
        case BNO085_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
            ESP_LOGI(TAG, "Mag (uncal): x=%.1f, y=%.1f, z=%.1f µT, bias x=%.1f, y=%.1f, z=%.1f",
                     value->data.magnetic_field_uncalibrated.x, value->data.magnetic_field_uncalibrated.y, value->data.magnetic_field_uncalibrated.z,
                     value->data.magnetic_field_uncalibrated.bias_x, value->data.magnetic_field_uncalibrated.bias_y, value->data.magnetic_field_uncalibrated.bias_z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GAME_ROTATION_VECTOR
        case BNO085_SENSOR_GAME_ROTATION_VECTOR: {
            #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
            ESP_LOGI(TAG, "Game Rotation Vector: i=%.6f, j=%.6f, k=%.6f, real=%.6f, accuracy=%.1f°",
                     value->data.game_rotation_vector.i, value->data.game_rotation_vector.j,
                     value->data.game_rotation_vector.k, value->data.game_rotation_vector.real,
                     value->data.game_rotation_vector.accuracy_rad * 57.2958f);
            #else
            float roll, pitch, yaw;
            quaternion_to_euler(value->data.game_rotation_vector.i, value->data.game_rotation_vector.j,
                              value->data.game_rotation_vector.k, value->data.game_rotation_vector.real,
                              &roll, &pitch, &yaw);
            float accuracy_deg = value->data.game_rotation_vector.accuracy_rad * 57.2958f;
            ESP_LOGI(TAG, "Game Rotation (Euler): roll=%.1f°, pitch=%.1f°, yaw=%.1f°, accuracy=%.1f°",
                     roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f, accuracy_deg);
            #endif
            break;
        }
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GEOMAGNETIC_ROTATION_VECTOR
        case BNO085_SENSOR_GEOMAGNETIC_ROTATION_VECTOR: {
            #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
            ESP_LOGI(TAG, "Geomagnetic Rotation Vector: i=%.6f, j=%.6f, k=%.6f, real=%.6f, accuracy=%.1f°",
                     value->data.geomagnetic_rotation_vector.i, value->data.geomagnetic_rotation_vector.j,
                     value->data.geomagnetic_rotation_vector.k, value->data.geomagnetic_rotation_vector.real,
                     value->data.geomagnetic_rotation_vector.accuracy_rad * 57.2958f);
            #else
            float roll, pitch, yaw;
            quaternion_to_euler(value->data.geomagnetic_rotation_vector.i, value->data.geomagnetic_rotation_vector.j,
                              value->data.geomagnetic_rotation_vector.k, value->data.geomagnetic_rotation_vector.real,
                              &roll, &pitch, &yaw);
            float accuracy_deg = value->data.geomagnetic_rotation_vector.accuracy_rad * 57.2958f;
            ESP_LOGI(TAG, "Geomagnetic Rotation (Euler): roll=%.1f°, pitch=%.1f°, yaw=%.1f°, accuracy=%.1f°",
                     roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f, accuracy_deg);
            #endif
            break;
        }
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_AR_VR_STABILIZED_RV
        case BNO085_SENSOR_AR_VR_STABILIZED_RV: {
            #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
            ESP_LOGI(TAG, "AR/VR Stabilized RV: i=%.6f, j=%.6f, k=%.6f, real=%.6f, accuracy=%.1f°",
                     value->data.ar_vr_stabilized_rv.i, value->data.ar_vr_stabilized_rv.j,
                     value->data.ar_vr_stabilized_rv.k, value->data.ar_vr_stabilized_rv.real,
                     value->data.ar_vr_stabilized_rv.accuracy_rad * 57.2958f);
            #else
            float roll, pitch, yaw;
            quaternion_to_euler(value->data.ar_vr_stabilized_rv.i, value->data.ar_vr_stabilized_rv.j,
                              value->data.ar_vr_stabilized_rv.k, value->data.ar_vr_stabilized_rv.real,
                              &roll, &pitch, &yaw);
            float accuracy_deg = value->data.ar_vr_stabilized_rv.accuracy_rad * 57.2958f;
            ESP_LOGI(TAG, "AR/VR Stabilized (Euler): roll=%.1f°, pitch=%.1f°, yaw=%.1f°, accuracy=%.1f°",
                     roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f, accuracy_deg);
            #endif
            break;
        }
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYRO_INTEGRATED_RV
        case BNO085_SENSOR_GYRO_INTEGRATED_RV: {
            #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
            ESP_LOGI(TAG, "Gyro Integrated RV (1kHz): i=%.6f, j=%.6f, k=%.6f, real=%.6f, accuracy=%.1f°",
                     value->data.gyro_integrated_rv.i, value->data.gyro_integrated_rv.j,
                     value->data.gyro_integrated_rv.k, value->data.gyro_integrated_rv.real,
                     value->data.gyro_integrated_rv.accuracy_rad * 57.2958f);
            #else
            float roll, pitch, yaw;
            quaternion_to_euler(value->data.gyro_integrated_rv.i, value->data.gyro_integrated_rv.j,
                              value->data.gyro_integrated_rv.k, value->data.gyro_integrated_rv.real,
                              &roll, &pitch, &yaw);
            float accuracy_deg = value->data.gyro_integrated_rv.accuracy_rad * 57.2958f;
            ESP_LOGI(TAG, "Gyro Integrated (Euler, 1kHz): roll=%.1f°, pitch=%.1f°, yaw=%.1f°, accuracy=%.1f°",
                     roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f, accuracy_deg);
            #endif
            break;
        }
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_RAW_ACCELEROMETER
        case BNO085_SENSOR_RAW_ACCELEROMETER:
            ESP_LOGI(TAG, "Raw Accel: x=%d, y=%d, z=%d (ADC counts)",
                     value->data.raw_accelerometer.x, value->data.raw_accelerometer.y, value->data.raw_accelerometer.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_RAW_GYROSCOPE
        case BNO085_SENSOR_RAW_GYROSCOPE:
            ESP_LOGI(TAG, "Raw Gyro: x=%d, y=%d, z=%d (ADC counts)",
                     value->data.raw_gyroscope.x, value->data.raw_gyroscope.y, value->data.raw_gyroscope.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_RAW_MAGNETOMETER
        case BNO085_SENSOR_RAW_MAGNETOMETER:
            ESP_LOGI(TAG, "Raw Mag: x=%d, y=%d, z=%d (ADC counts)",
                     value->data.raw_magnetometer.x, value->data.raw_magnetometer.y, value->data.raw_magnetometer.z);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_PRESSURE
        case BNO085_SENSOR_PRESSURE:
            ESP_LOGI(TAG, "Pressure: %.2f Pa", value->data.pressure.value);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_HUMIDITY
        case BNO085_SENSOR_HUMIDITY:
            ESP_LOGI(TAG, "Humidity: %.2f%%", value->data.humidity.value);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_TEMPERATURE
        case BNO085_SENSOR_TEMPERATURE:
            ESP_LOGI(TAG, "Temperature: %.2f°C", value->data.temperature.value);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_AMBIENT_LIGHT
        case BNO085_SENSOR_AMBIENT_LIGHT:
            ESP_LOGI(TAG, "Ambient Light: %.2f lux", value->data.ambient_light.value);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_STEP_COUNTER
        case BNO085_SENSOR_STEP_COUNTER:
            ESP_LOGI(TAG, "Step Counter: %u steps", value->data.step_counter.count);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_TAP_DETECTOR
        case BNO085_SENSOR_TAP_DETECTOR:
            ESP_LOGI(TAG, "Tap: type=%u (1=single, 2=double), direction=%u",
                     value->data.tap_detector.tap_type, value->data.tap_detector.direction);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_STABILITY_CLASSIFIER
        case BNO085_SENSOR_STABILITY_CLASSIFIER:
            ESP_LOGI(TAG, "Stability: %u (0=unknown, 1=on-table, 2=stationary, 3=stable, 4=motion)",
                     value->data.stability_classifier.activity);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_PERSONAL_ACTIVITY_CLASSIFIER
        case BNO085_SENSOR_PERSONAL_ACTIVITY_CLASSIFIER:
            ESP_LOGI(TAG, "Activity: %u (0=unknown, 1=still, 2=walking, 3=running, 4=on-bicycle, 5=in-vehicle, ...)",
                     value->data.personal_activity_classifier.activity);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_STEP_DETECTOR
        case BNO085_SENSOR_STEP_DETECTOR:
            ESP_LOGI(TAG, "Step Detector: event=%u", value->data.step_detector.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_SIGNIFICANT_MOTION
        case BNO085_SENSOR_SIGNIFICANT_MOTION:
            ESP_LOGI(TAG, "Significant Motion: event=%u", value->data.significant_motion.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_SHAKE_DETECTOR
        case BNO085_SENSOR_SHAKE_DETECTOR:
            ESP_LOGI(TAG, "Shake Detector: event=%u", value->data.shake_detector.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_FLIP_DETECTOR
        case BNO085_SENSOR_FLIP_DETECTOR:
            ESP_LOGI(TAG, "Flip Detector: event=%u", value->data.flip_detector.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_PICKUP_DETECTOR
        case BNO085_SENSOR_PICKUP_DETECTOR:
            ESP_LOGI(TAG, "Pickup Detector: event=%u", value->data.pickup_detector.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_TILT_DETECTOR
        case BNO085_SENSOR_TILT_DETECTOR:
            ESP_LOGI(TAG, "Tilt Detector: event=%u", value->data.tilt_detector.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_POCKET_DETECTOR
        case BNO085_SENSOR_POCKET_DETECTOR:
            ESP_LOGI(TAG, "Pocket Detector: event=%u", value->data.pocket_detector.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_CIRCLE_DETECTOR
        case BNO085_SENSOR_CIRCLE_DETECTOR:
            ESP_LOGI(TAG, "Circle Detector: event=%u", value->data.circle_detector.event);
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_SLEEP_DETECTOR
        case BNO085_SENSOR_SLEEP_DETECTOR:
            ESP_LOGI(TAG, "Sleep Detector: event=%u", value->data.sleep_detector.event);
            break;
#endif
    }
#endif
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing BNO085 sensor driver...");

    i2c_master_bus_config_t bus_config = {
        .i2c_port = BNO085_I2C_PORT,
        .sda_io_num = BNO085_SDA_GPIO,
        .scl_io_num = BNO085_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    ESP_LOGI(TAG, "I2C bus initialized");

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BNO085_I2C_ADDR,
        .scl_speed_hz = BNO085_I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
    ESP_LOGI(TAG, "BNO085 device added to bus at address 0x%02X", BNO085_I2C_ADDR);

    bno085_config_t config;
    bno085_config_default(&config);

    bno085_handle_t bno085_handle;
    ESP_ERROR_CHECK(bno085_init(&config, dev_handle, BNO085_INT_GPIO, BNO085_RST_GPIO, &bno085_handle));
    ESP_LOGI(TAG, "BNO085 driver initialized");

    ESP_ERROR_CHECK(bno085_register_sensor_callback(bno085_handle, sensor_callback, NULL));

    uint32_t report_interval_us = (1000000 / CONFIG_APP_BNO085_SAMPLING_RATE_HZ);

    /* Enable all sensors from Kconfig */
#ifdef CONFIG_APP_BNO085_ENABLE_ROTATION_VECTOR
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_ROTATION_VECTOR, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_ACCELEROMETER
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_ACCELEROMETER, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_LINEAR_ACCELERATION
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_LINEAR_ACCELERATION, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GRAVITY
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_GRAVITY, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_GYROSCOPE_CALIBRATED, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE_UNCALIBRATED
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_GYROSCOPE_UNCALIBRATED, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD_UNCALIBRATED
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GAME_ROTATION_VECTOR
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_GAME_ROTATION_VECTOR, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GEOMAGNETIC_ROTATION_VECTOR
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_AR_VR_STABILIZED_RV
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_AR_VR_STABILIZED_RV, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYRO_INTEGRATED_RV
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_GYRO_INTEGRATED_RV, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_RAW_ACCELEROMETER
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_RAW_ACCELEROMETER, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_RAW_GYROSCOPE
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_RAW_GYROSCOPE, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_RAW_MAGNETOMETER
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_RAW_MAGNETOMETER, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_PRESSURE
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_PRESSURE, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_HUMIDITY
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_HUMIDITY, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_TEMPERATURE
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_TEMPERATURE, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_AMBIENT_LIGHT
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_AMBIENT_LIGHT, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_STEP_COUNTER
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_STEP_COUNTER, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_STEP_DETECTOR
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_STEP_DETECTOR, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_STABILITY_CLASSIFIER
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_STABILITY_CLASSIFIER, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_PERSONAL_ACTIVITY_CLASSIFIER
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_PERSONAL_ACTIVITY_CLASSIFIER, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_SIGNIFICANT_MOTION
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_SIGNIFICANT_MOTION, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_SHAKE_DETECTOR
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_SHAKE_DETECTOR, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_TAP_DETECTOR
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_TAP_DETECTOR, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_FLIP_DETECTOR
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_FLIP_DETECTOR, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_PICKUP_DETECTOR
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_PICKUP_DETECTOR, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_TILT_DETECTOR
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_TILT_DETECTOR, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_POCKET_DETECTOR
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_POCKET_DETECTOR, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_CIRCLE_DETECTOR
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_CIRCLE_DETECTOR, report_interval_us);
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_SLEEP_DETECTOR
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_SLEEP_DETECTOR, report_interval_us);
#endif

    /* Enable sensors needed for calculations (may not be printing them).
       Rotation vectors depend on accel/gyro/mag being available; silently enable them if needed. */
#if defined(CONFIG_APP_BNO085_ACCEL_CALC_NEEDED) && !defined(CONFIG_APP_BNO085_ENABLE_ACCELEROMETER)
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_ACCELEROMETER, report_interval_us);
#endif

#if defined(CONFIG_APP_BNO085_GYRO_CALC_NEEDED) && !defined(CONFIG_APP_BNO085_ENABLE_GYROSCOPE)
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_GYROSCOPE_CALIBRATED, report_interval_us);
#endif

#if defined(CONFIG_APP_BNO085_MAG_CALC_NEEDED) && !defined(CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD)
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED, report_interval_us);
#endif

    ESP_LOGI(TAG, "Sensors enabled. Starting main loop...");

    while (1) {
        bno085_service(bno085_handle);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
