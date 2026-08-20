#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "bno085.h"
#include "sdkconfig.h"
#include "sensor.h"

#define MAX_SENSOR_ID 0x2B

static const char *TAG = "output";

#ifdef CONFIG_APP_BNO085_OUTPUT_CSV

#ifdef CONFIG_APP_BNO085_CSV_SEPARATOR_COMMA
#ifdef CONFIG_APP_BNO085_CSV_SPACE_AFTER_SEPARATOR
#define CSV_SEP ", "
#else
#define CSV_SEP ","
#endif
#elif defined(CONFIG_APP_BNO085_CSV_SEPARATOR_SEMICOLON)
#ifdef CONFIG_APP_BNO085_CSV_SPACE_AFTER_SEPARATOR
#define CSV_SEP "; "
#else
#define CSV_SEP ";"
#endif
#elif defined(CONFIG_APP_BNO085_CSV_SEPARATOR_PIPE)
#ifdef CONFIG_APP_BNO085_CSV_SPACE_AFTER_SEPARATOR
#define CSV_SEP "| "
#else
#define CSV_SEP "|"
#endif
#elif defined(CONFIG_APP_BNO085_CSV_SEPARATOR_TAB)
#define CSV_SEP "\t"
#endif

#ifdef CONFIG_APP_BNO085_TIMESTAMP_PRINT_MODE_SENSOR
#define FIRST_SENSOR_SEP CSV_SEP
#else
#define FIRST_SENSOR_SEP ""
#endif

static SemaphoreHandle_t output_ready_sem = NULL;
static bool csv_header_printed = false;

static void quaternion_to_euler(float i, float j, float k, float real,
                                float *roll, float *pitch, float *yaw)
{
    float q0 = real, q1 = i, q2 = j, q3 = k;

    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

    float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    sin_pitch = (sin_pitch > 1.0f) ? 1.0f : (sin_pitch < -1.0f) ? -1.0f : sin_pitch;
    *pitch = asinf(sin_pitch);

    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

static void print_csv_header(void)
{
    #ifdef CONFIG_APP_BNO085_TIMESTAMP_PRINT_MODE_SENSOR
    printf("sensor_timestamp_ms");
    #endif

    #ifdef CONFIG_APP_BNO085_PRINT_ROTATION_VECTOR
        #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
        #ifdef CONFIG_APP_BNO085_TIMESTAMP_PRINT_MODE_SENSOR
        printf("%srv_i%srv_j%srv_k%srv_real%srv_acc", CSV_SEP, CSV_SEP, CSV_SEP, CSV_SEP, CSV_SEP);
        #else
        printf("rv_i%srv_j%srv_k%srv_real%srv_acc", CSV_SEP, CSV_SEP, CSV_SEP, CSV_SEP);
        #endif
        #else
        #ifdef CONFIG_APP_BNO085_TIMESTAMP_PRINT_MODE_SENSOR
        printf("%srv_roll%srv_pitch%srv_yaw%srv_acc", CSV_SEP, CSV_SEP, CSV_SEP, CSV_SEP);
        #else
        printf("rv_roll%srv_pitch%srv_yaw%srv_acc", CSV_SEP, CSV_SEP, CSV_SEP);
        #endif
        #endif
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_GAME_ROTATION_VECTOR
        #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
        printf("%sgrv_i%sgrv_j%sgrv_k%sgrv_real%sgrv_acc", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP, CSV_SEP, CSV_SEP);
        #else
        printf("%sgrv_roll%sgrv_pitch%sgrv_yaw%sgrv_acc", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP, CSV_SEP);
        #endif
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_GEOMAGNETIC_ROTATION_VECTOR
        #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
        printf("%sgmrv_i%sgmrv_j%sgmrv_k%sgmrv_real%sgmrv_acc", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP, CSV_SEP, CSV_SEP);
        #else
        printf("%sgmrv_roll%sgmrv_pitch%sgmrv_yaw%sgmrv_acc", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP, CSV_SEP);
        #endif
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_AR_VR_STABILIZED_RV
        #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
        printf("%sarvr_i%sarvr_j%sarvr_k%sarvr_real%sarvr_acc", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP, CSV_SEP, CSV_SEP);
        #else
        printf("%sarvr_roll%sarvr_pitch%sarvr_yaw%sarvr_acc", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP, CSV_SEP);
        #endif
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_GYRO_INTEGRATED_RV
        #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
        printf("%sgirv_i%sgirv_j%sgirv_k%sgirv_real%sgirv_acc", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP, CSV_SEP, CSV_SEP);
        #else
        printf("%sgirv_roll%sgirv_pitch%sgirv_yaw%sgirv_acc", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP, CSV_SEP);
        #endif
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_ACCELEROMETER
    printf("%sac_x%sac_y%sac_z", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_LINEAR_ACCELERATION
    printf("%sla_x%sla_y%sla_z", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_GRAVITY
    printf("%sgr_x%sgr_y%sgr_z", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_GYROSCOPE
    printf("%sgy_x%sgy_y%sgy_z", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_GYROSCOPE_UNCALIBRATED
    printf("%sgyu_x%sgyu_y%sgyu_z%sgyu_bx%sgyu_by%sgyu_bz", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP, CSV_SEP, CSV_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_MAGNETIC_FIELD
    printf("%smg_x%smg_y%smg_z", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_MAGNETIC_FIELD_UNCALIBRATED
    printf("%smgu_x%smgu_y%smgu_z%smgu_bx%smgu_by%smgu_bz", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP, CSV_SEP, CSV_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_RAW_ACCELEROMETER
    printf("%srac_x%srac_y%srac_z", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_RAW_GYROSCOPE
    printf("%srgy_x%srgy_y%srgy_z", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_RAW_MAGNETOMETER
    printf("%srmg_x%srmg_y%srmg_z", FIRST_SENSOR_SEP, CSV_SEP, CSV_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_PRESSURE
    printf("%spr", FIRST_SENSOR_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_HUMIDITY
    printf("%shm", FIRST_SENSOR_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_TEMPERATURE
    printf("%stm", FIRST_SENSOR_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_AMBIENT_LIGHT
    printf("%sal", FIRST_SENSOR_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_STEP_COUNTER
    printf("%ssc", FIRST_SENSOR_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_TAP_DETECTOR
    printf("%stap_type%stap_dir", FIRST_SENSOR_SEP, CSV_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_STABILITY_CLASSIFIER
    printf("%sstab_class", FIRST_SENSOR_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_PERSONAL_ACTIVITY_CLASSIFIER
    printf("%spact_class", FIRST_SENSOR_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_STEP_DETECTOR
    printf("%sstep_ev", FIRST_SENSOR_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_SIGNIFICANT_MOTION
    printf("%ssig_motion", FIRST_SENSOR_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_SHAKE_DETECTOR
    printf("%sshake", FIRST_SENSOR_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_FLIP_DETECTOR
    printf("%sflip", FIRST_SENSOR_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_PICKUP_DETECTOR
    printf("%spickup", FIRST_SENSOR_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_TILT_DETECTOR
    printf("%stilt", FIRST_SENSOR_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_POCKET_DETECTOR
    printf("%spocket", FIRST_SENSOR_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_CIRCLE_DETECTOR
    printf("%scircle", FIRST_SENSOR_SEP);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_SLEEP_DETECTOR
    printf("%ssleep", FIRST_SENSOR_SEP);
    #endif
    printf("\n");
    fflush(stdout);
}

static bool print_csv_value(const bno085_sensor_value_t *value, bool first)
{
    const char *sep = first ? "" : CSV_SEP;
    const char *field_sep = CSV_SEP;

    switch (value->sensor_id) {
    #ifdef CONFIG_APP_BNO085_PRINT_ROTATION_VECTOR
        case BNO085_SENSOR_ROTATION_VECTOR: {
            #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
            printf("%s%.6f%s%.6f%s%.6f%s%.6f%s%.6f", sep,
                   value->data.quaternion.i, field_sep, value->data.quaternion.j,
                   field_sep, value->data.quaternion.k, field_sep, value->data.quaternion.real,
                   field_sep, value->data.quaternion.accuracy_rad);
            #else
            float roll, pitch, yaw;
            quaternion_to_euler(value->data.quaternion.i, value->data.quaternion.j,
                              value->data.quaternion.k, value->data.quaternion.real,
                              &roll, &pitch, &yaw);
            float accuracy_deg = value->data.quaternion.accuracy_rad * 57.2958f;
            printf("%s%.2f%s%.2f%s%.2f%s%.1f", sep,
                   roll * 57.2958f, field_sep, pitch * 57.2958f, field_sep, yaw * 57.2958f, field_sep, accuracy_deg);
            #endif
            return true;
        }
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_ACCELEROMETER
        case BNO085_SENSOR_ACCELEROMETER:
            printf("%s%.4f%s%.4f%s%.4f", sep,
                   value->data.accelerometer.x, field_sep, value->data.accelerometer.y, field_sep, value->data.accelerometer.z);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_LINEAR_ACCELERATION
        case BNO085_SENSOR_LINEAR_ACCELERATION:
            printf("%s%.4f%s%.4f%s%.4f", sep,
                   value->data.linear_acceleration.x, field_sep, value->data.linear_acceleration.y, field_sep, value->data.linear_acceleration.z);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_GRAVITY
        case BNO085_SENSOR_GRAVITY:
            printf("%s%.4f%s%.4f%s%.4f", sep,
                   value->data.gravity.x, field_sep, value->data.gravity.y, field_sep, value->data.gravity.z);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_GYROSCOPE
        case BNO085_SENSOR_GYROSCOPE_CALIBRATED:
            printf("%s%.6f%s%.6f%s%.6f", sep,
                   value->data.gyroscope_calibrated.x, field_sep, value->data.gyroscope_calibrated.y, field_sep, value->data.gyroscope_calibrated.z);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_GYROSCOPE_UNCALIBRATED
        case BNO085_SENSOR_GYROSCOPE_UNCALIBRATED:
            printf("%s%.6f%s%.6f%s%.6f%s%.6f%s%.6f%s%.6f", sep,
                   value->data.gyroscope_uncalibrated.x, field_sep, value->data.gyroscope_uncalibrated.y, field_sep, value->data.gyroscope_uncalibrated.z,
                   field_sep, value->data.gyroscope_uncalibrated.bias_x, field_sep, value->data.gyroscope_uncalibrated.bias_y, field_sep, value->data.gyroscope_uncalibrated.bias_z);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_MAGNETIC_FIELD
        case BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED:
            printf("%s%.4f%s%.4f%s%.4f", sep,
                   value->data.magnetic_field_calibrated.x, field_sep, value->data.magnetic_field_calibrated.y, field_sep, value->data.magnetic_field_calibrated.z);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_MAGNETIC_FIELD_UNCALIBRATED
        case BNO085_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
            printf("%s%.4f%s%.4f%s%.4f%s%.4f%s%.4f%s%.4f", sep,
                   value->data.magnetic_field_uncalibrated.x, field_sep, value->data.magnetic_field_uncalibrated.y, field_sep, value->data.magnetic_field_uncalibrated.z,
                   field_sep, value->data.magnetic_field_uncalibrated.bias_x, field_sep, value->data.magnetic_field_uncalibrated.bias_y, field_sep, value->data.magnetic_field_uncalibrated.bias_z);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_GAME_ROTATION_VECTOR
        case BNO085_SENSOR_GAME_ROTATION_VECTOR: {
            #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
            printf("%s%.6f%s%.6f%s%.6f%s%.6f%s%.6f", sep,
                   value->data.game_rotation_vector.i, field_sep, value->data.game_rotation_vector.j,
                   field_sep, value->data.game_rotation_vector.k, field_sep, value->data.game_rotation_vector.real,
                   field_sep, value->data.game_rotation_vector.accuracy_rad);
            #else
            float roll, pitch, yaw;
            quaternion_to_euler(value->data.game_rotation_vector.i, value->data.game_rotation_vector.j,
                              value->data.game_rotation_vector.k, value->data.game_rotation_vector.real,
                              &roll, &pitch, &yaw);
            float accuracy_deg = value->data.game_rotation_vector.accuracy_rad * 57.2958f;
            printf("%s%.2f%s%.2f%s%.2f%s%.1f", sep,
                   roll * 57.2958f, field_sep, pitch * 57.2958f, field_sep, yaw * 57.2958f, field_sep, accuracy_deg);
            #endif
            return true;
        }
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_GEOMAGNETIC_ROTATION_VECTOR
        case BNO085_SENSOR_GEOMAGNETIC_ROTATION_VECTOR: {
            #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
            printf("%s%.6f%s%.6f%s%.6f%s%.6f%s%.6f", sep,
                   value->data.geomagnetic_rotation_vector.i, field_sep, value->data.geomagnetic_rotation_vector.j,
                   field_sep, value->data.geomagnetic_rotation_vector.k, field_sep, value->data.geomagnetic_rotation_vector.real,
                   field_sep, value->data.geomagnetic_rotation_vector.accuracy_rad);
            #else
            float roll, pitch, yaw;
            quaternion_to_euler(value->data.geomagnetic_rotation_vector.i, value->data.geomagnetic_rotation_vector.j,
                              value->data.geomagnetic_rotation_vector.k, value->data.geomagnetic_rotation_vector.real,
                              &roll, &pitch, &yaw);
            float accuracy_deg = value->data.geomagnetic_rotation_vector.accuracy_rad * 57.2958f;
            printf("%s%.2f%s%.2f%s%.2f%s%.1f", sep,
                   roll * 57.2958f, field_sep, pitch * 57.2958f, field_sep, yaw * 57.2958f, field_sep, accuracy_deg);
            #endif
            return true;
        }
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_AR_VR_STABILIZED_RV
        case BNO085_SENSOR_AR_VR_STABILIZED_RV: {
            #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
            printf("%s%.6f%s%.6f%s%.6f%s%.6f%s%.6f", sep,
                   value->data.ar_vr_stabilized_rv.i, field_sep, value->data.ar_vr_stabilized_rv.j,
                   field_sep, value->data.ar_vr_stabilized_rv.k, field_sep, value->data.ar_vr_stabilized_rv.real,
                   field_sep, value->data.ar_vr_stabilized_rv.accuracy_rad);
            #else
            float roll, pitch, yaw;
            quaternion_to_euler(value->data.ar_vr_stabilized_rv.i, value->data.ar_vr_stabilized_rv.j,
                              value->data.ar_vr_stabilized_rv.k, value->data.ar_vr_stabilized_rv.real,
                              &roll, &pitch, &yaw);
            float accuracy_deg = value->data.ar_vr_stabilized_rv.accuracy_rad * 57.2958f;
            printf("%s%.2f%s%.2f%s%.2f%s%.1f", sep,
                   roll * 57.2958f, field_sep, pitch * 57.2958f, field_sep, yaw * 57.2958f, field_sep, accuracy_deg);
            #endif
            return true;
        }
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_GYRO_INTEGRATED_RV
        case BNO085_SENSOR_GYRO_INTEGRATED_RV: {
            #ifdef CONFIG_APP_BNO085_RV_FORMAT_QUATERNION
            printf("%s%.6f%s%.6f%s%.6f%s%.6f%s%.6f", sep,
                   value->data.gyro_integrated_rv.i, field_sep, value->data.gyro_integrated_rv.j,
                   field_sep, value->data.gyro_integrated_rv.k, field_sep, value->data.gyro_integrated_rv.real,
                   field_sep, value->data.gyro_integrated_rv.accuracy_rad);
            #else
            float roll, pitch, yaw;
            quaternion_to_euler(value->data.gyro_integrated_rv.i, value->data.gyro_integrated_rv.j,
                              value->data.gyro_integrated_rv.k, value->data.gyro_integrated_rv.real,
                              &roll, &pitch, &yaw);
            float accuracy_deg = value->data.gyro_integrated_rv.accuracy_rad * 57.2958f;
            printf("%s%.2f%s%.2f%s%.2f%s%.1f", sep,
                   roll * 57.2958f, field_sep, pitch * 57.2958f, field_sep, yaw * 57.2958f, field_sep, accuracy_deg);
            #endif
            return true;
        }
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_RAW_ACCELEROMETER
        case BNO085_SENSOR_RAW_ACCELEROMETER:
            printf("%s%d%s%d%s%d", sep, value->data.raw_accelerometer.x, field_sep, value->data.raw_accelerometer.y, field_sep, value->data.raw_accelerometer.z);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_RAW_GYROSCOPE
        case BNO085_SENSOR_RAW_GYROSCOPE:
            printf("%s%d%s%d%s%d", sep, value->data.raw_gyroscope.x, field_sep, value->data.raw_gyroscope.y, field_sep, value->data.raw_gyroscope.z);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_RAW_MAGNETOMETER
        case BNO085_SENSOR_RAW_MAGNETOMETER:
            printf("%s%d%s%d%s%d", sep, value->data.raw_magnetometer.x, field_sep, value->data.raw_magnetometer.y, field_sep, value->data.raw_magnetometer.z);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_PRESSURE
        case BNO085_SENSOR_PRESSURE:
            printf("%s%.2f", sep, value->data.pressure.value);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_HUMIDITY
        case BNO085_SENSOR_HUMIDITY:
            printf("%s%.2f", sep, value->data.humidity.value);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_TEMPERATURE
        case BNO085_SENSOR_TEMPERATURE:
            printf("%s%.2f", sep, value->data.temperature.value);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_AMBIENT_LIGHT
        case BNO085_SENSOR_AMBIENT_LIGHT:
            printf("%s%.2f", sep, value->data.ambient_light.value);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_STEP_COUNTER
        case BNO085_SENSOR_STEP_COUNTER:
            printf("%s%u", sep, value->data.step_counter.count);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_TAP_DETECTOR
        case BNO085_SENSOR_TAP_DETECTOR:
            printf("%s%u%s%u", sep, value->data.tap_detector.tap_type, field_sep, value->data.tap_detector.direction);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_STABILITY_CLASSIFIER
        case BNO085_SENSOR_STABILITY_CLASSIFIER:
            printf("%s%u", sep, value->data.stability_classifier.activity);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_PERSONAL_ACTIVITY_CLASSIFIER
        case BNO085_SENSOR_PERSONAL_ACTIVITY_CLASSIFIER:
            printf("%s%u", sep, value->data.personal_activity_classifier.activity);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_STEP_DETECTOR
        case BNO085_SENSOR_STEP_DETECTOR:
            printf("%s%u", sep, value->data.step_detector.event);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_SIGNIFICANT_MOTION
        case BNO085_SENSOR_SIGNIFICANT_MOTION:
            printf("%s%u", sep, value->data.significant_motion.event);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_SHAKE_DETECTOR
        case BNO085_SENSOR_SHAKE_DETECTOR:
            printf("%s%u", sep, value->data.shake_detector.event);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_FLIP_DETECTOR
        case BNO085_SENSOR_FLIP_DETECTOR:
            printf("%s%u", sep, value->data.flip_detector.event);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_PICKUP_DETECTOR
        case BNO085_SENSOR_PICKUP_DETECTOR:
            printf("%s%u", sep, value->data.pickup_detector.event);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_TILT_DETECTOR
        case BNO085_SENSOR_TILT_DETECTOR:
            printf("%s%u", sep, value->data.tilt_detector.event);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_POCKET_DETECTOR
        case BNO085_SENSOR_POCKET_DETECTOR:
            printf("%s%u", sep, value->data.pocket_detector.event);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_CIRCLE_DETECTOR
        case BNO085_SENSOR_CIRCLE_DETECTOR:
            printf("%s%u", sep, value->data.circle_detector.event);
            return true;
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_SLEEP_DETECTOR
        case BNO085_SENSOR_SLEEP_DETECTOR:
            printf("%s%u", sep, value->data.sleep_detector.event);
            return true;
    #endif
    }
    return false;
}

static void print_csv_row(void)
{
    bool first = true;

    #ifdef CONFIG_APP_BNO085_TIMESTAMP_PRINT_MODE_SENSOR
    {
        printf("%llu", sensor_get_latest_timestamp());
        first = false;
    }
    #endif

    for (int i = 0; i < MAX_SENSOR_ID; i++) {
        sensor_data_snapshot_t snapshot = sensor_get_data_snapshot(i);
        if (snapshot.has_data) {
            if (print_csv_value(&snapshot.data, first)) {
                first = false;
            }
        }
    }

    printf("\n");
    fflush(stdout);
}

static void output_timer_callback(TimerHandle_t xTimer)
{
    (void) xTimer;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (output_ready_sem != NULL) {
        xSemaphoreGiveFromISR(output_ready_sem, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

static void output_task(void *pvParameters)
{
    (void) pvParameters;
    ESP_LOGI(TAG, "Output task started");

    while (1) {
        if (xSemaphoreTake(output_ready_sem, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (!csv_header_printed) {
                print_csv_header();
                csv_header_printed = true;
            }

            print_csv_row();
        }
    }
}

void output_init(void)
{
    output_ready_sem = xSemaphoreCreateBinary();
    if (output_ready_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create output_ready_sem");
    }
}

void output_start_task(void)
{
    xTaskCreate(output_task, "output_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Output task created");
}

void output_start_timer(void)
{
    TimerHandle_t output_timer = xTimerCreate("output_timer",
                                              pdMS_TO_TICKS(CONFIG_APP_BNO085_OUTPUT_PERIOD_MS),
                                              pdTRUE, NULL, output_timer_callback);
    if (output_timer != NULL) {
        xTimerStart(output_timer, 0);
        ESP_LOGI(TAG, "Output timer started (period: %d ms)", CONFIG_APP_BNO085_OUTPUT_PERIOD_MS);
    }
}

#else

void output_init(void) {}
void output_start_task(void) {}
void output_start_timer(void) {}

#endif
