#ifndef __BNO085_H__
#define __BNO085_H__

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to a BNO085 device instance.
 *
 * Each handle represents one BNO085 sensor connected to the I2C bus.
 * Multiple instances may be initialized concurrently, up to CONFIG_BNO085_MAX_INSTANCES
 * (default 2, configurable at build time to 1–8). Attempting bno085_init() when all
 * slots are occupied returns ESP_ERR_NO_MEM.
 *
 * Note: sh2_multi_acquire_slot()/release_slot() are not thread-safe; call bno085_init()
 * and bno085_deinit() from a single task if used in a multitasking environment.
 */
typedef struct bno085_dev_t *bno085_handle_t;

/**
 * @brief Configuration for BNO085 initialization.
 */
typedef struct {
    uint32_t i2c_timeout_ms;        /**< I2C operation timeout in milliseconds (default: 100) */
    uint8_t  reset_retry_count;     /**< Soft-reset retry attempts (default: 5) */
    uint32_t reset_retry_delay_ms;  /**< Delay between reset retries in milliseconds (default: 30) */
    uint32_t reset_settle_delay_ms; /**< Delay after reset to let device boot (default: 300) */
} bno085_config_t;

/**
 * @brief Get default BNO085 configuration.
 *
 * Fills the provided config struct with recommended defaults.
 */
void bno085_config_default(bno085_config_t *config);

/**
 * @brief Sensor ID constants for all decoded sensor types.
 *
 * These correspond to SH2 report IDs. Decoding into named fields is implemented
 * for all available sensor types (0x01–0x2A). Unrecognized sensor IDs will have
 * only sensor_id, status, and timestamp_us populated; the data union remains zeroed.
 */
#define BNO085_SENSOR_ACCELEROMETER                    0x01
#define BNO085_SENSOR_GYROSCOPE_CALIBRATED             0x02
#define BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED        0x03
#define BNO085_SENSOR_LINEAR_ACCELERATION              0x04
#define BNO085_SENSOR_ROTATION_VECTOR                  0x05
#define BNO085_SENSOR_GRAVITY                          0x06
#define BNO085_SENSOR_GYROSCOPE_UNCALIBRATED           0x07
#define BNO085_SENSOR_GAME_ROTATION_VECTOR             0x08
#define BNO085_SENSOR_GEOMAGNETIC_ROTATION_VECTOR      0x09
#define BNO085_SENSOR_PRESSURE                         0x0A
#define BNO085_SENSOR_AMBIENT_LIGHT                    0x0B
#define BNO085_SENSOR_HUMIDITY                         0x0C
#define BNO085_SENSOR_TEMPERATURE                      0x0E
#define BNO085_SENSOR_MAGNETIC_FIELD_UNCALIBRATED      0x0F
#define BNO085_SENSOR_TAP_DETECTOR                     0x10
#define BNO085_SENSOR_STEP_COUNTER                     0x11
#define BNO085_SENSOR_SIGNIFICANT_MOTION               0x12
#define BNO085_SENSOR_STABILITY_CLASSIFIER             0x13
#define BNO085_SENSOR_RAW_ACCELEROMETER                0x14
#define BNO085_SENSOR_RAW_GYROSCOPE                    0x15
#define BNO085_SENSOR_RAW_MAGNETOMETER                 0x16
#define BNO085_SENSOR_STEP_DETECTOR                    0x18
#define BNO085_SENSOR_SHAKE_DETECTOR                   0x19
#define BNO085_SENSOR_FLIP_DETECTOR                    0x1A
#define BNO085_SENSOR_PICKUP_DETECTOR                  0x1B
#define BNO085_SENSOR_PERSONAL_ACTIVITY_CLASSIFIER     0x1E
#define BNO085_SENSOR_SLEEP_DETECTOR                   0x1F
#define BNO085_SENSOR_TILT_DETECTOR                    0x20
#define BNO085_SENSOR_POCKET_DETECTOR                  0x21
#define BNO085_SENSOR_CIRCLE_DETECTOR                  0x22
#define BNO085_SENSOR_AR_VR_STABILIZED_RV              0x28
#define BNO085_SENSOR_GYRO_INTEGRATED_RV               0x2A

/**
 * @brief Decoded sensor reading from the BNO085.
 *
 * Contains one sensor report with decoded data for all 35+ available SH2 sensors.
 * For supported sensor_id values (0x01–0x2A), the appropriate data union member
 * is populated. For unrecognized sensor_id values, only sensor_id, status, and
 * timestamp_us are set; the data union is zeroed.
 *
 * The comprehensive decoding supports orientation (rotation vectors, Euler angles),
 * raw IMU (accelerometer, gyroscope, magnetometer), environmental sensors (pressure,
 * humidity, temperature, light), activity classifiers, and gesture detectors.
 */
typedef struct {
    uint8_t  sensor_id;      /**< Sensor report ID (BNO085_SENSOR_* or raw SH2 report ID) */
    uint8_t  status;         /**< Accuracy/reliability: 0=unreliable, 1=low, 2=medium, 3=high */
    uint64_t timestamp_us;   /**< Timestamp in microseconds when report was generated */

    /**
     * @brief Decoded sensor data (populated based on sensor_id).
     *
     * Access the appropriate member based on the sensor_id value.
     */
    union {
        struct { float x, y, z; } xyz;                   /**< Generic 3-axis (accel, gyro, mag, etc.) */
        struct { float i, j, k, real, accuracy_rad; } quaternion;  /**< Quaternion + accuracy */
        struct { float roll, pitch, yaw, accuracy_rad; } euler;     /**< Euler angles + accuracy */

        struct { float x, y, z; } accelerometer;         /**< m/s² (includes gravity) */
        struct { float x, y, z; } linear_acceleration;   /**< m/s² (gravity removed) */
        struct { float x, y, z; } gyroscope_calibrated;  /**< rad/s (calibrated) */
        struct { float x, y, z, bias_x, bias_y, bias_z; } gyroscope_uncalibrated;  /**< rad/s + bias */
        struct { float x, y, z; } gravity;               /**< Gravity vector in m/s² */
        struct { float x, y, z; } magnetic_field_calibrated;   /**< µTesla (calibrated) */
        struct { float x, y, z, bias_x, bias_y, bias_z; } magnetic_field_uncalibrated;  /**< µTesla + bias */

        struct { float i, j, k, real, accuracy_rad; } game_rotation_vector;           /**< Quaternion (accel + gyro, no mag) */
        struct { float i, j, k, real, accuracy_rad; } geomagnetic_rotation_vector;    /**< Quaternion (accel + mag, no gyro) */
        struct { float i, j, k, real, accuracy_rad; } ar_vr_stabilized_rv;           /**< High-precision quaternion for AR/VR */
        struct { float i, j, k, real, accuracy_rad; } gyro_integrated_rv;            /**< High-frequency quaternion (1kHz) */

        struct { int16_t x, y, z; } raw_accelerometer;   /**< Raw ADC counts */
        struct { int16_t x, y, z; } raw_gyroscope;       /**< Raw ADC counts */
        struct { int16_t x, y, z; } raw_magnetometer;    /**< Raw ADC counts */

        struct { float value; } pressure;                /**< Pa (pascals) */
        struct { float value; } humidity;                /**< % relative humidity */
        struct { float value; } temperature;             /**< °C */
        struct { float value; } ambient_light;           /**< lux */

        struct { uint8_t tap_type; uint8_t direction; } tap_detector;     /**< tap_type: 1=single, 2=double; direction: 0-5 */
        struct { uint32_t count; } step_counter;         /**< Total step count since reset */
        struct { uint8_t event; } step_detector;         /**< Event on each step (binary trigger) */
        struct { uint8_t event; } significant_motion;    /**< Motion event trigger */
        struct { uint8_t event; } flip_detector;         /**< Flip event trigger */
        struct { uint8_t event; } pickup_detector;       /**< Pickup event trigger */
        struct { uint8_t event; } shake_detector;        /**< Shake event trigger */
        struct { uint8_t event; } tilt_detector;         /**< Tilt event trigger */
        struct { uint8_t event; } pocket_detector;       /**< Pocket detection state */
        struct { uint8_t event; } circle_detector;       /**< Circle motion detected */
        struct { uint8_t event; } sleep_detector;        /**< Sleep state detected */

        struct { uint8_t activity; } stability_classifier;          /**< Activity class: 0=unknown, 1=on-table, 2=stationary, 3=stable, 4=motion */
        struct { uint8_t activity; } personal_activity_classifier;  /**< Activity class: 0=unknown, 1=still, 2=walking, 3=running, 4=on-bicycle, 5=in-vehicle, etc. */
    } data;
} bno085_sensor_value_t;

/**
 * @brief Sensor data callback function type.
 *
 * Invoked by bno085_service() when a sensor report is available.
 * The callback receives a decoded sensor value and user context.
 * Must be fast and non-blocking.
 *
 * @param handle The BNO085 handle that triggered the callback.
 * @param value Pointer to decoded sensor value (valid only during callback execution).
 * @param user_context User-provided context passed to bno085_register_sensor_callback().
 */
typedef void (*bno085_sensor_callback_t)(bno085_handle_t handle,
                                          const bno085_sensor_value_t *value,
                                          void *user_context);

/**
 * @brief Initialize the BNO085 sensor.
 *
 * Configures GPIO pins for reset and interrupt, initializes the SH2 protocol layer,
 * and performs a hardware reset. The I2C device handle must already be configured
 * and added to the bus by the caller.
 *
 * **I2C Address (AD0 Pin):**
 * The BNO085 has an AD0 pin that selects its I2C address:
 *  - AD0 pulled LOW (GND)  → I2C address 0x28
 *  - AD0 pulled HIGH (3V3) → I2C address 0x29
 *
 * The caller must create the i2c_dev handle with the correct address matching
 * their AD0 pin configuration. For example:
 *  - If AD0=LOW:  create handle with address 0x28
 *  - If AD0=HIGH: create handle with address 0x29
 *
 * **Multiplexer Usage:**
 * With an I2C multiplexer (e.g., TCA9548A), you can have multiple BNO085 sensors
 * on different channels. Each sensor appears at its configured I2C address (0x28 or 0x29)
 * when its channel is selected. Initialize one BNO085 handle per sensor, then select
 * the multiplexer channel before calling bno085_service() on each handle.
 *
 * **Concurrent Instances:**
 * Up to CONFIG_BNO085_MAX_INSTANCES concurrent instances are supported (default 2,
 * range 1–8, configured at build time). Each unit costs roughly equal Flash/IRAM.
 *
 * @param config Pointer to configuration struct (can be NULL to use defaults).
 * @param i2c_dev Configured I2C master device handle (must not be NULL).
 *                Must be created with the correct address (0x28 or 0x29) matching AD0 state.
 * @param int_pin GPIO number for H_INTN (active-low data-ready signal).
 * @param reset_pin GPIO number for RSTN (active-low hardware reset).
 * @param out_handle Pointer to receive the allocated device handle.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if i2c_dev or out_handle is NULL.
 * @return ESP_ERR_NO_MEM if all CONFIG_BNO085_MAX_INSTANCES slots are in use or heap allocation fails.
 * @return ESP_FAIL on SH2 initialization or hardware communication failure.
 */
esp_err_t bno085_init(const bno085_config_t *config,
                      i2c_master_dev_handle_t i2c_dev,
                      gpio_num_t int_pin,
                      gpio_num_t reset_pin,
                      bno085_handle_t *out_handle);

/**
 * @brief Service the BNO085 sensor hub.
 *
 * Pumps the SH2 protocol layer to read pending data from the device and dispatch
 * any async events or sensor callbacks. Must be called periodically (recommended: 10 ms).
 *
 * @param handle Device handle from bno085_init().
 */
void bno085_service(bno085_handle_t handle);

/**
 * @brief Deinitialize the BNO085 sensor and close the SH2 session.
 *
 * Puts the device in hardware reset state and frees the device handle.
 * The handle must not be used after this call.
 *
 * @param handle Device handle from bno085_init().
 */
void bno085_deinit(bno085_handle_t handle);

/**
 * @brief Register a callback to receive decoded sensor data.
 *
 * Must be called after bno085_init() and before enabling any sensors.
 * Replaces any previously registered callback.
 *
 * @param handle Device handle from bno085_init().
 * @param callback Function to invoke on sensor events (can be NULL to unregister).
 * @param user_context Opaque context pointer passed to callback.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if handle is NULL.
 * @return ESP_ERR_INVALID_STATE if device not initialized.
 */
esp_err_t bno085_register_sensor_callback(bno085_handle_t handle,
                                          bno085_sensor_callback_t callback,
                                          void *user_context);

/**
 * @brief Enable a sensor with specified reporting rate.
 *
 * Configures the sensor to stream data at the specified interval.
 * Can be used for any SH2 sensor ID (0x01–0x2A); decoding into named fields
 * is only implemented for IDs 0x01, 0x02, 0x03, 0x04, 0x05.
 *
 * @param handle Device handle from bno085_init().
 * @param sensor_id SH2 sensor report ID (e.g., BNO085_SENSOR_ROTATION_VECTOR).
 * @param report_interval_us Reporting interval in microseconds (e.g., 100000 = 10Hz).
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if handle is NULL or sensor_id is out of range.
 * @return ESP_ERR_INVALID_STATE if device not initialized.
 * @return ESP_FAIL on SH2 communication failure.
 */
esp_err_t bno085_enable_sensor(bno085_handle_t handle,
                               uint8_t sensor_id,
                               uint32_t report_interval_us);

/**
 * @brief Disable a sensor.
 *
 * Stops receiving data from the specified sensor.
 *
 * @param handle Device handle from bno085_init().
 * @param sensor_id SH2 sensor report ID to disable.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if handle is NULL.
 * @return ESP_ERR_INVALID_STATE if device not initialized.
 * @return ESP_FAIL on SH2 communication failure.
 */
esp_err_t bno085_disable_sensor(bno085_handle_t handle,
                                uint8_t sensor_id);

#ifdef __cplusplus
}
#endif

#endif
