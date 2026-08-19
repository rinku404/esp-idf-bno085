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
 * Due to the underlying SH2 library's singleton design (sh2_t _sh2 at file scope),
 * only one BNO085 instance can be initialized at a time. Attempting bno085_init()
 * while another instance is active returns ESP_ERR_INVALID_STATE.
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
 * @brief Sensor ID constants for the five decoded sensor types.
 *
 * These correspond to SH2 report IDs. Other report IDs can be enabled/disabled
 * via bno085_enable_sensor()/bno085_disable_sensor() using raw uint8_t IDs,
 * but decoding into named fields is only implemented for these five types.
 */
#define BNO085_SENSOR_ACCELEROMETER               0x01
#define BNO085_SENSOR_GYROSCOPE_CALIBRATED        0x02
#define BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED   0x03
#define BNO085_SENSOR_LINEAR_ACCELERATION         0x04
#define BNO085_SENSOR_ROTATION_VECTOR             0x05

/**
 * @brief Decoded sensor reading from the BNO085.
 *
 * Contains one sensor report with decoded data. For sensor_id values not in
 * {0x01, 0x02, 0x03, 0x04, 0x05}, only sensor_id, status, and timestamp_us
 * are populated; the data union is zeroed (future sensors can be decoded when support is added).
 */
typedef struct {
    uint8_t  sensor_id;      /**< Sensor report ID (BNO085_SENSOR_* or raw SH2 report ID) */
    uint8_t  status;         /**< Accuracy/reliability: 0=unreliable, 1=low, 2=medium, 3=high */
    uint64_t timestamp_us;   /**< Timestamp in microseconds when report was generated */

    /**
     * @brief Decoded sensor data (populated based on sensor_id).
     *
     * Union is zeroed for unknown sensor types. Access the appropriate member
     * based on the sensor_id value.
     */
    union {
        struct {
            float i;              /**< Quaternion imaginary component i */
            float j;              /**< Quaternion imaginary component j */
            float k;              /**< Quaternion imaginary component k */
            float real;           /**< Quaternion real component (w) */
            float accuracy_rad;   /**< Accuracy estimate in radians */
        } rotation_vector;

        struct {
            float x;              /**< Acceleration in m/s² (includes gravity) */
            float y;
            float z;
        } accelerometer;

        struct {
            float x;              /**< Linear acceleration in m/s² (gravity removed) */
            float y;
            float z;
        } linear_acceleration;

        struct {
            float x;              /**< Angular velocity in rad/s */
            float y;
            float z;
        } gyroscope;

        struct {
            float x;              /**< Magnetic field in µTesla */
            float y;
            float z;
        } magnetic_field;
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
 * Due to underlying SH2 library constraints, only one BNO085 instance can be
 * initialized at a time in a process.
 *
 * @param config Pointer to configuration struct (can be NULL to use defaults).
 * @param i2c_dev Configured I2C master device handle (must not be NULL).
 * @param int_pin GPIO number for H_INTN (active-low data-ready signal).
 * @param reset_pin GPIO number for RSTN (active-low hardware reset).
 * @param out_handle Pointer to receive the allocated device handle.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if i2c_dev or out_handle is NULL.
 * @return ESP_ERR_INVALID_STATE if another BNO085 instance is already active.
 * @return ESP_ERR_NO_MEM if heap allocation fails.
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
