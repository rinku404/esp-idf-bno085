#ifndef __BNO085_H__
#define __BNO085_H__

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "sh2.h"
#include "sh2_SensorValue.h"

/**
 * @brief Initialize the BNO085 sensor via SH2 HAL.
 *
 * This function configures GPIO pins for hardware reset (RSTN) and data-ready interrupt (H_INTN),
 * then initializes the SH2 sensor hub protocol layer. The I2C device must be already configured
 * and added to the bus by the caller.
 *
 * @param i2c_dev Configured I2C master device handle (must not be NULL)
 * @param int_pin GPIO number for H_INTN (active-low data-ready signal)
 * @param reset_pin GPIO number for RSTN (active-low hardware reset)
 * @return ESP_OK on success, ESP_FAIL on SH2 initialization failure, or other esp_err_t on GPIO config error
 */
esp_err_t bno085_init(i2c_master_dev_handle_t i2c_dev, gpio_num_t int_pin, gpio_num_t reset_pin);

/**
 * @brief Service the BNO085 sensor hub.
 *
 * This function pumps the SH2 protocol layer to read pending data from the device and dispatch
 * async events. Must be called periodically (recommended: every 10 ms) from the application's main loop.
 */
void bno085_service(void);

/**
 * @brief Deinitialize the BNO085 sensor and close the SH2 session.
 *
 * Puts the device in hardware reset and closes the protocol layer.
 */
void bno085_deinit(void);

/**
 * @brief Register a callback to receive sensor data.
 *
 * The callback will be invoked whenever sensor data is available from the device.
 * Must be called after bno085_init().
 *
 * @param callback Function pointer to receive sensor events
 * @param cookie User context passed to callback (typically NULL)
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t bno085_register_sensor_callback(void (*callback)(sh2_SensorEvent_t *event), void *cookie);

/**
 * @brief Enable a sensor with specified reporting rate.
 *
 * Configures the sensor to stream data at the specified interval.
 * Must be called after bno085_init() and after registering a callback.
 *
 * @param sensor_id Sensor ID (e.g., SH2_ROTATION_VECTOR)
 * @param report_interval_us Reporting interval in microseconds (e.g., 100000 for 10Hz)
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t bno085_enable_sensor(uint8_t sensor_id, uint32_t report_interval_us);

/**
 * @brief Disable a sensor.
 *
 * Stops receiving data from the specified sensor.
 *
 * @param sensor_id Sensor ID to disable
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t bno085_disable_sensor(uint8_t sensor_id);

#endif