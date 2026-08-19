#include "bno085.h"
#include "sh2.h"
#include "sh2_hal.h"
#include "sh2_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BNO085_SHTP_HDR_LEN   (4)
#define BNO085_I2C_TIMEOUT_MS (100)

static const char *TAG = "bno085";

typedef struct {
    sh2_Hal_t base;
    i2c_master_dev_handle_t i2c_dev;
    gpio_num_t int_pin;
    gpio_num_t reset_pin;
} bno085_hal_t;

static bno085_hal_t s_hal;

// User-provided sensor callback
static void (*s_user_sensor_callback)(sh2_SensorEvent_t *event) = NULL;
static void *s_user_sensor_cookie = NULL;

static void bno085_sensor_cb(void *cookie, sh2_SensorEvent_t *pEvent)
{
    (void) cookie;
    if (s_user_sensor_callback) {
        s_user_sensor_callback(pEvent);
    }
}

static void bno085_event_cb(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    (void) cookie;
    switch (pEvent->eventId) {
        case SH2_RESET:
            ESP_LOGI(TAG, "BNO085 sensor hub reset event");
            break;
        case SH2_SHTP_EVENT:
            ESP_LOGD(TAG, "SHTP event: %d", pEvent->shtpEvent);
            break;
        case SH2_GET_FEATURE_RESP:
            ESP_LOGD(TAG, "Get-feature response received");
            break;
        default:
            ESP_LOGD(TAG, "Unhandled sh2 async event id=%lu", (unsigned long) pEvent->eventId);
            break;
    }
}

static int bno085_hal_open(sh2_Hal_t *self)
{
    bno085_hal_t *me = (bno085_hal_t *)self;

    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    bool success = false;

    for (uint8_t attempts = 0; attempts < 5; attempts++) {
        esp_err_t err = i2c_master_transmit(me->i2c_dev, softreset_pkt, sizeof(softreset_pkt), 100);
        if (err == ESP_OK) {
            success = true;
            ESP_LOGI(TAG, "Soft-reset I2C packet sent successfully");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    if (!success) {
        ESP_LOGE(TAG, "Failed to send soft-reset I2C packet after 5 attempts");
        return SH2_ERR_IO;
    }

    vTaskDelay(pdMS_TO_TICKS(300));
    return SH2_OK;
}

static void bno085_hal_close(sh2_Hal_t *self)
{
    bno085_hal_t *me = (bno085_hal_t *)self;
    gpio_set_level(me->reset_pin, 0);
}

static int bno085_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    bno085_hal_t *me = (bno085_hal_t *)self;

    if (gpio_get_level(me->int_pin) != 0) {
        return 0;
    }

    *t_us = (uint32_t) esp_timer_get_time();

    uint8_t header[BNO085_SHTP_HDR_LEN];
    esp_err_t err = i2c_master_receive(me->i2c_dev, header, sizeof(header),
                                        BNO085_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return 0;
    }

    uint16_t packet_len = (uint16_t)(header[0] | (header[1] << 8)) & 0x7FFF;
    if (packet_len < BNO085_SHTP_HDR_LEN) {
        return 0;
    }

    if (packet_len > len) {
        packet_len = (uint16_t)len;
    }
    if (packet_len > SH2_HAL_MAX_TRANSFER_IN) {
        packet_len = SH2_HAL_MAX_TRANSFER_IN;
    }

    err = i2c_master_receive(me->i2c_dev, pBuffer, packet_len, BNO085_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return 0;
    }

    return (int) packet_len;
}

static int bno085_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    bno085_hal_t *me = (bno085_hal_t *)self;

    esp_err_t err = i2c_master_transmit(me->i2c_dev, pBuffer, len, BNO085_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return 0;
    }
    return (int) len;
}

static uint32_t bno085_hal_getTimeUs(sh2_Hal_t *self)
{
    (void) self;
    return (uint32_t) esp_timer_get_time();
}

esp_err_t bno085_init(i2c_master_dev_handle_t i2c_dev, gpio_num_t int_pin, gpio_num_t reset_pin)
{
    if (i2c_dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_config_t int_cfg = {
        .pin_bit_mask = (1ULL << int_pin),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&int_cfg);
    if (err != ESP_OK) {
        return err;
    }

    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << reset_pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&rst_cfg);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(reset_pin, 1);

    ESP_LOGI(TAG, "Asserting NRST (reset pulse)...");
    gpio_set_level(reset_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(reset_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    s_hal.base.open      = bno085_hal_open;
    s_hal.base.close     = bno085_hal_close;
    s_hal.base.read      = bno085_hal_read;
    s_hal.base.write     = bno085_hal_write;
    s_hal.base.getTimeUs = bno085_hal_getTimeUs;
    s_hal.i2c_dev   = i2c_dev;
    s_hal.int_pin   = int_pin;
    s_hal.reset_pin = reset_pin;

    ESP_LOGI(TAG, "Calling sh2_open()...");
    int rc = sh2_open(&s_hal.base, bno085_event_cb, NULL);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "sh2_open() failed, rc=%d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BNO085 initialized successfully (int_pin=%d, reset_pin=%d)", int_pin, reset_pin);
    return ESP_OK;
}

void bno085_service(void)
{
    sh2_service();
}

void bno085_deinit(void)
{
    sh2_close();
}

esp_err_t bno085_register_sensor_callback(void (*callback)(sh2_SensorEvent_t *event), void *cookie)
{
    if (!callback) {
        return ESP_ERR_INVALID_ARG;
    }

    s_user_sensor_callback = callback;
    s_user_sensor_cookie = cookie;

    int rc = sh2_setSensorCallback(bno085_sensor_cb, cookie);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "sh2_setSensorCallback() failed, rc=%d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sensor callback registered");
    return ESP_OK;
}

esp_err_t bno085_enable_sensor(uint8_t sensor_id, uint32_t report_interval_us)
{
    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));

    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.alwaysOnEnabled = false;
    config.sniffEnabled = false;
    config.reportInterval_us = report_interval_us;

    int rc = sh2_setSensorConfig(sensor_id, &config);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "Failed to enable sensor 0x%02x, rc=%d", sensor_id, rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sensor 0x%02x enabled with interval %u us", sensor_id, report_interval_us);
    return ESP_OK;
}

esp_err_t bno085_disable_sensor(uint8_t sensor_id)
{
    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));
    config.reportInterval_us = 0;  // 0 = disable

    int rc = sh2_setSensorConfig(sensor_id, &config);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "Failed to disable sensor 0x%02x, rc=%d", sensor_id, rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sensor 0x%02x disabled", sensor_id);
    return ESP_OK;
}
