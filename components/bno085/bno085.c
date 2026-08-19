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
    (void) self;
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

    ESP_LOGI(TAG, "Waiting for H_INTN to assert (device ready)...");
    int timeout_ms = 500;
    int elapsed = 0;
    while (gpio_get_level(int_pin) != 0 && elapsed < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(1));
        elapsed++;
    }

    if (gpio_get_level(int_pin) != 0) {
        ESP_LOGE(TAG, "H_INTN never asserted after reset (timeout %d ms) — check RESET/power wiring", timeout_ms);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "H_INTN asserted after %d ms, device is ready", elapsed);

    s_hal.base.open      = bno085_hal_open;
    s_hal.base.close     = bno085_hal_close;
    s_hal.base.read      = bno085_hal_read;
    s_hal.base.write     = bno085_hal_write;
    s_hal.base.getTimeUs = bno085_hal_getTimeUs;
    s_hal.i2c_dev   = i2c_dev;
    s_hal.int_pin   = int_pin;
    s_hal.reset_pin = reset_pin;

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
