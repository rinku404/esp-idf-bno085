#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "bno085.h"
#include "sdkconfig.h"
#include "sensor.h"

#define MAX_SENSOR_ID 0x2B

static const char *TAG = "sensor";

typedef struct {
    bno085_sensor_value_t data;
    uint64_t timestamp_ms;
    SemaphoreHandle_t mutex;
    bool has_data;
} sensor_data_t;

static sensor_data_t sensor_data[MAX_SENSOR_ID];
static SemaphoreHandle_t data_ready_sem = NULL;

#define BNO085_INT_GPIO ((gpio_num_t)CONFIG_APP_BNO085_INT_GPIO)

static void IRAM_ATTR bno085_int_isr_handler(void *arg)
{
    (void) arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (data_ready_sem != NULL) {
        xSemaphoreGiveFromISR(data_ready_sem, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

static void sensor_callback(bno085_handle_t handle, const bno085_sensor_value_t *value, void *user_context)
{
    (void) handle;
    (void) user_context;

    if (!value || value->sensor_id >= MAX_SENSOR_ID) return;

    sensor_data_t *s = &sensor_data[value->sensor_id];
    if (xSemaphoreTake(s->mutex, pdMS_TO_TICKS(10))) {
        s->data = *value;
        s->timestamp_ms = value->timestamp_us / 1000;
        s->has_data = true;
        xSemaphoreGive(s->mutex);
    }
}

static void service_task(void *pvParameters)
{
    (void) pvParameters;
    bno085_handle_t handle = (bno085_handle_t)pvParameters;
    ESP_LOGI(TAG, "Service task started (update period: %d ms)", CONFIG_APP_BNO085_SENSOR_UPDATE_PERIOD_MS);

    while (1) {
        bno085_service(handle);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_APP_BNO085_SENSOR_UPDATE_PERIOD_MS));
    }
}

void sensor_init(void)
{
    data_ready_sem = xSemaphoreCreateBinary();
    if (data_ready_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create data_ready_sem");
        return;
    }

    for (int i = 0; i < MAX_SENSOR_ID; i++) {
        sensor_data[i].mutex = xSemaphoreCreateMutex();
        sensor_data[i].has_data = false;
        if (sensor_data[i].mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex for sensor %d", i);
            return;
        }
    }
}

void sensor_register_callback(bno085_handle_t handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Invalid handle for registering callback");
        return;
    }
    ESP_ERROR_CHECK(bno085_register_sensor_callback(handle, sensor_callback, NULL));
}

void sensor_install_isr(void)
{
    gpio_config_t int_gpio_config = {
        .pin_bit_mask = (1ULL << BNO085_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&int_gpio_config));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BNO085_INT_GPIO, bno085_int_isr_handler, NULL));
    ESP_LOGI(TAG, "INT pin interrupt configured (active low)");
}

void sensor_start_task(bno085_handle_t handle)
{
    xTaskCreate(service_task, "service_task", 4096, (void *)handle, 5, NULL);
    ESP_LOGI(TAG, "Service task created");
}

sensor_data_snapshot_t sensor_get_data_snapshot(int sensor_id)
{
    sensor_data_snapshot_t snapshot = {.has_data = false};

    if (sensor_id < 0 || sensor_id >= MAX_SENSOR_ID) {
        return snapshot;
    }

    sensor_data_t *s = &sensor_data[sensor_id];
    if (s->has_data && xSemaphoreTake(s->mutex, pdMS_TO_TICKS(5))) {
        snapshot.data = s->data;
        snapshot.has_data = true;
        xSemaphoreGive(s->mutex);
    }
    return snapshot;
}

uint64_t sensor_get_data_timestamp(int sensor_id)
{
    if (sensor_id < 0 || sensor_id >= MAX_SENSOR_ID) {
        return 0;
    }

    sensor_data_t *s = &sensor_data[sensor_id];
    if (s->has_data && xSemaphoreTake(s->mutex, pdMS_TO_TICKS(5))) {
        uint64_t ts = s->timestamp_ms;
        xSemaphoreGive(s->mutex);
        return ts;
    }
    return 0;
}

uint64_t sensor_get_latest_timestamp(void)
{
    for (int i = 0; i < MAX_SENSOR_ID; i++) {
        sensor_data_t *s = &sensor_data[i];
        if (s->has_data && xSemaphoreTake(s->mutex, pdMS_TO_TICKS(5))) {
            uint64_t ts = s->timestamp_ms;
            xSemaphoreGive(s->mutex);
            return ts;
        }
    }
    return 0;
}
