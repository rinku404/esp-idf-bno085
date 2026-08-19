#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "bno085.h"

#define BNO085_I2C_PORT     I2C_NUM_0
#define BNO085_SDA_GPIO     GPIO_NUM_8     // PLACEHOLDER — adjust for your board
#define BNO085_SCL_GPIO     GPIO_NUM_9     // PLACEHOLDER — adjust for your board
#define BNO085_INT_GPIO     GPIO_NUM_4     // PLACEHOLDER — H_INTN pin
#define BNO085_RST_GPIO     GPIO_NUM_5     // PLACEHOLDER — RSTN pin

#define BNO085_I2C_ADDR     0x4A
#define BNO085_I2C_FREQ_HZ  400000

static const char *TAG = "app_main";

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

    ESP_ERROR_CHECK(bno085_init(dev_handle, BNO085_INT_GPIO, BNO085_RST_GPIO));
    ESP_LOGI(TAG, "BNO085 driver initialized, starting service loop");

    while (1) {
        bno085_service();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
