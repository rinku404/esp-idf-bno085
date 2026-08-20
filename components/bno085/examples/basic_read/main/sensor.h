#pragma once

#include "bno085.h"

void sensor_init(void);
void sensor_register_callback(bno085_handle_t handle);
void sensor_install_isr(void);
void sensor_start_task(bno085_handle_t handle);

typedef struct {
    bno085_sensor_value_t data;
    bool has_data;
} sensor_data_snapshot_t;

sensor_data_snapshot_t sensor_get_data_snapshot(int sensor_id);
uint64_t sensor_get_data_timestamp(int sensor_id);
uint64_t sensor_get_latest_timestamp(void);
