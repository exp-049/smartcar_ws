#pragma once
#include "chassis_core.h"
#include "esp_err.h"

typedef enum {CMD_STOP, CMD_MODE, CMD_ARM, CMD_WHEEL, CMD_TWIST, CMD_BODY, CMD_LINK, CMD_CLEAR, CMD_ZERO, CMD_CONFIG, CMD_FILTER} chassis_command_type_t;
typedef struct {
    chassis_command_type_t type;
    int64_t time_us;
    double value[3];
    chassis_config_t config;
} chassis_command_t;

esp_err_t chassis_hw_start(const chassis_config_t *config);
bool chassis_submit(const chassis_command_t *command);
void chassis_emergency_stop(void);
void chassis_snapshot(chassis_core_t *snapshot);
void chassis_print_io(void);
