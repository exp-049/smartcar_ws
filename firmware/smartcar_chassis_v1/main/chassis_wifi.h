#pragma once
#include <stdbool.h>
#include "esp_err.h"

esp_err_t chassis_wifi_start(void);
bool chassis_wifi_route_ready(void);
