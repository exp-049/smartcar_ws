#include <math.h>
#include <stdint.h>
#include <string.h>

#include "chassis_hw.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "geometry_msgs/msg/twist.h"
#include "nav_msgs/msg/odometry.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rmw_microros/rmw_microros.h"
#include "rosidl_runtime_c/string_functions.h"
#include "sdkconfig.h"
#include "std_msgs/msg/bool.h"
#include "std_msgs/msg/u_int8.h"
#include "std_msgs/msg/u_int64.h"
#include "esp32_serial_transport.h"

#define ODOM_TOPIC "/car/chassis/odom"
#define ODOM_VALID_TOPIC "/car/chassis/odom_valid"
#define STATUS_TOPIC "/car/chassis/status"
#define PHASE_EDGES_TOPIC "/car/chassis/phase_edges"
#define MAINTENANCE_TOPIC "/car/chassis/maintenance"
#define CMD_VEL_TOPIC "/car/chassis/cmd_vel"
#define ODOM_FRAME "odom"
#define BASE_FRAME "base_link"
#define CHASSIS_UROS_UART UART_NUM_0

typedef struct {
    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;
    rcl_publisher_t odom_publisher;
    rcl_publisher_t valid_publisher;
    rcl_publisher_t status_publisher;
    rcl_publisher_t phase_edges_publisher;
    rcl_subscription_t command_subscription;
    rcl_subscription_t maintenance_subscription;
    rcl_timer_t publish_timer;
    rclc_executor_t executor;
    bool support_ready;
    bool node_ready;
    bool odom_publisher_ready;
    bool valid_publisher_ready;
    bool status_publisher_ready;
    bool phase_edges_publisher_ready;
    bool subscription_ready;
    bool maintenance_subscription_ready;
    bool timer_ready;
    bool executor_ready;
} ros_entities_t;

static nav_msgs__msg__Odometry s_odom_message;
static std_msgs__msg__Bool s_odom_valid_message;
static std_msgs__msg__UInt64 s_status_message;
static std_msgs__msg__UInt64 s_phase_edges_message;
static geometry_msgs__msg__Twist s_command_message;
static std_msgs__msg__UInt8 s_maintenance_message;
static volatile bool s_publish_failed;
static volatile uint16_t s_command_callback_count;
static volatile uint8_t s_command_result;
static ros_entities_t *s_active_entities;

static bool submit_simple(chassis_command_type_t type, double value)
{
    chassis_command_t command = {.type = type, .time_us = esp_timer_get_time()};
    command.value[0] = value;
    return chassis_submit(&command);
}

static void publish_odometry(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;

    int64_t stamp_ns = rmw_uros_epoch_nanos();
    if (stamp_ns <= 0) {
        s_publish_failed = true;
        return;
    }

    chassis_core_t state;
    chassis_snapshot(&state);
    s_odom_message.header.stamp.sec = (int32_t)(stamp_ns / 1000000000LL);
    s_odom_message.header.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000LL);
    s_odom_message.pose.pose.position.x = state.x;
    s_odom_message.pose.pose.position.y = state.y;
    s_odom_message.pose.pose.position.z = 0.0;
    s_odom_message.pose.pose.orientation.x = 0.0;
    s_odom_message.pose.pose.orientation.y = 0.0;
    s_odom_message.pose.pose.orientation.z = sin(state.yaw * 0.5);
    s_odom_message.pose.pose.orientation.w = cos(state.yaw * 0.5);
    s_odom_message.twist.twist.linear.x = state.linear;
    s_odom_message.twist.twist.linear.y = 0.0;
    s_odom_message.twist.twist.linear.z = 0.0;
    s_odom_message.twist.twist.angular.x = 0.0;
    s_odom_message.twist.twist.angular.y = 0.0;
    s_odom_message.twist.twist.angular.z = state.angular;

    memset(s_odom_message.pose.covariance, 0, sizeof(s_odom_message.pose.covariance));
    memset(s_odom_message.twist.covariance, 0, sizeof(s_odom_message.twist.covariance));
    const double unknown = state.odometry_valid ? 1.0 : 1000.0;
    s_odom_message.pose.covariance[0] = unknown;
    s_odom_message.pose.covariance[7] = unknown;
    s_odom_message.pose.covariance[14] = 1000000.0;
    s_odom_message.pose.covariance[21] = 1000000.0;
    s_odom_message.pose.covariance[28] = 1000000.0;
    s_odom_message.pose.covariance[35] = unknown;
    s_odom_message.twist.covariance[0] = unknown;
    s_odom_message.twist.covariance[7] = 1000000.0;
    s_odom_message.twist.covariance[14] = 1000000.0;
    s_odom_message.twist.covariance[21] = 1000000.0;
    s_odom_message.twist.covariance[28] = 1000000.0;
    s_odom_message.twist.covariance[35] = unknown;
    s_odom_valid_message.data = state.odometry_valid;
    s_status_message.data = (state.odometry_valid ? 1ull : 0ull) |
        (state.encoder_valid[0] ? 1ull << 1 : 0ull) |
        (state.encoder_valid[1] ? 1ull << 2 : 0ull) |
        (state.armed ? 1ull << 3 : 0ull) |
        (state.link_ready ? 1ull << 4 : 0ull) |
        (state.odometry_uncertain ? 1ull << 5 : 0ull) |
        ((uint64_t)state.fault << 8) |
        ((uint64_t)state.mode << 12) |
        ((uint64_t)(uint16_t)state.hardware_count[0] << 16) |
        ((uint64_t)(uint16_t)state.hardware_count[1] << 32) |
        ((uint64_t)(s_command_callback_count & 0x0fffu) << 48) |
        ((uint64_t)(s_command_result & 0x0fu) << 60);
    s_phase_edges_message.data = 0;
    for (size_t i = 0; i < 4; ++i) {
        s_phase_edges_message.data |= ((uint64_t)state.phase_edges[i] & 0xffffull) << (16 * i);
    }

    if (s_active_entities == NULL ||
        rcl_publish(&s_active_entities->odom_publisher, &s_odom_message, NULL) != RCL_RET_OK) {
        s_publish_failed = true;
        return;
    }
    if (rcl_publish(&s_active_entities->valid_publisher, &s_odom_valid_message, NULL) != RCL_RET_OK) {
        s_publish_failed = true;
        return;
    }
    if (rcl_publish(&s_active_entities->status_publisher, &s_status_message, NULL) != RCL_RET_OK) {
        s_publish_failed = true;
        return;
    }
    if (rcl_publish(&s_active_entities->phase_edges_publisher, &s_phase_edges_message, NULL) != RCL_RET_OK) {
        s_publish_failed = true;
    }
}

static void command_callback(const void *message)
{
    s_command_callback_count++;
    const geometry_msgs__msg__Twist *twist = message;
    const double v = twist->linear.x;
    const double w = twist->angular.z;
    if (!isfinite(v) || !isfinite(w) ||
        !isfinite(twist->linear.y) || !isfinite(twist->linear.z) ||
        !isfinite(twist->angular.x) || !isfinite(twist->angular.y) ||
        fabs(twist->linear.y) > 1e-9 || fabs(twist->linear.z) > 1e-9 ||
        fabs(twist->angular.x) > 1e-9 || fabs(twist->angular.y) > 1e-9) {
        s_command_result = 1;
        chassis_emergency_stop();
        return;
    }

    chassis_core_t state;
    chassis_snapshot(&state);
    if (!state.armed) {
        chassis_command_t arm = {.type = CMD_ARM, .time_us = esp_timer_get_time()};
        if (!chassis_submit(&arm)) {
            s_command_result = 2;
            return;
        }
    }
    chassis_command_t command = {.type = CMD_TWIST, .time_us = esp_timer_get_time()};
    command.value[0] = v;
    command.value[1] = w;
    if (!chassis_submit(&command)) {
        s_command_result = 3;
        chassis_emergency_stop();
    } else {
        s_command_result = 4;
    }
}

static void maintenance_callback(const void *message)
{
    const std_msgs__msg__UInt8 *request = message;
    if (request->data != 1 && request->data != 2) return;
    chassis_core_t state;
    chassis_snapshot(&state);
    if (state.armed) return;
    chassis_command_t command = {
        .type = request->data == 1 ? CMD_CLEAR : CMD_ZERO,
        .time_us = esp_timer_get_time(),
    };
    chassis_submit(&command);
}

static void fini_entities(ros_entities_t *entities)
{
    if (s_active_entities == entities) s_active_entities = NULL;
    if (entities->executor_ready) rclc_executor_fini(&entities->executor);
    if (entities->timer_ready) rcl_timer_fini(&entities->publish_timer);
    if (entities->subscription_ready) rcl_subscription_fini(&entities->command_subscription, &entities->node);
    if (entities->maintenance_subscription_ready) rcl_subscription_fini(&entities->maintenance_subscription, &entities->node);
    if (entities->valid_publisher_ready) rcl_publisher_fini(&entities->valid_publisher, &entities->node);
    if (entities->status_publisher_ready) rcl_publisher_fini(&entities->status_publisher, &entities->node);
    if (entities->phase_edges_publisher_ready) rcl_publisher_fini(&entities->phase_edges_publisher, &entities->node);
    if (entities->odom_publisher_ready) rcl_publisher_fini(&entities->odom_publisher, &entities->node);
    if (entities->node_ready) rcl_node_fini(&entities->node);
    if (entities->support_ready) rclc_support_fini(&entities->support);
    memset(entities, 0, sizeof(*entities));
}

static bool init_entities(ros_entities_t *entities)
{
    memset(entities, 0, sizeof(*entities));
    entities->allocator = rcl_get_default_allocator();
    entities->node = rcl_get_zero_initialized_node();
    entities->odom_publisher = rcl_get_zero_initialized_publisher();
    entities->valid_publisher = rcl_get_zero_initialized_publisher();
    entities->status_publisher = rcl_get_zero_initialized_publisher();
    entities->phase_edges_publisher = rcl_get_zero_initialized_publisher();
    entities->command_subscription = rcl_get_zero_initialized_subscription();
    entities->maintenance_subscription = rcl_get_zero_initialized_subscription();
    entities->publish_timer = rcl_get_zero_initialized_timer();
    entities->executor = rclc_executor_get_zero_initialized_executor();

    if (rclc_support_init(&entities->support, 0, NULL, &entities->allocator) != RCL_RET_OK) goto fail;
    entities->support_ready = true;
    if (rclc_node_init_default(&entities->node, "smartcar_chassis", "", &entities->support) != RCL_RET_OK) goto fail;
    entities->node_ready = true;
    if (rclc_publisher_init_default(&entities->odom_publisher, &entities->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), ODOM_TOPIC) != RCL_RET_OK) goto fail;
    entities->odom_publisher_ready = true;
    if (rclc_publisher_init_default(&entities->valid_publisher, &entities->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), ODOM_VALID_TOPIC) != RCL_RET_OK) goto fail;
    entities->valid_publisher_ready = true;
    if (rclc_publisher_init_default(&entities->status_publisher, &entities->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt64), STATUS_TOPIC) != RCL_RET_OK) goto fail;
    entities->status_publisher_ready = true;
    if (rclc_publisher_init_default(&entities->phase_edges_publisher, &entities->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt64), PHASE_EDGES_TOPIC) != RCL_RET_OK) goto fail;
    entities->phase_edges_publisher_ready = true;
    if (rclc_subscription_init_default(&entities->command_subscription, &entities->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), CMD_VEL_TOPIC) != RCL_RET_OK) goto fail;
    entities->subscription_ready = true;
    if (rclc_subscription_init_default(&entities->maintenance_subscription, &entities->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), MAINTENANCE_TOPIC) != RCL_RET_OK) goto fail;
    entities->maintenance_subscription_ready = true;
    if (rclc_timer_init_default2(&entities->publish_timer, &entities->support,
            RCL_MS_TO_NS(200), publish_odometry, true) != RCL_RET_OK) goto fail;
    entities->timer_ready = true;
    if (rclc_executor_init(&entities->executor, &entities->support.context, 3,
            &entities->allocator) != RCL_RET_OK) goto fail;
    entities->executor_ready = true;
    if (rclc_executor_add_subscription(&entities->executor, &entities->command_subscription,
            &s_command_message, command_callback, ON_NEW_DATA) != RCL_RET_OK) goto fail;
    if (rclc_executor_add_subscription(&entities->executor, &entities->maintenance_subscription,
            &s_maintenance_message, maintenance_callback, ON_NEW_DATA) != RCL_RET_OK) goto fail;
    if (rclc_executor_add_timer(&entities->executor, &entities->publish_timer) != RCL_RET_OK) goto fail;
    s_active_entities = entities;
    return true;

fail:
    fini_entities(entities);
    return false;
}

static void switch_to_ros_mode(void)
{
    chassis_command_t mode = {.type = CMD_MODE, .time_us = esp_timer_get_time()};
    mode.value[0] = MODE_ROS;
    if (!chassis_submit(&mode)) return;
    submit_simple(CMD_LINK, 1.0);
}

static void switch_to_safe_mode(void)
{
    chassis_emergency_stop();
    vTaskDelay(pdMS_TO_TICKS(30));
    submit_simple(CMD_LINK, 0.0);
    chassis_command_t mode = {.type = CMD_MODE, .time_us = esp_timer_get_time()};
    mode.value[0] = MODE_ENCODER;
    chassis_submit(&mode);
}

static void micro_ros_task(void *argument)
{
    (void)argument;
    uart_port_t uart_port = CHASSIS_UROS_UART;
    rmw_uros_set_custom_transport(true, &uart_port, smartcar_serial_open,
                                  smartcar_serial_close, smartcar_serial_write,
                                  smartcar_serial_read);

    while (true) {
        if (rmw_uros_ping_agent(250, 1) != RMW_RET_OK) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        ros_entities_t entities;
        if (!init_entities(&entities)) {
            switch_to_safe_mode();
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        s_publish_failed = false;
        if (rmw_uros_sync_session(1000) != RMW_RET_OK) {
            fini_entities(&entities);
            switch_to_safe_mode();
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        switch_to_ros_mode();
        bool connected = true;
        while (connected && !s_publish_failed) {
            rcl_ret_t result = rclc_executor_spin_some(&entities.executor, RCL_MS_TO_NS(25));
            if (result != RCL_RET_OK && result != RCL_RET_TIMEOUT) connected = false;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        switch_to_safe_mode();
        fini_entities(&entities);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static bool load_chassis_config(chassis_config_t *config)
{
    *config = chassis_default_config();
    nvs_handle_t nvs;
    if (nvs_open("chassis_v1", NVS_READONLY, &nvs) != ESP_OK) return true;
    chassis_config_t saved;
    size_t size = sizeof(saved);
    esp_err_t result = nvs_get_blob(nvs, "config_v3", &saved, &size);
    nvs_close(nvs);
    if (result == ESP_OK && size == sizeof(saved) && chassis_config_valid(&saved)) *config = saved;
    return true;
}

void app_main(void)
{
    esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES || nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (nvs_flash_erase() != ESP_OK) return;
        nvs_result = nvs_flash_init();
    }
    if (nvs_result != ESP_OK) return;
    if (geometry_msgs__msg__Twist__init(&s_command_message) != true ||
        nav_msgs__msg__Odometry__init(&s_odom_message) != true ||
        std_msgs__msg__Bool__init(&s_odom_valid_message) != true ||
        std_msgs__msg__UInt8__init(&s_maintenance_message) != true ||
        std_msgs__msg__UInt64__init(&s_status_message) != true ||
        std_msgs__msg__UInt64__init(&s_phase_edges_message) != true) return;
    if (rosidl_runtime_c__String__assign(&s_odom_message.header.frame_id, ODOM_FRAME) != true ||
        rosidl_runtime_c__String__assign(&s_odom_message.child_frame_id, BASE_FRAME) != true) return;
    chassis_config_t config;
    load_chassis_config(&config);
    if (chassis_hw_start(&config) != ESP_OK) return;
    xTaskCreate(micro_ros_task, "micro_ros", CONFIG_MICRO_ROS_APP_STACK,
                NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}
