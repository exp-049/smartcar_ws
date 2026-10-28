#pragma once
#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_PCNT_LIMIT 32767
#define CHASSIS_COMMAND_TIMEOUT_US 300000
#define CHASSIS_MAX_INTERVAL_US 50000
#define CHASSIS_MAX_COUNT_RATE 60000.0

typedef enum {MODE_ENCODER=0, MODE_PWM=1, MODE_PID=2, MODE_ROS=3} chassis_mode_t;
typedef enum {FAULT_NONE=0, FAULT_TIMING=1, FAULT_COUNTS=2, FAULT_STALL_L=3, FAULT_STALL_R=4, FAULT_DIRECTION_L=5, FAULT_DIRECTION_R=6} chassis_fault_t;
typedef enum {ENCODER_QUADRATURE=0, ENCODER_SINGLE_A=1} encoder_mode_t;
// How a single-phase wheel resolves pulse direction, since one phase cannot
// tell forward from reverse.
typedef enum {
    SINGLE_A_DIR_FROM_TARGET=0, // sign every pulse by the commanded target sign
    SINGLE_A_DIR_LATCHED=1      // learn it once the wheel has settled, then hold
} single_a_direction_t;
typedef struct { double kp, ki, kd, ks, kv; } wheel_gains_t;
typedef struct { double ks, kv, start_duty; } wheel_model_t;
typedef struct {
    double radius[2], separation, cpr[2];
    int encoder_sign[2], motor_sign[2];
    encoder_mode_t encoder_mode[2];
    double max_duty, max_speed, acceleration;
    wheel_gains_t gains[2];
    wheel_model_t model[2][2]; // wheel, direction: 0 forward / 1 reverse
    bool feedback_commissioned[2];
    single_a_direction_t single_a_direction;
} chassis_config_t;
typedef struct {
    double integral, previous_speed, derivative;
    double p, i, d, ff, output;
} wheel_pid_t;
typedef struct {
    uint32_t seq;
    int64_t sample_us, command_us;
    double dt, speed[2], raw_speed[2], position[2], target[2], duty[2];
    int64_t counts[2];
    int64_t delta[2];
    int64_t phase_edges[4];
    int phase_levels[4];
    int64_t hardware_count[2];
    int64_t motor_edges[2];
    uint32_t hardware_duty[4];
    double x, y, yaw, linear, angular;
    wheel_pid_t pid[2];
    chassis_mode_t mode;
    chassis_fault_t fault;
    bool armed, link_ready, initialized;
    bool encoder_valid[2], odometry_valid;
    // Single A gives magnitude only. Keep the previous drive direction while
    // coasting; never sign old pulses using a newly received target.
    int inferred_direction[2];
    double still_s[2];
    int64_t unresolved_edges[2];
    bool reversing[2], odometry_uncertain;
    bool kick_active[2];
    double kick_elapsed[2], restart_delay[2], motion_wait_s[2];
    double requested[2], ramped[2];
    int64_t raw_previous[2];
    double stalled_s[2];
    double wrong_direction_s[2];
    uint64_t timeouts, invalid_commands, overruns;
    chassis_config_t config;
} chassis_core_t;

chassis_config_t chassis_default_config(void);
bool chassis_config_valid(const chassis_config_t *config);
void chassis_core_init(chassis_core_t *state, const chassis_config_t *config);
void chassis_stop(chassis_core_t *state, bool disarm);
void chassis_set_mode(chassis_core_t *state, chassis_mode_t mode);
bool chassis_arm(chassis_core_t *state);
void chassis_set_link(chassis_core_t *state, bool ready);
bool chassis_command_wheels(chassis_core_t *state, double left, double right, int64_t now_us);
bool chassis_command_body(chassis_core_t *state, double linear, double angular, int64_t now_us);
bool chassis_command_normalized_wheels(chassis_core_t *state, double left, double right, int64_t now_us);
int64_t chassis_accum_counter_delta(int current, int previous);
void chassis_core_tick(chassis_core_t *state, int64_t now_us, int64_t raw_left, int64_t raw_right);
