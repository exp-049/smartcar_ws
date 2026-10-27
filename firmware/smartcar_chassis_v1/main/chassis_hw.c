#include "chassis_hw.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"

static const int MOTOR_PINS[4]={15,13,25,14};
static const int ENCODER_PINS[2][2]={{16,17},{18,19}};
// GPIO5 is a passive duplicate input of the user's GPIO5--GPIO17--B junction.
static const int AUX_INPUT_PINS[2]={15,5};
static pcnt_unit_handle_t s_pcnt[2];
static pcnt_unit_handle_t s_phase_pcnt[4];
static int s_phase_previous[4];
static pcnt_unit_handle_t s_motor_pcnt[2];
static int s_motor_previous[2];
static TaskHandle_t s_control_task;
static QueueHandle_t s_commands;
static portMUX_TYPE s_lock=portMUX_INITIALIZER_UNLOCKED;
static chassis_core_t s_state, s_snapshot;
static bool s_stop_requested;
static int s_last_direction[2];
static uint32_t s_boot_output_enable;
static int64_t s_phase_window_start[4], s_phase_check_us;

static void pwm_write(int channel, double duty)
{
    uint32_t value=(uint32_t)lround(fmin(1.0,fmax(0.0,duty))*2047.0);
    // Only this control task writes LEDC; no fade-service / ISR is needed.
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,(ledc_channel_t)channel,value));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,(ledc_channel_t)channel));
}

static void motor_write(int wheel, double value)
{
    int direction=(value>1e-8)-(value<-1e-8);
    if(direction!=s_last_direction[wheel]) {
        pwm_write(wheel*2,0); pwm_write(wheel*2+1,0);
        if(direction!=0 && s_last_direction[wheel]!=0) {
            s_last_direction[wheel]=0; return;
        }
    }
    s_last_direction[wheel]=direction;
    if(direction>0) pwm_write(wheel*2,fabs(value));
    if(direction<0) pwm_write(wheel*2+1,fabs(value));
}

static bool IRAM_ATTR timer_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *event, void *context)
{
    (void)timer; (void)event; (void)context;
    BaseType_t woken=pdFALSE;
    vTaskNotifyGiveFromISR(s_control_task,&woken);
    return woken==pdTRUE;
}

static void process_command(const chassis_command_t *c)
{
    if(c->type==CMD_STOP) chassis_stop(&s_state,true);
    else if(c->type==CMD_MODE) chassis_set_mode(&s_state,(int)c->value[0]);
    else if(c->type==CMD_ARM) chassis_arm(&s_state);
    else if(c->type==CMD_LINK) chassis_set_link(&s_state,c->value[0]!=0);
    else if(c->type==CMD_CLEAR && !s_state.armed) {
        s_state.fault=FAULT_NONE;
    } else if(c->type==CMD_ZERO && !s_state.armed) {
        s_state.x=s_state.y=s_state.yaw=0;
        for(int j=0;j<2;j++) {s_state.counts[j]=0; s_state.position[j]=0;}
        s_state.odometry_uncertain=false;
        for(int j=0;j<2;j++) s_state.unresolved_edges[j]=0;
        for(int j=0;j<4;j++) s_state.phase_edges[j]=0;
        memset(s_phase_window_start,0,sizeof(s_phase_window_start));
        s_phase_check_us=esp_timer_get_time();
        for(int j=0;j<2;j++) s_state.motor_edges[j]=0;
    } else if(c->type==CMD_CONFIG && !s_state.armed && chassis_config_valid(&c->config)) {
        s_state.config=c->config;
        chassis_stop(&s_state,true);
    } else if(c->type==CMD_FILTER && !s_state.armed && s_state.mode==MODE_ENCODER) {
        uint32_t ns=(uint32_t)c->value[0];
        pcnt_glitch_filter_config_t filter={.max_glitch_ns=ns};
        for(int j=0;j<8;j++) {
            pcnt_unit_handle_t unit=j<2?s_pcnt[j]:j<6?s_phase_pcnt[j-2]:s_motor_pcnt[j-6];
            esp_err_t err=pcnt_unit_stop(unit);
            if(err==ESP_OK) err=pcnt_unit_disable(unit);
            if(err==ESP_OK) err=pcnt_unit_set_glitch_filter(unit,ns?&filter:NULL);
            if(err==ESP_OK) err=pcnt_unit_enable(unit);
            if(err==ESP_OK) err=pcnt_unit_start(unit);
            if(err!=ESP_OK) {s_state.fault=FAULT_COUNTS;chassis_stop(&s_state,true);break;}
        }
    } else if(c->type==CMD_WHEEL || c->type==CMD_TWIST || c->type==CMD_BODY) {
        int64_t age=esp_timer_get_time()-c->time_us;
        if(age<0 || age>CHASSIS_COMMAND_TIMEOUT_US) { chassis_stop(&s_state,true); return; }
        if(c->type==CMD_WHEEL) chassis_command_wheels(&s_state,c->value[0],c->value[1],c->time_us);
        else if(c->type==CMD_BODY) chassis_command_body(&s_state,c->value[0],c->value[1],c->time_us);
        else chassis_command_twist(&s_state,c->value[0],c->value[1],c->time_us);
    }
}

static void control_task(void *argument)
{
    (void)argument;
    while(true) {
        uint32_t notifications=ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(30));
        bool stop;
        portENTER_CRITICAL(&s_lock);
        stop=s_stop_requested; s_stop_requested=false;
        portEXIT_CRITICAL(&s_lock);
        if(stop) {xQueueReset(s_commands); chassis_stop(&s_state,true);}
        if(notifications==0 || notifications>4) {
            s_state.fault=FAULT_TIMING; ++s_state.overruns; chassis_stop(&s_state,true);
        }
        chassis_command_t command;
        for(int i=0;!stop && i<16 && xQueueReceive(s_commands,&command,0)==pdTRUE;i++) process_command(&command);
        for(int j=0;j<4;j++) {
            int phase_count;
            if(pcnt_unit_get_count(s_phase_pcnt[j],&phase_count)==ESP_OK) {
                s_state.phase_edges[j]+=chassis_counter_delta(phase_count,s_phase_previous[j]);
                s_phase_previous[j]=phase_count;
            }
            s_state.phase_levels[j]=gpio_get_level(ENCODER_PINS[j/2][j%2]);
        }
        int64_t phase_now=esp_timer_get_time();
        if(phase_now-s_phase_check_us>=200000) {
            for(int j=0;j<2;j++) {
                int64_t ea=s_state.phase_edges[2*j]-s_phase_window_start[2*j];
                int64_t eb=s_state.phase_edges[2*j+1]-s_phase_window_start[2*j+1];
                if(s_state.config.encoder_mode[j]==ENCODER_SINGLE_A) {
                    if(ea>16) s_state.encoder_valid[j]=true;
                    continue;
                }
                if(ea>16 && eb>16 && llabs(ea-eb)<(ea+eb)/4+4) s_state.encoder_valid[j]=true;
                bool missing=(ea>64 && eb<=1)||(eb>64 && ea<=1);
                if(missing) {
                    s_state.encoder_valid[j]=false;
                    if(s_state.armed && fabs(s_state.duty[j])>.1) {s_state.fault=FAULT_COUNTS;chassis_stop(&s_state,true);}
                }
            }
            memcpy(s_phase_window_start,s_state.phase_edges,sizeof(s_phase_window_start));
            s_phase_check_us=phase_now;
        }
        int counts[2];
        esp_err_t a=pcnt_unit_get_count(s_pcnt[0],&counts[0]);
        esp_err_t b=pcnt_unit_get_count(s_pcnt[1],&counts[1]);
        if(a!=ESP_OK || b!=ESP_OK) {
            s_state.fault=FAULT_COUNTS; s_state.odometry_valid=false;chassis_stop(&s_state,true);
        } else {
            s_state.hardware_count[0]=counts[0];s_state.hardware_count[1]=counts[1];
            chassis_core_tick(&s_state,esp_timer_get_time(),counts[0],counts[1]);
        }
        for(int j=0;j<2;j++) {
            int raw_motor;
            if(pcnt_unit_get_count(s_motor_pcnt[j],&raw_motor)==ESP_OK) {
                s_state.motor_edges[j]+=chassis_counter_delta(raw_motor,s_motor_previous[j]);
                s_motor_previous[j]=raw_motor;
            }
            motor_write(j,s_state.duty[j]*s_state.config.motor_sign[j]);
        }
        if(s_state.fault!=FAULT_NONE) s_state.odometry_valid=false;
        for(int j=0;j<4;j++) s_state.hardware_duty[j]=ledc_get_duty(LEDC_LOW_SPEED_MODE,(ledc_channel_t)j);
        portENTER_CRITICAL(&s_lock); s_snapshot=s_state; portEXIT_CRITICAL(&s_lock);
    }
}

bool chassis_submit(const chassis_command_t *command)
{
    if(!s_commands || !command) return false;
    if(xQueueSend(s_commands,command,0)!=pdTRUE) {chassis_emergency_stop(); return false;}
    return true;
}

void chassis_emergency_stop(void)
{
    portENTER_CRITICAL(&s_lock); s_stop_requested=true; portEXIT_CRITICAL(&s_lock);
}

void chassis_snapshot(chassis_core_t *snapshot)
{
    portENTER_CRITICAL(&s_lock); *snapshot=s_snapshot; portEXIT_CRITICAL(&s_lock);
}

void chassis_print_io(void)
{
    printf("BOOT_ENCODER_OUTPUT_ENABLE=0x%08"PRIx32"\n",s_boot_output_enable & 0x000f0000);
    gpio_dump_io_configuration(stdout,(1ULL<<5)|(1ULL<<16)|(1ULL<<17)|(1ULL<<18)|(1ULL<<19));
}

esp_err_t chassis_hw_start(const chassis_config_t *config)
{
    chassis_core_init(&s_state,config);
    s_state.encoder_valid[0]=config->feedback_commissioned[0];
    s_state.encoder_valid[1]=config->feedback_commissioned[1];s_snapshot=s_state;
    s_boot_output_enable=REG_READ(GPIO_ENABLE_REG);
    for(int i=0;i<4;i++) {gpio_reset_pin(MOTOR_PINS[i]); gpio_set_direction(MOTOR_PINS[i],GPIO_MODE_OUTPUT); gpio_set_level(MOTOR_PINS[i],0);}
    gpio_hold_dis(GPIO_NUM_5);
    ESP_ERROR_CHECK(gpio_reset_pin(GPIO_NUM_5));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_5,GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(GPIO_NUM_5,GPIO_PULLUP_ONLY));
    const ledc_timer_config_t pwm_timer={.speed_mode=LEDC_LOW_SPEED_MODE,.duty_resolution=LEDC_TIMER_11_BIT,.timer_num=LEDC_TIMER_0,.freq_hz=20000,.clk_cfg=LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));
    for(int i=0;i<4;i++) {
        const ledc_channel_config_t c={.gpio_num=MOTOR_PINS[i],.speed_mode=LEDC_LOW_SPEED_MODE,.channel=i,.timer_sel=LEDC_TIMER_0,.duty=0,.hpoint=0};
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }
    for(int j=0;j<2;j++) {
        for(int k=0;k<2;k++) {
            gpio_hold_dis(ENCODER_PINS[j][k]);
            ESP_ERROR_CHECK(gpio_reset_pin(ENCODER_PINS[j][k]));
            ESP_ERROR_CHECK(gpio_set_direction(ENCODER_PINS[j][k],GPIO_MODE_INPUT));
            ESP_ERROR_CHECK(gpio_set_pull_mode(ENCODER_PINS[j][k],GPIO_PULLUP_ONLY));
        }
        const pcnt_unit_config_t unit={.high_limit=CHASSIS_PCNT_LIMIT,.low_limit=-CHASSIS_PCNT_LIMIT,.flags.accum_count=false};
        ESP_ERROR_CHECK(pcnt_new_unit(&unit,&s_pcnt[j]));
        const pcnt_glitch_filter_config_t filter={.max_glitch_ns=1000};
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_pcnt[j],&filter));
        bool single=config->encoder_mode[j]==ENCODER_SINGLE_A;
        for(int k=0;k<(single?1:2);k++) {
            const pcnt_chan_config_t c={.edge_gpio_num=ENCODER_PINS[j][k],.level_gpio_num=single?-1:ENCODER_PINS[j][1-k]};
            pcnt_channel_handle_t channel;
            ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt[j],&c,&channel));
            ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel,single?PCNT_CHANNEL_EDGE_ACTION_INCREASE:k==0?PCNT_CHANNEL_EDGE_ACTION_DECREASE:PCNT_CHANNEL_EDGE_ACTION_INCREASE,single?PCNT_CHANNEL_EDGE_ACTION_INCREASE:k==0?PCNT_CHANNEL_EDGE_ACTION_INCREASE:PCNT_CHANNEL_EDGE_ACTION_DECREASE));
            ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel,PCNT_CHANNEL_LEVEL_ACTION_KEEP,single?PCNT_CHANNEL_LEVEL_ACTION_KEEP:PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
            gpio_pullup_en(ENCODER_PINS[j][k]);
        }
        // No GPIO ISR, no PCNT watch callbacks, no interrupt-based accum_count.
        ESP_ERROR_CHECK(pcnt_unit_enable(s_pcnt[j]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt[j]));
        ESP_ERROR_CHECK(pcnt_unit_start(s_pcnt[j]));
    }
    for(int j=0;j<4;j++) {
        const pcnt_unit_config_t unit={.high_limit=CHASSIS_PCNT_LIMIT,.low_limit=-CHASSIS_PCNT_LIMIT,.flags.accum_count=false};
        ESP_ERROR_CHECK(pcnt_new_unit(&unit,&s_phase_pcnt[j]));
        const pcnt_glitch_filter_config_t filter={.max_glitch_ns=1000};
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_phase_pcnt[j],&filter));
        const pcnt_chan_config_t c={.edge_gpio_num=ENCODER_PINS[j/2][j%2],.level_gpio_num=-1};
        pcnt_channel_handle_t channel;
        ESP_ERROR_CHECK(pcnt_new_channel(s_phase_pcnt[j],&c,&channel));
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel,PCNT_CHANNEL_EDGE_ACTION_INCREASE,PCNT_CHANNEL_EDGE_ACTION_INCREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel,PCNT_CHANNEL_LEVEL_ACTION_KEEP,PCNT_CHANNEL_LEVEL_ACTION_KEEP));
        ESP_ERROR_CHECK(pcnt_unit_enable(s_phase_pcnt[j]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(s_phase_pcnt[j]));
        ESP_ERROR_CHECK(pcnt_unit_start(s_phase_pcnt[j]));
    }
    for(int j=0;j<2;j++) {
        const pcnt_unit_config_t unit={.high_limit=CHASSIS_PCNT_LIMIT,.low_limit=-CHASSIS_PCNT_LIMIT,.flags.accum_count=false};
        ESP_ERROR_CHECK(pcnt_new_unit(&unit,&s_motor_pcnt[j]));
        const pcnt_glitch_filter_config_t filter={.max_glitch_ns=1000};
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_motor_pcnt[j],&filter));
        const pcnt_chan_config_t c={.edge_gpio_num=AUX_INPUT_PINS[j],.level_gpio_num=-1};
        pcnt_channel_handle_t channel;
        ESP_ERROR_CHECK(pcnt_new_channel(s_motor_pcnt[j],&c,&channel));
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel,PCNT_CHANNEL_EDGE_ACTION_INCREASE,PCNT_CHANNEL_EDGE_ACTION_INCREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel,PCNT_CHANNEL_LEVEL_ACTION_KEEP,PCNT_CHANNEL_LEVEL_ACTION_KEEP));
        ESP_ERROR_CHECK(pcnt_unit_enable(s_motor_pcnt[j]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(s_motor_pcnt[j]));
        ESP_ERROR_CHECK(pcnt_unit_start(s_motor_pcnt[j]));
    }
    s_commands=xQueueCreate(16,sizeof(chassis_command_t));
    if(!s_commands) return ESP_ERR_NO_MEM;
    if(xTaskCreatePinnedToCore(control_task,"chassis_control",8192,NULL,20,&s_control_task,1)!=pdPASS) return ESP_ERR_NO_MEM;
    const gptimer_config_t timer_config={.clk_src=GPTIMER_CLK_SRC_DEFAULT,.direction=GPTIMER_COUNT_UP,.resolution_hz=1000000};
    gptimer_handle_t timer;
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config,&timer));
    const gptimer_event_callbacks_t callbacks={.on_alarm=timer_alarm};
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer,&callbacks,NULL));
    const gptimer_alarm_config_t alarm={.alarm_count=10000,.reload_count=0,.flags.auto_reload_on_alarm=true};
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer,&alarm));
    ESP_ERROR_CHECK(gptimer_enable(timer));
    return gptimer_start(timer);
}
