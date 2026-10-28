#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "chassis_hw.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

// Detailed hardware diagnostics use 10 Hz to fit the 115200 baud UART.
// The normal wheel/PID/odometry stream remains 20 Hz.
static volatile bool s_diagnostics;

static void report_task(void *argument)
{
    (void)argument;
    TickType_t last_wake=xTaskGetTickCount();
    while(true) {
        bool diagnostics=s_diagnostics;
        chassis_core_t s; chassis_snapshot(&s);
        if(!s.initialized || s.dt<=0) {vTaskDelayUntil(&last_wake,pdMS_TO_TICKS(50));continue;}
        char extra[480]={0};
        char frame[1024];
        if(diagnostics) snprintf(extra,sizeof(extra),",\"dir\":[%d,%d],\"rev\":[%d,%d],\"unknown\":[%"PRId64",%"PRId64"],\"s\":[%.4f,%.4f],\"edges\":[%"PRId64",%"PRId64",%"PRId64",%"PRId64"],\"levels\":[%d,%d,%d,%d],\"raw\":[%"PRId64",%"PRId64"],\"aux_pins\":[15,5],\"aux_edges\":[%"PRId64",%"PRId64"],\"pwm_hw\":[%u,%u,%u,%u]",
            s.inferred_direction[0],s.inferred_direction[1],s.reversing[0],s.reversing[1],s.unresolved_edges[0],s.unresolved_edges[1],s.position[0],s.position[1],s.phase_edges[0],s.phase_edges[1],s.phase_edges[2],s.phase_edges[3],s.phase_levels[0],s.phase_levels[1],s.phase_levels[2],s.phase_levels[3],s.hardware_count[0],s.hardware_count[1],s.motor_edges[0],s.motor_edges[1],(unsigned)s.hardware_duty[0],(unsigned)s.hardware_duty[1],(unsigned)s.hardware_duty[2],(unsigned)s.hardware_duty[3]);
        snprintf(frame,sizeof(frame),"{\"type\":\"chassis\",\"seq\":%"PRIu32",\"t_us\":%"PRId64",\"dt\":%.5f,\"n\":[%"PRId64",%"PRId64"],\"d\":[%"PRId64",%"PRId64"],\"v\":[%.4f,%.4f],\"vr\":[%.4f,%.4f],\"target\":[%.4f,%.4f],\"u\":[%.4f,%.4f],\"pid\":[[%.4f,%.4f,%.4f,%.4f],[%.4f,%.4f,%.4f,%.4f]],\"pose\":[%.4f,%.4f,%.4f],\"twist\":[%.4f,%.4f],\"start\":[%d,%d],\"enc_mode\":[%d,%d],\"enc_ok\":[%d,%d],\"odom_valid\":%d,\"mode\":%d,\"armed\":%d,\"fault\":%d,\"timeouts\":%"PRIu64",\"overruns\":%"PRIu64"%s",
            s.seq,s.sample_us,s.dt,s.counts[0],s.counts[1],s.delta[0],s.delta[1],s.speed[0],s.speed[1],s.raw_speed[0],s.raw_speed[1],s.target[0],s.target[1],s.duty[0],s.duty[1],s.pid[0].p,s.pid[0].i,s.pid[0].d,s.pid[0].ff,s.pid[1].p,s.pid[1].i,s.pid[1].d,s.pid[1].ff,s.x,s.y,s.yaw,s.linear,s.angular,s.kick_active[0],s.kick_active[1],s.config.encoder_mode[0],s.config.encoder_mode[1],s.encoder_valid[0],s.encoder_valid[1],s.odometry_valid,s.mode,s.armed,s.fault,s.timeouts,s.overruns,extra);
        uint32_t crc=esp_rom_crc32_le(0,(const uint8_t *)frame,strlen(frame));
        printf("%s,\"crc\":%"PRIu32"}\n",frame,crc);
        vTaskDelayUntil(&last_wake,pdMS_TO_TICKS(diagnostics?100:50));
    }
}

static void save_config(const chassis_core_t *state)

{
    chassis_config_t config=state->config;
    for(int j=0;j<2;j++) config.feedback_commissioned[j]=state->encoder_valid[j];
    nvs_handle_t nvs;
    esp_err_t result=nvs_open("chassis_v1",NVS_READWRITE,&nvs);
    if(result==ESP_OK) {
        result=nvs_set_blob(nvs,"config_v3",&config,sizeof(config));
        if(result==ESP_OK) result=nvs_commit(nvs);
        nvs_close(nvs);
    }
    printf("CONFIG_SAVE %s\n",esp_err_to_name(result));
}

// Preserve partial UART input across EAGAIN. Parse only a complete line.
static void read_console_line(char *line, size_t capacity)
{
    size_t used=0;bool overflow=false;
    while(true) {
        int c=getchar();
        if(c==EOF) {clearerr(stdin);vTaskDelay(pdMS_TO_TICKS(1));continue;}
        if(c=='\r') continue;
        if(c=='\n') {
            if(overflow) {printf("REJECTED line-too-long\n");used=0;overflow=false;continue;}
            line[used]=0;return;
        }
        if(used+1<capacity) line[used++]=(char)c;else overflow=true;
    }
}

static void console_task(void *argument)
{
    (void)argument;
    char line[256];
    while(true) {
        read_console_line(line,sizeof(line));
        char *payload=line;bool framed=false;
        if(line[0]=='@') {
            char *end=NULL;uint32_t expected=(uint32_t)strtoul(line+1,&end,16);
            if(end!=line+9 || line[9]!=' ') {printf("REJECTED crc-header\n");continue;}
            payload=line+10;
            uint32_t actual=esp_rom_crc32_le(0,(const uint8_t *)payload,strlen(payload));
            if(expected!=actual) {printf("REJECTED crc\n");continue;}
            framed=true;
        }
        char name[24]={0}; double a=0,b=0,c=0,d=0,e=0,f=0,g=0;
        int fields=sscanf(payload,"%23s %lf %lf %lf %lf %lf %lf %lf",name,&a,&b,&c,&d,&e,&f,&g);
        if(fields<1) continue;
        if((!strcmp(name,"wheel")||!strcmp(name,"drive")) && !framed) {printf("REJECTED motion-needs-crc\n");continue;}
        chassis_core_t s; chassis_snapshot(&s);
        chassis_command_t command={.time_us=esp_timer_get_time()};
        bool known=true;
        if(!strcmp(name,"stop")) {chassis_emergency_stop(); printf("STOP\n"); continue;}
        if(!strcmp(name,"arm")) command.type=CMD_ARM;
        else if(!strcmp(name,"clear")) command.type=CMD_CLEAR;
        else if(!strcmp(name,"zero")) command.type=CMD_ZERO;
        else if(!strcmp(name,"io")) {chassis_print_io();continue;}
        else if(!strcmp(name,"diag") && fields==2 && (a==0||a==1)) {s_diagnostics=a!=0;printf("DIAG %d\n",s_diagnostics);continue;}
        else if(!strcmp(name,"filter") && fields==2 && !s.armed && s.mode==MODE_ENCODER && isfinite(a) && a>=0 && a<=2000 && floor(a)==a) {command.type=CMD_FILTER;command.value[0]=a;}
        else if(!strcmp(name,"mode") && fields==2 && a>=0 && a<=2 && floor(a)==a) {command.type=CMD_MODE;command.value[0]=a;}
        else if(!strcmp(name,"drive") && fields==3) {command.type=CMD_BODY;command.value[0]=a;command.value[1]=b;}
        else if(!strcmp(name,"wheel") && fields==3) {command.type=CMD_WHEEL;command.value[0]=a;command.value[1]=b;}
        else if(!strcmp(name,"gains") && fields==7 && !s.armed && (a==0||a==1)) {
            command.type=CMD_CONFIG;command.config=s.config;
            command.config.gains[(int)a]=(wheel_gains_t){b,c,d,e,f};
            if(!chassis_config_valid(&command.config)) known=false;
        } else if(!strcmp(name,"model") && fields==6 && !s.armed && (a==0||a==1) && fabs(b)==1) {
            command.type=CMD_CONFIG;command.config=s.config;
            command.config.model[(int)a][b<0]=(wheel_model_t){c,d,e};
            if(!chassis_config_valid(&command.config)) known=false;
        } else if(!strcmp(name,"limits") && fields==4 && !s.armed) {
            command.type=CMD_CONFIG;command.config=s.config;
            command.config.max_duty=a;command.config.max_speed=b;command.config.acceleration=c;
            if(!chassis_config_valid(&command.config)) known=false;
        } else if(!strcmp(name,"polarity") && fields==5 && !s.armed && fabs(a)==1 && fabs(b)==1 && fabs(c)==1 && fabs(d)==1) {
            command.type=CMD_CONFIG;command.config=s.config;
            command.config.motor_sign[0]=a;command.config.motor_sign[1]=b;
            command.config.encoder_sign[0]=c;command.config.encoder_sign[1]=d;
        } else if(!strcmp(name,"geometry") && fields==6 && !s.armed) {
            command.type=CMD_CONFIG;command.config=s.config;
            command.config.radius[0]=a;command.config.radius[1]=b;command.config.separation=c;
            command.config.cpr[0]=d;command.config.cpr[1]=e;
            if(!chassis_config_valid(&command.config)) known=false;
        } else if(!strcmp(name,"save") && !s.armed && s.mode==MODE_ENCODER) {save_config(&s);continue;}
        else if(!strcmp(name,"config")) {
            printf("CONFIG radius=%.8f,%.8f separation=%.8f cpr=%.1f,%.1f motor=%d,%d encoder=%d,%d duty_max=%.3f feedback=%d,%d\n",s.config.radius[0],s.config.radius[1],s.config.separation,s.config.cpr[0],s.config.cpr[1],s.config.motor_sign[0],s.config.motor_sign[1],s.config.encoder_sign[0],s.config.encoder_sign[1],s.config.max_duty,s.config.encoder_mode[0],s.config.encoder_mode[1]);
            for(int j=0;j<2;j++) printf("GAINS %d %.6f %.6f %.6f %.6f %.6f\n",j,s.config.gains[j].kp,s.config.gains[j].ki,s.config.gains[j].kd,s.config.gains[j].ks,s.config.gains[j].kv);
            for(int j=0;j<2;j++) for(int k=0;k<2;k++) printf("MODEL %d %d %.7f %.7f %.7f\n",j,k==0?1:-1,s.config.model[j][k].ks,s.config.model[j][k].kv,s.config.model[j][k].start_duty);
            continue;
        } else if(!strcmp(name,"help")) {printf("mode 0|1|2; arm; wheel L R (CRC); model wheel sign ks kv start; limits duty speed accel; drive v w; stop; clear; zero; config; gains wheel kp ki kd ks kv; polarity ml mr el er; geometry rl rr width cprl cprr; save (mode0 only)\n");continue;}
        else known=false;
        if(known && chassis_submit(&command)) {if(command.type!=CMD_WHEEL && command.type!=CMD_BODY) printf("QUEUED %s\n",name);}
        else printf("REJECTED %s\n",name);
    }
}

void app_main(void)
{
    esp_err_t error=nvs_flash_init();
    if(error==ESP_ERR_NVS_NO_FREE_PAGES||error==ESP_ERR_NVS_NEW_VERSION_FOUND) {ESP_ERROR_CHECK(nvs_flash_erase());error=nvs_flash_init();}
    ESP_ERROR_CHECK(error);
    chassis_config_t config=chassis_default_config();
    nvs_handle_t nvs;
    if(nvs_open("chassis_v1",NVS_READONLY,&nvs)==ESP_OK) {
        chassis_config_t stored;size_t size=sizeof(stored);
        if(nvs_get_blob(nvs,"config_v3",&stored,&size)==ESP_OK && size==sizeof(stored) && chassis_config_valid(&stored)) config=stored;
        nvs_close(nvs);
    }
    ESP_ERROR_CHECK(chassis_hw_start(&config));
    setvbuf(stdin,NULL,_IONBF,0);
    printf("V1 C firmware: LEFT single A both edges + inferred drive direction, RIGHT PCNT x4; NO encoder GPIO ISR/PCNT accumulation ISR; GPTimer=100Hz; PWM=20kHz; motors DISARMED; CPR provisional %.0f/%.0f\n",config.cpr[0],config.cpr[1]);
    xTaskCreate(console_task,"console",8192,NULL,4,NULL);
    xTaskCreate(report_task,"report",8192,NULL,3,NULL);
}
