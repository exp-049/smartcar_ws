#include "chassis_core.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const double PI = 3.14159265358979323846;
static double clamp(double x, double limit) { return fmax(-limit, fmin(limit, x)); }

chassis_config_t chassis_default_config(void)
{
    chassis_config_t c = {
        .radius={0.024,0.024}, .separation=0.133, .cpr={1000,2000},
        .encoder_sign={1,-1}, .motor_sign={1,-1},
        .encoder_mode={ENCODER_SINGLE_A,ENCODER_QUADRATURE},
        .max_duty=0.65, .max_speed=0.35, .acceleration=0.5,
        .gains={{0,0,0,0,0},{0,0,0,0,0}}
    };
    return c;
}

bool chassis_config_valid(const chassis_config_t *c)
{
    if (!c || !isfinite(c->separation) || c->separation<0.03 || c->separation>1 ||
        !isfinite(c->max_duty) || c->max_duty<=0 || c->max_duty>0.85 ||
        !isfinite(c->max_speed) || c->max_speed<=0 || c->max_speed>2 ||
        !isfinite(c->acceleration) || c->acceleration<=0 || c->acceleration>3) return false;
    for(int j=0;j<2;j++) {
        if(!isfinite(c->radius[j]) || c->radius[j]<0.005 || c->radius[j]>0.3 ||
           !isfinite(c->cpr[j]) || c->cpr[j]<4 || c->cpr[j]>100000 ||
           abs(c->encoder_sign[j])!=1 || abs(c->motor_sign[j])!=1 ||
           (c->encoder_mode[j]!=ENCODER_QUADRATURE && c->encoder_mode[j]!=ENCODER_SINGLE_A)) return false;
        const wheel_gains_t *g=&c->gains[j];
        if(!isfinite(g->kp)||!isfinite(g->ki)||!isfinite(g->kd)||!isfinite(g->ks)||!isfinite(g->kv)||
           g->kp<0||g->kp>100||g->ki<0||g->ki>500||g->kd<0||g->kd>5||
           g->ks<0||g->ks>0.65||g->kv<0||g->kv>20) return false;
        for(int k=0;k<2;k++) {
            const wheel_model_t *m=&c->model[j][k];
            if(!isfinite(m->ks)||!isfinite(m->kv)||!isfinite(m->start_duty)||
               m->ks<0||m->ks>c->max_duty||m->kv<0||m->kv>20||
               m->start_duty<0||m->start_duty>c->max_duty) return false;
        }
    }
    return true;
}

void chassis_core_init(chassis_core_t *s, const chassis_config_t *config)
{
    memset(s,0,sizeof(*s));
    s->config=chassis_config_valid(config)?*config:chassis_default_config();
    s->mode=MODE_ENCODER;
    // The pure mathematical core assumes validated input. The hardware
    // adapter overrides these until both physical phases have been verified.
    s->encoder_valid[0]=s->encoder_valid[1]=true;
}

void chassis_stop(chassis_core_t *s, bool disarm)
{
    for(int j=0;j<2;j++) {
        s->requested[j]=s->target[j]=s->ramped[j]=s->duty[j]=0;
        s->stalled_s[j]=0;
        s->wrong_direction_s[j]=0;
        s->reversing[j]=false;
        s->kick_active[j]=false;s->kick_elapsed[j]=s->restart_delay[j]=s->motion_wait_s[j]=0;
        memset(&s->pid[j],0,sizeof(s->pid[j]));
        s->pid[j].previous_speed=s->speed[j];
    }
    if(disarm) s->armed=false;
}

void chassis_set_mode(chassis_core_t *s, chassis_mode_t mode)
{
    chassis_stop(s,true);
    s->mode=(mode>=MODE_ENCODER && mode<=MODE_ROS)?mode:MODE_ENCODER;
}

bool chassis_arm(chassis_core_t *s)
{
    if(s->fault!=FAULT_NONE || s->mode==MODE_ENCODER || (s->mode==MODE_ROS && !s->link_ready)) return false;
    if((s->mode==MODE_PID || s->mode==MODE_ROS) && (!s->encoder_valid[0] || !s->encoder_valid[1])) return false;
    if(s->mode==MODE_PID) for(int j=0;j<2;j++) if(s->config.gains[j].kp==0 && s->config.gains[j].ki==0) return false;
    chassis_stop(s,false);
    s->armed=true;
    s->command_us=s->sample_us;
    return true;
}

void chassis_set_link(chassis_core_t *s, bool ready)
{
    if(s->link_ready!=ready && s->mode==MODE_ROS) chassis_stop(s,true);
    s->link_ready=ready;
}

bool chassis_command_wheels(chassis_core_t *s, double left, double right, int64_t now)
{
    if(!isfinite(left)||!isfinite(right)||now<0) {
        ++s->invalid_commands; chassis_stop(s,true); return false;
    }
    if(!s->armed || (s->mode!=MODE_PWM && s->mode!=MODE_PID)) return false;
    double limit=s->mode==MODE_PWM?s->config.max_duty:s->config.max_speed;
    double scale=fmax(1.0,fmax(fabs(left),fabs(right))/limit);
    s->requested[0]=left/scale; s->requested[1]=right/scale;
    s->command_us=now;
    return true;
}

bool chassis_command_body(chassis_core_t *s, double v, double w, int64_t now)
{
    if(s->mode!=MODE_PID) return false;
    if(!isfinite(v)||!isfinite(w)||fabs(v)>2||fabs(w)>10||now<0) {
        ++s->invalid_commands;chassis_stop(s,true);return false;
    }
    return chassis_command_wheels(s,v-w*s->config.separation/2,v+w*s->config.separation/2,now);
}

bool chassis_command_twist(chassis_core_t *s, double v, double w, int64_t now)
{
    if(s->mode!=MODE_ROS) return false;
    if(!isfinite(v)||!isfinite(w)||fabs(v)>2||fabs(w)>10||now<0) {
        ++s->invalid_commands; chassis_stop(s,true); return false;
    }
    if(!s->link_ready || s->fault!=FAULT_NONE) return false;
    if(v==0 && w==0) {
        chassis_stop(s,false);
        if(!s->encoder_valid[0] || !s->encoder_valid[1]) return false;
        s->armed=true; s->command_us=now; return true;
    }
    if(!s->armed) return false;
    double l=v-w*s->config.separation/2, r=v+w*s->config.separation/2;
    double scale=fmax(1.0,fmax(fabs(l),fabs(r))/s->config.max_speed);
    s->requested[0]=l/scale; s->requested[1]=r/scale; s->command_us=now;
    return true;
}

int32_t chassis_counter_delta(int current, int previous)
{
    int32_t d=(current-previous)%CHASSIS_PCNT_LIMIT;
    if(d>CHASSIS_PCNT_LIMIT/2) d-=CHASSIS_PCNT_LIMIT;
    if(d<-CHASSIS_PCNT_LIMIT/2) d+=CHASSIS_PCNT_LIMIT;
    return d;
}

static double pid_update(wheel_pid_t *p, const wheel_gains_t *g, const wheel_model_t *model, double target, double speed, double dt, double limit, bool single_phase, bool freeze_integral)
{
    double error=target-speed;
    double derivative=(speed-p->previous_speed)/dt;
    p->derivative += dt/(0.03+dt)*(derivative-p->derivative);
    p->previous_speed=speed;
    p->p=g->kp*error;
    p->d=-g->kd*p->derivative;
    double ks=model->ks>0?model->ks:g->ks;
    double kv=model->ks>0?model->kv:g->kv;
    p->ff=target==0?0:copysign(ks,target)+kv*target;
    double proposed=clamp(p->integral+g->ki*error*dt,limit);
    double unconstrained=p->ff+p->p+proposed+p->d;
    // With one phase we cannot observe direction under reverse torque. Use
    // coast deceleration rather than commanding torque opposite the target.
    double lower=single_phase && target>=0?0:-limit;
    double upper=single_phase && target<=0?0:limit;
    if(!freeze_integral && ((unconstrained>=lower && unconstrained<=upper) || (unconstrained>upper && error<0) || (unconstrained<lower && error>0))) p->integral=proposed;
    p->i=p->integral;
    p->output=fmin(upper,fmax(lower,p->ff+p->p+p->i+p->d));
    return p->output;
}

void chassis_core_tick(chassis_core_t *s, int64_t now, int raw_l, int raw_r)
{
    int raw[2]={raw_l,raw_r};
    if(!s->initialized) {
        s->initialized=true; s->sample_us=now;
        s->raw_previous[0]=raw_l; s->raw_previous[1]=raw_r;
        return;
    }
    int64_t elapsed=now-s->sample_us;
    s->sample_us=now; ++s->seq;
    if(elapsed<=0 || elapsed>CHASSIS_MAX_INTERVAL_US) {
        s->odometry_valid=false;
        ++s->overruns; s->fault=FAULT_TIMING; chassis_stop(s,true);
        for(int j=0;j<2;j++) s->raw_previous[j]=raw[j];
        return;
    }
    s->dt=elapsed/1e6;
    double ds[2];
    int32_t deltas[2];
    for(int j=0;j<2;j++) {
        deltas[j]=chassis_counter_delta(raw[j],s->raw_previous[j]);
        s->raw_previous[j]=raw[j];
    }
    if(fabs((double)deltas[0])>CHASSIS_MAX_COUNT_RATE*s->dt+8 || fabs((double)deltas[1])>CHASSIS_MAX_COUNT_RATE*s->dt+8) {
        s->odometry_valid=false;
        s->fault=FAULT_COUNTS; chassis_stop(s,true); return;
    }
    for(int j=0;j<2;j++) {
        int32_t delta=deltas[j];
        s->still_s[j]=delta==0?s->still_s[j]+s->dt:0;
        if(s->config.encoder_mode[j]==ENCODER_SINGLE_A) {
            if(delta<0) {s->fault=FAULT_COUNTS;s->odometry_valid=false;chassis_stop(s,true);return;}
            if(delta && s->inferred_direction[j]==0) {
                s->unresolved_edges[j]+=delta;
                s->odometry_uncertain=true;
            }
            delta*=s->inferred_direction[j];
        } else delta*=s->config.encoder_sign[j];
        s->delta[j]=delta; s->counts[j]+=delta;
        ds[j]=delta*2*PI*s->config.radius[j]/s->config.cpr[j];
        s->position[j]+=ds[j]; s->raw_speed[j]=ds[j]/s->dt;
        s->speed[j]+=s->dt/(0.04+s->dt)*(s->raw_speed[j]-s->speed[j]);
    }
    double distance=(ds[0]+ds[1])/2;
    double angle=(ds[1]-ds[0])/s->config.separation;
    double half=angle/2;
    double sinc=fabs(half)<1e-8?1:sin(half)/half;
    s->odometry_valid=s->encoder_valid[0] && s->encoder_valid[1] && s->fault==FAULT_NONE && !s->odometry_uncertain;
    if(s->odometry_valid) {
        s->x+=distance*sinc*cos(s->yaw+half);
        s->y+=distance*sinc*sin(s->yaw+half);
        s->yaw=remainder(s->yaw+angle,2*PI);
        s->linear=(s->raw_speed[0]+s->raw_speed[1])/2;
        s->angular=(s->raw_speed[1]-s->raw_speed[0])/s->config.separation;
    } else s->linear=s->angular=0;
    if(s->armed && (now<s->command_us || now-s->command_us>CHASSIS_COMMAND_TIMEOUT_US)) {
        ++s->timeouts; chassis_stop(s,true);
    }
    if(!s->armed || s->fault!=FAULT_NONE || s->mode==MODE_ENCODER || (s->mode==MODE_ROS && !s->link_ready)) {
        chassis_stop(s,true); return;
    }
    for(int j=0;j<2;j++) {
        bool single=s->config.encoder_mode[j]==ENCODER_SINGLE_A;
        double requested=s->requested[j];
        int requested_direction=(requested>1e-8)-(requested<-1e-8);
        s->reversing[j]=false;
        if(single && requested_direction && requested_direction!=s->inferred_direction[j]) {
            // Finish deceleration with the old sign. A zero command retains
            // the old sign so the full coast distance remains accounted for.
            bool settled=s->still_s[j]>=.10 && fabs(s->speed[j])<.01 && fabs(s->duty[j])<1e-8 && fabs(s->ramped[j])<1e-6;
            if(settled) {
                s->inferred_direction[j]=requested_direction;
                memset(&s->pid[j],0,sizeof(s->pid[j]));
                s->pid[j].previous_speed=s->speed[j];
            } else {requested=0;s->reversing[j]=true;}
        }
        if(s->mode==MODE_PWM) {
            s->target[j]=requested; s->duty[j]=clamp(requested,s->config.max_duty);
        } else {
            // Cross zero before reversing, rather than jumping to reverse torque.
            if(requested*s->ramped[j]<0) requested=0;
            s->ramped[j]+=clamp(requested-s->ramped[j],s->config.acceleration*s->dt);
            if(fabs(s->ramped[j])<1e-6) s->ramped[j]=0;
            s->target[j]=s->ramped[j];
            const wheel_model_t *model=&s->config.model[j][s->target[j]<0?1:0];
            s->restart_delay[j]=fmax(0,s->restart_delay[j]-s->dt);
            if(s->kick_active[j]) {
                s->kick_elapsed[j]+=s->dt;
                if(abs(s->delta[j])>=4 || s->kick_elapsed[j]>=.08 || s->target[j]==0) {
                    s->kick_active[j]=false;s->restart_delay[j]=.3;
                }
            } else if(model->start_duty>0 && fabs(s->target[j])>.005 &&
                      fabs(s->speed[j])<.01 && s->still_s[j]>=.10 &&
                      s->restart_delay[j]==0 && !s->reversing[j]) {
                s->kick_active[j]=true;s->kick_elapsed[j]=0;
            }
            s->duty[j]=pid_update(&s->pid[j],&s->config.gains[j],model,s->target[j],s->speed[j],s->dt,s->config.max_duty,single,s->kick_active[j]);
            if(s->kick_active[j]) s->duty[j]=copysign(fmax(fabs(s->duty[j]),model->start_duty),s->target[j]);
            if(s->target[j]==0 && requested==0) { s->duty[j]=0; s->kick_active[j]=false;memset(&s->pid[j],0,sizeof(s->pid[j])); }
            if(fabs(s->target[j])>.03 && s->delta[j]==0) s->motion_wait_s[j]+=s->dt;
            else s->motion_wait_s[j]=0;
            if(s->motion_wait_s[j]>1.0) {s->fault=j==0?FAULT_STALL_L:FAULT_STALL_R;chassis_stop(s,true);return;}
        }
        // Bench measurements put the start deadzone above 55%; don't label
        // a deliberately sub-start open-loop point as a mechanical stall.
        if(fabs(s->duty[j])>=0.60 && s->delta[j]==0) s->stalled_s[j]+=s->dt;
        else s->stalled_s[j]=0;
        if(s->stalled_s[j]>0.5) { s->fault=j==0?FAULT_STALL_L:FAULT_STALL_R; chassis_stop(s,true); return; }
        if(!single && s->mode!=MODE_PWM && s->target[j]*s->speed[j]<0 && fabs(s->speed[j])>0.03 && fabs(s->target[j])>0.03) s->wrong_direction_s[j]+=s->dt;
        else s->wrong_direction_s[j]=0;
        if(s->wrong_direction_s[j]>0.2) {s->fault=j==0?FAULT_DIRECTION_L:FAULT_DIRECTION_R;chassis_stop(s,true);return;}
    }
}
