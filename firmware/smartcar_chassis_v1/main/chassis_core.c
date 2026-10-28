#include "chassis_core.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const double PI = 3.14159265358979323846;
static double clamp(double x, double limit) { return fmax(-limit, fmin(limit, x)); }
static double relu(double x) { return fmax(0.0, x); }

static double normalized_pwm_duty(double command)
{
    const double start_duty=0.58;
    const double full_duty=1.0;
    const double forward=relu(command);
    const double reverse=relu(-command);
    const double magnitude=fmin(1.0,forward+reverse);
    if(magnitude<=1e-6) return 0.0;
    const double duty=start_duty+(full_duty-start_duty)*magnitude;
    return (forward>0.0?1.0:-1.0)*duty;
}

chassis_config_t chassis_default_config(void)
{
    chassis_config_t c = {
        .radius={0.024,0.024}, .separation=0.133, .cpr={1000,2000},
        .encoder_sign={1,-1}, .motor_sign={1,-1},
        .encoder_mode={ENCODER_SINGLE_A,ENCODER_QUADRATURE},
        .single_a_direction=SINGLE_A_DIR_FROM_TARGET,
        .max_duty=1.0, .max_speed=0.35, .acceleration=2.0,
        // The host owns the single speed_per_pwm gain. The chassis receives
        // normalized wheel commands and maps them open loop with signed ReLU.
        .gains={{0,0,0,0,0},{0,0,0,0,0}}
    };
    return c;
}

bool chassis_config_valid(const chassis_config_t *c)
{
    if (!c || !isfinite(c->separation) || c->separation<0.03 || c->separation>1 ||
        (c->single_a_direction!=SINGLE_A_DIR_FROM_TARGET && c->single_a_direction!=SINGLE_A_DIR_LATCHED) ||
        !isfinite(c->max_duty) || c->max_duty<=0 || c->max_duty>1.0 ||
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
    // Assume forward on the single-phase wheel so a sync message sent while the
    // car already coasts backward is not integrated with the wrong sign.
    for(int j=0;j<2;j++) if(s->config.encoder_mode[j]==ENCODER_SINGLE_A) s->inferred_direction[j]=1;
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
    // Only the link is required: no encoder, gain or fault precondition.
    if(s->mode==MODE_ENCODER) return false;
    chassis_stop(s,false);
    s->armed=true;
    s->command_us=s->sample_us;
    return true;
}

void chassis_set_link(chassis_core_t *s, bool ready)
{
    // Losing the host link does not stop the wheels; the host's 5 s lease is the
    // single mechanism that brings the robot to rest.
    s->link_ready=ready;
}

bool chassis_command_wheels(chassis_core_t *s, double left, double right, int64_t now)
{
    if(!isfinite(left)||!isfinite(right)||now<0) {
        ++s->invalid_commands; chassis_stop(s,true); return false;
    }
    if(s->mode!=MODE_PWM && s->mode!=MODE_PID) return false;
    double limit=s->mode==MODE_PWM?1.0:s->config.max_speed;
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

bool chassis_command_normalized_wheels(chassis_core_t *s, double left, double right, int64_t now)
{
    if(s->mode!=MODE_ROS) return false;
    if(!isfinite(left)||!isfinite(right)||now<0) {
        ++s->invalid_commands; chassis_stop(s,true); return false;
    }
    if(left==0 && right==0) {
        chassis_stop(s,false);
        s->command_us=now; return true;
    }
    double scale=fmax(1.0,fmax(fabs(left),fabs(right)));
    s->requested[0]=left/scale; s->requested[1]=right/scale; s->command_us=now;
    return true;
}

int64_t chassis_accum_counter_delta(int current, int previous)
{
    // pcnt_unit_get_count() exposes the driver's accumulated count as a signed
    // 32-bit value. Extend its wrap while the hardware layer keeps a 64-bit
    // software total. This is unambiguous as long as fewer than 2^31 counts
    // arrive between samples, which is far above the chassis pulse rate.
    int64_t delta=(int64_t)current-(int64_t)previous;
    if(delta>INT32_MAX) delta-=INT64_C(1)<<32;
    else if(delta<INT32_MIN) delta+=INT64_C(1)<<32;
    return delta;
}


void chassis_core_tick(chassis_core_t *s, int64_t now, int64_t raw_l, int64_t raw_r)
{
    int64_t raw[2]={raw_l,raw_r};
    if(!s->initialized) {
        s->initialized=true; s->sample_us=now;
        s->raw_previous[0]=raw_l; s->raw_previous[1]=raw_r;
        return;
    }
    int64_t elapsed=now-s->sample_us;
    s->sample_us=now; ++s->seq;
    // No timing fault: a late tick only means the estimate for this sample is
    // coarse. Keep the last valid dt instead of latching a stop.
    if(elapsed<=0) elapsed=1;
    s->dt=elapsed/1e6;
    double ds[2];
    int64_t deltas[2];
    for(int j=0;j<2;j++) {
        // The hardware adapter passes a monotonic 64-bit total, so no wrap
        // correction belongs in the motion core.
        deltas[j]=raw[j]-s->raw_previous[j];
        s->raw_previous[j]=raw[j];
    }
    // No count-rate fault. A bogus encoder pulse can only mislead odometry,
    // which is already reported through odometry_valid; it must not stop the
    // drivetrain.
    for(int j=0;j<2;j++) {
        int64_t delta=deltas[j];
        s->still_s[j]=delta==0?s->still_s[j]+s->dt:0;
        if(s->config.encoder_mode[j]==ENCODER_SINGLE_A) {
            // A single phase reports magnitude only, so its raw delta sign says
            // nothing about travel direction; the target command supplies it.
            // A negative delta is therefore legal (hand-rolling or back-driving
            // a reversing wheel) and must not be treated as a count fault. The
            // Magnitude-only counts still contribute to wheel distance; their
            // inferred sign is a best effort rather than a direction sensor.
            // One phase only measures magnitude, so the sign comes from the
            // commanded direction. With no target the wheel keeps coasting the
            // way it was last driven. The latched mode is the older behaviour:
            // it waits for the wheel to settle before flipping, so pulses that
            // arrive while the car is already reversing (or hand-rolled) are
            // integrated with the previous sign.
            if(!delta) s->unresolved_edges[j]=0;
            else if(s->config.single_a_direction==SINGLE_A_DIR_FROM_TARGET) {
                if(fabs(s->requested[j])>1e-8) {
                    s->inferred_direction[j]=(s->requested[j]>0)?1:-1;
                } else if(!s->inferred_direction[j]) {
                    s->unresolved_edges[j]+=delta;
                    s->odometry_uncertain=true;
                }
            } else if(s->inferred_direction[j]==0) {
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
    // Command lease is enforced by the host's 5 s cmd_vel lease, so the chassis
    // no longer times out on its own. It keeps driving the last target until a
    // new command arrives.
    if(s->mode==MODE_ENCODER) {
        chassis_stop(s,true); return;
    }
    for(int j=0;j<2;j++) {
        // From-target resolution already follows the command, so only the
        // latched mode needs to hold the wheel until it has stopped.
        bool single=s->config.encoder_mode[j]==ENCODER_SINGLE_A &&
                    s->config.single_a_direction==SINGLE_A_DIR_LATCHED;
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
            s->target[j]=clamp(requested,1.0);
            s->duty[j]=normalized_pwm_duty(s->target[j]);
        } else {
            // MODE_ROS requests are normalized wheel PWM. MODE_PID keeps its
            // legacy body-speed interface and normalizes it by max_speed.
            double normalized=s->mode==MODE_PID?requested/s->config.max_speed:requested;
            if(normalized*s->ramped[j]<0) normalized=0;
            s->ramped[j]+=clamp(normalized-s->ramped[j],s->config.acceleration*s->dt);
            if(fabs(s->ramped[j])<1e-6) s->ramped[j]=0;
            s->target[j]=s->ramped[j];
            s->duty[j]=normalized_pwm_duty(s->target[j]);
        }
    }
}
