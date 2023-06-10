#pragma once
#include "app_entry.h"

typedef struct{
    float kp,ki,kd;
    float err_limit;
    float out_limit;
    float integral_limit;
}dn_pid_param_t;

#ifdef APP_ENTRY_ENABEL_PID_TRACER
typedef   void (*pid_tracer_callback_t) (void *cbk_param, float target, float measurement, float output);
#endif

typedef struct{
    dn_pid_param_t param;
    float set;
    float get;

    float err;
    float last_err;

    float pout;
    float iout;
    float dout;
    float out;

#ifdef APP_ENTRY_ENABEL_PID_TRACER
    void *tracer_cbk_param;
    pid_tracer_callback_t tracer_callback;
#endif

}dn_pid_t;

void pid_init(dn_pid_t *pid, dn_pid_param_t *param);
float pid_update(dn_pid_t *pid, float target, float measurement);

#ifdef APP_ENTRY_ENABEL_PID_TRACER
void pid_set_tracer_cbk(dn_pid_t *pid, pid_tracer_callback_t callback, void *cbk_param);
#endif