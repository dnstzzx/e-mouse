#include "string.h"
#include "pid.h"
#include "pid_tracer.h"
#include "basic_algs.h"
#include "app_entry.h"
#include "math.h"
#include "stdio.h"

void pid_init(dn_pid_t* pid, dn_pid_param_t* param){
    memset(pid, 0, sizeof(*pid));
    memcpy(&(pid->param), param, sizeof(*param));

#ifdef APP_ENTRY_ENABEL_PID_TRACER
    pid->tracer_cbk_param = NULL;
    pid->tracer_callback = NULL;
#endif
}


static inline void abs_limit(float* a, float ABS_MAX){
    if (*a > ABS_MAX){
        *a = ABS_MAX;
    }
    if (*a < -ABS_MAX){
        *a = -ABS_MAX;
    }
}


float pid_update(dn_pid_t* pid, float target, float measurement){
    pid->get = measurement;
    pid->set = target;
    pid->last_err = pid->err;
    pid->err = target - measurement;
    if ((pid->param.err_limit != 0) && (fabs(pid->err) > pid->param.err_limit))
        return 0;

    pid->pout = pid->param.kp * pid->err;
    pid->iout += pid->param.ki * pid->err;
    pid->dout = pid->param.kd * (pid->err - pid->last_err);

    abs_limit(&(pid->iout), pid->param.integral_limit);
    pid->out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->out), pid->param.out_limit);

    float rst = pid->out;
    printf("%f %f %f\n", measurement, target - measurement, rst);
#ifdef APP_ENTRY_ENABEL_PID_TRACER
    if (pid->tracer_callback != NULL)
        pid->tracer_callback(pid->tracer_cbk_param, target, measurement, rst);
#endif
    return rst;
}

#ifdef APP_ENTRY_ENABEL_PID_TRACER
void pid_set_tracer_cbk(dn_pid_t* pid, pid_tracer_callback_t callback, void* cbk_param){
    pid->tracer_cbk_param = cbk_param;
    pid->tracer_callback = callback;
}
#endif