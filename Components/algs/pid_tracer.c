#include "pid_tracer.h"

#ifdef APP_ENTRY_ENABEL_PID_TRACER


pid_tracer_t channels[PID_TRACER_MAX_CHANNEL_COUNT] = {};

void pid_tracer_init(){
    memset(channels, 0, sizeof(channels));
}

void pid_tracer_add(){

}

#endif