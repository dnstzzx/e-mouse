#pragma once
#include "app_entry.h"
#include "stdbool.h"

#ifdef APP_ENTRY_ENABEL_PID_TRACER
#include "pid.h"

#define PID_TRACER_MAX_CHANNEL_COUNT (6)

typedef struct{
    bool enabled;
    uint8_t channel;
    dn_pid_t *pid;
} pid_tracer_t;



#endif