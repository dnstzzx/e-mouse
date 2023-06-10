#pragma once
#include "pid_tracer.h"

// set
void pid_tracer_protocol_rx_process(uint8_t *data, size_t len);

// report
void pid_tracer_report(pid_tracer_t *tracer, float target, float measurement, float output);
