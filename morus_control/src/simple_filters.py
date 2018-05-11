#!/usr/bin/env python

__author__ = "aivanovic"

def deadzone(value, lower_limit, upper_limit):

    if(value > upper_limit): return(value - upper_limit)
    elif(value < lower_limit): return(value - lower_limit)
    else: return 0.0


def filterPT1(previous_output, current_value, T, Ts, K):

    a = T / (T + Ts)
    b = K*Ts / (T + Ts)

    return (a*previous_output + b*current_value)

def signum(a):
    if a > 0.0:
        return 1.0
    elif a < 0.0:
        return -1.0
    else:
        return 0

def ramp(previous_output, setpoint, Ts, K):
    delta = setpoint - previous_output
    if abs(delta) < K*Ts:
        return setpoint
    return K * Ts * signum(delta) + previous_output