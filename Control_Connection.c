/*
 * File:   Control_Connection.c
 * Author: hugofva
 *
 * Created on 10 de junio de 2024, 19:16
 */

#include "Control_Connection.h"
#include <stdio.h>
#include <string.h>
#include <xc.h>


void send_control_signals(float linear_velocity, float angular_velocity) {
    char buffer[50];
    sprintf(buffer, "V: %f W: %f\n", linear_velocity, angular_velocity);
    uart_write_string(buffer);
}

