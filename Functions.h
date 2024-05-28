/* 
 * File:   Functions.h
 * Author: verooo
 *
 * Created on 23 de mayo de 2024, 17:01
 */

#ifndef FUNCTIONS_H
#define	FUNCTIONS_H

#include "xc.h"
#include <libpic30.h> /* Needed for __delay32 */
#include <stdio.h>
#include <p33EP512GM604.h>

int calculate_safety_distance(float actual_floor_distance, float safety_floor_distance);

float data_conversion(unsigned int data);

void wait_servo_movement();

void move_servo();

unsigned int LiDAR();

unsigned int ultrasonic();

unsigned int coordinates();

unsigned int floor();

void run_ADC(int analog_pin);



#endif	/* FUNCTIONS_H */

