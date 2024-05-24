/* 
 * File:   Inits.h
 * Author: verooo
 *
 * Created on 24 de mayo de 2024, 13:13
 */

#ifndef INITS_H
#define	INITS_H

void init_ports();
void init_clock_signal();
void init_adc();
void init_pwm();
void init_i2c();
void init_timer1(); //para el mk
void init_timer2(); //para el ultrasonidos
void init_uart2(); //para el lidar
void init_uart3(); //para comunicacion con control
void init_ultrasound_sensor();

#endif	/* INITS_H */

