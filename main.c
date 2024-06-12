/* 
 * File:   main.c
 * Author: verooo
 *
 * Created on May 23, 2024, 2:06 PM
 */
// DSPIC33EP512GM604 Configuration Bit Settings

// 'C' source line config statements

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config BOREN = OFF              //  (BOR is disabled)
#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25           // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS64           // Watchdog Timer Postscaler bits (1:64)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = XT              // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = PRI              // Oscillator Source Selection (Primary Oscillator (XT, HS, EC))
#pragma config PWMLOCK = OFF            // PWM Lock Enable bit (PWM registers may be written without key sequence)
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)

#include "xc.h"
#include <libpic30.h> /* Needed for __delay32 */
#include <stdio.h>
#include <p33EP512GM604.h>
#include "AuK.h"
#include "Functions.h"
#include "Inits.h"
#include "Topo_Map.h"
#include "Metric_Map.h"
#include "Route_Planning.h"
#include "Perception_Connection.h"
#include "Control_Connection.h"
#include "Map_Management.h"

#define TRUE 1
#define FALSE 0

#define BNO055_I2C_ADDRESS 0x28 //cambiar segun la direccion

/*Semaphores*/
Tsemaphore distSem1;
Tsemaphore distSem2;
Tsemaphore coorSem1;
Tsemaphore coorSem2;
Tsemaphore floorSem1;
Tsemaphore floorSem2;
Tsemaphore driverUARTSem; 
Tsemaphore topo_map_semaphore;
Tsemaphore metric_map_semaphore;
Tsemaphore velocity_calc_semaphore;
/*Mutex*/
Tmutex distMutex;
Tmutex coorMutex;
//Tmutex floorMutex;

typedef union{
    unsigned int value;
    float fvalue;
    char v[4];
}union_t;

union_t distance_LiDAR;
union_t distance_ultrasonic;
union_t actual_floor_distance;
float safety_floor_distance, distance;
char E_Off, ignorar_LiDAR, data, state, coorValues[6];
int degrees, echo_received, ADC_finished, ADC_result;
unsigned int Low_data, High_data;

int id1, id2, id3, id4, in1, in2, in3, in4;

/*Perception Tasks*/
void task_distance(){
    while(1){
        ignorar_LiDAR = TRUE;
        wait(&distSem1);
        /*mutex_lock(&distMutex);//recoger valor region critica
        degrees = PORTB; //saber si navegacion lo tiene que poner antes como salida
        mutex_unlock(&distMutex);*/
        move_servo(degrees);//dejar valor en PDC6
        //distance_ultrasonic.value = ultrasonic();
        send_ultrasonic_chirp(); //revisar cuanto tarda
        ignorar_LiDAR = FALSE;
        //distance_LiDAR.value = LiDAR();
        //distance_ultrasonic.fvalue = data_conversion(distance_ultrasonic.value);
        distance_LiDAR.fvalue = (float)distance_LiDAR.value / 100.0; //in meters
        //evaluar con cual de la dos me quedo
        if(distance_ultrasonic.fvalue < distance_LiDAR.fvalue){
            distance = distance_LiDAR.fvalue;
        }else{
            distance = distance_ultrasonic.fvalue;
        }
        degrees = 0;
        move_servo(degrees);
        /*for (int i = 0; i < 5; i++){
            mutex_lock(&distMutex);//dejar en region critica
            //dependiendo de cual me quedo
            LATC = distance_LiDAR.v[i];
            mutex_unlock(&distMutex);
        }*/
        signal(&distSem2);
    }
}

void task_floor(){
    while(1){
        wait(&floorSem1);
        while(!E_Off){
            unsigned long wake_up_time, task_period;
            task_period = 1; // 0.2 ms, revisar esto
            wake_up_time = clock();
            for(int i = 0; i < 4; i++){
                actual_floor_distance.value = floor(i);
                if((safety_floor_distance - actual_floor_distance.fvalue) >= 0.05 ){ //se ha puesto 0.5 como ejemplo, el valor puede ser otro
                    E_Off = TRUE;
                    //activar el pin de emergencia
                    //pin 44 y RB8
                    //Resgistro: RPOR3 (pág 171 del pic)
                    signal(&floorSem2);
                    /*mutex_lock(&floorMutex);
                    //dejar mensaje en region critica
                    mutex_unlock(&floorMutex);*/
                }else{
                    E_Off = FALSE;
                }
            }
            wake_up_time += task_period;
            delay_until(wake_up_time);//periodo de comprobacion
        }
    }
}

void task_coordinates(){
    while(1){
        wait(&coorSem1);
        
        char buffer[6];
        
        coordinates(BNO055_I2C_ADDRESS, 0x28, &buffer, 6);
        
        coorValues[0] = (buffer[1] << 8) | buffer[0]; //accel_x
        coorValues[1] = (buffer[3] << 8) | buffer[2]; //accel_y
        coorValues[2] = (buffer[5] << 8) | buffer[4]; //accel_z
        
        coordinates(BNO055_I2C_ADDRESS, 0x14, &buffer, 6);
        
        coorValues[3] = (buffer[1] << 8) | buffer[0]; //gyro_x
        coorValues[4] = (buffer[3] << 8) | buffer[2]; //gyro_y
        coorValues[5] = (buffer[5] << 8) | buffer[4]; //gyro_z

        
        // Leer un byte del dispositivo esclavo
        //speed_coordinates.value = coordinates(slave_address, register_address);
        //speed_coordinates.fvalue = data_conversion(speed_coordinates.value);
        /*for(int i = 0; i < 7; i++){
            mutex_lock(&coorMutex);
            LATC = coorValues[i];
            mutex_unlock(&coorMutex);
        }*/
        signal(&coorSem2);
    }
}

void task_driverUART(){
    while(1){
        wait_within_driver(&driverUARTSem, IEC1, 14);
        switch(state){
            case 0: 
                Low_data = (unsigned int)data;
                state = 1;
                break;
            case 1:
                High_data = (unsigned int)data;
                distance_LiDAR.value = High_data << 8;
                distance_LiDAR.value |= Low_data;                
                state = 2;
                break;
            case 2:
                state = 0;
                signal(&driverUARTSem); //esto no lo termino de pillar
                break;  
        }
    }
    
}

void send_ultrasonic_chirp(void)
{
    echo_received = PORTAbits.RA7; // Interrupt on change armed
    echo_received = FALSE;
    TMR2 = 0;
    T2CONbits.TON = 1;
    PORTAbits.RA10 = 1; //Set init
    IEC1bits.CNIE = 1; //Enable interrupt on change of echo bit
}

/*Navigation Tasks*/
void task_topological_map() {
    while (1) {
        wait(&topo_map_semaphore);
        a_star_topological(4, 6);  // Ejecuta el algoritmo A* en el mapa topológico
        signal(&metric_map_semaphore);  // Desbloquea la tarea del mapa métrico
    }
}

void task_metric_map() {
    while (1) {
        wait(&metric_map_semaphore);
        a_star_metric();  // Ejecuta el algoritmo A* en el mapa métrico usando final_path del topológico
        signal(&velocity_calc_semaphore);  // Desbloquea la tarea de cálculo de velocidades
    }
}

void task_velocity_calculation() {
    while (1) {
        wait(&velocity_calc_semaphore);
        calculate_velocity();  // Calcula las velocidades angular y lineal
        signal(&topo_map_semaphore);  // Desbloquea la tarea del mapa topológico para el siguiente ciclo
    }
}

void task_update_map() {
    while (1) {
        update_map_with_sensor_data();  // Actualiza el mapa métrico con los datos de los sensores
    }
}

int main() {
    int x;
    
    init_clock_signal();
    init_ports();
    init_adc();
    init_pwm();
    init_i2c();
    init_uart2(); //para el lidar
    init_uart1(); //para comunicacion con control
    init_ultrasound_sensor();
    
    // Inicializa semáforos
    init_semaphore(&topo_map_semaphore, 0);  
    init_semaphore(&metric_map_semaphore, 0);
    init_semaphore(&velocity_calc_semaphore, 0);
    
    x = init_AuK(59.904E+6, 0.0002);
    
    if(x == 0){
        id1 = create_task(__builtin_tblpage(task_distance),
                          __builtin_tbloffset(task_distance), 100, 1);
        id2 = create_task(__builtin_tblpage(task_floor),
                          __builtin_tbloffset(task_floor), 100, 3);
        id3 = create_task(__builtin_tblpage(task_coordinates),
                          __builtin_tbloffset(task_coordinates), 100, 1);
        id4 = create_task(__builtin_tblpage(task_driverUART),
                          __builtin_tbloffset(task_driverUART), 100, 1);
        //navigation tasks
        in1 = create_task(__builtin_tblpage(task_topological_map),
                          __builtin_tbloffset(task_topological_map), 100, 1);
        in2 = create_task(__builtin_tblpage(task_metric_map),
                          __builtin_tbloffset(task_metric_map), 100, 2);
        in3 = create_task(__builtin_tblpage(task_velocity_calculation),
                          __builtin_tbloffset(task_velocity_calculation), 100, 3);
        in4 = create_task(__builtin_tblpage(task_update_map),
                          __builtin_tbloffset(task_update_map), 100, 4);
        start_AuK();
        
        signal(&topo_map_semaphore);  // Inicia la primera tarea
    }
    
    Sleep();
    return(0);
}

/*Interrupts*/

void __attribute__((__interrupt__, no_auto_psv)) _CNInterrupt (void)
{
    echo_received = PORTAbits.RA7;
    if(echo_received)
    {
        // 344 ms
        // a TMR2 count is equal to 1.47 mm. => 2 TMR2 count are almost 3 mm
        // Then divide by 2 and multiply by 3
        distance_ultrasonic.value = TMR2;
        // ultrasonic_distance /= 2;
        distance_ultrasonic.value >>= 1;
        distance_ultrasonic.value *= 3;
        // This is the distance of the full travel 
        // (robot to obstacle + obstacle to robot).
        // It is needed to be divided by 2
        // ultrasonic_distance /= 2;
        distance_ultrasonic.value >>= 1;
        // An to offer distance in meters
        distance_ultrasonic.fvalue = distance_ultrasonic.fvalue / 1000.0;
        IEC1bits.CNIE = 0; //Disable interrupt on change of echo pin.
        T2CONbits.TON = 0;
        TMR2 = 0;
        PORTAbits.RA10 = 0; // Unset init
    }
    IFS1bits.CNIF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt (void)
{
    distance_ultrasonic.value = 0xFFFF;
    IEC1bits.CNIE = 0; //Disable interrupt on change of echo pin.
    T2CONbits.TON = 0;
    TMR2 = 0;
    PORTAbits.RA10 = 0; // Unset init
    
    IFS0bits.T2IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _AD1Interrupt(void)
{
    ADC_finished = TRUE;
    ADC_result = ADC1BUF0;
    IFS0bits.AD1IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt (void)
{
    data = U2RXREG;
    
    if(!ignorar_LiDAR){
        data = U2RXREG;
        if(data == 0x59){
            data = U2RXREG;
            IEC1bits.U2RXIE = 0;
            IFS1bits.U2RXIF = 0;
            signal(&driverUARTSem);
        }
    }
}

void __attribute__((__interrupt__, no_auto_psv)) _OscillatorFail(void) {
    INTCON1bits.OSCFAIL = 0;
    PORTAbits.RA1 = 1;
    while(1);
}

void __attribute__((__interrupt__, no_auto_psv)) _AddressError(void) {
    INTCON1bits.ADDRERR = 0;
    PORTAbits.RA1 = 1;
    while(1);
}

void __attribute__((__interrupt__, no_auto_psv)) _HardTrapError(void) {
    INTCON1bits.ADDRERR = 0;
    PORTAbits.RA1 = 1;
    while(1);
}

void __attribute__((__interrupt__, no_auto_psv)) _StackError(void) {
    INTCON1bits.STKERR = 0;
    PORTAbits.RA1 = 1;
    while(1);
}

void __attribute__((__interrupt__, no_auto_psv)) _MathError(void) {
    INTCON1bits.MATHERR = 0;
    PORTAbits.RA1 = 1;
    while(1);
}

void __attribute__((__interrupt__, no_auto_psv)) _DMACError(void) {
    INTCON3bits.DAE = 0;
    PORTAbits.RA1 = 1;
    while(1);
}

void __attribute__((__interrupt__, no_auto_psv)) _SoftTrapError(void) {
    INTCON4bits.SGHT = 0;
    PORTAbits.RA1 = 1;
    while(1);
}