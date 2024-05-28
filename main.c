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

#define TRUE 1
#define FALSE 0

/*Semaphores*/
Tsemaphore distSem1;
Tsemaphore distSem2;
Tsemaphore coorSem1;
Tsemaphore coorSem2;
Tsemaphore floorSem1;
Tsemaphore floorSem2;
Tsemaphore driverUARTSem; //falta por plantear
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
union_t speed_coordinates;
union_t actual_floor_distance;
float safety_floor_distance;
char E_Off, ignorar_LiDAR = TRUE;
int degrees, echo_received, ADC_finished, ADC_result;

int id1, id2, id3;

/*Perception Tasks*/
void task_distance(){
    while(1){
        wait(&distSem1);
        mutex_lock(&distMutex);//recoger valor region critica
        degrees = PORTB; //saber si navegacion lo tiene que poner antes como salida
        mutex_unlock(&distMutex);
        move_servo(degrees);//dejar valor en PDC6
        //distance_ultrasonic.value = ultrasonic();
        send_ultrasonic_chirp();
        ignorar_LiDAR = FALSE;
        distance_LiDAR.value = LiDAR();
        //distance_ultrasonic.fvalue = data_conversion(distance_ultrasonic.value);
        distance_LiDAR.fvalue = data_conversion(distance_LiDAR.value);
        //evaluar con cual de la dos me quedo
        degrees = 0;
        move_servo(degrees);
        for (int i = 0; i < 5; i++){
            mutex_lock(&distMutex);//dejar en region critica
            //dependiendo de cual me quedo
            LATC = distance_LiDAR.v[i];
            mutex_unlock(&distMutex);
        }
        signal(&distSem2);
    }
}
void task_floor(){
    while(1){
        wait(&floorSem1);
        while(!E_Off){
            for(int i = 0; i < 4; i++){
                actual_floor_distance.value = floor(i);
                if(calculate_safety_distance(actual_floor_distance.fvalue, safety_floor_distance) <= 0){ //se ha puesto 0 como ejemplo, el valor puede ser otro
                    E_Off = TRUE;
                    //activar el pin de emergencia
                    signal(&floorSem2);
                    /*mutex_lock(&floorMutex);
                    //dejar mensaje en region critica
                    mutex_unlock(&floorMutex);*/
                }else{
                    E_Off = FALSE;
                }
            }
            delay_until(0);//periodo de comprobacion
        }
    }
}
void task_coordinates(){
    while(1){
        wait(&coorSem1);
        speed_coordinates.value = coordinates();
        speed_coordinates.fvalue = data_conversion(speed_coordinates.value);
        for(int i = 0; i < 5; i++){
            mutex_lock(&coorMutex);
            LATC = speed_coordinates.v[i];
            mutex_unlock(&coorMutex);
        }
        signal(&coorSem2);
    }
}
//task_driverUART

/*Navigation Tasks*/


void send_ultrasonic_chirp(void)
{
    echo_received = PORTAbits.RA7; // Interrupt on change armed
    echo_received = FALSE;
    TMR2 = 0;
    T2CONbits.TON = 1;
    PORTAbits.RA10 = 1; //Set init
    IEC1bits.CNIE = 1; //Enable interrupt on change of echo bit
}



/*
 * 
 */
int main() {
    int x;
    
    init_clock_signal();
    init_ports();
    init_adc();
    init_pwm();
    //init_i2c();
    init_uart2(); //para el lidar
    //init_uart1(); //para comunicacion con control
    init_ultrasound_sensor();
    
    x = init_AuK(59.904E+6, 0.0002);
    
    if(x == 0){
        id1 = create_task(__builtin_tblpage(task_distance),
                          __builtin_tbloffset(task_distance), 100, 1);
        id2 = create_task(__builtin_tblpage(task_floor),
                          __builtin_tbloffset(task_floor), 100, 3);
        id3 = create_task(__builtin_tblpage(task_coordinates),
                          __builtin_tbloffset(task_coordinates), 100, 1);
        
        //navigation tasks
        
        start_AuK();
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
