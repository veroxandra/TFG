#include "Functions.h"

#define TRUE 1
#define FALSE 0

int ADC_finished1, ADC_result1;

int calculate_safety_distance(float actual_floor_distance, float safety_floor_distance){
    return 0;
}

float data_conversion(unsigned int data){
    return 0;
}

void move_servo(int degrees){
  
     /* Parallax imposes 20ms low level period between two high
     * high level of the PWM signal (something no really compatible with PWM).
     * Here we are going to define a PWM signal with period equal to 20ms plus
     * the highest high level interval
     * The center of the signal is at 1.44 ms duty cycle  (0 degrees).
     * 90 degrees to the right is at 1.44ms - 0.9ms = 0.54 ms (PDC6 = 1011).
     * 90 degrees to the left is at 1.44ms + 0.85ms = 2.24ms (PDC6 = 4287)*/
    
    int value;
    
    //ajustar degrees al valor necesario
    value = degrees;
    
    PDC6 = value;
    
    __delay32(119808000UL); // 2 seconds to assegue movement finished
}

void run_ADC(int analog_pin){
    
    ADC_finished1 = FALSE;
            
    // Select channel
    AD1CHS0bits.CH0SA = analog_pin; //AN1 (RA1), AN2 (RB0), AN3 (RB1))
            
    // Start sampling
    AD1CON1bits.SAMP = 1;
      
    // Time to charge sample and hold capacitor up to 99% is less than 1 us
    __delay32(60); // 1 us
            
    // Start ADC conversion
    AD1CON1bits.SAMP = 0;
            
    while(!ADC_finished1);  
}

unsigned int LiDAR(){
    return 0;
}

unsigned int coordinates(){
    return 0;
}

unsigned int floor(int i){
    
    run_ADC(i);
    return ADC_result1;
}
