#include "Inits.h"

void init_ports(){
    
    ANSELB = 0;
    ANSELC = 0;
    
    TRISB = 0xFFFF; //configurado como entrada
    TRISC = 0x0000;
}

void init_clock_signal(){
    RCONbits.SWDTEN = 0; // Disable Watchdog Timer

    // This chip is taged with I-Temp stamp (-I/SP) then the temperature range
    // is -40ºC to 85ºC which implies a maximum of 70 MIPS (Fosc = 140 MHz)
    // Chapter 33.1 DC characteristics in dsPIC33EP512GM604 manual.
    
    // We take a conservative configuration
    
    // Configure Oscillator to operate the device at 40 Mhz
    // Fosc = Fin*M/(N1*N2), Fcy = Fosc/2
    // Fosc = 7.3728*65/(2*2) = 119.808 MHz
    // Fcy = Fosc/2 = 59.904 MHz

    // Configure PLL prescaler, PLL postscaler and PLL divisor

    PLLFBDbits.PLLDIV = 63; // M = PLLDIV + 2 = 65 -> PLLDIV = 65 - 2 = 63
    CLKDIVbits.PLLPOST = 0; // N2 = 2 (Output/2)
    CLKDIVbits.PLLPRE = 0; // N1 = 2 (Input/2)
    
    // clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x03); // Initiate Clock Switch to Primary
    __builtin_write_OSCCONL(0x01); // Start clock switching

    while (OSCCONbits.COSC != 0b011); // Wait for Clock switch to occur
    while (OSCCONbits.LOCK != 1) {}; // Wait for PLL to lock (If LOCK = 1 -> PLL start-up timer is satisfied)
}

void init_adc(){
    // Set In/Out pins
    
    TRISAbits.TRISA1 = 1;
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;
        
    ANSELAbits.ANSA1 = 1;   // AN1
    ANSELBbits.ANSB0 = 1;   // AN2
    ANSELBbits.ANSB1 = 1;   // AN3
    
    
    AD1CON1bits.AD12B = 1;  // 12 Bits single channel conversion
    AD1CON1bits.ADSIDL = 1; // Discontinues operation on idle mode.
    AD1CON1bits.FORM = 0;   // Data output integer mode
    
    // Clearing SAMP bits ends sampling and stars conversion
    AD1CON1bits.SSRCG = 0;  
    AD1CON1bits.SSRC = 0;
    
    AD1CON1bits.ASAM = 0;   // Sampling begins when SAMP bit is set
    
    // Voltage reference configuration bits
    AD1CON2bits.VCFG = 0;   // VrefH = AVDD, VrefL = AVSS
    
    AD1CON2bits.CSCNA = 0;  // Do not scan inputs
    
    // Generates interrupt after completion of every sample/conversion operation
    AD1CON2bits.SMPI = 0;
    
    // Buffer fill mode (Does not affect the sampling in this case)
    // 16 bufffers whill be used if more than one conversion per interrupt
    // is used
    AD1CON2bits.BUFM = 0;
    
    AD1CON2bits.ALTS = 0;   // Always select MUXA
    
    AD1CON3bits.ADRC = 0;   // Clock derived from system clock
    
    // Fcy = (119.808E+6)/2 Hz = 59.904E+6 Hz => Tcy = 1.669337607E-8 s
    // for a Tad > 75E-9 s 
    //ADCS = Tad*Fcy -1
    // ADCS = 75E-9 * 59.904E+6 -1 = 4.4928 -1 = 3.4928 = 4
    // ADCS = 4 => Tad = 83.47 ns > 75 ns
    
    AD1CON3bits.ADCS = 5;
    
    // Conversion results are stored in the ADC1BUF0 through ADC1BUFF registers
    // DMA will no used.
    AD1CON4bits.ADDMAEN = 0; 
    
    AD1CHS0bits.CH0NA = 0; //Channel 0 negative input is Vrefl
    
    IPC3bits.AD1IP = 6;
    IFS0bits.AD1IF = 0;
    IEC0bits.AD1IE = 1;
    
    AD1CON1bits.ADON = 1; //Start operation  
}

void init_pwm(){
    /* PWM module is going to be configured as TRUE INDEPENDIENT output mode. */
    /* PWM6H is the channel conected to the servomotor PWM input. */
    
    // Set In/Out pin
    TRISCbits.TRISC6 = 0; //PWM6H (right motor);
    
    /* Parallax imposes 20ms low level period between two high
     * high level of the PWM signal (something no really compatible with PWM).
     * Here we are going to define a PWM signal with period equal to 20ms plus
     * the highest high level interval
     * The center of the signal is at 1.44 ms duty cycle  (0 degrees).
     * 90 degrees to the right is at 1.44ms - 0.9ms = 0.54 ms (PDC6 = 1011).
     * 90 degrees to the left is at 1.44ms + 0.85ms = 2.24ms (PDC6 = 4287)*/
    
        
    
    /* Set period of primary time base.     
     * Fpwm = 1/(20ms + 2.25ms)*64 Prescaler, Fpwm = 
     * As the period is to long for Fosc a prescaler of 64 is selected.
     * PTPER = (119.808 MHZ / 64) * 0.02225 = 41652
     */
    
    PTCON2bits.PCLKDIV = 0b110; // Time base prescaler of 64
    PTPER = 41652;  // 20 ms + 2.25 ms
    
    
    // Null phases
    PHASE6 = 0;
    SPHASE6 = 0;
    
    // Initial duty cycle
    //PDC6 = 2696; // 1.44 ms
    
    // No dead times
    DTR6 = 0;
    ALTDTR6 = 0;
    
    IOCON6bits.PMOD = 0b11; // True independent output mode
    FCLCON6bits.FLTMOD = 0b11; // Fault input is disabled
    IOCON6bits.SWAP = 0; // No swap high and low outputs.
    
    // Who controls PWM pins
    IOCON1bits.PENH = 0; //GPIO
    IOCON1bits.PENL = 0; //GPIO
    IOCON2bits.PENH = 0; //GPIO
    IOCON2bits.PENL = 0; //GPIO
    IOCON3bits.PENH = 0; //GPIO
    IOCON3bits.PENL = 0; //GPIO
    IOCON4bits.PENH = 0; //GPIO
    IOCON4bits.PENL = 0; //GPIO
    IOCON5bits.PENH = 0; //GPIO
    IOCON5bits.PENL = 0; //GPIO
    IOCON6bits.PENH = 1; //PMW module
    IOCON6bits.PENL = 0; //GPIO
    
    // PWCMCON 
    // Fault interruot is disabled.
    // Current limit interruot is disabled.
    // Triger event interrupt is disabled.
    // PTPER provides timing for this PWM generator.
    // Master duty cycle NOT used.
    // Primary time base used.
    // Edge aligned mode is enabled.
    
    PWMCON6 = 0;
    
    PWMCON6bits.DTC = 0b10; // Dead time is disabled
    
    // Enable pwm module
    PTCONbits.PTEN =1;
}

void init_i2c(){
    
}

void init_uart2(){ //para el lidar
    /* Specified pins for UART2 */
    
    /* RX RPI44 */
    RPINR19bits.U2RXR = 0b0101100; 
    TRISBbits.TRISB12 = 1;
    
    /* TX RP43 */
    RPOR4bits.RP43R = 0b000011;
    TRISBbits.TRISB11 = 0;
    
    
    /* Configure UART */
    U2MODEbits.USIDL = 1;   // Stop on idle mode
    U2MODEbits.IREN = 0;    // disable IrDA operation
    U2MODEbits.UEN = 0;     // Only RX and TX are used (non CTS, RTS and BCLK)
    U2MODEbits.WAKE = 0;    // Wake up on start bit is disabled
    U2MODEbits.LPBACK = 0;  // Loopback mode disabled
    U2MODEbits.ABAUD = 0;   // Baud rate measurement disabled
    U2MODEbits.URXINV = 0;  // Non polarity inversion. Idle state is 1
    U2MODEbits.BRGH = 0;    // High baude rate disabled
    U2MODEbits.PDSEL = 0;   // 8 bit data with no parity
    U2MODEbits.STSEL = 0;   // One stop bit.
    
    
    U2STAbits.URXISEL = 0;  // Interrupt on each character received
    
    // U1BRG = (Fcy/(16*Baud_rate) -1)
    //U1BRG = 389;            // For 9600 bauds (119.808/2 MHz)/(16*9600) - 1
    U2BRG = 32; // For 115200 bauds
    
    /* Configure interrupts */
    IPC7bits.U2RXIP = 2;
    IFS1bits.U2RXIF = 0;
    IEC1bits.U2RXIE = 1;
    
    U2MODEbits.UARTEN = 1; // Enable UART operation
    U2STAbits.UTXEN = 1;    // Enable uart1 TX (must be done after UARTEN)
    
    /* It is needed to wait one transmision bit to ensure start bit detection 
     When 9600 Baud rate is selected it is necessary to wait 104 us */
    __delay32(383);
}

void init_uart1(){ //para comunicacion con control
    
}

void init_ultrasound_sensor(){
    
    TRISAbits.TRISA7 = 1; //Echo
    TRISAbits.TRISA10 = 0; //Init
    CNENAbits.CNIEA7 = 1; // Enable interrupt on change of RA7 bit
    
    // configure Timer2
    T2CONbits.TCKPS = 0b11; // 1:256 prescale
    PR2 = 0xFFFF;
    TMR2 = 0;
    
    IPC1bits.T2IP = 7;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
    
    IPC4bits.CNIP = 7;
    IFS1bits.CNIF = 0;
    IEC1bits.CNIE = 0;
}
