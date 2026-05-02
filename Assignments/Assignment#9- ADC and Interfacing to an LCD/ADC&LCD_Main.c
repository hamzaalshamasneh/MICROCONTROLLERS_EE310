/*
 * Title:   Accelerometer Sensor Interfacing with LCD 
 * Author:  Hamza Alshamasneh
 *
 * Version History:
 * V1.0 : 5/01/2026 - Final working version with ADC, LCD, and accelerometer
 *
 * ------------------------------------------------------------
 * Device:   PIC18F47K42
 * Compiler: MPLAB X IDE + XC8 v3.10
 * Includes: MY_C_CONFIG.h
 * ------------------------------------------------------------
 *
 * Purpose:
 * This program reads an analog accelerometer signal using the ADC
 * of the PIC18F47K42 microcontroller and displays the detected
 * motion state on a 16x2 LCD.
 *
 * The system uses:
 * - One analog accelerometer input connected to RA1 / ANA1
 * - One push button interrupt connected to RC2 / IOCC2
 * - One red LED connected to RC3
 * - One 16x2 LCD connected using PORTB and PORTD
 *
 * Program Operation:
 * A) The system initializes the LCD, ADC, GPIO pins, and IOC interrupt.
 * B) The ADC continuously reads the accelerometer X-axis voltage.
 * C) The voltage is converted into an approximate acceleration value.
 * D) The system detects Tilt Left, Tilt Right, Flat, or Shake.
 * E) The detected state and X-direction value are displayed on the LCD.
 * F) When the push button is pressed, an interrupt occurs.
 * G) The system enters HALT mode for 10 seconds.
 * H) During HALT mode, ADC reading stops and the red LED blinks.
 * I) After 10 seconds, the system returns to normal operation.
 *
 * Inputs:
 * RA1 : Accelerometer analog X-axis input
 * RC2 : Interrupt push button input
 *
 * Outputs:
 * RB0-RB7 : LCD data pins
 * RD0     : LCD RS control pin
 * RD1     : LCD EN control pin
 * RC3     : Red LED output
 */

#include "MY_C_CONFIG.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define RS LATD0                  // LCD Register Select pin
#define EN LATD1                  // LCD Enable pin
#define ldata LATB                // LCD data connected to PORTB

#define LCD_Port TRISB
#define LCD_Control TRISD

#define LED LATCbits.LATC3        // LED connected to RC3

#define Vref 5.0                  // ADC reference voltage

int digital;                      // Stores ADC digital result
float voltage;                    // Stores converted voltage value
float prev_voltage = 0;           // Stores previous voltage for shake detection
float accel_x;                    // Stores calculated acceleration value

char line2[17];                   // LCD second line text buffer
char state[17];                   // Stores current motion state text

volatile unsigned char halt_flag = 0;   // Set by interrupt when button is pressed

/******** Function Prototypes ********/
void LCD_Init(void);                         // Initializes LCD
void LCD_Command(char);                      // Sends command to LCD
void LCD_Char(char);                         // Sends one character to LCD
void LCD_String(const char *);               // Sends string to LCD
void LCD_String_xy(char, char, const char *); // Sends string to LCD position
void MSdelay(unsigned int);                  // Software delay function

void ADC_Init(void);                         // Initializes ADC
void IOC_Init(void);                         // Initializes interrupt-on-change

/***************************** INTERRUPT *****************************/
void __interrupt() ISR(void)
{
    if(PIR0bits.IOCIF && IOCCFbits.IOCCF2)   // Checks if RC2 interrupt happened
    {
        halt_flag = 1;                       // Tells main program to enter HALT mode

        IOCCFbits.IOCCF2 = 0;                // Clears RC2 interrupt flag
        PIR0bits.IOCIF = 0;                  // Clears main IOC interrupt flag
    }
}

/***************************** MAIN *****************************/
void main(void)
{
    ANSELA = 0x00;                           // Makes PORTA pins digital
    ANSELB = 0x00;                           // Makes PORTB pins digital
    ANSELC = 0x00;                           // Makes PORTC pins digital
    ANSELD = 0x00;                           // Makes PORTD pins digital

    TRISB = 0x00;                            // PORTB is output for LCD data
    TRISD = 0x00;                            // PORTD is output for LCD control

    TRISCbits.TRISC2 = 1;                    // RC2 is input for interrupt button
    TRISCbits.TRISC3 = 0;                    // RC3 is output for red LED

    LATB = 0x00;                             // Clears PORTB outputs
    LATD = 0x00;                             // Clears PORTD outputs
    LATC = 0x00;                             // Clears PORTC outputs

    LCD_Init();                              // Starts the LCD
    ADC_Init();                              // Starts the ADC
    IOC_Init();                              // Starts the button interrupt

    while(1)                                 // Main program loop
    {
        if(halt_flag == 1)                   // Checks if interrupt button was pressed
        {
            halt_flag = 0;                   // Clears HALT request

            PIE0bits.IOCIE = 0;              // Disables IOC during HALT

            LCD_Command(0x01);               // Clears LCD screen
            LCD_String_xy(1, 0, "SYSTEM HALT"); // Shows HALT message
            LCD_String_xy(2, 0, "Wait 10 sec"); // Shows wait message

            for(int i = 0; i < 20; i++)      // Runs blink loop for about 10 seconds
            {
                LED = 1;                     // Turns LED ON
                __delay_ms(250);             // Waits 250 ms
                LED = 0;                     // Turns LED OFF
                __delay_ms(250);             // Waits 250 ms
            }

            LED = 0;                         // Makes sure LED is OFF after HALT

            while(PORTCbits.RC2 == 1);       // Waits until button is released
            __delay_ms(150);                 // Small debounce delay

            IOCCFbits.IOCCF2 = 0;            // Clears RC2 flag again
            PIR0bits.IOCIF = 0;              // Clears IOC flag again

            PIE0bits.IOCIE = 1;              // Enables IOC again

            LCD_Command(0x01);               // Clears LCD before normal mode
        }

        ADCON0bits.GO = 1;                   // Starts ADC conversion
        while(ADCON0bits.GO);                // Waits until ADC is done

        digital = ((int)ADRESH << 8) | ADRESL; // Combines high and low ADC bytes
        voltage = digital * (Vref / 4096.0); // Converts ADC value to voltage

        accel_x = (voltage - 1.65) / 0.33;   // Converts voltage to approximate X acceleration

        if(fabs(voltage - prev_voltage) > 0.10) // Checks for sudden voltage change
        {
            strcpy(state, "Shake");          // Sets state to Shake
        }
        else if(voltage < 1.63)              // Checks if voltage is below flat range
        {
            strcpy(state, "Tilt Left");      // Sets state to Tilt Left
        }
        else if(voltage > 1.70)              // Checks if voltage is above flat range
        {
            strcpy(state, "Tilt Right");     // Sets state to Tilt Right
        }
        else                                 // If none of the above
        {
            strcpy(state, "Flat");           // Sets state to Flat
        }

        prev_voltage = voltage;              // Saves voltage for next comparison

        sprintf(line2, "%.3f X-dir", accel_x); // Formats acceleration text

        LCD_Command(0x01);                   // Clears LCD screen

        LCD_String_xy(1, 0, "State:");       // Prints label on line 1
        LCD_String_xy(1, 7, state);          // Prints motion state on line 1

        LCD_String_xy(2, 0, line2);          // Prints X direction value on line 2

        __delay_ms(300);                     // Small delay before next reading
    }
}

/**************************** IOC INIT ****************************/
void IOC_Init(void)
{
    TRISCbits.TRISC2 = 1;                    // Sets RC2 as button input
    ANSELCbits.ANSELC2 = 0;                  // Makes RC2 digital

    TRISCbits.TRISC3 = 0;                    // Sets RC3 as LED output
    ANSELCbits.ANSELC3 = 0;                  // Makes RC3 digital
    LED = 0;                                 // Starts LED OFF

    IOCCPbits.IOCCP2 = 1;                    // Enables positive edge interrupt on RC2
    IOCCNbits.IOCCN2 = 0;                    // Disables negative edge interrupt on RC2

    IOCCFbits.IOCCF2 = 0;                    // Clears RC2 IOC flag
    PIR0bits.IOCIF = 0;                      // Clears main IOC flag

    PIE0bits.IOCIE = 1;                      // Enables IOC interrupt
    INTCON0bits.GIE = 1;                     // Enables global interrupts
}

/**************************** ADC INIT ****************************/
void ADC_Init(void)
{
    ADCON0bits.FM = 1;                       // Makes ADC result right justified
    ADCON0bits.CS = 1;                       // Uses internal ADC clock

    TRISAbits.TRISA1 = 1;                    // Sets RA1 as input
    ANSELAbits.ANSELA1 = 1;                  // Makes RA1 analog

    ADPCH = 0x01;                            // Selects ANA1 as ADC input channel

    ADCLK = 0x00;                            // Sets ADC clock divider

    ADRESH = 0;                              // Clears ADC high result register
    ADRESL = 0;                              // Clears ADC low result register

    ADPREL = 0;                              // No ADC precharge low byte
    ADPREH = 0;                              // No ADC precharge high byte

    ADACQL = 0;                              // No extra acquisition low byte
    ADACQH = 0;                              // No extra acquisition high byte

    ADREF = 0x00;                            // Uses VDD and VSS as ADC reference

    ADCON0bits.ON = 1;                       // Turns ADC module ON
}

/**************************** LCD ****************************/
void LCD_Init(void)
{
    MSdelay(15);                             // LCD power-on delay

    LCD_Port = 0x00;                         // Sets LCD data pins as output
    LCD_Control = 0x00;                      // Sets LCD control pins as output

    LCD_Command(0x01);                       // Clears LCD
    LCD_Command(0x38);                       // Sets LCD to 8-bit, 2-line mode
    LCD_Command(0x0C);                       // Display ON, cursor OFF
    LCD_Command(0x06);                       // Auto-increment cursor
}

void LCD_Command(char cmd)
{
    ldata = cmd;                             // Sends command to data pins
    RS = 0;                                  // Selects command register
    EN = 1;                                  // Enables LCD pulse
    NOP();                                   // Small delay
    EN = 0;                                  // Ends LCD pulse
    MSdelay(3);                              // Waits for LCD command
}

void LCD_Char(char dat)
{
    ldata = dat;                             // Sends character to data pins
    RS = 1;                                  // Selects data register
    EN = 1;                                  // Enables LCD pulse
    NOP();                                   // Small delay
    EN = 0;                                  // Ends LCD pulse
    MSdelay(1);                              // Waits for LCD data write
}

void LCD_String(const char *msg)
{
    while(*msg)                              // Loops until string ends
    {
        LCD_Char(*msg++);                    // Sends each character to LCD
    }
}

void LCD_String_xy(char row, char pos, const char *msg)
{
    char location;                           // Stores LCD cursor address

    if(row == 1)                             // Checks if line 1 is selected
    {
        location = 0x80 + pos;               // Sets address for LCD line 1
    }
    else                                     // Otherwise use line 2
    {
        location = 0xC0 + pos;               // Sets address for LCD line 2
    }

    LCD_Command(location);                   // Moves LCD cursor
    LCD_String(msg);                         // Prints string at location
}

void MSdelay(unsigned int val)
{
    unsigned int i, j;                       // Delay loop counters

    for(i = 0; i < val; i++)                 // Outer delay loop
    {
        for(j = 0; j < 165; j++);            // Inner delay loop
    }
}
