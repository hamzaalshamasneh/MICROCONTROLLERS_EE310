/*
 * ============================================================================
 * Title:   Multi-Mode Motion Controlled Servo System
 * Authors:  Hamza Alshamasneh / Taine Harrison
 *
 * Device:   PIC18F47K42
 * Compiler: MPLAB X IDE + XC8
 * Header:   MY_C_CONFIG.h
 *
 * ============================================================================
 * Project Description:
 * ----------------------------------------------------------------------------
 * This project reads the X-axis output of an ADXL335 analog accelerometer using
 * the ADC module on the PIC18 microcontroller. Based on the tilt direction, the
 * system controls a servo motor using hardware PWM from Timer2 and CCP2.
 *
 * The system has two operating modes:
 *
 * MODE 1: TRACK
 * - Tilting the accelerometer left moves the servo left.
 * - Tilting the accelerometer right moves the servo right.
 * - Flat position centers the servo.
 *
 * MODE 2: INVERTED
 * - Tilting the accelerometer left moves the servo right.
 * - Tilting the accelerometer right moves the servo left.
 * - Flat position centers the servo.
 *
 * The project also detects an UNSTABLE state. This happens when the
 * accelerometer is moved quickly left and right multiple times. When this
 * happens, the LCD displays UNSTABLE and the servo performs a left-center-right
 * center motion pattern.
 *
 * ============================================================================
 * Hardware Connections:
 * ----------------------------------------------------------------------------
 * RA1 / ANA1  -> ADXL335 accelerometer X-axis output
 * RC2         -> Reset pushbutton using Interrupt-On-Change
 * RC4         -> Mode select pushbutton using Interrupt-On-Change
 * RC3         -> Reset LED output
 * RC5         -> Servo motor PWM signal using CCP2 and Timer2
 * RB0-RB7     -> LCD data pins
 * RD0         -> LCD RS pin
 * RD1         -> LCD EN pin
 *
 * ============================================================================
 * Main Features Used:
 * ----------------------------------------------------------------------------
 * 1. ADC        -> Reads accelerometer analog voltage
 * 2. LCD        -> Displays selected mode and motion state
 * 3. Interrupt  -> Handles reset and mode select buttons
 * 4. Timer2     -> Creates PWM timing base
 * 5. CCP2 PWM   -> Controls servo motor position
 *
 * ============================================================================
 * Program Behavior:
 * ----------------------------------------------------------------------------
 * - On startup, the LCD asks the user to select a mode.
 * - Press mode button once and wait 3 seconds to select TRACK mode.
 * - Press mode button twice within the time window to select INVERTED mode.
 * - Reset button clears the mode, centers the servo, and blinks the LED.
 * - The LCD always shows the current mode on line 1 and state on line 2.
 *
 * ============================================================================
 */

#include "MY_C_CONFIG.h"          // includes PIC config and _XTAL_FREQ
#include <stdio.h>                // standard input/output library
#include <string.h>               // used for strcpy and strcmp
#include <math.h>                 // math library
#include <stdint.h>               // used for uint16_t type

/**************** LCD CONNECTIONS ****************/
#define RS LATD0                  // LCD RS pin connected to RD0
#define EN LATD1                  // LCD Enable pin connected to RD1
#define ldata LATB                // LCD data pins connected to PORTB

#define LCD_Port TRISB            // direction register for LCD data pins
#define LCD_Control TRISD         // direction register for LCD control pins

/**************** OUTPUTS ****************/
#define LED LATCbits.LATC3        // red LED connected to RC3

/**************** INPUTS ****************/
#define RESET_BUTTON PORTCbits.RC2 // reset button connected to RC2
#define MODE_BUTTON  PORTCbits.RC4 // mode select button connected to RC4

/**************** ADC SETTINGS ****************/
#define Vref 5.0                  // ADC reference voltage is 5V

/**************** ACCELEROMETER THRESHOLDS ****************/
#define LEFT_LIMIT   1.45         // below this voltage means tilt left
#define RIGHT_LIMIT  1.85         // above this voltage means tilt right
#define FLAT_LOW     1.60         // lower voltage limit for flat state
#define FLAT_HIGH    1.72         // upper voltage limit for flat state

/**************** UNSTABLE DETECTION ****************/
#define UNSTABLE_TRANSITIONS_REQUIRED  3  // number of fast left/right changes needed
#define UNSTABLE_TIMEOUT_COUNT         35 // timeout before transition count resets

/**************** SERVO DUTY VALUES ****************/
#define SERVO_LEFT_DUTY    31      // PWM duty value for left servo position
#define SERVO_CENTER_DUTY  47      // PWM duty value for center servo position
#define SERVO_RIGHT_DUTY   63      // PWM duty value for right servo position

/**************** MODES ****************/
#define MODE_NONE      0           // no mode selected yet
#define MODE_TRACK     1           // normal tracking mode
#define MODE_INVERTED  2           // inverted tracking mode

int digital;                       // stores ADC digital value
float voltage;                     // stores converted voltage value

char state[17] = "FLAT";           // stores current accelerometer state
char last_state[17] = "";          // stores previous state for LCD update
char last_mode[17] = "";           // stores previous mode for LCD update

unsigned char current_mode = MODE_NONE;       // stores selected mode
unsigned char last_servo_position = 255;      // stores previous servo position

volatile unsigned char reset_flag = 0;        // set when reset interrupt happens
volatile unsigned char mode_button_flag = 0;  // set when mode button interrupt happens

/**************** PROTOTYPES ****************/
void Clock_Init(void);              // sets internal clock

void LCD_Init(void);                // initializes LCD
void LCD_Command(char);             // sends command to LCD
void LCD_Char(char);                // sends one character to LCD
void LCD_String(const char *);      // sends string to LCD
void LCD_String_xy(char, char, const char *); // prints string at LCD position
void LCD_Clear_Line(char row);      // clears one LCD line

void ADC_Init(void);                // initializes ADC
int ADC_Read(void);                 // reads ADC result
float Read_Average_Voltage(void);   // averages ADC readings
void Detect_State(void);            // detects LEFT, RIGHT, FLAT, or UNSTABLE

void IOC_Init(void);                // initializes button interrupts

void TMR2_Initialize(void);         // initializes Timer2 for PWM
void PWM_Output_RC5_Enable(void);   // maps PWM output to RC5
void PWM2_Initialize(void);         // initializes CCP2 PWM
void PWM2_LoadDutyValue(uint16_t dutyValue); // loads PWM duty value

void Servo_Set(unsigned char position); // sets servo position
void Servo_Center(void);            // moves servo to center
void Servo_Left(void);              // moves servo left
void Servo_Right(void);             // moves servo right
void Servo_Unstable(void);          // servo unstable movement

void Reset_System(void);            // handles reset mode
unsigned char Select_Mode(void);    // handles mode selection
void Update_LCD(const char *mode_text, const char *state_text); // updates LCD

/**************** INTERRUPT ****************/
void __interrupt() ISR(void)
{
    if(PIR0bits.IOCIF)              // checks if interrupt-on-change happened
    {
        if(IOCCFbits.IOCCF2)        // checks if RC2 caused interrupt
        {
            reset_flag = 1;         // tells main loop to reset
            IOCCFbits.IOCCF2 = 0;   // clears RC2 interrupt flag
        }

        if(IOCCFbits.IOCCF4)        // checks if RC4 caused interrupt
        {
            mode_button_flag = 1;   // tells program mode button was pressed
            IOCCFbits.IOCCF4 = 0;   // clears RC4 interrupt flag
        }

        PIR0bits.IOCIF = 0;         // clears main IOC flag
    }
}

/**************** MAIN ****************/
void main(void)
{
    Clock_Init();                   // starts internal clock at 4 MHz

    ANSELA = 0x00;                  // makes PORTA digital first
    ANSELB = 0x00;                  // makes PORTB digital
    ANSELC = 0x00;                  // makes PORTC digital
    ANSELD = 0x00;                  // makes PORTD digital

    TRISB = 0x00;                   // PORTB is output for LCD data
    TRISD = 0x00;                   // PORTD is output for LCD control

    TRISCbits.TRISC2 = 1;           // RC2 is reset button input
    TRISCbits.TRISC3 = 0;           // RC3 is LED output
    TRISCbits.TRISC4 = 1;           // RC4 is mode button input
    TRISCbits.TRISC5 = 0;           // RC5 is servo PWM output

    LATB = 0x00;                    // clears PORTB output
    LATD = 0x00;                    // clears PORTD output
    LATC = 0x00;                    // clears PORTC output

    LCD_Init();                     // starts LCD
    ADC_Init();                     // starts ADC

    TMR2_Initialize();              // starts Timer2 setup for PWM
    PWM_Output_RC5_Enable();        // sends CCP2 PWM output to RC5
    PWM2_Initialize();              // starts PWM module

    Servo_Center();                 // centers servo at startup

    IOC_Init();                     // enables button interrupts

    current_mode = Select_Mode();   // asks user to select mode

    while(1)                        // main program loop
    {
        if(reset_flag == 1)         // checks if reset button was pressed
        {
            reset_flag = 0;         // clears reset flag
            Reset_System();         // runs reset routine
            current_mode = Select_Mode(); // asks user to select mode again
        }

        Detect_State();             // reads accelerometer and updates state

        if(current_mode == MODE_TRACK) // checks if track mode is selected
        {
            Update_LCD("Mode: TRACK", state); // displays mode and state

            if(strcmp(state, "LEFT") == 0)    // checks left state
                Servo_Left();                 // moves servo left
            else if(strcmp(state, "RIGHT") == 0) // checks right state
                Servo_Right();                // moves servo right
            else if(strcmp(state, "UNSTABLE") == 0) // checks unstable state
                Servo_Unstable();             // does unstable servo motion
            else
                Servo_Center();               // flat state centers servo
        }
        else if(current_mode == MODE_INVERTED) // checks if inverted mode is selected
        {
            Update_LCD("Mode: INVERTED", state); // displays mode and state

            if(strcmp(state, "LEFT") == 0)    // left tilt in inverted mode
                Servo_Right();                // moves servo right
            else if(strcmp(state, "RIGHT") == 0) // right tilt in inverted mode
                Servo_Left();                 // moves servo left
            else if(strcmp(state, "UNSTABLE") == 0) // unstable state
                Servo_Unstable();             // does unstable servo motion
            else
                Servo_Center();               // flat state centers servo
        }
        else
        {
            current_mode = Select_Mode();     // if no mode, select one
        }

        __delay_ms(40);                       // small delay to stabilize readings
    }
}

/**************** CLOCK ****************/
void Clock_Init(void)
{
    OSCCON1 = 0x60;                // selects HFINTOSC
    OSCFRQ = 0x02;                 // sets oscillator to 4 MHz
}

/**************** STATE DETECTION ****************/
float Read_Average_Voltage(void)
{
    long total = 0;                // stores total ADC readings

    for(int i = 0; i < 12; i++)    // takes 12 ADC samples
    {
        total += ADC_Read();       // adds ADC reading to total
        __delay_ms(3);             // small delay between samples
    }

    digital = total / 12;          // calculates average ADC value
    return digital * (Vref / 4096.0); // converts ADC value to voltage
}

void Detect_State(void)
{
    float avg;                     // stores average voltage
    static unsigned char last_direction = 0;     // previous direction
    static unsigned char transition_count = 0;   // counts left/right changes
    static unsigned char timeout_count = 0;      // timeout counter

    avg = Read_Average_Voltage();  // reads filtered accelerometer voltage

    unsigned char current_direction = 0; // stores current direction number

    if(avg < LEFT_LIMIT)           // checks if voltage is left tilt
        current_direction = 1;     // 1 means left
    else if(avg > RIGHT_LIMIT)     // checks if voltage is right tilt
        current_direction = 2;     // 2 means right
    else if(avg >= FLAT_LOW && avg <= FLAT_HIGH) // checks flat range
        current_direction = 3;     // 3 means flat
    else
        current_direction = last_direction; // holds previous direction if unclear

    if((last_direction == 1 && current_direction == 2) ||
       (last_direction == 2 && current_direction == 1)) // checks left-right switching
    {
        transition_count++;        // counts fast direction changes
        timeout_count = 0;         // resets timeout because movement happened
    }
    else
    {
        if(timeout_count < UNSTABLE_TIMEOUT_COUNT) // checks if still inside time window
            timeout_count++;       // increases timeout counter
        else
            transition_count = 0;  // clears count if movement was too slow
    }

    if(transition_count >= UNSTABLE_TRANSITIONS_REQUIRED) // checks unstable condition
    {
        strcpy(state, "UNSTABLE"); // sets state to unstable
        transition_count = 0;      // resets transition counter
        timeout_count = 0;         // resets timeout counter
    }
    else if(current_direction == 1) // checks left direction
    {
        strcpy(state, "LEFT");     // sets state to left
    }
    else if(current_direction == 2) // checks right direction
    {
        strcpy(state, "RIGHT");    // sets state to right
    }
    else if(current_direction == 3) // checks flat direction
    {
        strcpy(state, "FLAT");     // sets state to flat
    }

    last_direction = current_direction; // saves direction for next loop
    voltage = avg;                      // saves voltage value
}

/**************** MODE SELECT ****************/
unsigned char Select_Mode(void)
{
    unsigned char press_count = 0; // counts mode button presses

    mode_button_flag = 0;          // clears button flag

    strcpy(last_mode, "");         // clears last mode display memory
    strcpy(last_state, "");        // clears last state display memory

    Servo_Center();                // centers servo during selection

    LCD_Command(0x01);             // clears LCD
    LCD_String_xy(1, 0, "Select the mode"); // shows mode select message
    LCD_String_xy(2, 0, "Press button");    // asks user to press button

    while(press_count == 0)        // waits for first button press
    {
        if(reset_flag == 1)        // checks reset during selection
            return MODE_NONE;      // returns no mode

        if(mode_button_flag == 1 || MODE_BUTTON == 1) // checks mode button press
        {
            __delay_ms(120);       // debounce delay

            if(MODE_BUTTON == 1 || mode_button_flag == 1) // confirms press
            {
                mode_button_flag = 0; // clears flag
                press_count = 1;      // first press means track option

                LCD_Command(0x01);    // clears LCD
                LCD_String_xy(1, 0, "Option: TRACK"); // shows track option
                LCD_String_xy(2, 0, "Wait 3 sec");    // tells user to wait

                while(MODE_BUTTON == 1); // waits until button released
                __delay_ms(150);         // debounce after release
            }
        }
    }

    for(int i = 0; i < 30; i++)    // 3 second window for second press
    {
        if(reset_flag == 1)        // checks reset during mode window
            return MODE_NONE;      // returns no mode

        if(mode_button_flag == 1 || MODE_BUTTON == 1) // checks second press
        {
            __delay_ms(120);       // debounce delay

            if(MODE_BUTTON == 1 || mode_button_flag == 1) // confirms second press
            {
                mode_button_flag = 0; // clears button flag
                press_count = 2;      // second press means inverted

                LCD_Command(0x01);    // clears LCD
                LCD_String_xy(1, 0, "Option: INVERT"); // shows inverted option
                LCD_String_xy(2, 0, "Wait 3 sec");     // tells user to wait

                while(MODE_BUTTON == 1); // waits for button release
                __delay_ms(150);         // debounce delay
            }
        }

        __delay_ms(100);           // creates the 3 second selection window
    }

    LCD_Command(0x01);             // clears LCD

    if(press_count == 1)           // one press means track mode
    {
        LCD_String_xy(1, 0, "Selected:"); // shows selected label
        LCD_String_xy(2, 0, "TRACK");     // shows track selected
        Servo_Center();            // centers servo
        __delay_ms(700);           // short display delay
        LCD_Command(0x01);         // clears LCD
        return MODE_TRACK;         // returns track mode
    }
    else                           // two presses means inverted mode
    {
        LCD_String_xy(1, 0, "Selected:"); // shows selected label
        LCD_String_xy(2, 0, "INVERTED");  // shows inverted selected
        Servo_Center();            // centers servo
        __delay_ms(700);           // short display delay
        LCD_Command(0x01);         // clears LCD
        return MODE_INVERTED;      // returns inverted mode
    }
}

/**************** RESET ****************/
void Reset_System(void)
{
    PIE0bits.IOCIE = 0;            // disables IOC while resetting

    current_mode = MODE_NONE;      // clears current mode

    strcpy(last_mode, "");         // clears LCD mode memory
    strcpy(last_state, "");        // clears LCD state memory

    Servo_Center();                // centers servo during reset

    LCD_Command(0x01);             // clears LCD
    LCD_String_xy(1, 0, "Resetting...");  // shows reset message
    LCD_String_xy(2, 0, "Servo Center"); // shows servo message

    for(int i = 0; i < 6; i++)     // 6 changes at 500ms equals about 3 seconds
    {
        LED = !LED;                // toggles LED
        __delay_ms(500);           // delay for blink timing
    }

    LED = 0;                       // makes sure LED is off

    while(RESET_BUTTON == 1);      // waits for reset button release
    __delay_ms(100);               // debounce delay

    IOCCFbits.IOCCF2 = 0;          // clears RC2 flag
    IOCCFbits.IOCCF4 = 0;          // clears RC4 flag
    PIR0bits.IOCIF = 0;            // clears IOC flag

    reset_flag = 0;                // clears reset flag
    mode_button_flag = 0;          // clears mode button flag

    PIE0bits.IOCIE = 1;            // enables IOC again

    LCD_Command(0x01);             // clears LCD before returning
}

/**************** SERVO ****************/
void Servo_Set(unsigned char position)
{
    if(last_servo_position != position) // only updates PWM if position changed
    {
        if(position == 0)          // position 0 means left
            PWM2_LoadDutyValue(SERVO_LEFT_DUTY); // sends left duty value
        else if(position == 1)     // position 1 means center
            PWM2_LoadDutyValue(SERVO_CENTER_DUTY); // sends center duty value
        else if(position == 2)     // position 2 means right
            PWM2_LoadDutyValue(SERVO_RIGHT_DUTY); // sends right duty value

        last_servo_position = position; // saves new servo position
    }
}

void Servo_Left(void)
{
    Servo_Set(0);                  // sets servo left
}

void Servo_Center(void)
{
    Servo_Set(1);                  // sets servo center
}

void Servo_Right(void)
{
    Servo_Set(2);                  // sets servo right
}

void Servo_Unstable(void)
{
    LCD_Clear_Line(2);             // clears LCD line 2
    LCD_String_xy(2, 0, "State: UNSTABLE"); // shows unstable state

    for(int i = 0; i < 3; i++)     // repeats unstable motion pattern
    {
        PWM2_LoadDutyValue(SERVO_LEFT_DUTY); // moves servo left
        last_servo_position = 0;             // updates servo position variable
        __delay_ms(250);                     // waits

        if(reset_flag == 1)                  // checks reset during motion
            break;

        PWM2_LoadDutyValue(SERVO_CENTER_DUTY); // moves servo center
        last_servo_position = 1;                // updates position
        __delay_ms(250);                        // waits

        if(reset_flag == 1)                     // checks reset
            break;

        PWM2_LoadDutyValue(SERVO_RIGHT_DUTY); // moves servo right
        last_servo_position = 2;              // updates position
        __delay_ms(250);                      // waits

        if(reset_flag == 1)                   // checks reset
            break;

        PWM2_LoadDutyValue(SERVO_CENTER_DUTY); // moves servo center again
        last_servo_position = 1;                // updates position
        __delay_ms(250);                        // waits

        if(reset_flag == 1)                     // checks reset
            break;
    }

    Servo_Center();                // centers servo after unstable movement
    strcpy(last_state, "");        // forces LCD to update after unstable state
}

/**************** HARDWARE PWM ****************/
void TMR2_Initialize(void)
{
    T2CLKCON = 0x01;               // Timer2 clock source is FOSC/4
    T2HLT = 0x00;                  // Timer2 free running mode
    T2RST = 0x00;                  // no external reset source

    T2PR = 155;                    // period value for about 20ms PWM
    T2TMR = 0x00;                  // clears Timer2 count

    PIR4bits.TMR2IF = 0;           // clears Timer2 interrupt flag

    T2CON = 0xF0;                  // turns Timer2 on with 1:128 prescaler
}

void PWM_Output_RC5_Enable(void)
{
    PPSLOCK = 0x55;                // unlock PPS step 1
    PPSLOCK = 0xAA;                // unlock PPS step 2
    PPSLOCKbits.PPSLOCKED = 0;     // unlocks PPS registers

    RC5PPS = 0x0A;                 // maps CCP2 PWM output to RC5

    PPSLOCK = 0x55;                // lock PPS step 1
    PPSLOCK = 0xAA;                // lock PPS step 2
    PPSLOCKbits.PPSLOCKED = 1;     // locks PPS registers again
}

void PWM2_Initialize(void)
{
    CCP2CON = 0x8C;                // enables CCP2 in PWM mode
    CCPR2H = 0x00;                 // clears high duty register
    CCPR2L = 0x00;                 // clears low duty register

    CCPTMRS0bits.C2TSEL = 0x1;     // selects Timer2 for CCP2
}

void PWM2_LoadDutyValue(uint16_t dutyValue)
{
    dutyValue &= 0x03FF;           // keeps duty value within 10 bits

    if(CCP2CONbits.FMT)            // checks if left aligned
    {
        dutyValue <<= 6;           // shifts duty for left alignment
        CCPR2H = dutyValue >> 8;   // loads high byte
        CCPR2L = dutyValue;        // loads low byte
    }
    else                           // otherwise right aligned
    {
        CCPR2H = dutyValue >> 8;   // loads high byte
        CCPR2L = dutyValue;        // loads low byte
    }
}

/**************** LCD UPDATE ****************/
void Update_LCD(const char *mode_text, const char *state_text)
{
    if(strcmp(last_mode, mode_text) != 0) // updates mode only if changed
    {
        LCD_Clear_Line(1);         // clears line 1
        LCD_String_xy(1, 0, mode_text); // prints mode
        strcpy(last_mode, mode_text);   // saves mode
    }

    if(strcmp(last_state, state_text) != 0) // updates state only if changed
    {
        LCD_Clear_Line(2);         // clears line 2
        LCD_String_xy(2, 0, "State: "); // prints state label
        LCD_String_xy(2, 7, state_text); // prints state value
        strcpy(last_state, state_text);  // saves state
    }
}

void LCD_Clear_Line(char row)
{
    LCD_String_xy(row, 0, "                "); // writes spaces to clear row
}

/**************** ADC ****************/
void ADC_Init(void)
{
    ADCON0bits.FM = 1;             // right justified ADC result
    ADCON0bits.CS = 1;             // uses internal ADC clock

    TRISAbits.TRISA1 = 1;          // RA1 is input
    ANSELAbits.ANSELA1 = 1;        // RA1 is analog

    ADPCH = 0x01;                  // selects ANA1 channel

    ADCLK = 0x00;                  // ADC clock setting

    ADRESH = 0;                    // clears high result register
    ADRESL = 0;                    // clears low result register

    ADPREL = 0;                    // no ADC precharge low byte
    ADPREH = 0;                    // no ADC precharge high byte

    ADACQL = 0;                    // no extra acquisition low byte
    ADACQH = 0;                    // no extra acquisition high byte

    ADREF = 0x00;                  // uses VDD and VSS as ADC references

    ADCON0bits.ON = 1;             // turns ADC on
}

int ADC_Read(void)
{
    ADCON0bits.GO = 1;             // starts ADC conversion

    while(ADCON0bits.GO);          // waits until conversion finishes

    return ((int)ADRESH << 8) | ADRESL; // combines high and low ADC bytes
}

/**************** IOC ****************/
void IOC_Init(void)
{
    TRISCbits.TRISC2 = 1;          // RC2 input for reset button
    ANSELCbits.ANSELC2 = 0;        // RC2 digital

    TRISCbits.TRISC4 = 1;          // RC4 input for mode button
    ANSELCbits.ANSELC4 = 0;        // RC4 digital

    TRISCbits.TRISC3 = 0;          // RC3 output for LED
    ANSELCbits.ANSELC3 = 0;        // RC3 digital

    TRISCbits.TRISC5 = 0;          // RC5 output for servo PWM
    ANSELCbits.ANSELC5 = 0;        // RC5 digital

    LED = 0;                       // starts LED off

    IOCCPbits.IOCCP2 = 1;          // enables positive edge IOC on RC2
    IOCCNbits.IOCCN2 = 0;          // disables negative edge IOC on RC2

    IOCCPbits.IOCCP4 = 1;          // enables positive edge IOC on RC4
    IOCCNbits.IOCCN4 = 0;          // disables negative edge IOC on RC4

    IOCCFbits.IOCCF2 = 0;          // clears RC2 IOC flag
    IOCCFbits.IOCCF4 = 0;          // clears RC4 IOC flag

    PIR0bits.IOCIF = 0;            // clears main IOC flag

    PIE0bits.IOCIE = 1;            // enables IOC interrupt
    INTCON0bits.GIE = 1;           // enables global interrupts
}

/**************** LCD ****************/
void LCD_Init(void)
{
    __delay_ms(20);                // LCD startup delay

    LCD_Port = 0x00;               // sets LCD data pins as output
    LCD_Control = 0x00;            // sets LCD control pins as output

    LCD_Command(0x01);             // clears LCD
    LCD_Command(0x38);             // 8-bit mode, 2 lines
    LCD_Command(0x0C);             // display on, cursor off
    LCD_Command(0x06);             // auto increment cursor
}

void LCD_Command(char cmd)
{
    ldata = cmd;                   // puts command on data pins
    RS = 0;                        // selects command register
    EN = 1;                        // enable pulse high
    NOP();                         // tiny delay
    EN = 0;                        // enable pulse low
    __delay_ms(2);                 // wait for LCD command
}

void LCD_Char(char dat)
{
    ldata = dat;                   // puts character on data pins
    RS = 1;                        // selects data register
    EN = 1;                        // enable pulse high
    NOP();                         // tiny delay
    EN = 0;                        // enable pulse low
    __delay_ms(1);                 // wait for LCD write
}

void LCD_String(const char *msg)
{
    while(*msg)                    // loops until end of string
    {
        LCD_Char(*msg++);          // prints each character
    }
}

void LCD_String_xy(char row, char pos, const char *msg)
{
    char location;                 // stores LCD memory address

    if(row == 1)                   // checks if row 1
        location = 0x80 + pos;     // row 1 address
    else                           // otherwise row 2
        location = 0xC0 + pos;     // row 2 address

    LCD_Command(location);         // moves cursor
    LCD_String(msg);               // prints message
}
