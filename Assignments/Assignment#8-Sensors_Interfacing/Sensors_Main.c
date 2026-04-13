/*
 * Title:   Sensors Interfacing
 * Author:  Hamza Alshamasneh
 *
 * Version History:
 * V1.0 : 4/11/2026 - Initial C version of the project
 *
 * ------------------------------------------------------------
 * Device:   PIC18F47K42
 * Compiler: MPLAB X IDE + XC8 v3.10
 * Includes: My_C_Config.h, Initialization.h, Functions.h
 * ------------------------------------------------------------
 *
 * Purpose:
 * This program controls the sensor interfacing system using a
 * PIC18F47K42 microcontroller.
 *
 * The system uses:
 * - Two photoresistor circuits as touchless input sensors
 * - One emergency push button switch using interrupt
 * - One 7-segment display for user feedback
 * - One system LED to indicate power / system ON
 * - One relay module connected to a buzzer
 *
 * Program Operation:
 * A) The system starts by initializing GPIO, interrupt settings,
 *    relay state, and display state.
 * B) The system waits for the first touchless input from PR1.
 * C) After the first input is completed, the system waits for
 *    the second touchless input from PR2.
 * D) The entered input sequence is compared with the stored
 *    secret code.
 * E) If the entered code is correct, the correct-code action
 *    is executed.
 * F) If the entered code is wrong, the relay is activated and
 *    the buzzer turns on for a fixed amount of time.
 * G) If the emergency switch is pressed, an interrupt occurs
 *    and the emergency melody is generated inside the ISR.
 *
 * Inputs:
 * RA1 : PR1 touchless sensor input
 * RA2 : PR2 touchless sensor input
 * RB0 : Emergency switch input
 *
 * Outputs:
 * RA0      : System LED
 * RB1      : Relay module control
 * RD0-RD6  : 7-segment display outputs
 */

#include "My_C_Config.h"
#include "Initialization.h"
#include "Functions.h"

/* =========================
   GLOBAL VARIABLES
   ========================= */

volatile uint8_t Check_Emergency_SW = 0; // this flag is raised by the ISR so the main loop knows an emergency happened

uint8_t PR1_Count = 0;                   // keeps track of how many valid times PR1 was activated
uint8_t PR2_Count = 0;                   // keeps track of how many valid times PR2 was activated

uint16_t PR1_DONE = 0;                   // counts how long PR1 has been idle after the first trigger
uint16_t PR2_DONE = 0;                   // counts how long PR2 has been idle after the first trigger

uint8_t PR1_Debounce = 0;                // helps ignore repeated PR1 readings caused by unstable transitions
uint8_t PR2_Debounce = 0;                // helps ignore repeated PR2 readings caused by unstable transitions

bool PR1Prev = false;                    // stores the previous PR1 state so the code can detect a new edge
bool PR2Prev = false;                    // stores the previous PR2 state so the code can detect a new edge

system_state_t SystemState = waitFor_PR1; // when the system starts, it always expects the first input from PR1

/* =========================
   7-SEGMENT LOOKUP TABLE
   bit0=A, bit1=B, ... bit6=G
   ========================= */

static const uint8_t Seg7_Digits[10] =
{
    0x3F, /* 0 */   // segment pattern used to display digit 0
    0x06, /* 1 */   // segment pattern used to display digit 1
    0x5B, /* 2 */   // segment pattern used to display digit 2
    0x4F, /* 3 */   // segment pattern used to display digit 3
    0x66, /* 4 */   // segment pattern used to display digit 4
    0x6D, /* 5 */   // segment pattern used to display digit 5
    0x7D, /* 6 */   // segment pattern used to display digit 6
    0x07, /* 7 */   // segment pattern used to display digit 7
    0x7F, /* 8 */   // segment pattern used to display digit 8
    0x6F  /* 9 */   // segment pattern used to display digit 9
};

/* =========================
   INTERRUPT SERVICE ROUTINE
   Emergency switch on RB0
   ========================= */

void __interrupt(irq(IRQ_IOC), base(8)) ISR_IOC(void) // this ISR runs immediately when RB0 changes and triggers the emergency input
{
    if (PIR0bits.IOCIF && IOCBFbits.IOCBF0) // makes sure the interrupt really came from the RB0 interrupt-on-change source
    {
        IOCBFbits.IOCBF0 = 0;               // clears the RB0-specific interrupt flag so the same interrupt can be detected again later
        PIR0bits.IOCIF = 0;                 // clears the global IOC interrupt flag to complete interrupt servicing

        Check_Emergency_SW = 1;             // tells the main loop that an emergency button press happened
        SystemState = Emergency_Pressed;    // forces the program state into the emergency condition

        // The relay is switched ON and OFF here to produce the emergency buzzer pattern.
        for (uint8_t i = 0; i < 2; i++)     // first group of emergency pulses
        {
            RELAY_On();                     // energizes the relay so the buzzer sounds
            __delay_ms(150);                // keeps the buzzer active long enough to hear the first tone
            RELAY_Off();                    // silences the buzzer between tones
            __delay_ms(100);                // creates spacing so the sound pattern is distinct
        }

        __delay_ms(250);                    // larger pause between the first and second melody groups

        for (uint8_t i = 0; i < 1; i++)     // second group of emergency pulses
        {
            RELAY_On();                     // turns the buzzer back on for the second part of the pattern
            __delay_ms(350);                // makes these tones longer than the first group
            RELAY_Off();                    // turns the buzzer off again
            __delay_ms(100);                // leaves a short gap before the next pulse
        }
    }
}

/* =========================
   MAIN
   ========================= */

void main(void)                             // program starts here after reset
{
    SYSTEM_Initialize();                    // prepares the ports, interrupt system, relay, and display
    Reset_To_Start();                       // clears any old values and makes sure the program begins from the first expected input
    SYS_LED_On();                           // turns on the status LED so the user knows the system is running

    while (1)                               // the main loop keeps the system running forever
    {
        if (Check_Emergency_SW)             // if the interrupt raised the emergency flag, handle that first before doing anything else
        {
            EmergencyOn();                  // clears the emergency condition and resets the program to a safe starting point
            continue;                       // skips the normal code-entry process for this loop cycle
        }

        Update_SensorsAndCounts();          // reads the photoresistors and updates any valid user input counts
        Process_System();                   // checks the current state and decides what should happen next

        __delay_ms(LOOP_DELAY_MS);          // slows the loop slightly so the timing logic works in controlled steps
    }
}

/* =========================
   INITIALIZATION
   ========================= */

void SYSTEM_Initialize(void)                // prepares the whole system before the main loop begins
{
    GPIO_Initialize();                      // sets up pin directions and ensures the used pins behave digitally
    Emergency_Initialize();                 // enables the emergency switch interrupt system

    RELAY_Off();                            // starts with relay OFF so the buzzer does not activate immediately at power-up
    SEG_Clear();                            // clears the 7-segment so no random digit appears at startup
}

void GPIO_Initialize(void)                  // configures the hardware pins based on their role in the circuit
{
    ANSELA = 0x00;                          // disables analog mode on PORTA so sensor pins act like digital inputs
    ANSELB = 0x00;                          // disables analog mode on PORTB so the switch and relay pins work digitally
    ANSELD = 0x00;                          // disables analog mode on PORTD so the display outputs behave correctly

    SYS_LED_TRIS = 0;                       // makes the LED pin an output because the PIC must drive it
    PR1_TRIS = 1;                           // makes PR1 an input because the PIC only reads its voltage level
    PR2_TRIS = 1;                           // makes PR2 an input because the PIC only reads its voltage level
    EMG_SW_TRIS = 1;                        // makes the emergency switch pin an input because it is an external event source
    RELAY_TRIS = 0;                         // makes the relay control pin an output because the PIC drives the relay module
    SEG_PORT_TRIS = 0x00;                   // makes all display lines outputs so the PIC can send digit patterns

    LATA = 0x00;                            // clears output latches on PORTA so nothing starts in an unknown state
    LATB = 0x00;                            // clears output latches on PORTB for the same reason
    LATD = 0x00;                            // clears output latches on PORTD so the display starts blank

    WPUBbits.WPUB0 = 1;                     // enables the weak pull-up on RB0 so the switch stays at a stable HIGH level when not pressed
}

void Emergency_Initialize(void)             // configures the interrupt behavior for the emergency switch
{
    IOCBNbits.IOCBN0 = 1;                   // tells the PIC to trigger an interrupt when RB0 goes from HIGH to LOW, which happens when the switch is pressed
    IOCBPbits.IOCBP0 = 0;                   // disables rising-edge interrupt because only the press event matters here

    IOCBFbits.IOCBF0 = 0;                   // clears any old interrupt flag on RB0 before the system begins
    PIR0bits.IOCIF = 0;                     // clears the global interrupt-on-change flag for a clean start

    PIE0bits.IOCIE = 1;                     // enables the interrupt-on-change source so RB0 events can actually trigger the ISR

    INTCON0bits.GIEH = 1;                   // enables high-priority interrupts so the emergency can interrupt normal execution immediately
    INTCON0bits.GIEL = 1;                   // enables low-priority interrupts too, completing the interrupt system setup
}

/* =========================
   BASIC OUTPUT FUNCTIONS
   ========================= */

void SYS_LED_On(void)                       // turns on the system indicator LED
{
    SYS_LED_LAT = 1;                        // drives the LED pin high so current can flow through the LED circuit
}

void SYS_LED_Off(void)                      // turns off the system indicator LED
{
    SYS_LED_LAT = 0;                        // removes the active output from the LED pin so the LED stops glowing
}

void RELAY_On(void)                         // activates the relay module
{
    RELAY_LAT = 0;                          // because the module is active-low, writing 0 energizes the relay input
}

void RELAY_Off(void)                        // deactivates the relay module
{
    RELAY_LAT = 1;                          // for active-low hardware, writing 1 returns the relay to its inactive state

}

void Seg7_Display(uint8_t digit)            // sends one decimal digit to the 7-segment display
{
    uint8_t pattern = 0x00;                 // starts with all segments off until a valid digit pattern is selected

    if (digit <= 9U)                        // makes sure only valid decimal digits use the lookup table
    {
        pattern = Seg7_Digits[digit];       // selects the correct bit pattern for the chosen digit
    }

    SEG_PORT_LAT = pattern;                 // sends the normal pattern directly because the display is treated as common cathode
}

void SEG_Clear(void)                        // blanks the 7-segment display
{

    SEG_PORT_LAT = 0x00;                    // with common cathode, writing zeros turns all segments off
}

/* =========================
   INPUT FUNCTIONS
   ========================= */

bool PR1_IsActive(void)                     // checks whether PR1 should currently be considered "triggered"
{
    return (PR1_PORT == 0);                 // returns true when PR1 reads LOW because the sensor circuit was defined as active-low
}

bool PR2_IsActive(void)                     // checks whether PR2 should currently be considered "triggered"
{
    return (PR2_PORT == 0);                 // returns true when PR2 reads LOW because the same input logic is used for both sensors
}

/* =========================
   RESET
   ========================= */

void Reset_InputData(void)                  // clears all user-entry values and helper timers
{
    PR1_Count = 0;                          // removes any previous first-digit count so the next code entry starts fresh
    PR2_Count = 0;                          // removes any previous second-digit count for the same reason

    PR1_DONE = 0;                           // clears the idle timer used to decide when PR1 entry is finished
    PR2_DONE = 0;                           // clears the idle timer used to decide when PR2 entry is finished

    PR1_Debounce = 0;                       // clears any debounce delay left from earlier PR1 triggers
    PR2_Debounce = 0;                       // clears any debounce delay left from earlier PR2 triggers

    PR1Prev = false;                        // resets the remembered PR1 state so edge detection starts cleanly
    PR2Prev = false;                        // resets the remembered PR2 state so edge detection starts cleanly
}

void Reset_To_Start(void)                   // returns the whole system to its normal starting condition
{
    Reset_InputData();                      // first clears all counters and helper values from the previous cycle
    SystemState = waitFor_PR1;              // then puts the state machine back to waiting for the first sensor input
    SEG_Clear();                            // clears the display so no old digit remains visible to the user
    RELAY_Off();                            // makes sure the buzzer output is off before a new attempt begins
}

/* =========================
   SENSOR / COUNTING LOGIC
   ========================= */

void Update_SensorsAndCounts(void)          // reads the sensors and updates valid counts only when a new trigger happens
{
    bool pr1_active = PR1_IsActive();       // gets the current active/inactive condition of PR1
    bool pr2_active = PR2_IsActive();       // gets the current active/inactive condition of PR2

    if (PR1_Debounce > 0)                   // if PR1 is still in its debounce period
    {
        PR1_Debounce--;                     // count down until PR1 is allowed to be accepted again
    }

    if (PR2_Debounce > 0)                   // if PR2 is still in its debounce period
    {
        PR2_Debounce--;                     // count down until PR2 is allowed to be accepted again
    }

    switch (SystemState)                    // counting rules depend on which part of the code entry the user is in
    {
        case waitFor_PR1:                   // first stage: only PR1 input is meaningful here
        {
            if (pr1_active && !PR1Prev && (PR1_Debounce == 0)) // counts only a new PR1 edge, not a sensor being held continuously
            {
                if (PR1_Count < 4U)         // stops the value from growing beyond the allowed range
                {
                    PR1_Count++;            // stores one more valid PR1 trigger
                    Seg7_Display(PR1_Count);// shows the current first-digit count so the user gets feedback
                }

                PR1_DONE = 0;               // resets the timeout counter because a fresh PR1 trigger just happened
                PR1_Debounce = DEBOUNCE_TICKS; // starts the debounce delay so noise or holding does not count again immediately
            }

            if (PR1_Count > 0U)             // once at least one PR1 entry exists, begin timing the pause after it
            {
                PR1_DONE++;                 // each loop increases the idle counter until the pause is long enough
            }

            break;                          // leaves this state after finishing the first-input handling
        }

        case STATE_WAIT_PR2:                // second stage: now only PR2 input is meaningful
        {
            if (pr2_active && !PR2Prev && (PR2_Debounce == 0)) // counts only a new PR2 edge, not a held condition
            {
                if (PR2_Count < 4U)         // keeps the second digit within the same allowed maximum
                {
                    PR2_Count++;            // stores one more valid PR2 trigger
                    Seg7_Display(PR2_Count);// shows the current second-digit count on the display
                }

                PR2_DONE = 0;               // resets the second-input timeout because a new PR2 trigger just happened
                PR2_Debounce = DEBOUNCE_TICKS; // starts the debounce timer to prevent accidental repeated counts
            }

            if (PR2_Count > 0U)             // once PR2 starts being entered, begin timing the pause after it
            {
                PR2_DONE++;                 // this allows the code to decide when the second digit is finished
            }

            break;                          // leaves this state after handling the second-input logic
        }

        default:                            // in all other states, sensors are not counted as code-entry inputs
            break;
    }

    PR1Prev = pr1_active;                   // stores current PR1 condition so next loop can detect a change properly
    PR2Prev = pr2_active;                   // stores current PR2 condition so next loop can detect a change properly
}

/* =========================
   MAIN STATE MACHINE
   ========================= */

void Process_System(void)                   // controls the overall behavior by moving between the program states
{
    switch (SystemState)                    // decides what should happen based on the current operating state
    {
        case waitFor_PR1:                   // first stage: system is waiting for the user to finish PR1 entry
        {
            /*
             * The first number is considered complete only after
             * PR1 has been triggered at least once and then stays idle
             * long enough to reach the timeout.
             */
            if ((PR1_Count > 0U) && (PR1_DONE >= DIGIT_DONE_TIMEOUT_TICKS)) // checks whether the first digit entry is finished
            {
                SystemState = STATE_WAIT_PR2; // once the first digit is done, move to waiting for the second digit
                PR2_DONE = 0;                 // clears PR2 timeout counter so the second stage starts fresh
            }
            break;                            // end of first-state processing
        }

        case STATE_WAIT_PR2:                  // second stage: system is waiting for the user to finish PR2 entry
        {
            if ((PR2_Count > 0U) && (PR2_DONE >= DIGIT_DONE_TIMEOUT_TICKS)) // checks whether the second digit entry is finished
            {
                SystemState = STATE_CHECK_CODE; // both digits are ready, so move on to compare them with the secret code
            }
            break;                            // end of second-state processing
        }

        case STATE_CHECK_CODE:                // compare what the user entered against the stored secret code
        {
            if ((PR1_Count == SECRET_CODE_PR1) && (PR2_Count == SECRET_CODE_PR2)) // both digits must match exactly for success
            {
                SystemState = Correct_Secret_Code; // marks the code as correct so the success action can run
            }
            else
            {
                SystemState = Wrong_Secret_Code;   // any mismatch sends the program to the wrong-code action
            }
            break;                                 // end of code-check state
        }

        case Correct_Secret_Code:                 // state reached when the entered code is correct
        {
            Handle_CorrectCode();                 // performs the success behavior for the current design
            Reset_To_Start();                     // clears everything so the next attempt starts from the beginning
            break;                                // end of success state
        }

        case Wrong_Secret_Code:                   // state reached when the entered code is wrong
        {
            Handle_WrongCode();                   // performs the failure behavior, which is the buzzer through the relay
            Reset_To_Start();                     // resets all values after the warning sound finishes
            break;                                // end of failure state
        }

        case Emergency_Pressed:                   // state entered after the ISR marks an emergency event
        {
            EmergencyOn();                        // handles the software-side emergency cleanup after the melody
            break;                                // end of emergency state
        }

        default:                                  // safety case in case the state ever becomes invalid unexpectedly
        {
            Reset_To_Start();                     // safest response is to reset to the known idle state
            break;                                // end of default state
        }
    }
}

/* =========================
   ACTION HANDLERS
   ========================= */

void Handle_CorrectCode(void)                // runs when the correct code is entered
{
    /*
     * In this version, the correct code does not activate the buzzer.
     * The display is simply cleared and the system resets afterwards.
     */
    SEG_Clear();                             // removes the last entered digit so the display returns to a neutral state
}

void Handle_WrongCode(void)                  // runs when the entered code is incorrect
{
    RELAY_On();                              // turns on the relay so the buzzer receives power
    DelayMs_Blocking(WRONG_CODE_ON_MS);      // keeps the buzzer active long enough for the user to clearly hear the warning
    RELAY_Off();                             // turns the buzzer back off after the warning duration ends

    SEG_Clear();                             // clears the display so the next attempt starts without leftover numbers
}

void EmergencyOn(void)                       // runs after the interrupt-driven emergency behavior has finished
{
    Check_Emergency_SW = 0;                  // clears the emergency flag so the main loop stops treating the event as active
    Reset_To_Start();                        // resets the whole program so it returns to normal operation from the beginning
}

/* =========================
   SIMPLE BLOCKING DELAY
   ========================= */

void DelayMs_Blocking(uint16_t ms)           // creates a manual blocking delay one millisecond at a time
{
    while (ms--)                             // repeats until the requested number of milliseconds has fully passed
    {
        __delay_ms(1);                       // each loop adds one millisecond, giving a simple controlled total delay
    }
}
