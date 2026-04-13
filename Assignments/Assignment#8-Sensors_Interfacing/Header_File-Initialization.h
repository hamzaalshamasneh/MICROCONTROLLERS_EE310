/*
 * File:   Initialization.h
 * Author: Hamza Alshamasneh
 *
 * This file collects the main hardware definitions and software constants
 * in one place. It makes the rest of the program easier to read because
 * pin names, timing values, and state names are all organized here.
 */

#ifndef Initialization_H
#define Initialization_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

/* =========================
   PIN DEFINITIONS
   ========================= */

/* System LED pin:
 * this LED is used as a visual indicator that the system is powered and operating.
 */
#define SYS_LED_LAT     LATAbits.LATA0      // writes a 1 or 0 to the LED output pin
#define SYS_LED_TRIS    TRISAbits.TRISA0    // decides whether that LED pin behaves as input or output

/* Photoresistor input pins:
 * these two pins receive the sensor voltages from the two LDR divider circuits.
 */
#define PR1_PORT        PORTAbits.RA1       // reads the actual logic level seen on PR1
#define PR1_TRIS        TRISAbits.TRISA1    // configures PR1 pin direction

#define PR2_PORT        PORTAbits.RA2       // reads the actual logic level seen on PR2
#define PR2_TRIS        TRISAbits.TRISA2    // configures PR2 pin direction

/* Emergency switch pin:
 * this push button is used as the interrupt source for the emergency condition.
 */
#define EMG_SW_PORT     PORTBbits.RB0       // reads whether the emergency switch is pressed or not
#define EMG_SW_TRIS     TRISBbits.TRISB0    // configures the emergency pin as an input

/* Relay control pin:
 * this pin drives the input side of the relay module.
 */
#define RELAY_LAT       LATBbits.LATB1      // writes the control signal that turns the relay module ON or OFF
#define RELAY_TRIS      TRISBbits.TRISB1    // configures the relay control pin as output

/* 7-segment display port:
 * the display is connected to PORTD, so the whole port is used to output segment patterns.
 */
#define SEG_PORT_LAT    LATD                // sends the selected segment pattern to the display
#define SEG_PORT_TRIS   TRISD               // makes the display pins outputs

/* =========================
   LOGIC OPTIONS
   ========================= */

/*
 * The sensor circuit is wired so that covering the sensor produces a LOW signal.
 * That means the software should treat logic 0 as the active event.
 */
#define SENSOR_ACTIVE_LOW   1               // tells the code that sensor activation happens when the pin reads LOW

/*
 * The relay module is active-low, which means sending a LOW control signal turns it ON.
 * This keeps the relay functions reusable even if the hardware changes later.
 */
#define RELAY_ACTIVE_LOW    1               // tells relay functions what logic level means ON

/*
 * The 7-segment is being treated as common cathode,
 * so normal segment patterns can be sent directly without inversion.
 */
#define SEG7_TYPE  1                        // this lets the display function know what output style to use

/* =========================
   TIMING / LOGIC CONSTANTS
   ========================= */

/*
 * These two values are the actual secret code:
 * PR1 must be triggered 3 times first,
 * then PR2 must be triggered 2 times.
 */
#define SECRET_CODE_PR1         3U          // expected count from the first sensor
#define SECRET_CODE_PR2         2U          // expected count from the second sensor

/*
 * These timing values control how fast the main loop runs,
 * how long the user is allowed to pause before a digit is considered finished,
 * and how long the buzzer sounds for a wrong code.
 */
#define LOOP_DELAY_MS           10U         // one loop delay unit, used to pace the main loop
#define DIGIT_DONE_TIMEOUT_MS   1500U       // if no new trigger happens for this long, the digit is considered complete
#define WRONG_CODE_ON_MS        2000U       // how long the buzzer stays active for an incorrect code
#define UNLOCK_ON_MS            2000U       // kept here from the original design, even if not currently used
#define DEBOUNCE_MS             60U         // helps ignore repeated false triggers caused by unstable sensor transitions

/*
 * Since the loop works in steps of LOOP_DELAY_MS,
 * these formulas convert milliseconds into loop counts.
 */
#define DIGIT_DONE_TIMEOUT_TICKS   (DIGIT_DONE_TIMEOUT_MS / LOOP_DELAY_MS) // number of loops that equal the digit-finished timeout
#define DEBOUNCE_TICKS             (DEBOUNCE_MS / LOOP_DELAY_MS)           // number of loops used as debounce duration

/* =========================
   APPLICATION STATES
   ========================= */

/*
 * The program works like a state machine.
 * Each state describes what the system is currently waiting for or doing.
 */
typedef enum
{
    waitFor_PR1 = 0,            // system is waiting for the first code entry using PR1
    STATE_WAIT_PR2,             // system is waiting for the second code entry using PR2
    STATE_CHECK_CODE,           // system has both inputs and is now comparing them with the secret code
    Correct_Secret_Code,        // the entered values matched the secret code
    Wrong_Secret_Code,          // the entered values did not match the secret code
    Emergency_Pressed           // emergency switch was pressed and emergency condition is active
} system_state_t;

/* =========================
   GLOBAL FLAGS / VARIABLES
   ========================= */

/*
 * These variables are declared here as extern so the source file can define them,
 * while the other files can still access them.
 */

extern volatile uint8_t Check_Emergency_SW; // becomes 1 when the interrupt says the emergency switch was pressed

extern uint8_t PR1_Count;                   // stores how many valid PR1 triggers happened
extern uint8_t PR2_Count;                   // stores how many valid PR2 triggers happened

extern uint16_t PR1_DONE;                   // counts idle loop time after PR1 input to decide when first digit is finished
extern uint16_t PR2_DONE;                   // counts idle loop time after PR2 input to decide when second digit is finished

extern uint8_t PR1_Debounce;                // temporary countdown used to stop PR1 from being counted too quickly
extern uint8_t PR2_Debounce;                // temporary countdown used to stop PR2 from being counted too quickly

extern bool PR1Prev;                        // remembers the previous PR1 state so the code can detect a new edge
extern bool PR2Prev;                        // remembers the previous PR2 state so the code can detect a new edge

extern system_state_t SystemState;          // stores the current state of the full program

#endif
