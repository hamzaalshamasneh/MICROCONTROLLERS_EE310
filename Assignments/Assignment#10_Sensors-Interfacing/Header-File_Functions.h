/*
 * File:   Functions.h
 * Author: Hamza Alshamasneh
 *
 * This file is used to declare all the functions before they are
 * used in the main source file.
 */

#ifndef Functions_H
#define Functions_H

#include "My_C_Config.h"
#include "Initialization.h"

/* =========================
   FUNCTION PROTOTYPES
   ========================= */

/* These functions prepare the microcontroller and its pins before the program starts working. */
void SYSTEM_Initialize(void);        // prepares the full system so all hardware starts in a known condition
void GPIO_Initialize(void);          // sets which pins are inputs and which are outputs
void Emergency_Initialize(void);     // enables and prepares the emergency interrupt on the switch pin

/* These functions control what is shown on the 7-segment display. */
void Seg7_Display(uint8_t digit);    // sends the pattern of a digit to the 7-segment so the user sees the entered count
void SEG_Clear(void);                // turns off all display segments so no number remains shown

/* These functions control the relay output. */
void RELAY_On(void);                 // activates the relay so the buzzer/load can be powered
void RELAY_Off(void);                // deactivates the relay so the buzzer/load is turned off

/* These functions control the system status LED. */
void SYS_LED_On(void);               // turns on the LED to show the system is powered and running
void SYS_LED_Off(void);              // turns off the LED if needed

/* These functions check whether the photoresistors are currently active. */
bool PR1_IsActive(void);             // checks the first sensor based on the selected input logic
bool PR2_IsActive(void);             // checks the second sensor based on the selected input logic

/* These functions reset the system data when a cycle finishes or an emergency happens. */
void Reset_InputData(void);          // clears all counts, timers, and previous sensor states
void Reset_To_Start(void);           // brings the whole program back to the starting state

/* These are the main program logic functions used repeatedly in the loop. */
void Update_SensorsAndCounts(void);  // reads the sensors and updates counts using debounce and edge detection
void Process_System(void);           // moves the program from one operating state to another

/* These functions decide what the system does after a specific event happens. */
void Handle_CorrectCode(void);       // action taken when the entered code matches the secret code
void Handle_WrongCode(void);         // action taken when the entered code is incorrect
void EmergencyOn(void);              // action taken after the emergency interrupt has occurred

/* This function creates a blocking delay in milliseconds. */
void DelayMs_Blocking(uint16_t ms);  // keeps the CPU waiting for a chosen number of milliseconds
#endif
