; Title: 7-Segment Counter
; Author: Hamza Alshamasneh
; Versions:
; V1: 3/20/2026 - Initial Versions
; V2: 3/24/2026 - extended delay time (~0.6s) for slower counting
;-----------------------------------------------------------
; Device: PIC18F47K42
; Compiler: MPLAB X IDE - pic-as Assembler
; Includes: MyConfig.inc, xc.inc
;-------------------------------------------------------------
; Purpose:
; This program controls a 7-segment display using a PIC18F47K42.
; Two push-button switches are used to control counting behavior.
; A)Press and hold Switch A, count UP (0 -> F)
; B) Press and hold Switch B, count DOWN (F -> 0)
; C) Press both switches, reset display to 0
; D) No switch pressed, hold last displayed value
; The display cycles through hexadecimal values (0?F).
;------------------------------------------------------------
; Inputs:
; RB0: Switch A (active HIGH)
; RB1: Switch B (active HIGH)
;---------------------------------------------------------------
; Outputs:
; RD0-RD6: 7-Segment display segments (a-g) (Common Cathode)
;------------------------------------------------------------
; Program Features:
; Uses CALL instruction to implement delay between counts
; Uses indirect addressing for 7-segment lookup
;================================================================

; Initialization
#include "./MyConfig.inc"
#include <xc.inc>

;--------------------------------------------------------------------------------
; Program Constants
NonePressed    equ     0x00      ; no switch pressed
A_Pressed      equ     0x01      ; only switch A pressed
B_Pressed      equ     0x02      ; only switch B pressed
BothPressed    equ     0x03      ; both switches pressed together
COUNT       equ     0x20         ; current count value in HEX
ProcessReg  equ     0x21         ; to store operation
OutDelay    equ     0x22         ; outer delay loop counter
InDelay     equ     0x23         ; inner delay loop counter

;---------------------------------------------------------------------------------
; 7-segment addresses
SEG0 equ 0x30
SEG1 equ 0x31
SEG2 equ 0x32
SEG3 equ 0x33
SEG4 equ 0x34
SEG5 equ 0x35
SEG6 equ 0x36
SEG7 equ 0x37
SEG8 equ 0x38
SEG9 equ 0x39
SEGA equ 0x3A
SEGB equ 0x3B
SEGC equ 0x3C
SEGD equ 0x3D
SEGE equ 0x3E
SEGF equ 0x3F

;---------------------------------------------------------------------------------
; Program section
PSECT absdata,abs,ovrld

ORG     0
        GOTO    _start

ORG     0020H                    ; main program starts at 0x20
_start:
        CALL    InitialSetup     ; to initialize ports and variables
        CALL    ShowCount        ; display initial count value (0)

;--------------------------------------------------------------------------
; Main program loop
MainProgram:
        CALL    SwitchCheck      ; read switches and update ProcessReg

        MOVF    ProcessReg,0,0   ; load ProcessReg into W
        XORLW   BothPressed      ; compare with BothPressed
        BTFSC   STATUS,2         ; if equal (Z=1)
        CALL    ResetCount       ; reset count

        MOVF    ProcessReg,0,0   ; reload ProcessReg
        XORLW   A_Pressed        ; compare with A_Pressed
        BTFSC   STATUS,2         ; if equal
        CALL    COUNT_INC        ; count up

        MOVF    ProcessReg,0,0   ; reload ProcessReg
        XORLW   B_Pressed        ; compare with B_Pressed
        BTFSC   STATUS,2         ; if equal
        CALL    COUNT_DEC        ; count down

        MOVF    ProcessReg,0,0   ; reload ProcessReg
        XORLW   NonePressed      ; compare with NonePressed
        BTFSC   STATUS,2         ; if equal
        CALL    COUNT_STAY       ; hold value

        GOTO    MainProgram      ; loop forever

;==================================================================================
;InitialSetup: Sets Inputs/Outputs, clears registers,and loads 7-seg lookup table
InitialSetup:
        BANKSEL ANSELB
        CLRF    ANSELB,1         ; make PORTB digital
        CLRF    ANSELD,1         ; make PORTD digital

        BANKSEL TRISB
        MOVLW   0x03             ; RB0 & RB1 inputs
        MOVWF   TRISB,1

        BANKSEL TRISD
        MOVLW   0x80             ; RD0?RD6 outputs
        MOVWF   TRISD,1

        BANKSEL LATD
        CLRF    LATD,1           ; clear outputs

        CLRF    COUNT,0          ; initialize count
        CLRF    ProcessReg,0     ; clear operation register
        CLRF    OutDelay,0       ; clear delay registers
        CLRF    InDelay,0

        CALL    Show7Seg         ; load lookup table into RAM
        RETURN                   ; return to main

;==================================================================================
;Show7Seg: Loads 7-segment patterns (0?F) into RAM
Show7Seg:
        MOVLW 0x3F               ; pattern for 0
        MOVWF SEG0,0
        MOVLW 0x06               ; pattern for 1
        MOVWF SEG1,0
        MOVLW 0x5B               ; pattern for 2
        MOVWF SEG2,0
        MOVLW 0x4F               ; pattern for 3
        MOVWF SEG3,0
        MOVLW 0x66               ; pattern for 4
        MOVWF SEG4,0
        MOVLW 0x6D               ; pattern for 5
        MOVWF SEG5,0
        MOVLW 0x7D               ; pattern for 6
        MOVWF SEG6,0
        MOVLW 0x07               ; pattern for 7
        MOVWF SEG7,0
        MOVLW 0x7F               ; pattern for 8
        MOVWF SEG8,0
        MOVLW 0x6F               ; pattern for 9
        MOVWF SEG9,0
        MOVLW 0x77               ; pattern for A
        MOVWF SEGA,0
        MOVLW 0x7C               ; pattern for b
        MOVWF SEGB,0
        MOVLW 0x39               ; pattern for C
        MOVWF SEGC,0
        MOVLW 0x5E               ; pattern for d
        MOVWF SEGD,0
        MOVLW 0x79               ; pattern for E
        MOVWF SEGE,0
        MOVLW 0x71               ; pattern for F
        MOVWF SEGF,0
        RETURN

;================================================================================
;SwitchCheck: Reads switches and decide which operation to perform
SwitchCheck:
        CLRF    ProcessReg,0     ; no switch pressed

        BANKSEL PORTB            ; access PORTB

        BTFSC   PORTB,0          ; check RB0
        GOTO    SW_A             ; if high, A pressed

        BTFSC   PORTB,1          ; check RB1
        GOTO    SW_B             ; if high, B pressed

        RETURN                   ; none pressed

SW_A:
        BTFSC   PORTB,1          ; check if both pressed
        GOTO    A_AND_B

        MOVLW   A_Pressed        ; only A
        MOVWF   ProcessReg,0
        RETURN

SW_B:
        MOVLW   B_Pressed        ; only B
        MOVWF   ProcessReg,0
        RETURN

A_AND_B:
        MOVLW   BothPressed      ; both pressed
        MOVWF   ProcessReg,0
        RETURN

;==============================================================================
;ResetCount: Resets counter to 0 and display it
ResetCount:
        CLRF    COUNT,0          ; reset count
        CALL    ShowCount        ; update display
        RETURN

;=========================================================================
;COUNT_INC: Increments count
COUNT_INC:
        CALL    DelayLoops
	CALL    DelayLoops	; call delay 3 time to get 0.57s delay
	CALL    DelayLoops
        INCF    COUNT,1,0        ; increment

        MOVF    COUNT,0,0        ; check if overflow
        XORLW   0x10
        BTFSC   STATUS,2
        CLRF    COUNT,0          ; reset to 0

        CALL    ShowCount        ; display new value
        RETURN

;=============================================================================
;COUNT_DEC: Decrements count
COUNT_DEC:
        CALL    DelayLoops
	CALL    DelayLoops	; call delay 3 time to get 0.57s delay
	CALL    DelayLoops

        MOVF    COUNT,0,0        ; check if 0
        XORLW   0x00
        BTFSC   STATUS,2
        GOTO    LOAD_F           ; set to F

        DECF    COUNT,1,0        ; decrement
        CALL    ShowCount
        RETURN

LOAD_F:
        MOVLW   0x0F             ; load F
        MOVWF   COUNT,0
        CALL    ShowCount
        RETURN

;============================================================================
;COUNT_STAY: Holds current value on display
COUNT_STAY:
        CALL    ShowCount        ; just refresh display
        RETURN

;==============================================================================
;ShowCount: Displays COUNT using indirect addressing
ShowCount:
        MOVLW   0x30             ; table start
        MOVWF   FSR0L,0
        CLRF    FSR0H,0

        MOVF    COUNT,0,0
        ADDWF   FSR0L,1,0

        MOVF    INDF0,0,0        ; read pattern
        BANKSEL LATD
        MOVWF   LATD,1           ; output to display

        RETURN

;===========================================================================
;DelayLoops: Create visible delay of 0.19s
DelayLoops:
        MOVLW   0xFF             ; outer delay start
        MOVWF   OutDelay,0

OuterDelay:
        MOVLW   0xFF             ; inner delay start
        MOVWF   InDelay,0

InnerDelay:
        DECFSZ  InDelay,1,0      ; inner loop
        GOTO    InnerDelay

        DECFSZ  OutDelay,1,0     ; outer loop
        GOTO    OuterDelay

        RETURN

        END
