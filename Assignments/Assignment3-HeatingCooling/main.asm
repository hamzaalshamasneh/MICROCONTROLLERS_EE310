; Title: Heating and Cooling Control System
;------------------------------------------------------------
; Purpose:
; This program implements a heating and cooling control system
; using assembly. The program reads the desired temperature (refTemp)
; from the keypad and the actual room temperature (measuredTemp)
; from the temperature sensor. It compares both values and controls
; the heating or cooling system through PORTD output registers.
;
; Dependencies:
; PIC18F46K42 device include file and MyConfig.inc file
;
; Compiler: MPLAB X IDE - XC8 PIC Assembler
;
; Author: Hamza Alshamasneh
;
; Outputs:
; PORTD.1 -> Heating system (Hot Air Blower / LED)
; PORTD.2 -> Cooling system (Cooling Fan / LED)
;
; Inputs:
; Keypad: Desired temperature input (refTemp)
; Temperature Sensor: Measured room temperature input (measuredTemp)
;
; Registers Used:
; refTemp: REG 0x20
; measuredTemp REG: 0x21
; contReg: REG 0x22
;
; Decimal Storage:
; refTemp digits:
;   REG 0x60: low byte digit
;   REG 0x61: high byte digit
;   REG 0x62: upper byte digit
; measuredTemp digits:
;   REG 0x70: low byte digit
;   REG 0x71: high byte digit
;   REG 0x72: upper byte digit
;
; Program Operation:
; If measuredTemp > refTemp, contReg = 2 and PORTD.2 turns ON
; for the cooling system.
; If measuredTemp < refTemp, contReg = 1 and PORTD.1 turns ON
; for the heating system.
; If measuredTemp = refTemp, contReg = 0 and both outputs stay OFF.
;
; Input Ranges:
; refTemp can range from +10 C to +50 C
; measuredTemp can range from -10 C to +60 C
;
; Versions:
; V1.0: 2026-03-06 - First version
;------------------------------------------------------------
PROCESSOR 18F46K42

#include "MyConfig.inc"              ; include header file
#include <xc.inc>                    ; include device definitions

;-------------------------
; PROGRAM INPUTS
#define measuredTempInput    -5      ; test measured temperature input
#define refTempInput         15     ; test reference temperature input

;-------------------------
; OUTPUT DEFINITIONS
#define HEAT_ON  PORTD,1             ; heating output bit
#define COOL_ON  PORTD,2             ; cooling output bit

;-------------------------
; PROGRAM CONSTANTS
refTemp         equ     0x20         ; register for reference temperature
measuredTemp    equ     0x21         ; register for measured temperature
contReg         equ     0x22         ; control register
tempReg         equ     0x23         ; temporary working register

refLow          equ     0x60         ; refTemp low digit
refHigh         equ     0x61         ; refTemp high digit
refUpper        equ     0x62         ; refTemp upper digit

measLow         equ     0x70         ; measuredTemp low digit
measHigh        equ     0x71         ; measuredTemp high digit
measUpper       equ     0x72         ; measuredTemp upper digit

REF_MIN         equ     10           ; minimum reference temperature
REF_MAX         equ     50           ; maximum reference temperature
MEAS_MAX        equ     60           ; maximum measured temperature
MEAS_MIN        equ     0xF6         ; minimum measured temperature (-10)

;---------------------------
; Program section
PSECT absdata,abs,ovrld              ; absolute code section

ORG     0
        GOTO    _start               ; jump to program start

ORG     0020H                        ; start main code at 0x20
_start:
        clrf    BSR,0                ; clear bank select register
        clrf    TRISD,0              ; make PORTD pins outputs
        clrf    PORTD,0              ; clear PORTD outputs

        movlw   refTempInput         ; load reference temperature value
        movwf   refTemp,0            ; store into refTemp register

        movlw   measuredTempInput    ; load measured temperature value
        movwf   measuredTemp,0       ; store into measuredTemp register

        clrf    contReg,0            ; clear control register

        ; Convert refTemp to decimal digits
        movf    refTemp,0,0          ; move refTemp into W
        movwf   tempReg,0           ; copy W into tempWork
        call    refConvert     ; convert refTemp digits

        ; Convert measuredTemp to decimal digits
        movf    measuredTemp,0,0     ; move measuredTemp into W
        movwf   tempReg,0           ; copy W into tempWork
        call    Magnitude        ; make value positive if negative
        call    measConvert    ; convert measuredTemp digits

        ; Compare measuredTemp with refTemp
        movf    refTemp,0,0          ; move refTemp into W
        subwf   measuredTemp,0,0     ; W = measuredTemp - refTemp

        btfsc   STATUS,2,0           ; test zero flag
        goto    TempEqual            ; branch if equal

        btfsc   STATUS,4,0           ; test negative flag
        goto    TempLess             ; branch if measuredTemp < refTemp

        goto    TempGreater          ; otherwise measuredTemp > refTemp


;============================================================
; measuredTemp > refTemp
; contReg = 2
; cooling ON
ORG     0200H
TempGreater:
        movlw   02h                  ; load value 2
        movwf   contReg,0            ; contReg = 2
        bcf     PORTD,1,0            ; turn heating off
        bcf     PORTD,2,0            ; clear cooling bit first
        bsf     PORTD,2,0            ; turn cooling on
        goto    ProgramEnd           ; hold final result

;============================================================
; measuredTemp < refTemp
; contReg = 1
; heating ON
ORG     0300H
TempLess:
        movlw   01h                  ; load value 1
        movwf   contReg,0            ; contReg = 1
        bcf     PORTD,2,0            ; turn cooling off
        bcf     PORTD,1,0            ; clear heating bit first
        bsf     PORTD,1,0            ; turn heating on
        goto    ProgramEnd           ; go hold final result

;============================================================
; measuredTemp = refTemp
; contReg = 0
; all OFF
ORG     0400H
TempEqual:
        clrf    contReg,0            ; contReg = 0
        bcf     PORTD,1,0            ; turn heating off
        bcf     PORTD,2,0            ; turn cooling off
        goto    ProgramEnd           ; go hold final result

;============================================================
; MakeMagnitude
; If tempWork is negative, convert to positive magnitude
ORG     0500H
Magnitude:
        btfss   tempReg,7,0         ; test sign bit
        return                       ; return if already positive

        negf    tempReg,0           ; take two's complement magnitude
        return                       ; return to caller

;============================================================
; ConvertRefDigits
; refUpper = hundreds digit
; refHigh  = tens digit
; refLow   = ones digit

ORG     0600H
refConvert:
        clrf    refLow,1             ; clear low digit
        clrf    refHigh,1            ; clear high digit
        clrf    refUpper,1           ; clear upper digit

RefHundredsLoop:
        movlw   100                  ; load 100
        subwf   tempReg,0,0         ; subtract 100 into W
        btfss   STATUS,0,0           ; test carry flag
        goto    RefTensLoop          ; go to tens if less than 100

        movlw   100                  ; load 100 again
        subwf   tempReg,1,0         ; subtract 100 from tempWork
        incf    refUpper,1,1         ; increment hundreds digit
        goto    RefHundredsLoop      ; repeat hundreds loop

RefTensLoop:
        movlw   10                   ; load 10
        subwf   tempReg,0,0         ; subtract 10 into W
        btfss   STATUS,0,0           ; test carry flag
        goto    RefStoreOnes         ; go store ones if less than 10

        movlw   10                   ; load 10 again
        subwf   tempReg,1,0         ; subtract 10 from tempWork
        incf    refHigh,1,1          ; increment tens digit
        goto    RefTensLoop          ; repeat tens loop

RefStoreOnes:
        movf    tempReg,0,0         ; move remaining value into W
        movwf   refLow,1             ; store ones digit
        return                       ; return to caller


;============================================================
; ConvertMeasDigits
; measUpper = hundreds digit
; measHigh  = tens digit
; measLow   = ones digit
ORG     0700H
measConvert:
        clrf    measLow,1            ; clear low digit
        clrf    measHigh,1           ; clear high digit
        clrf    measUpper,1          ; clear upper digit

MeasHundredsLoop:
        movlw   100                  ; load 100
        subwf   tempReg,0,0         ; subtract 100 into W
        btfss   STATUS,0,0           ; test carry flag
        goto    MeasTensLoop         ; go to tens if less than 100

        movlw   100                  ; load 100 again
        subwf   tempReg,1,0         ; subtract 100 from tempWork
        incf    measUpper,1,1        ; increment hundreds digit
        goto    MeasHundredsLoop     ; repeat hundreds loop

MeasTensLoop:
        movlw   10                   ; load 10
        subwf   tempReg,0,0         ; subtract 10 into W
        btfss   STATUS,0,0           ; test carry flag
        goto    MeasStoreOnes        ; go store ones if less than 10

        movlw   10                   ; load 10 again
        subwf   tempReg,1,0         ; subtract 10 from tempWork
        incf    measHigh,1,1         ; increment tens digit
        goto    MeasTensLoop         ; repeat tens loop

MeasStoreOnes:
        movf    tempReg,0,0         ; move remaining value into W
        movwf   measLow,1            ; store ones digit
        return                       ; return to caller


;============================================================
; Stop here to check results
ProgramEnd:                          ; end loop label
        bra     ProgramEnd           ; stay here forever

        END                          ; end of source file
