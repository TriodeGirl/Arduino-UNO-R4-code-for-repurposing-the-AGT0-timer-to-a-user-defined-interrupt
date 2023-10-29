/*  Arduino UNO R4 code for general method to attach an interrupt to ANY event
 *    + fast digital pin operation
 *    + non-blocking ADC operation
 *    + DAC output
 *    + Internal 48MHz HOCO or external XTAL clock switch (when XTAL fitted to PCB)
 *    + AGT timer (used in millis() and delay() functions) reasignment for local user mS type operations
 *
 *  Susan Parker - 22nd July 2023.
 *
 *  Susan Parker - 28th October 2023.
 *    Add internal HOCO or external XTAL clock for ADC stability test 
 *
 *  Susan Parker - 29th October 2023.
 *    Add AGT hijack for use as background timer
 *
 *
 * This code is "AS IS" without warranty or liability. 

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

/* Fastest possible pin writes and reads with the RA4M1 processor
  *PFS_P107PFS_BY = 0x05;      // Each Port Output bit clear to set takes c. 83nS 
  *PFS_P107PFS_BY = 0x04;      // Each Port Output bit set to clear takes c. 83nS 

  *PFS_P107PFS_BY = 0x05;      // Set HIGH
  char_val = *PFS_P107PFS_BY;  // Port State Input read - takes about 165nS
  *PFS_P107PFS_BY = 0x04;      // Read plus Set LOW = c. 250nS
 */   

/* For external aref - ADR4540 - Ultralow Noise, High Accuracy Voltage Reference
   Using an aref gives c. +- 1 to 2 value in 14 bit reads on USB power
https://www.analog.com/media/en/technical-documentation/data-sheets/adr4520_4525_4530_4533_4540_4550.pdf
*/


#include "Arduino.h"

// ARM-developer - Accessing memory-mapped peripherals
// https://developer.arm.com/documentation/102618/0100

#define ICUBASE 0x40000000 // ICU Base - See 13.2.6 page 233
// 32 bits - 
#define IELSR 0x6300 // ICU Event Link Setting Register n
#define ICU_IELSR00 ((volatile unsigned int *)(ICUBASE + IELSR))            //
#define ICU_IELSR01 ((volatile unsigned int *)(ICUBASE + IELSR + ( 1 * 4))) // 
#define ICU_IELSR02 ((volatile unsigned int *)(ICUBASE + IELSR + ( 2 * 4))) // 
#define ICU_IELSR03 ((volatile unsigned int *)(ICUBASE + IELSR + ( 3 * 4))) // 
#define ICU_IELSR04 ((volatile unsigned int *)(ICUBASE + IELSR + ( 4 * 4))) // 
#define ICU_IELSR05 ((volatile unsigned int *)(ICUBASE + IELSR + ( 5 * 4))) // 
#define ICU_IELSR06 ((volatile unsigned int *)(ICUBASE + IELSR + ( 6 * 4))) // 
#define ICU_IELSR07 ((volatile unsigned int *)(ICUBASE + IELSR + ( 7 * 4))) // 
#define ICU_IELSR08 ((volatile unsigned int *)(ICUBASE + IELSR + ( 8 * 4))) // 
#define ICU_IELSR09 ((volatile unsigned int *)(ICUBASE + IELSR + ( 9 * 4))) // 
#define ICU_IELSR10 ((volatile unsigned int *)(ICUBASE + IELSR + (10 * 4))) // 
#define ICU_IELSR11 ((volatile unsigned int *)(ICUBASE + IELSR + (11 * 4))) // 
#define ICU_IELSR12 ((volatile unsigned int *)(ICUBASE + IELSR + (12 * 4))) // 
#define ICU_IELSR13 ((volatile unsigned int *)(ICUBASE + IELSR + (13 * 4))) // 
#define ICU_IELSR14 ((volatile unsigned int *)(ICUBASE + IELSR + (14 * 4))) // 
#define ICU_IELSR15 ((volatile unsigned int *)(ICUBASE + IELSR + (15 * 4))) // 
#define ICU_IELSR16 ((volatile unsigned int *)(ICUBASE + IELSR + (16 * 4))) // 
#define ICU_IELSR17 ((volatile unsigned int *)(ICUBASE + IELSR + (17 * 4))) // 
#define ICU_IELSR18 ((volatile unsigned int *)(ICUBASE + IELSR + (18 * 4))) // 
#define ICU_IELSR19 ((volatile unsigned int *)(ICUBASE + IELSR + (19 * 4))) // 
#define ICU_IELSR20 ((volatile unsigned int *)(ICUBASE + IELSR + (20 * 4))) // 
#define ICU_IELSR21 ((volatile unsigned int *)(ICUBASE + IELSR + (21 * 4))) // 
#define ICU_IELSR22 ((volatile unsigned int *)(ICUBASE + IELSR + (22 * 4))) // 
#define ICU_IELSR23 ((volatile unsigned int *)(ICUBASE + IELSR + (23 * 4))) // 
#define ICU_IELSR24 ((volatile unsigned int *)(ICUBASE + IELSR + (24 * 4))) // 
#define ICU_IELSR25 ((volatile unsigned int *)(ICUBASE + IELSR + (25 * 4))) // 
#define ICU_IELSR26 ((volatile unsigned int *)(ICUBASE + IELSR + (26 * 4))) // 
#define ICU_IELSR27 ((volatile unsigned int *)(ICUBASE + IELSR + (27 * 4))) // 
#define ICU_IELSR28 ((volatile unsigned int *)(ICUBASE + IELSR + (28 * 4))) // 
#define ICU_IELSR29 ((volatile unsigned int *)(ICUBASE + IELSR + (29 * 4))) // 
#define ICU_IELSR30 ((volatile unsigned int *)(ICUBASE + IELSR + (30 * 4))) // 
#define ICU_IELSR31 ((volatile unsigned int *)(ICUBASE + IELSR + (31 * 4))) // 

#define ICU_SELSR0  ((volatile unsigned short  *)(ICUBASE + 0x6200))         // SYS Event Link Setting Register

// Low Power Mode Control - See datasheet section 10
#define SYSTEM 0x40010000 // System Registers
#define SYSTEM_SBYCR   ((volatile unsigned short *)(SYSTEM + 0xE00C))      // Standby Control Register
#define SYSTEM_MSTPCRA ((volatile unsigned int   *)(SYSTEM + 0xE01C))      // Module Stop Control Register A

#define MSTP 0x40040000 // Module Registers
#define MSTP_MSTPCRB   ((volatile unsigned int   *)(MSTP + 0x7000))      // Module Stop Control Register B
#define MSTPB2   2 // CAN0
#define MSTPB8   8 // IIC1
#define MSTPB9   9 // IIC0
#define MSTPB18 18 // SPI1
#define MSTPB19 19 // SPI0
#define MSTPB22 22 // SCI9
#define MSTPB29 29 // SCI2
#define MSTPB30 30 // SCI1
#define MSTPB31 31 // SCI0

#define MSTP_MSTPCRC   ((volatile unsigned int   *)(MSTP + 0x7004))      // Module Stop Control Register C
#define MSTP_MSTPCRD   ((volatile unsigned int   *)(MSTP + 0x7008))      // Module Stop Control Register D
#define MSTPD2   2 // AGT1   - Asynchronous General Purpose Timer 1 Module
#define MSTPD3   3 // AGT0   - Asynchronous General Purpose Timer 0 Module
#define MSTPD5   5 // GPT320 and GPT321 General 32 bit PWM Timer Module
#define MSTPD6   6 // GPT162 to GPT167 General 16 bit PWM Timer Module
#define MSTPD14 14 // POEG   - Port Output Enable for GPT Module Stop
#define MSTPD16 16 // ADC140 - 14-Bit A/D Converter Module
#define MSTPD19 19 // DAC8   -  8-Bit D/A Converter Module
#define MSTPD20 20 // DAC12  - 12-Bit D/A Converter Module
#define MSTPD29 29 // ACMPLP - Low-Power Analog Comparator Module
#define MSTPD31 31 // OPAMP  - Operational Amplifier Module

// The Mode Control bits are read as 1, the write value should be 1.
// Bit value 0: Cancel the module-stop state 
// Bit value 1: Enter the module-stop state.


// =========== ADC14 ============
// 35.2 Register Descriptions

#define ADCBASE 0x40050000 /* ADC Base */

#define ADC140_ADCSR   ((volatile unsigned short *)(ADCBASE + 0xC000)) // A/D Control Register
#define ADC140_ADANSA0 ((volatile unsigned short *)(ADCBASE + 0xC004)) // A/D Channel Select Register A0
#define ADC140_ADANSA1 ((volatile unsigned short *)(ADCBASE + 0xC006)) // A/D Channel Select Register A1
#define ADC140_ADADS0  ((volatile unsigned short *)(ADCBASE + 0xC008)) // A/D-Converted Value Addition/Average Channel Select Register 0
#define ADC140_ADADS1  ((volatile unsigned short *)(ADCBASE + 0xC00A)) // A/D-Converted Value Addition/Average Channel Select Register 1
#define ADC140_ADCER   ((volatile unsigned short *)(ADCBASE + 0xC00E)) // A/D Control Extended Register 
#define ADC140_ADSTRGR ((volatile unsigned short *)(ADCBASE + 0xC010)) // A/D Conversion Start Trigger Select Register
#define ADC140_ADEXICR ((volatile unsigned short *)(ADCBASE + 0xC012)) // A/D Conversion Extended Input Control Register
#define ADC140_ADANSB0 ((volatile unsigned short *)(ADCBASE + 0xC014)) // A/D Channel Select Register B0
#define ADC140_ADANSB1 ((volatile unsigned short *)(ADCBASE + 0xC016)) // A/D Channel Select Register B1
#define ADC140_ADTSDR  ((volatile unsigned short *)(ADCBASE + 0xC01A)) // A/D conversion result of temperature sensor output
#define ADC140_ADOCDR  ((volatile unsigned short *)(ADCBASE + 0xC01C)) // A/D result of internal reference voltage
#define ADC140_ADRD    ((volatile unsigned short *)(ADCBASE + 0xC01E)) // A/D Self-Diagnosis Data Register

#define ADC140_ADDR00 ((volatile unsigned short *)(ADCBASE + 0xC020))      // A1 (P000 AN00 AMP+)
#define ADC140_ADDR01 ((volatile unsigned short *)(ADCBASE + 0xC020 +  2)) // A2 (P001 AN01 AMP-) 
#define ADC140_ADDR02 ((volatile unsigned short *)(ADCBASE + 0xC020 +  4)) // A3 (P002 AN02 AMPO) 
#define ADC140_ADDR05 ((volatile unsigned short *)(ADCBASE + 0xC020 + 10)) // Aref (P010 AN05 VrefH0)
#define ADC140_ADDR09 ((volatile unsigned short *)(ADCBASE + 0xC020 + 18)) // A0 (P014 AN09 DAC)
#define ADC140_ADDR21 ((volatile unsigned short *)(ADCBASE + 0xC040 + 10)) // A4 (P101 AN21 SDA) 
#define ADC140_ADDR22 ((volatile unsigned short *)(ADCBASE + 0xC040 + 12)) // A5 (P100 AN20 SCL) 

#define ADC140_ADHVREFCNT ((volatile unsigned char  *)(ADCBASE + 0xC08A)) // A/D High-Potential/Low-Potential Reference Voltage Control Register
#define ADC140_ADADC      ((volatile unsigned char  *)(ADCBASE + 0xC00C)) // A/D-Converted Value Addition/Average Count Select Register

#define ADC140_ADSSTR00 ((volatile unsigned char *)(ADCBASE + 0xC0E0))      // AN00 A/D Sampling State Register

// 12-Bit D/A Converter
#define DACBASE 0x40050000          // DAC Base - DAC output on A0 (P014 AN09 DAC)
#define DAC12_DADR0    ((volatile unsigned short *)(DACBASE + 0xE000))      // D/A Data Register 0 
#define DAC12_DACR     ((volatile unsigned char  *)(DACBASE + 0xE004))      // D/A Control Register
#define DAC12_DADPR    ((volatile unsigned char  *)(DACBASE + 0xE005))      // DADR0 Format Select Register
#define DAC12_DAADSCR  ((volatile unsigned char  *)(DACBASE + 0xE006))      // D/A A/D Synchronous Start Control Register
#define DAC12_DAVREFCR ((volatile unsigned char  *)(DACBASE + 0xE007))      // D/A VREF Control Register

// =========== Ports ============
// 19.2.5 Port mn Pin Function Select Register (PmnPFS/PmnPFS_HA/PmnPFS_BY) (m = 0 to 9; n = 00 to 15)
#define PORTBASE 0x40040000 /* Port Base */

#define P000PFS 0x0800  // Port 0 Pin Function Select Register
#define PFS_P000PFS ((volatile unsigned int *)(PORTBASE + P000PFS))            // 
#define PFS_P001PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 1 * 4))) // 
#define PFS_P002PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 2 * 4))) // 
#define PFS_P003PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 3 * 4))) // 
#define PFS_P004PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 4 * 4))) // 
#define PFS_P005PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 5 * 4))) // 
#define PFS_P006PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 6 * 4))) // 
#define PFS_P007PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 7 * 4))) // 
#define PFS_P008PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 8 * 4))) // 
// #define PFS_P009PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 9 * 4))) // Does not exist
#define PFS_P010PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (10 * 4))) // 
#define PFS_P011PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (11 * 4))) // 
#define PFS_P012PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (12 * 4))) // 
#define PFS_P013PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (13 * 4))) // N/C
#define PFS_P014PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (14 * 4))) // N/A
#define PFS_P015PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (15 * 4))) // N/A

#define PFS_P100PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843))   // 8 bits - A5
#define PFS_P101PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 1 * 4))) // A4
#define PFS_P102PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 2 * 4))) // D5
#define PFS_P103PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 3 * 4))) // D4
#define PFS_P104PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 4 * 4))) // D3
#define PFS_P105PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 5 * 4))) // D2
#define PFS_P106PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 6 * 4))) // D6
#define PFS_P107PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 7 * 4))) // D7
#define PFS_P108PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 8 * 4))) // SWDIO
#define PFS_P109PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 9 * 4))) // D11 / MOSI
#define PFS_P110PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + (10 * 4))) // D12 / MISO
#define PFS_P111PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + (11 * 4))) // D13 / SCLK
#define PFS_P112PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + (12 * 4))) // D10 / CS
#define PFS_P300PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3))            // SWCLK (P300)
#define PFS_P301PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (01 * 4))) // D0 / RxD (P301)
#define PFS_P302PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (02 * 4))) // D1 / TxD (P302) 
#define PFS_P303PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (03 * 4))) // D9
#define PFS_P304PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (04 * 4))) // D8

#define P100PFS 0x0840  // Port 1 Pin Function Select Register
#define PFS_P102PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 2 * 4))) // D5 - RSPCKA


// ====  Asynchronous General Purpose Timer (AGT) =====
#define AGTBASE 0x40084000
#define AGT0_AGT    ((volatile unsigned short *)(AGTBASE))         // AGT Counter Register
#define AGT1_AGT    ((volatile unsigned short *)(AGTBASE + 0x100))
#define AGT0_AGTCMA ((volatile unsigned short *)(AGTBASE + 0x002)) // AGT Compare Match A Register
#define AGT1_AGTCMA ((volatile unsigned short *)(AGTBASE + 0x102))
#define AGT0_AGTCMB ((volatile unsigned short *)(AGTBASE + 0x004)) // AGT Compare Match B Register
#define AGT1_AGTCMB ((volatile unsigned short *)(AGTBASE + 0x104))

// 8 bit registers
#define AGT0_AGTCR    ((volatile unsigned char  *)(AGTBASE + 0x008))  // AGT Control Register
#define AGT1_AGTCR    ((volatile unsigned char  *)(AGTBASE + 0x108))  //
#define AGTCR_TSTART 0  // R/W - AGT Count Start; 1: Count starts, 0: Count stops   
#define AGTCR_TCSTF  1  // R   - AGT Count Status Flag; 1: Count in progress, 0: Count is stopped   
#define AGTCR_TSTOP  2  // W   - AGT Count Forced Stop; 1: The count is forcibly stopped, 0: Writing 0 is invalid!!!   
#define AGT0_AGTMR1   ((volatile unsigned char  *)(AGTBASE + 0x009))  // AGT Mode Register 1
#define AGT1_AGTMR1   ((volatile unsigned char  *)(AGTBASE + 0x109))  //
#define AGT0_AGTMR2   ((volatile unsigned char  *)(AGTBASE + 0x00A))  // AGT Mode Register 2
#define AGT1_AGTMR2   ((volatile unsigned char  *)(AGTBASE + 0x10A))  //
#define AGT0_AGTIOC   ((volatile unsigned char  *)(AGTBASE + 0x00C))  // AGT I/O Control Register
#define AGT1_AGTIOC   ((volatile unsigned char  *)(AGTBASE + 0x10C))  //
#define AGTIOC_TOE   2  // AGTOn Output Enable   
#define AGT0_AGTISR   ((volatile unsigned char  *)(AGTBASE + 0x00D))  // AGT Event Pin Select Register
#define AGT1_AGTISR   ((volatile unsigned char  *)(AGTBASE + 0x10D))  //
#define AGT0_AGTCMSR  ((volatile unsigned char  *)(AGTBASE + 0x00E))  // AGT Compare Match Function Select Register
#define AGT1_AGTCMSR  ((volatile unsigned char  *)(AGTBASE + 0x10E))  //
#define AGT0_AGTIOSEL ((volatile unsigned char  *)(AGTBASE + 0x00F))  // AGT Pin Select Register
#define AGT1_AGTIOSEL ((volatile unsigned char  *)(AGTBASE + 0x10F))  //


// ==== System & Clock Generation ====
#define SYSTEM 0x40010000 // ICU Base - See 13.2.6 page 233

// Register Write Protection - See section 12
// PRC0 Registers related to the clock generation circuit:
//   SCKDIVCR, SCKSCR, PLLCR, PLLCCR2, MEMWAIT, MOSCCR, HOCOCR, MOCOCR, CKOCR, TRCKCR,
//   OSTDCR, OSTDSR, SLCDSCKCR, EBCKOCR, MOCOUTCR, HOCOUTCR, MOSCWTCR, MOMCR, SOSCCR,
//   SOMCR, LOCOCR, LOCOUTCR, HOCOWTCR, USBCKCR
//
//   *SYSTEM_PRCR = 0xA501;     // Enable writing to the clock registers
//   *SYSTEM_PRCR = 0xA500;     // Disable writing to the clock registers

#define SYSTEM_PRCR  ((volatile unsigned short *)(SYSTEM + 0xE3FE))    // Protect Register
#define PRCR_PRC0           0   // Enables or disables writing to clock generation registers
#define PRCR_PRC1           1   // En/Dis writing to the low power modes and battery backup function registers
#define PRCR_PRC3           3   // Enables or disables writing to the registers related to the LVD
#define PRCR_PRKEY_7_0      8   // Control write access to the PRCR register.
#define PRCR_PRKEY       0xA5   // PRC Key Code - write to the upper 8 bits

#define SYSTEM_SCKDIVCR  ((volatile unsigned int *)(SYSTEM + 0xE020))  // System Clock Division Control Register
                                // SYSTEM_SCKDIVCR = 100010100 
#define SCKDIVCR_PCKD_2_0   0   // Peripheral Module Clock D           = 4; 1/16
#define SCKDIVCR_PCKC_2_0   4   // Peripheral Module Clock C           = 1; 1/2
#define SCKDIVCR_PCKB_2_0   8   // Peripheral Module Clock B           = 1; 1/2
#define SCKDIVCR_PCKA_2_0  12   // Peripheral Module Clock A           = 0
#define SCKDIVCR_ICK_2_0   24   // System Clock (ICLK) Select          = 0
#define SCKDIVCR_FCK_2_0   28   // Flash Interface Clock (FCLK) Select = 0
#define SYSTEM_SCKSCR  ((volatile unsigned char *)(SYSTEM + 0xE026))  // System Clock Source Control Register
#define SCKSCR_CKSEL_2_0    0   // Clock Source Select - See section 8.2.2
#define SYSTEM_PLLCR   ((volatile unsigned char *)(SYSTEM + 0xE02A))  // PLL Control Register
#define PLLCR_PLLSTP        0   // PLL Stop Control; 0: PLL is operating, 1: PLL is stopped
#define SYSTEM_PLLCCR2 ((volatile unsigned char *)(SYSTEM + 0xE02B))  // PLL Clock Control Register 2
#define PLLCCR2_PLLMUL_4_0  0   // PLL Frequency Multiplication Factor Select
#define PLLCCR2_PLODIV_1_0  6   // PLL Output Frequency Division Ratio Select
#define SYSTEM_MEMWAIT ((volatile unsigned char *)(SYSTEM + 0xE031))  // Memory Wait Cycle Control Register
#define MEMWAIT_MEMWAIT     0   // Memory Wait Cycle Select; 0: No wait, 1: Wait
#define SYSTEM_MOSCCR   ((volatile unsigned char *)(SYSTEM + 0xE032))  // Main Clock Oscillator Control Register
#define MOSCCR_MOSTP        0   // Main Clock Oscillator Stop; 0: Main clock oscillator is operating, 1: MCO is stopped
#define SYSTEM_HOCOCR   ((volatile unsigned char *)(SYSTEM + 0xE036))  // High-Speed On-Chip Oscillator Control Register
#define HOCOCR_HCSTP        0   // HOCO Stop; 0: HOCO is operating, 1: HOCO is stopped
#define SYSTEM_MOCOCR   ((volatile unsigned char *)(SYSTEM + 0xE038))  // Middle-Speed On-Chip Oscillator Control Register
#define MOCOCR_MCSTP        0   // MOCO Stop; 0: MOCO is operating, 1: MOCO is stopped
#define SYSTEM_OSCSF    ((volatile unsigned char *)(SYSTEM + 0xE03C))  // Oscillation Stabilization Flag Register
#define OSCSF_HOCOSF        0   // HOCO Clock Oscillation Stabilization Flag; 0: The HOCO clock is stopped or not stable, 1: The clock is stable
#define OSCSF_MOSCSF        3   // Main Clock Oscillation Stabilization Flag; 0: The Main clock is stopped or not stable, 1: The clock is stable
#define OSCSF_PLLSF         5   // PLL  Clock Oscillation Stabilization Flag; 0: The PLL  clock is stopped or not stable, 1: The clock is stable
#define SYSTEM_CKOCR    ((volatile unsigned char *)(SYSTEM + 0xE03E))  // Clock Out Control Register
#define CKOCR_CKOSEL_2_0    0   // Clock Out Source Select; 000: HOCO, 001: MOCO, 010: LOCO, 011: MOSC, 100: SOSC
#define CKOCR_CKODIV_2_0    4   // Clock Out Input Frequency Division Select; 000: Ã—1, 001: /2, 010: /4, ... , 111: /128
#define CKOCR_CKOEN         7   // Clock Out Enable; 0: Disable clock out, 1: Enable clock out
#define SYSTEM_TRCKCR   ((volatile unsigned char *)(SYSTEM + 0xE03F))  // Trace Clock Control Register
#define TRCKCR_TRCK_3_0     0   // Trace Clock Operation Frequency Select; 0000: /1, 0001: /2, 0010: /4 ( /2 = value after reset )
#define TRCKCR_TRCKEN       7   // Trace Clock Operating Enable; 0: Disable clock, 1: Enable clock

#define SYSTEM_OSTDCR   ((volatile unsigned char *)(SYSTEM + 0xE040))  // Oscillation Stop Detection Control Register
#define OSTDCR_OSTDIE       0   // Oscillation Stop Detection Interrupt Enable; 0: Disable oscillation stop detection interrupt, 1: Enable OSDI
#define OSTDCR_OSTDE        7   // Oscillation Stop Detection Function Enable; 0: Disable the oscillation stop detection function, 1: Enable OSDF
#define SYSTEM_OSTDSR   ((volatile unsigned char *)(SYSTEM + 0xE041))  // Oscillation Stop Detection Control Register
#define OSTDSR_OSTDF        0   // Oscillation Stop Detection Flag; 0: Main clock oscillation stop not detected, 1: Main clock oscillation stop detected

#define SYSTEM_SLCDSCKCR    ((volatile unsigned char *)(SYSTEM + 0xE050))  // Segment LCD Source Clock Control Register
#define SLCDSCKCR_LCDSCKSEL_2_0  0  // LCD Source Clock Select; 000: LOCO, 001: SOSC, 010: MOSC, 100: HOCO
#define SLCDSCKCR_LCDSCKEN       7  // LCD Source Clock Out Enable; 0: LCD source clock out disabled, 1: LCD source clock out enabled

#define SYSTEM_MOCOUTCR   ((volatile unsigned char *)(SYSTEM + 0xE061))  // MOCO User Trimming Control Register
#define MOCOUTCR_MOCOUTRM_7_0   0  // MOCO User Trimming - See: 8.2.21
#define SYSTEM_HOCOUTCR   ((volatile unsigned char *)(SYSTEM + 0xE062))  // HOCO User Trimming Control Register
#define HOCOUTCR_HOCOUTRM_7_0   0  // HOCO User Trimming - See: 8.2.21

#define SYSTEM_MOSCWTCR ((volatile unsigned char *)(SYSTEM + 0xE0A2))  // Main Clock Oscillator Wait Control Register
#define MOSCWTCR_MSTS_3_0   0   // Main Clock Oscillator Wait Time Setting
#define SYSTEM_HOCOWTCR ((volatile unsigned char *)(SYSTEM + 0xE0A5))  // High-Speed On-Chip Oscillator Wait Control Register
#define HOCOWTCR_MSTS_2_0   0   // HOCO Wait Time Setting

#define SYSTEM_USBCKCR  ((volatile unsigned char *)(SYSTEM + 0xE0D0))  // USB Clock Control Register
#define USBCKCR_USBCLKSEL   0   // USB Clock Source Select; 0: PLL (value after reset), 1: HOCO

#define SYSTEM_MOMCR    ((volatile unsigned char *)(SYSTEM + 0xE413))  // Main Clock Oscillator Mode Oscillation Control Register
#define MOMCR_MODRV1        3   // Main Clock Oscillator Drive Capability 1 Switching; 0: 10 MHz to 20 MHz, 1: 1 MHz to 10 MHz
#define MOMCR_MOSEL         6   // Main Clock Oscillator Switching; 0: Resonator, 1: External clock input

#define SYSTEM_SOSCCR   ((volatile unsigned char *)(SYSTEM + 0xE480))  // Sub-Clock Oscillator Control Register
#define SOSCCR_SOSTP        0   // Sub-Clock Oscillator Stop; 0: Operate the sub-clock oscillator, 1: Stop the sub-clock osc
#define SYSTEM_SOMCR    ((volatile unsigned char *)(SYSTEM + 0xE481))  // Sub-Clock Oscillator Mode Control Register
#define SOMCR_SODRV_1_0     0   // Sub-Clock Oscillator Drive Capability Switching; 00: Normal, 01: Low-power 1, 10: Low-power 2, 11: Low-power 3

#define SYSTEM_LOCOCR   ((volatile unsigned char *)(SYSTEM + 0xE490))  // Low-Speed On-Chip Oscillator Control Register
#define LOCOCR_LCSTP        0   // LOCO Stop; 0: Operate the LOCO clock, 1: Stop the LOCO clock
#define SYSTEM_LOCOUTCR ((volatile unsigned char *)(SYSTEM + 0xE492))  // LOCO User Trimming Control Register
#define LOCOUTCR_LOCOUTRM_7_0   0  // LOCO User Trimming - See: 8.2.20

#define SYSTEM_RSTSR0   ((volatile unsigned char *)(SYSTEM + 0xE410))  // Reset Status Register 0
#define RSTSR0_PORF         0    // Power-On Reset Detect Flag
#define RSTSR0_LVD0RF       1    // Voltage Monitor 0 Reset Detect Flag
#define RSTSR0_LVD1RF       2    // Voltage Monitor 1 Reset Detect Flag
#define RSTSR0_LVD2RF       3    // Voltage Monitor 2 Reset Detect Flag
#define SYSTEM_RSTSR1   ((volatile unsigned char *)(SYSTEM + 0xE0C0))  // Reset Status Register 1
#define RSTSR1_IWDTRF       0    // Independent Watchdog Timer Reset Detect Flag
#define RSTSR1_WDTRF        1    // Watchdog Timer Reset Detect Flag
#define RSTSR1_SWRF         2    // Software Reset Detect Flag
#define SYSTEM_RSTSR2   ((volatile unsigned char *)(SYSTEM + 0xE411))  // Reset Status Register 2
#define RSTSR2_CWSF         0    // Cold/Warm Start Determination Flag - 0: Cold start, 1: Warm start

#define NVICBASE 0xE0000000 // NVIC Interrupt Controller
#define NVICIPR  0xE400     // Interrupt Priority Register
#define NVIC_IPR04_BY  ((volatile unsigned char  *)(NVICBASE + NVICIPR +  4 ))     // AGT for millis() etc = 0x80
#define NVIC_IPR05_BY  ((volatile unsigned char  *)(NVICBASE + NVICIPR +  5 ))     // 
#define NVIC_IPR06_BY  ((volatile unsigned char  *)(NVICBASE + NVICIPR +  6 ))     // 

// ==== Event Link Controller ====
#define ELCBASE 0x40040000 // Event Link Controller
#define ELC_ELCR     ((volatile unsigned char  *)(ELCBASE + 0x1000))              // Event Link Controller Register


// === Local Defines

// #define DIAGS_PRINT       // Use to check setups

#define CLOCK_PLL            // Use when external 12MHz crystal fitted
#define ADC_EXT_AREF         // Use external ADC Aref source
#define ADC_AVARAGE          // Enable 4x averaging i.e. four sucessive conversions
#define ADSSTR00             // Enable Sampling State Register change from default
#define ADSSTR00_VAL 0x3F    // A/D Sampling State Register 0 - Default is 0x0D

// Time from Start ADC conversion to ADC complete interrupt pin-flags
// HOCO at 12.000MHz; 4x Averaging; Default ADSSTR00 at 0x0D  =  7.95uS
// XTAL at 12.288MHz; 4x Averaging; ADSSTR00 at 0x3F          = 11.83uS

#ifdef CLOCK_PLL
#define AGT0_RELOAD  3061    // AGT0 value for 1.000 mS time with 12.288 MHz PLL derived 4x clock
// With 12.288 PLL clock: 3061 = 1.000mS, 4000 = 1.306 mS
#else
#define AGT0_RELOAD  2988    // AGT0 value to give 1mS time - refine as needed
// With 48.00 HOCO clock: 3000 = 1.004mS; 2990 = 1.001mS; 2988 = 1.000mS (if no other interrupts get in)
#endif

// These variables are in effect static - but take longer to access than a variable inside a function

unsigned int int_val;
unsigned short short_val;
unsigned char char_val; 

// volatile uint32_t agt_count;    // Max 32bit count is 4,294,967 seconds = 49 days, 17 hours, 2 minutes, 47 seconds
volatile uint64_t agt_count;    // Max 64bit count is 18,446,744,073,709,551 seconds = 584,942,417 Years
uint16_t adc_val_16;

#define MS_DELAY_PRINT 100    // Delay by xxx nominal milliseconds


void setup()
  {                                                   // Pins for interrupts: 0, 1, 2, 3, 8, 12, 13, 15, 16, 17, 18 and 19

  *ICU_IELSR04 = 0x021;                                 // Reasign mS timer Slot 04 IELSR04 to non-active IRQ e.g AGT1_AGTI

  attachInterrupt(15, agtUnderflowInterrupt, FALLING);  // This IRQ will be asigned to Slot 05 IELSR05 as 0x001 PORT_IRQ0 - Table 13.4
  *ICU_IELSR05 = 0x01E;                                 // HiJack Slot 05 IELSR05 for AGT0_AGTI
  *PFS_P000PFS = 0x00000000;                            // Clear A1/D15 ISEL pin assigned Interrupt Enable
	asm volatile("dsb");                                  // Data bus Synchronization instruction
  *NVIC_IPR05_BY = 0xE0;                                // Drop the priority down from 0x80 to 0xE0
// Warning: Cannot use   delay(); etc mS functions.

  attachInterrupt(16, adcCompleteInterrupt, RISING);    // This IRQ will be asigned to Slot 06 IELSR06 as 0x002 PORT_IRQ1 - Table 13.4
  *PFS_P001PFS = 0x00000000;                            // Clear A2/D16 ISEL pin assigned Interrupt Enable
  *ICU_IELSR06 = 0x029;                                 // Assign Slot 06 IELSR06 for ADC140_ADI Interrupt
	asm volatile("dsb");                                  // Data bus Synchronization instruction
  *NVIC_IPR06_BY = 0x40;                                // Bounce the priority up from 0xC0 to 0x40

  *PFS_P107PFS_BY = 0x04;                               // Set D7 output low - IRQ time flag pin

  Serial.begin(115200);
  while (!Serial){};

#ifdef CLOCK_PLL
  sys_clock_pll_setup();
#else
  Serial.println("Default HOCO Clock"); 
#endif

  setup_adc();
  setup_dac();
  enable_agt0_output();

#ifdef DIAGS_PRINT
  Serial.print("Sampling State Register = "); 
  Serial.println(*ADC140_ADSSTR00, HEX);
  print_icu_event_links();
  print_agt0_regs();
  Serial.print("Event Link Controller Register = "); 
  Serial.println(*ELC_ELCR, HEX);
  Serial.print("SYS Event Link Setting Register = "); 
  Serial.println(*ICU_SELSR0, HEX);
#endif  

  *ADC140_ADCSR   |= (0x01 << 15);   // Start an ADC conversion
  }


void loop()
  {
  static uint32_t agt_count_last;                   // Note use static variable, otherwise reset to 0 each loop()
  static  int16_t mS_delay_count = MS_DELAY_PRINT;   
  static bool     mS_flag = false;

  if(agt_count_last != agt_count)
    {
    agt_count_last = agt_count;  

    *PFS_P103PFS_BY = 0x05;      // Pulse on D4 to trigger scope 
    *PFS_P103PFS_BY = 0x04;      //  

    // Set High, read ADC register, trigger Conversion, set Low = c. 500nS
    *PFS_P107PFS_BY = 0x05;         // digitalWrite(monitorPin, HIGH);   // Digital Pin D7
    adc_val_16 = *ADC140_ADDR00;    // adcValue = analogRead(analogPin); // Internal 16bit register read = c. 123nS 
    *ADC140_ADCSR |= (0x01 << 15);  // Next ADC conversion = write to register c. 300nS
    *PFS_P107PFS_BY = 0x04;         // digitalWrite(monitorPin, LOW);  

    mS_flag = true;
    }

  if(mS_flag == true)
    {
    mS_flag = false;
    if(--mS_delay_count <= 0)       // Only print value every xxx "milliseconds"
      {
      mS_delay_count = MS_DELAY_PRINT;
      Serial.print(agt_count);
      Serial.print("\t");
      Serial.println(adc_val_16);
      }
    }

  }


void agtUnderflowInterrupt(void)
  {
  *PFS_P111PFS_BY = 0x05;            // D13
  *AGT0_AGT = AGT0_RELOAD;           //
  agt_count++;                       // 
  *PFS_P111PFS_BY = 0x04;            //  
  }

void adcCompleteInterrupt(void)
  {
  *PFS_P107PFS_BY = 0x05;            // D7 
  *DAC12_DADR0 = (adc_val_16 >> 2);  // DAC will update after all ADC conversions have finished
  *PFS_P107PFS_BY = 0x04;            //  
  }

void setup_adc(void)
  {
  *MSTP_MSTPCRD &= (0xFFFFFFFF - (0x01 << MSTPD16));  // Enable ADC140 module
#ifdef ADC_EXT_AREF
  *ADC140_ADHVREFCNT = 0x01;         // Set External Aref = analogReference(AR_EXTERNAL);      
#endif
  *ADC140_ADCER = 0x06;              // 14 bit mode, clear ACE bit 5
  *ADC140_ADANSA0 |= (0x01 << 0);    // Selected ANSA00 = A1 as DAC is on A0
#ifdef ADC_AVARAGE
  *ADC140_ADADC    = 0x83;           // Average mode - 4x and b7 to enable averaging
  *ADC140_ADADS0  |= (0x01 << 0);    // Enable Averaging for ANSA00 channel
#endif
#ifdef ADSSTR00
  *ADC140_ADSSTR00 = ADSSTR00_VAL;   // A/D Sampling State Register 0 - Default is 0x0D
#endif
  *ADC140_ADCSR |= (0x01 << 6);      // Set b6 - GBADIE Group B Scan End Interrupt Enable
  }

void setup_dac(void)       // Note make sure ADC is stopped before setup DAC
  {
  *MSTP_MSTPCRD &= (0xFFFFFFFF - (0x01 << MSTPD20));  // Enable DAC12 module
  *DAC12_DADPR    = 0x00;        // DADR0 Format Select Register - Set right-justified format
//  *DAC12_DAADSCR  = 0x80;        // D/A A/D Synchronous Start Control Register - Enable
  *DAC12_DAADSCR  = 0x00;        // D/A A/D Synchronous Start Control Register - Default
// 36.3.2 Notes on Using the Internal Reference Voltage as the Reference Voltage
  *DAC12_DAVREFCR = 0x00;        // D/A VREF Control Register - Write 0x00 first - see 36.2.5
  *DAC12_DADR0    = 0x0000;      // D/A Data Register 0 
   delayMicroseconds(10);        
  *DAC12_DAVREFCR = 0x01;        // D/A VREF Control Register - Select AVCC0/AVSS0 for Vref
//  *DAC12_DAVREFCR = 0x03;        // D/A VREF Control Register - Select Internal reference voltage/AVSS0
//  *DAC12_DAVREFCR = 0x06;        // D/A VREF Control Register - Select External Vref; set VREFH&L pins used for LEDs
  *DAC12_DACR     = 0x5F;        // D/A Control Register - 
   delayMicroseconds(5);         // 
  *DAC12_DADR0    = 0x0800;      // D/A Data Register 0 
  *PFS_P014PFS   = 0x00000000;   // Make sure all cleared
  *PFS_P014PFS  |= (0x1 << 15);  // Port Mode Control - Used as an analog pin
  }

void sys_clock_pll_setup(void)
  {
  Serial.println("Setup MOSC - XTAL & PLL"); 
  *SYSTEM_PRCR     = 0xA501;          // Enable writing to the clock registers
  *SYSTEM_MOSCCR   = 0x01;            // Make sure XTAL is stopped
  *SYSTEM_MOMCR    = 0x00;            // MODRV1 = 0 (10 MHz to 20 MHz); MOSEL = 0 (Resonator)
  *SYSTEM_MOSCWTCR = 0x07;            // Set stability timeout period 
  *SYSTEM_MOSCCR   = 0x00;            // Enable XTAL
	asm volatile("dsb");                // Data bus Synchronization instruction
  char enable_ok = *SYSTEM_MOSCCR;    // Check bit 
  delay(100);                         // wait for XTAL to stabilise  
  *SYSTEM_PLLCR    = 0x01;            // Disable PLL
  *SYSTEM_PLLCCR2  = 0x07;            // Setup PLLCCR2_PLLMUL_4_0 PLL PLL Frequency Multiplication to 8x
  *SYSTEM_PLLCCR2 |= 0x40;            // Setup PLLCCR2_PLODIV_1_0 PLL Output Frequency Division to /2
  *SYSTEM_PLLCR    = 0x00;            // Enable PLL
  delayMicroseconds(1000);            // wait for PLL to stabilise
  *SYSTEM_SCKSCR   = 0x05;            // Select PLL as the system clock 
  *SYSTEM_PRCR     = 0xA500;          // Disable writing to the clock registers
  }

void enable_agt0_output(void)          // Output on D5 - NOTE AGT is a DOWN counter.
  {
  *AGT0_AGTIOC = (0x1 << AGTIOC_TOE);  // AGTO0 Output Enable 
  *PFS_P102PFS   = (0b00001 << 24);    // Select PSEL[4:0] for AGTO0 - See Table 19.6
  *PFS_P102PFS  |= (0x1 << 16);        // Port Mode Control - Used as an I/O port for peripheral function
  }

void print_agt0_regs(void)
  {
  Serial.print("AGT0_AGT      = "); 
  Serial.println(*AGT0_AGT, HEX);
  Serial.print("AGT0_AGTCMA   = "); 
  Serial.println(*AGT0_AGTCMA, HEX);
  Serial.print("AGT0_AGTCMB   = "); 
  Serial.println(*AGT0_AGTCMB, HEX);
  Serial.print("AGT0_AGTCR    = ");
  Serial.println(*AGT0_AGTCR, BIN);    // 0b00000011 b0 AGT Count Start, b1 AGT Count Status flag
  Serial.print("AGT0_AGTMR1   = "); 
  Serial.println(*AGT0_AGTMR1, BIN);   // 0b00010001 b0-2 Mode = 001: Pulse output mode, b4-6 Count source = 001: PCLKB/8
  Serial.print("AGT0_AGTMR2   = "); 
  Serial.println(*AGT0_AGTMR2, BIN);
  Serial.print("AGT0_AGTIOC   = "); 
  Serial.println(*AGT0_AGTIOC, BIN);
  Serial.print("AGT0_AGTISR   = "); 
  Serial.println(*AGT0_AGTISR, BIN);
  Serial.print("AGT0_AGTCMSR  = "); 
  Serial.println(*AGT0_AGTCMSR, BIN);
  Serial.print("AGT0_AGTIOSEL = "); 
  Serial.println(*AGT0_AGTIOSEL, BIN);
  }

#ifdef DIAGS_PRINT

// Function: print_icu_event_links();
// The following is to determin which interrupts are in use
// RA4M1 Group ICU Event Number Table 13.4 
//
// Slot - Event Number - Name of the Interrupt
// 0 - 33 - USBFS_USBI
// 1 - 34 - USBFS_USBR
// 2 - 31 - USBFS_D0FIFO
// 3 - 32 - USBFS_D1FIFO
// 4 - 1E - AGT0_AGTI
//
// The above 5 entries are always present before the code gets to setup()

// Use PROGMEM structures to place strings into program memory 
// https://www.arduino.cc/reference/en/language/variables/utilities/progmem/

const char string_00[] PROGMEM = "No_Event";
const char string_01[] PROGMEM = "PORT_IRQ0";
const char string_02[] PROGMEM = "PORT_IRQ1";
const char string_03[] PROGMEM = "PORT_IRQ2";
const char string_04[] PROGMEM = "PORT_IRQ3";
const char string_05[] PROGMEM = "PORT_IRQ4";
const char string_06[] PROGMEM = "PORT_IRQ5";
const char string_07[] PROGMEM = "PORT_IRQ6";
const char string_08[] PROGMEM = "PORT_IRQ7";
const char string_09[] PROGMEM = "PORT_IRQ8";
const char string_0A[] PROGMEM = "PORT_IRQ9";
const char string_0B[] PROGMEM = "PORT_IRQ10";
const char string_0C[] PROGMEM = "PORT_IRQ11";
const char string_0D[] PROGMEM = "PORT_IRQ12";
const char string_0E[] PROGMEM = "PORT_UNUSED";
const char string_0F[] PROGMEM = "PORT_IRQ14";
const char string_10[] PROGMEM = "PORT_IRQ15";
const char string_11[] PROGMEM = "DMAC0_INT";
const char string_12[] PROGMEM = "DMAC1_INT";
const char string_13[] PROGMEM = "DMAC2_INT";
const char string_14[] PROGMEM = "DMAC3_INT";
const char string_15[] PROGMEM = "DTC_COMPLETE";
const char string_16[] PROGMEM = "UNUSED";
const char string_17[] PROGMEM = "ICU_SNZCANCEL";
const char string_18[] PROGMEM = "FCU_FRDYI";
const char string_19[] PROGMEM = "LVD_LVD1";
const char string_1A[] PROGMEM = "LVD_LVD2";
const char string_1B[] PROGMEM = "VBATT_LVD";
const char string_1C[] PROGMEM = "MOSC_STOP";
const char string_1D[] PROGMEM = "SYSTEM_SNZREQ";
const char string_1E[] PROGMEM = "AGT0_AGTI";
const char string_1F[] PROGMEM = "AGT0_AGTCMAI";
const char string_20[] PROGMEM = "AGT0_AGTCMBI";
const char string_21[] PROGMEM = "AGT1_AGTI";
const char string_22[] PROGMEM = "AGT1_AGTCMAI";
const char string_23[] PROGMEM = "AGT1_AGTCMBI";
const char string_24[] PROGMEM = "IWDT_NMIUNDF";
const char string_25[] PROGMEM = "WDT_NMIUNDF";
const char string_26[] PROGMEM = "RTC_ALM";
const char string_27[] PROGMEM = "RTC_PRD";
const char string_28[] PROGMEM = "RTC_CUP";
const char string_29[] PROGMEM = "ADC140_ADI";
const char string_2A[] PROGMEM = "ADC140_GBADI";
const char string_2B[] PROGMEM = "ADC140_CMPAI";
const char string_2C[] PROGMEM = "ADC140_CMPBI";
const char string_2D[] PROGMEM = "ADC140_WCMPM";
const char string_2E[] PROGMEM = "ADC140_WCMPUM";
const char string_2F[] PROGMEM = "ACMP_LP0";
const char string_30[] PROGMEM = "ACMP_LP1";
const char string_31[] PROGMEM = "USBFS_D0FIFO";
const char string_32[] PROGMEM = "USBFS_D1FIFO";
const char string_33[] PROGMEM = "USBFS_USBI";
const char string_34[] PROGMEM = "USBFS_USBR";
const char string_35[] PROGMEM = "IIC0_RXI";
const char string_36[] PROGMEM = "IIC0_TXI";
const char string_37[] PROGMEM = "IIC0_TEI";
const char string_38[] PROGMEM = "IIC0_EEI";
const char string_39[] PROGMEM = "IIC0_WUI";
const char string_3A[] PROGMEM = "IIC1_RXI";
const char string_3B[] PROGMEM = "IIC1_TXI";
const char string_3C[] PROGMEM = "IIC1_TEI";
const char string_3D[] PROGMEM = "IIC1_EEI";
const char string_3E[] PROGMEM = "SSIE0_SSITXI";
const char string_3F[] PROGMEM = "SSIE0_SSIRXI";
const char string_40[] PROGMEM = "UNUSED";
const char string_41[] PROGMEM = "SSIE0_SSIF";
const char string_42[] PROGMEM = "CTSU_CTSUWR";
const char string_43[] PROGMEM = "CTSU_CTSURD";
const char string_44[] PROGMEM = "CTSU_CTSUFN";
const char string_45[] PROGMEM = "KEY_INTKR";
const char string_46[] PROGMEM = "DOC_DOPCI";
const char string_47[] PROGMEM = "CAC_FERRI";
const char string_48[] PROGMEM = "CAC_MENDI";
const char string_49[] PROGMEM = "CAC_OVFI";
const char string_4A[] PROGMEM = "CAN0_ERS";
const char string_4B[] PROGMEM = "CAN0_RXF";
const char string_4C[] PROGMEM = "CAN0_TXF";
const char string_4D[] PROGMEM = "CAN0_RXM";
const char string_4E[] PROGMEM = "CAN0_TXM";
const char string_4F[] PROGMEM = "IOPORT_GROUP1";
const char string_50[] PROGMEM = "IOPORT_GROUP2";
const char string_51[] PROGMEM = "IOPORT_GROUP3";
const char string_52[] PROGMEM = "IOPORT_GROUP4";
const char string_53[] PROGMEM = "ELC_SWEVT0";
const char string_54[] PROGMEM = "ELC_SWEVT1";
const char string_55[] PROGMEM = "POEG_GROUP0";
const char string_56[] PROGMEM = "POEG_GROUP1";
const char string_57[] PROGMEM = "GPT0_CCMPA";
const char string_58[] PROGMEM = "GPT0_CCMPB";
const char string_59[] PROGMEM = "GPT0_CMPC";
const char string_5A[] PROGMEM = "GPT0_CMPD";
const char string_5B[] PROGMEM = "GPT0_CMPE";
const char string_5C[] PROGMEM = "GPT0_CMPF";
const char string_5D[] PROGMEM = "GPT0_OVF";
const char string_5E[] PROGMEM = "GPT0_UDF";
const char string_5F[] PROGMEM = "GPT1_CCMPA";
const char string_60[] PROGMEM = "GPT1_CCMPB";
const char string_61[] PROGMEM = "GPT1_CMPC";
const char string_62[] PROGMEM = "GPT1_CMPD";
const char string_63[] PROGMEM = "GPT1_CMPE";
const char string_64[] PROGMEM = "GPT1_CMPF";
const char string_65[] PROGMEM = "GPT1_OVF";
const char string_66[] PROGMEM = "GPT1_UDF";
const char string_67[] PROGMEM = "GPT2_CCMPA";
const char string_68[] PROGMEM = "GPT2_CCMPB";
const char string_69[] PROGMEM = "GPT2_CMPC";
const char string_6A[] PROGMEM = "GPT2_CMPD";
const char string_6B[] PROGMEM = "GPT2_CMPE";
const char string_6C[] PROGMEM = "GPT2_CMPF";
const char string_6D[] PROGMEM = "GPT2_OVF";
const char string_6E[] PROGMEM = "GPT2_UDF";
const char string_6F[] PROGMEM = "GPT3_CCMPA";
const char string_70[] PROGMEM = "GPT3_CCMPB";
const char string_71[] PROGMEM = "GPT3_CMPC";
const char string_72[] PROGMEM = "GPT3_CMPD";
const char string_73[] PROGMEM = "GPT3_CMPE";
const char string_74[] PROGMEM = "GPT3_CMPF";
const char string_75[] PROGMEM = "GPT3_OVF";
const char string_76[] PROGMEM = "GPT3_UDF";
const char string_77[] PROGMEM = "GPT4_CCMPA";
const char string_78[] PROGMEM = "GPT4_CCMPB";
const char string_79[] PROGMEM = "GPT4_CMPC";
const char string_7A[] PROGMEM = "GPT4_CMPD";
const char string_7B[] PROGMEM = "GPT4_CMPE";
const char string_7C[] PROGMEM = "GPT4_CMPF";
const char string_7D[] PROGMEM = "GPT4_OVF";
const char string_7E[] PROGMEM = "GPT4_UDF";
const char string_7F[] PROGMEM = "GPT5_CCMPA";
const char string_80[] PROGMEM = "GPT5_CCMPB";
const char string_81[] PROGMEM = "GPT5_CMPC";
const char string_82[] PROGMEM = "GPT5_CMPD";
const char string_83[] PROGMEM = "GPT5_CMPE";
const char string_84[] PROGMEM = "GPT5_CMPF";
const char string_85[] PROGMEM = "GPT5_OVF";
const char string_86[] PROGMEM = "GPT5_UDF";
const char string_87[] PROGMEM = "GPT6_CCMPA";
const char string_88[] PROGMEM = "GPT6_CCMPB";
const char string_89[] PROGMEM = "GPT6_CMPC";
const char string_8A[] PROGMEM = "GPT6_CMPD";
const char string_8B[] PROGMEM = "GPT6_CMPE";
const char string_8C[] PROGMEM = "GPT6_CMPF";
const char string_8D[] PROGMEM = "GPT6_OVF";
const char string_8E[] PROGMEM = "GPT6_UDF";
const char string_8F[] PROGMEM = "GPT7_CCMPA";
const char string_90[] PROGMEM = "GPT7_CCMPB";
const char string_91[] PROGMEM = "GPT7_CMPC";
const char string_92[] PROGMEM = "GPT7_CMPD";
const char string_93[] PROGMEM = "GPT7_CMPE";
const char string_94[] PROGMEM = "GPT7_CMPF";
const char string_95[] PROGMEM = "GPT7_OVF";
const char string_96[] PROGMEM = "GPT7_UDF";
const char string_97[] PROGMEM = "GPT_UVWEDGE";
const char string_98[] PROGMEM = "SCI0_RXI";
const char string_99[] PROGMEM = "SCI0_TXI";
const char string_9A[] PROGMEM = "SCI0_TEI";
const char string_9B[] PROGMEM = "SCI0_ERI";
const char string_9C[] PROGMEM = "SCI0_AM";
const char string_9D[] PROGMEM = "SCI0_RXI_OR_ERI";
const char string_9E[] PROGMEM = "SCI1_RXI";
const char string_9F[] PROGMEM = "SCI1_TXI";
const char string_A0[] PROGMEM = "SCI1_TEI";
const char string_A1[] PROGMEM = "SCI1_ERI";
const char string_A2[] PROGMEM = "SCI1_AM";
const char string_A3[] PROGMEM = "SCI2_RXI";
const char string_A4[] PROGMEM = "SCI2_TXI";
const char string_A5[] PROGMEM = "SCI2_TEI";
const char string_A6[] PROGMEM = "SCI2_ERI";
const char string_A7[] PROGMEM = "SCI2_AM";
const char string_A8[] PROGMEM = "SCI9_RXI";
const char string_A9[] PROGMEM = "SCI9_TXI";
const char string_AA[] PROGMEM = "SCI9_TEI";
const char string_AB[] PROGMEM = "SCI9_ERI";
const char string_AC[] PROGMEM = "SCI9_AM";
const char string_AD[] PROGMEM = "SPI0_SPRI";
const char string_AE[] PROGMEM = "SPI0_SPTI";
const char string_AF[] PROGMEM = "SPI0_SPII";
const char string_B0[] PROGMEM = "SPI0_SPEI";
const char string_B1[] PROGMEM = "SPI0_SPTEND";
const char string_B2[] PROGMEM = "SPI1_SPRI";
const char string_B3[] PROGMEM = "SPI1_SPTI";
const char string_B4[] PROGMEM = "SPI1_SPII";
const char string_B5[] PROGMEM = "SPI1_SPEI";
const char string_B6[] PROGMEM = "SPI1_SPTEND";
const char string_B7[] PROGMEM = "UNUSED";

const char *const string_table[] PROGMEM = {
string_00, string_01, string_02, string_03, string_04, string_05, string_06, string_07,
string_08, string_09, string_0A, string_0B, string_0C, string_0D, string_0E, string_0F,
string_10, string_11, string_12, string_13, string_14, string_15, string_16, string_17,
string_18, string_19, string_1A, string_1B, string_1C, string_1D, string_1E, string_1F,
string_20, string_21, string_22, string_23, string_24, string_25, string_26, string_27,
string_28, string_29, string_2A, string_2B, string_2C, string_2D, string_2E, string_2F,
string_30, string_31, string_32, string_33, string_34, string_35, string_36, string_37,
string_38, string_39, string_3A, string_3B, string_3C, string_3D, string_3E, string_3F,
string_40, string_41, string_42, string_43, string_44, string_45, string_46, string_47,
string_48, string_49, string_4A, string_4B, string_4C, string_4D, string_4E, string_4F,
string_50, string_51, string_52, string_53, string_54, string_55, string_56, string_57,
string_58, string_59, string_5A, string_5B, string_5C, string_5D, string_5E, string_5F,
string_60, string_61, string_62, string_63, string_64, string_65, string_66, string_67,
string_68, string_69, string_6A, string_6B, string_6C, string_6D, string_6E, string_6F,
string_70, string_71, string_72, string_73, string_74, string_75, string_76, string_77,
string_78, string_79, string_7A, string_7B, string_7C, string_7D, string_7E, string_7F,
string_80, string_81, string_82, string_83, string_84, string_85, string_86, string_87,
string_88, string_89, string_8A, string_8B, string_8C, string_8D, string_8E, string_8F,
string_90, string_91, string_92, string_93, string_94, string_95, string_96, string_97,
string_98, string_99, string_9A, string_9B, string_9C, string_9D, string_9E, string_9F,
string_A0, string_A1, string_A2, string_A3, string_A4, string_A5, string_A6, string_A7,
string_A8, string_A9, string_AA, string_AB, string_AC, string_AD, string_AE, string_AF,
string_B0, string_B1, string_B2, string_B3, string_B4, string_B5, string_B6, string_B7
};

char message_buffer[30];  // 

void print_icu_event_links(void)
  {
  unsigned int local_icu_val = 0;
  unsigned char icu_val_index = 0;

  for(icu_val_index = 0; icu_val_index < 32; icu_val_index++)
    {
    Serial.print(icu_val_index);  
    Serial.print(" - ");  
    local_icu_val = *((volatile unsigned int *)(ICUBASE + IELSR + (icu_val_index * 4)));            //
    Serial.print(local_icu_val, HEX);
    strcpy_P(message_buffer, (char *)pgm_read_word(&(string_table[local_icu_val])));  // 
    Serial.print(" - ");  
    Serial.println(message_buffer);
    if(local_icu_val == 0) break;      // Only print active allocations - these are always contigious from 0
    }
  }

#endif