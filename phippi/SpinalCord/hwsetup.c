/*
 *  Copyright (c) 2011, Tõnu Samuel
 *  All rights reserved.
 *
 *  This file is part of TYROS.
 *
 *  TYROS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  TYROS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with TYROS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "ior32c111.h"
#include "hwsetup.h"
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include "main.h"
#include "uart.h"

#define TIMERB2COUNT	1200
<<<<<<< HEAD
#define TIMER4COUNT	60
=======
>>>>>>> 0af8a399d5edef392ad9bddf7f0a7d3cdd74b071

static void ConfigureOperatingFrequency(char mode);
static void ConfigurePortPins(void);
volatile struct statuses status;
unsigned int base_freq;

void 
OLED_On(void) {
    // Make ports safe
    p4_0 = 0;                                              // DC (data/command)
    p4_2 = 0;                                              // reset pin
    p4_4 = 0;                                              // VCC pin
    p4_5 = 0;                                              // VDD is reversed (0-on, 1-off), but switched ON first, so keep it like that  :)

    // Port function is IO
    p4_0s = 0;
    p4_2s = 0;
    p4_4s = 0;
    p4_5s = 0;

    // Ports are output
    pd4_0 = 1;                                             // Reset pin
    pd4_2 = 1;                                             // Reset pin
    pd4_4 = 1;                                             // VCC
    pd4_5 = 0;                                             // VDD

    // Make sure VCC is Off
    p4_4 = 0;
    uDelay(10);

    // Reset sequence
    p4_2 = 1;
    uDelay(100);
    p4_2 = 0;
    uDelay(100);
    p4_2 = 1;
    uDelay(100);

    // Enable VCC
    p4_4 = 1;
    uDelay(100);
}


void
PWM_Init(void)
{

    /* 
     * Removes Protection for INVC0 & INVC1 Registers 
     */
    prc1 = 1;

    /* 
     * Three-Phase PWM Control Register0 - 00001100b b1:b0 INV01:INV00
     * ICTB2 Count Condition: Underflow of timer B2 b2 INV02 Use
     * Three-Phase Motor Control Timers b3 INV03 Enable Three-Phase Motor 
     * Control Timer Output b4 INV04 Ignore Simultaneous turn-on Signal
     * output b5 INV05 Simultaneous Conduction Not Detected b6 INV06
     * Triangular Wave Modulation b7 INV07 Software Trigger Select Bit 
     */
    invc0 = 0x1C;

    /* 
     * Three-Phase PWM Control Register1 - 00110000b b0 INV10 Timers A1,
     * A2 & A4 Trigger : Underflow of Timer B2 b1 INV11 Timers A1-1, A2-1 
     * & A4-1 Control : Thee-phase mode 0 b2 INV12 Dead Time Timer Count
     * Source : f1 b3 INV13 Timer A reload control signal is 0 b4 INV14
     * Active High output b5 INV15 Dead Time Disable b6 INV16 Dead Time
     * Timer Trigger : Rising edge of the three-phase output shift
     * register b7 - Reserved 
     */
    invc1 = 0x40;

    /* 
     * Three-Phase Output Use pins U, U, V, V, W, and W of port P3 
     */
    tbsout = 1;

    /* 
     * Protects INVC0 & INVC1 Registers 
     */
    prc1 = 0;

    /* 
     * Timer B2 Reload every time an underflow occure 
     */
    tb2sc = 0x00;

    /* 
     * Three-Phase Output Buffer Register0 - 00111111b b0 DU0 U-Phase
     * Output Buffer0 ON b1 DUB0 U'-Phase Output Buffer0 ON b2 DV0
     * V-Phase Output Buffer0 ON b3 DVB0 V'-Phase Output Buffer0 ON
     * b4 DW0 W-Phase Output Buffer0 ON b5 DWB0 W'-Phase Output
     * Buffer0 OFF b7:b6 - Reserved 
     */
    idb0 = 0x1A;

    /* 
     * Three-Phase Output Buffer Register1 - 00000000b b0 DU1 U-Phase
     * Output Buffer1 OFF b1 DUB1 U'-Phase Output Buffer1 OFF b2 DV1 
     * V-Phase Output Buffer1 OFF b3 DVB1 V'-Phase Output Buffer1 OFF 
     * b4 , DW1 W-Phase Output Buffer1 OFF b5 DWB1 W'-Phase Output
     * Buffer1 OFF b7:b6 - Reserved 
     */
    idb1 = 0x25;

    /* 
     * Timer A4 Mode Register - 00010001b b1:b0 TMOD0:TMOD1 One shot
     * mode b2 MR0 Reserved b3 MR1 External Trigger Select Bit b4
     * MR2 Trigger selected by TRGSR register b5 MR3 Reserved b6:b7
     * TCK0:TCK1 Count Source f1 
     */
<<<<<<< HEAD
    ta4mr = 0x13; // Toon! Mootori reziim, t2itetegurit muudetakse
=======
    ta4mr = 0x12;
>>>>>>> 0af8a399d5edef392ad9bddf7f0a7d3cdd74b071

    /* 
     * Timer A1 Mode Register - 00010001b b1:b0 TMOD0:TMOD1 One shot
     * mode b2 MR0 Reserved b3 MR1 External Trigger Select Bit b4
     * MR2 Trigger selected by TRGSR register b5 MR3 Reserved b6:b7
     * TCK0:TCK1 Count Source f1 
     */
    ta1mr = 0x12;

    /* 
     * Timer A2 Mode Register - 00010001b b1:b0 TMOD0:TMOD1 One shot
     * mode b2 MR0 Reserved b3 MR1 External Trigger Select Bit b4
     * MR2 Trigger selected by TRGSR register b5 MR3 Reserved b6:b7
     * TCK0:TCK1 Count Source f1 
     */
    ta2mr = 0x12;

    /* 
     * Timer B2 Mode Register - 00000000b b1:b0 TMOD0:TMOD1 Timer
     * Mode b2:b3 mr0:mr1 Reserved b4 mr2 Reserved b5 mr3 Reserved
     * b6:b7 TCK0:TCK1 Count Source f1 
     */
    tb2mr = 0x00;

    /* 
     * Trigger Select Register - 01000101b b1:b0 TA1TGL:TA1TGH Timer
     * A1 Trigger the underflow of TB2 b2:b3 TA2TGL:TA2TGH Timer A2
     * Trigger the underflow of TB2 b4:b5 TA3TGL:TA3TGH Timer A3
     * Trigger : input to TA3IN pin b6:b7 TA4TGL:TA4TGH Timer A4
     * Trigger the underflow of TB2 
     */
    trgsr = 0x45;

    /* 
     * Configures 3-Phase motor control timers 
     */
    /* 
     * Loading the register 
     */
    tb2 = TIMERB2COUNT;

    /* 
     * Timer A4 register 
     */
<<<<<<< HEAD
    ta1 = 0;
    ta2 = 0;
    ta4 = 0;
=======
    ta1 = 1;
    ta2 = 1;
    ta4 = 1;
>>>>>>> 0af8a399d5edef392ad9bddf7f0a7d3cdd74b071

    /* 
     * Timer A1 register 
     */
    // ta1 = ((TIMERB2COUNT * 2) / 3);

    /* 
     * Timer A2 register 
     */
    // ta2 = ((TIMERB2COUNT * 1) / 3);

    /* 
     * Removes Protection for P3_6S,P3_2S & P3_4S Registers 
     */
    // prc30 = 1;
    /* 
     * Initialize port 3 for output 
     */
    pd3 = 0xFF;

    /* 
     * Port P3_2 Function Select Register - 00000010b b2:b0
     * PSEL2:PSEL0 Three-phase motor control output b7:b3 - No
     * register bits; 
     */
    p3_2s = 0x01;

    /* 
     * Port P3_,3 Function Select Register - 00000010b b2:b0
     * PSEL2:PSEL0 Three-phase motor control output b7:b3 - No
     * register bits; 
     */
    // p3_3s = 0x02;
    /* 
     * Port P3_4 Function Select Register - 00000010b b2:b0
     * PSEL2:PSEL0 Three-phase motor control output b7:b3 - No
     * register bits; 
     */
    p3_4s = 0x01;

    /* 
     * Port P3_5 Function Select Register - 00000010b b2:b0
     * PSEL2:PSEL0 Three-phase motor control output b7:b3 - No
     * register bits; 
     */
    // p3_5s = 0x02;
    /* 
     * Port P3_6 Function Select Register - 00000010b b2:b0
     * PSEL2:PSEL0 Three-phase motor control output b7:b3 - No
     * register bits 
     */
    p3_6s = 0x01;

    /* 
     * Port P3_7 Function Select Register - 00000010b b2:b0
     * PSEL2:PSEL0 Three-phase motor control output b7:b3 - No
     * register bits; 
     */
    // p3_7s = 0x02;
    p3_7s = 0;
    pd3_7 = 1;
    p3_7 = 0;

    /* 
     * Protects P3_6S,P3_2S & P3_4S Registers 
     */
    // prc30 = 0;
    /* 
     * Setting timer A4, timer A1, timer A2 and timer B2 start flag 
     */
    tabsr = 0x96;

    // FIXME, timer3 start into right place
    TABSR_bit.TA3S = 1;
}

void
HardwareSetup(void)
{
    /* 
     * Configures CPU clock 
     */
    DISABLE_IRQ;
    ConfigureOperatingFrequency(1);

    // Init_TMRB5 1 mS timer
    tb5mr = 0x80;                                          // timer mode,fc/8 = 1,0 MHz
    tb5 = 9999;                                            // 1MHz/25 - 1; Fi = 40kHz
    tb5ic = 1;                                             // level 1 interrupt
    tb5s = 1;
    ticks = 0;
    
    ConfigurePortPins();

<<<<<<< HEAD
    SPI3_Init(); // OLED
    SPI4_Init(); // Melexis sensor left
    SPI6_Init(); // gyro
    SPI7_Init(); // Melexis sensor right
    uart5_init(); // Panda
//    uart8_init(); // ?
=======
    SPI3_Init(); // OLED!
    SPI4_Init(); // Melexis sensor
    uart5_init();
    uart7_init();
    uart8_init();
>>>>>>> 0af8a399d5edef392ad9bddf7f0a7d3cdd74b071
#if 1
    OLED_On();
    OLED_Init();
#endif    
    ENABLE_IRQ;
    PWM_Init();

}

static void
ConfigureOperatingFrequency(char mode)
{
    unsigned short i;

    prr = 0xAA;
    ccr = 0x1F;
    prr = 0x00;
    prc0 = 1;
    pm3 = 0x40;                                        // peripheral clock 24MHz
    prc0 = 0;
    prc2 = 1;
    *(unsigned short *) &plc0 = 0x0226;                // 48MHz, PLL = 96MHz
    prc2 = 0;
    base_freq = 24000000;

    for (i = 0; i < 0x8000u; i++);                         /* Add delay
                                                            * for PLL to
                                                            * stabilise. */
    /* 
     * Disable the port pins 
     */
    pd8_7 = 0;
    pd8_6 = 0;

    /* 
     * Disable the pull-up resistor 
     */
    pu25 = 0;

    /* 
     * Enable writing to CM0 
     */
    prc0 = 1;

    /* 
     * Start the 32KHz crystal 
     */
    cm04 = 1;

    /* 
     * Disable writing to CM0 
     */
    prc0 = 0;

    /* 
     * Enable writing to PM2 
     */
    prc1 = 1;
    /* 
     * Disable clock changes 
     */
    pm21 = 1;
    pm26 = 1;                                              
    /* 
     * Disable writing to PM2 
     */
    prc1 = 0;
    cst_tcspr = 0;
    tcspr = 0x08;
    cst_tcspr = 1;                                        
}

static void
ConfigurePortPins(void)
{
    /* 
     * All pins are input by default 
     */
    pur0 = 0;                            

    pd0 = 0;
    pd1 = 4; // charge is output                                
    p0 = p1 = 0;
    p0_0s = p0_1s = p0_2s = p0_3s = p0_4s = p0_5s = p0_6s = p0_7s = 0;
    p1_0s = p1_1s = p1_3s = p1_4s = p1_5s = p1_6s = p1_7s = 0;

    p1_2s = 0; // charge
   
    pu02=1;
    pu03=1;

    p2 = 0x07;    
    pd2 = 0xFF;                             
    p2_0s = p2_1s = p2_2s = p2_3s = 0;
    p2_4s = p2_5s = p2_6s = p2_7s = 0x80;

    p3 = 0;                                                 
    pd3 = 0xAB;                                            
    p3_0s = p3_1s = p3_3s = p3_5s = p3_7s = 0;
    p3_2s = p3_4s = 0x01;// PWM ports
    pur1 = 0x04;        
    pd5 =  (1 << 7); 
    p5_0s = p5_1s = p5_2s = p5_3s = p5_5s = p5_7s = 0;
    p5_6s = p5_4s = 0x03;
   
    p5 = 0;                                                // 

    // P6_4...P6_7 is E8a
    pur2 = 0;       
    pd6_0 = 0;
    pd6_1 = 0;
    pd6_2 = 0;
    pd6_3 = 0;
    p6_2s = 3;
    p6_3s = 3;

    pd7 = 0x90;                                            
    p7 = 0;                                               
    p7_0s = p7_1s = p7_3s = p7_5s = p7_6s = 3;             
    p7_2s = p7_4s = p7_7s = 0;                             
    pd8 = 2;                                               
    p8 = 0;
    p8_0s = 3;

    p8_1s = 0;

    p8_2s = 0;                                             
    p8_3s = 0;                                             
    pd8_4 = 0;                                             
    p8_4s = 0;                                             
    pu24 = 1;  
    pd10 = 0;                                              
    p10 = 0;
    p10_1 = 0;
    p10_0s = p10_1s = p10_2s = p10_3s = p10_4s = p10_5s = p10_6s = p10_7s = 0x80;
// FIXME temportary hack
//    p1_2=1; // charge on
//    p3_5=1; // panda on

}

// 1000 Hz interrupt
#pragma vector=TIMER_A3
__interrupt void
ms_int(void)
{

}

#pragma vector=TIMER_B5
__interrupt void
<<<<<<< HEAD
s_int(void) {
    if(++ticks % 48 == 0) {
      status.sek_flag=1;
      ta4=(int)abs(64 -  p1_3*2 + p1_4*2 + p1_5*4 + p1_6*8 + p1_3*16 *TIMER4COUNT);
    }
//    tabsr = 0x0;

//    ta4=(int)abs(15 -  p1_3*32 + p1_4*2 + p1_5*4 + p1_6*8 + p1_3*16   *TIMER4COUNT);
//   tb2=(int)abs(15 -  p1_3*32 + p1_4*2 + p1_5*4 + p1_6*8 + p1_3*16   *TIMER4COUNT);
   // tabsr = 0x96;

#if 0
      p1_3 ? ""    :"up ",
      p1_4 ? ""  :"down ",
      p1_5 ? ""  :"left ",
      p1_6 ? "" :"right ",
      p1_7 ? "":"center" );
#endif   
   
     if(ticks % 48 == 1  ) {
         ta4=0;
    }
    
    
=======
s_int(void)
{
    if(++ticks % 48 == 0)
      status.sek_flag=1;
>>>>>>> 0af8a399d5edef392ad9bddf7f0a7d3cdd74b071
}

