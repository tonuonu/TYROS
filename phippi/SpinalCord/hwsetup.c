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

#define TIMERB2COUNT	100
#define TIMER4COUNT	100

volatile struct statuses status;
unsigned int base_freq;

#if 0
static void
ConfigurePortPins(void) {
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

    // port 2 is all about motor driving simple digital signals
    // usually all 8 bits are outputs
    p2 = 0x00;    
    pd2 = 0xFF;                             
    RIGHT_INAs = RIGHT_INBs = LEFT_INAs = LEFT_INBs = 0;
    RIGHT_DIAGAs = RIGHT_DIAGBs = LEFT_DIAGAs = LEFT_DIAGBs = 0;

    // port 3 contains some PWMs for motors and buzzer
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
#endif

static void
ConfigureOperatingFrequency(char mode) {
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

void 
OLED_On(void) {
    // Ports are output
    OLED_DATACOMMANDd = PD_OUTPUT;                                             
    OLED_RESETd       = PD_OUTPUT;   // Reset pin
    OLED_ENABLEd      = PD_OUTPUT;   // VCC
    OLED_VDDd         = PD_OUTPUT;   // VDD is reversed (0-on, 1-off), but switched ON first, so keep it like that  :)

    OLED_CSd          = PD_OUTPUT;
    
    // Make sure VCC is Off
    OLED_ENABLE = 0;
    uDelay(10);

    // Reset sequence
    OLED_RESET = 1;
    uDelay(100);
    OLED_RESET = 0;
    uDelay(100);
    OLED_RESET = 1;
    uDelay(100);

    // Enable VCC
    OLED_ENABLE = 1;
    uDelay(100);
}


static void 
PWM_Init(void) {
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

    prc1 = 1;
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
    ta4mr = 0x13; // Toon! Mootori reziim, t2itetegurit muudetakse

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
    ta1 = 0;
    ta2 = 0;
    ta4 = 0;

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
     * Port P3_4 Function Select Register - 00000010b b2:b0
     * PSEL2:PSEL0 Three-phase motor control output b7:b3 - No
     * register bits; 
     */
    p3_4s = 0x01;

    /* 
     * Setting timer A4, timer A1, timer A2 and timer B2 start flag 
     */
    tabsr = 0x96;

    LEFT_INAd    = PD_OUTPUT;
    LEFT_INBd    = PD_OUTPUT;
    RIGHT_INAd   = PD_OUTPUT;
    RIGHT_INBd   = PD_OUTPUT;

    LEFT_DIAGAd  = PD_OUTPUT;
    LEFT_DIAGBd  = PD_OUTPUT;
    RIGHT_DIAGAd = PD_OUTPUT;
    RIGHT_DIAGBd = PD_OUTPUT;
    
    LEFT_PWMd    = PD_OUTPUT;
    LEFT_PWMs    = PF_TIMER;
    RIGHT_PWMd   = PD_OUTPUT;
    RIGHT_PWMs   = PF_TIMER;

    // FIXME, timer3 start into right place
    TABSR_bit.TA3S = 1;
}

static void 
Heartbeat_Init(void) {
    // Init_TMRB5 1 mS timer
    tb5mr = 0x80;                                          // timer mode,fc/8 = 1,0 MHz
    tb5 = 9999;                                            // 1MHz/25 - 1; Fi = 40kHz
    tb5ic = 1;                                             // level 1 interrupt
    tb5s = 1;
    ticks = 0;
    LED1d  = PD_OUTPUT; 
    BUZZERd  = PD_OUTPUT; 
}

static void 
Buzzer_Init(void) {
    BUZZERs = PF_MOTOR;
    BUZZERd = PD_OUTPUT; 
}

static void 
Panda_Init(void) {
    PANDAd = PD_OUTPUT; 
    PANDA  = 0;
}

static void 
Coilgun_Init(void) {
    KICKd = PD_OUTPUT; 
    pu22 = 1; // ball detect p7_3 pullup on
}

static void 
CapacitorCharger_Init(void) {
    CHARGEd = PD_OUTPUT; 
}

static void 
Analog_Init(void) {

}

static void 
Joy_Init(void) {
//    JOY_LEFTd = PD_OUTPUT;
//    JOY_RIGHTd = PD_OUTPUT;
//    JOY_UPd = PD_OUTPUT;
//    JOY_DOWNd = PD_OUTPUT;
//    JOY_CENTERd = PD_OUTPUT;
    pu02 = 1; // P1_0 to P1_3 Pull-up Control Bit
    pu03 = 1; // P1_4 to P1_7 Pull-up Control Bit
}

void
HardwareSetup(void) {
    /* 
     * Configures CPU clock 
     */
    DISABLE_IRQ;
    ConfigureOperatingFrequency(1);
//    ConfigurePortPins();

    Buzzer_Init();
    Heartbeat_Init();
    Analog_Init();
    CapacitorCharger_Init();
    Coilgun_Init();
    Panda_Init();
    Joy_Init();
    SPI0_Init();  // Accel sensor left
    SPI2_Init();  // Accel sensor right
    SPI3_Init();  // OLED
    SPI4_Init();  // Melexis sensor left
    uart5_init(); // Panda
    SPI6_Init();  // gyro
    SPI7_Init();  // Melexis sensor right
    OLED_On();    // ?
    OLED_Init();
    ENABLE_IRQ;
    PWM_Init();
}


// 1000 Hz interrupt
#pragma vector=TIMER_A3
__interrupt void
ms_int(void) {

}

#pragma vector=TIMER_B5
__interrupt void
s_int(void) {
    // This interrupt gets called 48 times per second
  
    // We may want to do something once per second in main loop
    // so we set flag to indicate when to do.
    if(++ticks % 48 == 0) {
      status.sek_flag=1;
      // This turns on PWM on buzzer
        ta4=(int)abs(64 -  p1_3*2 + p1_4*2 + p1_5*4 + p1_6*8 + p1_3*16 *TIMER4COUNT);
    } else if(ticks % 48 == 1  ) {
        // Turn off buzzer
        ta4=0;
    }
    
    // Make sure pwm-s get closer to targets but not too fast. 
    if(pwmtarget[0] < pwm[0]) {
        pwm[0]--;
    } else if(pwmtarget[0] > pwm[0]) { 
        pwm[0]++;
    }
    
    if(pwmtarget[1] < pwm[1]) {
        pwm[0]--;
    } else if(pwmtarget[1] > pwm[1]) {
        pwm[1]++;
    }

    // Update MCU PWM timers for new values
    ta1=(int)abs(pwm[0]*TIMERB2COUNT);
    ta2=(int)abs(pwm[1]*TIMERB2COUNT);


    // Make sure proper bits set on motor drivers to go forward or backward
    if(pwm[0] > 0) {
        RIGHT_INA=1; // right in a
        RIGHT_INB=0; // right in b
    } else {
        RIGHT_INA=0; // right in a
        RIGHT_INB=1; // right in b
    }
    
    if(pwm[1] > 0) {
        LEFT_INA=1; // left in a
        LEFT_INB=0; // left in b
    } else {
        LEFT_INA=0; // left in a
        LEFT_INB=1; // left in b
    }

    // Enable motors
    RIGHT_DIAGA=1; // right diag a (enable)
    RIGHT_DIAGB=1; // right diag b (enable)
    LEFT_DIAGA=1; // left diag a (enable)
    LEFT_DIAGB=1; // left diag b (enable)
        
    // Reduce PWM targets for next turn. This makes motors slow down in 
    // ~2 seconds if no new commands are received.
    if(pwmtarget[0] > 0) {
        pwmtarget[0]--;
    } else if(pwmtarget[0] < 0) {
        pwmtarget[0]++;
    }

    if(pwmtarget[1] > 0) {
        pwmtarget[1]--;
    } else if(pwmtarget[1] < 0) { 
        pwmtarget[1]++;
    }
    
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
}

