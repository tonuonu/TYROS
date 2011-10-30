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

    SPI3_Init(); // OLED!
    SPI4_Init(); // Melexis sensor
    uart5_init();
    uart7_init();
    uart8_init();
#if 1
    OLED_On();
    OLED_Init();
#endif    
    ENABLE_IRQ;
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
    pd2 = 0x0F;                             
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
}

// 1000 Hz interrupt
#pragma vector=TIMER_A3
__interrupt void
ms_int(void)
{

}

#pragma vector=TIMER_B5
__interrupt void
s_int(void)
{
    if(++ticks % 48 == 0)
      status.sek_flag=1;    
}

