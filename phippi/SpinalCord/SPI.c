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
#include <intrinsics.h>
#include "hwsetup.h"


void
SPI0_Init(void) { // Accel sensor left
    /* 
     * CS - no CS on accel sensor?!
     */
//    prc2=1;
//    pd9_4 = PD_OUTPUT;
//    p9_4=1;

    /* 
     * CLK0
     */
//    prc2=1;
    CLOCK6d = PD_OUTPUT;

//    prc2=1;
    CLOCK6s = PF_UART;
    /* 
     * TXD0
     */
//    prc2=1;
    TX0d = PD_OUTPUT;

//    prc2=1;
    TX0s = PF_UART;
    
//    pu27=1;

    smd0_u0mr  = 1;                                        // \ 
    smd1_u0mr  = 0;                                        //  | Synchronous Serial Mode
    smd2_u0mr  = 0;                                        // /

    ckdir_u0mr = 0;                                        // 0=internal clock 
    
    stps_u0mr  = 0;                                        // 0=1 stop bit, 0 required
    pry_u0mr   = 0;                                        // Parity, 0=odd, 0 required 
    prye_u0mr  = 0;                                        // Parity Enable? 0=disable, 0 required 
    iopol_u0mr = 0;                                        // IO Polarity, 0=not inverted, 0 required

    clk0_u0c0 = 0;                                         // Clock source f1 for u4brg
    clk1_u0c0 = 0;                                         // 
    txept_u0c0 = 0;                                        // Transmit register empty flag 
    crd_u0c0 = 1;                                          // CTS disabled when 1
    nch_u0c0 = 1;                                          // 0=Output mode "open drain" for TXD and CLOCK pin 
    ckpol_u0c0 = 1;                                        // CLK Polarity 
    uform_u0c0 = 1;                                        // 1=MSB first

    te_u0c1 = 1;                                           // 1=Transmission Enable
    ti_u0c1 = 0;                                           // Must be 0 to send or receive
    re_u0c1 = 1;                                           // Reception Enable when 1
    ri_u0c1 = 0;                                           // Receive complete flag - U4RB is empty.
    u0irs_u0c1 = 0;                                        // Interrupt  when transmission  is completed, U4TB is empty. 
    u0rrm_u0c1 = 1;                                        // Continuous receive mode off
    u0lch_u0c1 = 0;                                        // Logical inversion off 

    u0smr = 0x00;                                          // Set 0 
    u0smr2 = 0x00;                                         // Set 0 

    sse_u0smr3 = 0;                                        // SS is disabled when 0
    ckph_u0smr3 = 0;                                       // Non clock delayed 
    dinc_u0smr3 = 0;                                       // Master mode when 0
    nodc_u0smr3 = 0;                                       // Select a clock output  mode "push-pull" when 0 
    err_u0smr3 = 0;                                        // Error flag, no error when 0 
    dl0_u0smr3 = 0;                                        // Set 0 for no  delay 
    dl1_u0smr3 = 0;                                        // Set 0 for no  delay 
    dl2_u0smr3 = 0;                                        // Set 0 for no  delay 

    u0smr4 = 0x00;                                         // Set 0. u4c0 must be set before this function

//    u4brg = 55 /* 435kHz */ ;                                             // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);
    u0brg = 0x55;                                             // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);
    
    
//    pu27=1;
}

void
SPI2_Init(void) { // Accel sensor right
    /* 
     * CS - no CS on accel sensor?!
     */
//    prc2=1;
//    pd9_4 = PD_OUTPUT;
//    p9_4=1;

    /* 
     * CLK0
     */
//    prc2=1;
    pd7_2 = PD_OUTPUT;

//    prc2=1;
    p7_2s = PF_UART;
    /* 
     * TXD0
     */
//    prc2=1;
    pd7_0 = PD_OUTPUT;

//    prc2=1;
    p7_0s = PF_UART;
    
//    pu27=1;

    smd0_u2mr  = 1;                                        // \ 
    smd1_u2mr  = 0;                                        //  | Synchronous Serial Mode
    smd2_u2mr  = 0;                                        // /

    ckdir_u2mr = 0;                                        // 0=internal clock 
    
    stps_u2mr  = 0;                                        // 0=1 stop bit, 0 required
    pry_u2mr   = 0;                                        // Parity, 0=odd, 0 required 
    prye_u2mr  = 0;                                        // Parity Enable? 0=disable, 0 required 
    iopol_u2mr = 0;                                        // IO Polarity, 0=not inverted, 0 required

    clk0_u2c0 = 0;                                         // Clock source f1 for u4brg
    clk1_u2c0 = 0;                                         // 
    txept_u2c0 = 0;                                        // Transmit register empty flag 
    crd_u2c0 = 1;                                          // CTS disabled when 1
    nch_u2c0 = 1;                                          // 0=Output mode "open drain" for TXD and CLOCK pin 
    ckpol_u2c0 = 1;                                        // CLK Polarity 
    uform_u2c0 = 1;                                        // 1=MSB first

    te_u2c1 = 1;                                           // 1=Transmission Enable
    ti_u2c1 = 0;                                           // Must be 0 to send or receive
    re_u2c1 = 1;                                           // Reception Enable when 1
    ri_u2c1 = 0;                                           // Receive complete flag - U4RB is empty.
    u2irs_u2c1 = 0;                                        // Interrupt  when transmission  is completed, U4TB is empty. 
    u2rrm_u2c1 = 1;                                        // Continuous receive mode off
    u2lch_u2c1 = 0;                                        // Logical inversion off 

    u2smr = 0x00;                                          // Set 0 
    u2smr2 = 0x00;                                         // Set 0 

    sse_u2smr3 = 0;                                        // SS is disabled when 0
    ckph_u2smr3 = 0;                                       // Non clock delayed 
    dinc_u2smr3 = 0;                                       // Master mode when 0
    nodc_u2smr3 = 0;                                       // Select a clock output  mode "push-pull" when 0 
    err_u2smr3 = 0;                                        // Error flag, no error when 0 
    dl0_u2smr3 = 0;                                        // Set 0 for no  delay 
    dl1_u2smr3 = 0;                                        // Set 0 for no  delay 
    dl2_u2smr3 = 0;                                        // Set 0 for no  delay 

    u2smr4 = 0x00;                                         // Set 0. u4c0 must be set before this function

//    u4brg = 55 /* 435kHz */ ;                                             // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);
    u2brg = 0x55;                                             // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);
    
    
//    pu27=1;
}

void
SPI4_Init(void) { // Melexis 90316
    /* 
     * CS
     */
    prc2=1;
    pd9_4 = PD_OUTPUT;
    p9_4=1;

    /* 
     * CLK4 
     */
    prc2=1;
    CLOCK4d = PD_OUTPUT;

    prc2=1;
    CLOCK4s = PF_UART;
    /* 
     * TXD4 
     */
    prc2=1;
    TX4d = PD_OUTPUT;

    prc2=1;
    TX4s = PF_UART;
    
//    pu27=1;

    smd0_u4mr  = 1;                                        // \ 
    smd1_u4mr  = 0;                                        //  | Synchronous Serial Mode
    smd2_u4mr  = 0;                                        // /

    ckdir_u4mr = 0;                                        // 0=internal clock 
    
    stps_u4mr  = 0;                                        // 0=1 stop bit, 0 required
    pry_u4mr   = 0;                                        // Parity, 0=odd, 0 required 
    prye_u4mr  = 0;                                        // Parity Enable? 0=disable, 0 required 
    iopol_u4mr = 0;                                        // IO Polarity, 0=not inverted, 0 required

    clk0_u4c0 = 0;                                         // Clock source f1 for u4brg
    clk1_u4c0 = 0;                                         // 
    txept_u4c0 = 0;                                        // Transmit register empty flag 
    crd_u4c0 = 1;                                          // CTS disabled when 1
    nch_u4c0 = 1;                                          // 0=Output mode "open drain" for TXD and CLOCK pin 
    ckpol_u4c0 = 1;                                        // CLK Polarity 
    uform_u4c0 = 1;                                        // 1=MSB first

    te_u4c1 = 1;                                           // 1=Transmission Enable
    ti_u4c1 = 0;                                           // Must be 0 to send or receive
    re_u4c1 = 1;                                           // Reception Enable when 1
    ri_u4c1 = 0;                                           // Receive complete flag - U4RB is empty.
    u4irs_u4c1 = 0;                                        // Interrupt  when transmission  is completed, U4TB is empty. 
    u4rrm_u4c1 = 1;                                        // Continuous receive mode off
    u4lch_u4c1 = 0;                                        // Logical inversion off 

    u4smr = 0x00;                                          // Set 0 
    u4smr2 = 0x00;                                         // Set 0 

    sse_u4smr3 = 0;                                        // SS is disabled when 0
    ckph_u4smr3 = 0;                                       // Non clock delayed 
    dinc_u4smr3 = 0;                                       // Master mode when 0
    nodc_u4smr3 = 0;                                       // Select a clock output  mode "push-pull" when 0 
    err_u4smr3 = 0;                                        // Error flag, no error when 0 
    dl0_u4smr3 = 0;                                        // Set 0 for no  delay 
    dl1_u4smr3 = 0;                                        // Set 0 for no  delay 
    dl2_u4smr3 = 0;                                        // Set 0 for no  delay 
    u4smr4 = 0x00;                                         // Set 0. u4c0 must be set before this function

//    u4brg = 55 /* 435kHz */ ;                                             // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);
    u4brg = 0x55;                                             // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);    
    pu27=1;
}

void
SPI6_Init(void) { // Gyro
    CS6d = PD_OUTPUT;
    CS6=1; // CS is high, means disabled

    CLOCK6d = PD_OUTPUT;
    CLOCK6s = PF_UART;

    TX6d = PD_OUTPUT;
    TX6s = PF_UART;

    RX6s = PF_UART;
    
    smd0_u6mr  = 1;                                        // \ 
    smd1_u6mr  = 0;                                        //  | Synchronous Serial Mode
    smd2_u6mr  = 0;                                        // /

    ckdir_u6mr = 0;                                        // 0=internal clock 
    
    stps_u6mr  = 0;                                        // 0=1 stop bit, 0 required
    pry_u6mr   = 0;                                        // Parity, 0=odd, 0 required 
    prye_u6mr  = 0;                                        // Parity Enable? 0=disable, 0 required 
    iopol_u6mr = 0;                                        // IO Polarity, 0=not inverted, 0 required

    clk0_u6c0 = 0;                                         // Clock source f1 for u4brg
    clk1_u6c0 = 0;                                         // 
    txept_u6c0 = 0;                                        // Transmit register empty flag 
    crd_u6c0 = 1;                                          // CTS disabled when 1
    nch_u6c0 = 0;                                          // 0=Output mode "open drain" for TXD and CLOCK pin 
    ckpol_u6c0 = 1;                                        // CLK Polarity 
    uform_u6c0 = 1;                                        // 1=MSB first

    te_u6c1 = 1;                                          // 1=Transmission Enable
    ti_u6c1 = 0;                                           // Must be 0 to send or receive
    re_u6c1 = 1;                                           // Reception Enable when 1
    ri_u6c1 = 0;                                           // Receive complete flag - U4RB is empty.
    u6irs_u6c1 = 0;                                        // Interrupt  when transmission  is completed, U4TB is empty. 
    u6rrm_u6c1 = 1;                                        // Continuous receive mode off
    u6lch_u6c1 = 0;                                        // Logical inversion off 

    u6smr = 0x00;                                          // Set 0 
    u6smr2 = 0x00;                                         // Set 0 

    sse_u6smr3 = 0;                                        // SS is disabled when 0
    ckph_u6smr3 = 0;                                       // Non clock delayed 
    dinc_u6smr3 = 0;                                       // Master mode when 0
    nodc_u6smr3 = 0;                                       // Select a clock output  mode "push-pull" when 0 
    err_u6smr3 = 0;                                        // Error flag, no error when 0 
    dl0_u6smr3 = 0;                                        // Set 0 for no  delay 
    dl1_u6smr3 = 0;                                        // Set 0 for no  delay 
    dl2_u6smr3 = 0;                                        // Set 0 for no  delay 

    u6smr4 = 0x00;                                         // Set 0. u4c0 must be set before this function

//    u4brg = 55 /* 435kHz */ ;                                             // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);
    u6brg = 0x10;                                             // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);

    pu11 = 1; // gyro RX interface needs pullup on RX^ or p4_6
}

void
SPI7_Init(void) { // Melexis 90316
    CS7d = PD_OUTPUT;
    CS7 = 1;// CS is high, means disabled

    CLOCK7d = PD_OUTPUT;
    CLOCK7s = PF_UART;

    TX7d = PD_OUTPUT;
    TX7s = PF_UART;
    
    smd0_u7mr  = 1;                                        // \ 
    smd1_u7mr  = 0;                                        //  | Synchronous Serial Mode
    smd2_u7mr  = 0;                                        // /

    ckdir_u7mr = 0;                                        // 0=internal clock 
    
    stps_u7mr  = 0;                                        // 0=1 stop bit, 0 required
    pry_u7mr   = 0;                                        // Parity, 0=odd, 0 required 
    prye_u7mr  = 0;                                        // Parity Enable? 0=disable, 0 required 
//    iopol_u7mr = 0;                                        // IO Polarity, 0=not inverted, 0 required

    clk0_u7c0 = 0;                                         // Clock source f1 for u4brg
    clk1_u7c0 = 0;                                         // 
    txept_u7c0 = 0;                                        // Transmit register empty flag 
    crd_u7c0 = 1;                                          // CTS disabled when 1
//    nch_u7c0 = 1;                                          // 0=Output mode "open drain" for TXD and CLOCK pin 
    ckpol_u7c0 = 1;                                        // CLK Polarity 
    uform_u7c0 = 1;                                        // 1=MSB first

    te_u7c1 = 1;                                           // 1=Transmission Enable
    ti_u7c1 = 0;                                           // Must be 0 to send or receive
    re_u7c1 = 1;                                           // Reception Enable when 1
    ri_u7c1 = 0;                                           // Receive complete flag - U4RB is empty.
#if 0
    u7irs_u7c1 = 0;                                        // Interrupt  when transmission  is completed, U4TB is empty. 
    u7rrm_u7c1 = 1;                                        // Continuous receive mode off
    u7lch_u7c1 = 0;                                        // Logical inversion off 

    u7smr = 0x00;                                          // Set 0 
    u7smr2 = 0x00;                                         // Set 0 

    sse_u7smr3 = 0;                                        // SS is disabled when 0
    ckph_u7smr3 = 0;                                       // Non clock delayed 
    dinc_u7smr3 = 0;                                       // Master mode when 0
    nodc_u7smr3 = 0;                                       // Select a clock output  mode "push-pull" when 0 
    err_u7smr3 = 0;                                        // Error flag, no error when 0 
    dl0_u7smr3 = 0;                                        // Set 0 for no  delay 
    dl1_u7smr3 = 0;                                        // Set 0 for no  delay 
    dl2_u7smr3 = 0;                                        // Set 0 for no  delay 

    u7smr4 = 0x00;                                         // Set 0. u4c0 must be set before this function
#endif
//    u4brg = 55 /* 435kHz */ ;                                             // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);
    u7brg = 0x55;                                             // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);

}

void
SPI3_Init(void) // OLED
{
    CLOCK3d = PD_OUTPUT;
    CLOCK3s = PF_UART;

    TX3d = PD_OUTPUT;
    TX3s = PF_UART;
    
    smd0_u3mr = 1;                                        // \ 
    smd1_u3mr = 0;                                         // >    // Synchronous Serial Mode
    smd2_u3mr = 0;                                         // /
    ckdir_u3mr = 0;                                        // internal clock , 243
    stps_u3mr = 0;                                         // 0 required
    pry_u3mr = 0;                                          // 0 required
    prye_u3mr = 0;                                         // 0 required
    iopol_u3mr = 0;                                        // 0 required
    clk0_u3c0 = 0;                                         // \ Clock
    clk1_u3c0 = 0;                                         // /
    txept_u3c0 = 0;                                        // Transmit
    crd_u3c0 = 1;                                          // CTS disabled 
    nch_u3c0 = 0;                                          // Output mode
    ckpol_u3c0 = 0;                                        // Polarity
    uform_u3c0 = 1;                                        // MSB first
    te_u3c1 = 1;                                           // Transmission 
    ti_u3c1 = 0;                                           // Must be 0 to 
    re_u3c1 = 0;                                           // Reception is 
    ri_u3c1 = 0;                                           // Receive
    u3irs_u3c1 = 0;                                        // 1 when
    u3rrm_u3c1 = 1;                                        // Continuous
    u3lch_u3c1 = 0;                                        // Logical
    u3smr = 0x00;                                          // Set 0
    u3smr2 = 0x00;                                         // Set 0 
    sse_u3smr3 = 0;                                        // SS is
    ckph_u3smr3 = 0;                                       // Non clock
    dinc_u3smr3 = 0;                                       // Master mode
    nodc_u3smr3 = 0;                                       // Select a
    err_u3smr3 = 0;                                        // Error flag,
    dl0_u3smr3 = 0;                                        // Set 0 for no 
    dl1_u3smr3 = 0;                                        // Set 0 for no 
    dl2_u3smr3 = 0;                                        // Set 0 for no 
    u3smr4 = 0x00;                                         // Set 0 (page
    u3brg = 3;                                             
    s3tic = 0x0;
}

