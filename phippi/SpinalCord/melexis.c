/*
 *  Copyright (c) 2011,2012 Tonu Samuel
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
#include "main.h"
#include "SPI.h"

int mlx1whoamistatus=0;
int mlx2whoamistatus=0;
int mlxRerrcode=0;
int mlxLerrcode=0;
long int MLXLold=-1; // -1 indicates no old data known
long int MLXRold=-1;
unsigned int MLXLdata=0;
unsigned int MLXRdata=0;
unsigned char MLXLbyte1;
unsigned char MLXLbyte2;
unsigned char MLXLbyte3;
unsigned char MLXLbyte4;

unsigned char MLXRbyte1;
unsigned char MLXRbyte2;
unsigned char MLXRbyte3;
unsigned char MLXRbyte4;

char mlxRstatus=0,mlxLstatus=0;

void
SPI4_Init(void) { // Right Melexis 90316
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
    nch_u4c0 = 0;                                          // 0=Output mode "open drain" for TXD and CLOCK pin 
    ckpol_u4c0 = 1;                                        // CLK Polarity 0 rising edge, 1 falling edge (1 - OK)
    uform_u4c0 = 1;                                        // 1=MSB first (1 OK)

    te_u4c1 = 1;                                           // 1=Transmission Enable
    ti_u4c1 = 0;                                           // Must be 0 to send or receive
    re_u4c1 = 1;                                           // Reception Enable when 1
    ri_u4c1 = 0;                                           // Receive complete flag - U4RB is empty.
    u4irs_u4c1 = 1;                                        // Interrupt  when transmission  is completed, U4TB is empty. 
    u4rrm_u4c1 = 0;                                        // Continuous receive mode off
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
    // 0x60 produces 8uS waveform which is 125kHz
    u4brg = 0x60;                                          // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);    
    
    pu27=1;
    DISABLE_IRQ
    /* 
     * Lowest interrupt priority
     * we do not care about speed
     */
    ilvl_s4ric =1; 
    ir_s4ric   =0;            
    ENABLE_IRQ

}

#pragma vector = UART4_RX
__interrupt void _uart4_receive(void) {
    ERRORLED=1;

  /* Used to reference a specific location in the array while string the
  received data.   */
  unsigned char b=u4rb; 
  /*
  This chip protocol:
  2 bytes sent to chip - AA, FF
  2 bytes responded with data
  2 bytes of same data inverted responded
  4 bytes of 0xFF (we ignore them)
  10 in total.
  From state machine perspective 0 and 1 are AA,FF
  The 2 and 3 contain some data, rest can be jsut ignored
  But we read also 4...5 and implement error checking on it
  */
  switch(mlx1whoamistatus) {
  case 2:
      MLXLbyte1=(int)b;
      ta3  = 25; // Set timer 50us 
      ta3os = 1; // start timer
      break;
  case 3:
      MLXLbyte2=(int)b;
      ta3  = 25; // Set timer 50us 
      ta3os = 1; // start timer
      break;
  case 4:
      MLXLbyte3=(int)b;
      ta3  = 25; // Set timer 50us 
      ta3os = 1; // start timer
      break;
  case 5:
      MLXLbyte4=(int)b;
      uDelay(200); 
      if( MLXLbyte1 == (unsigned char)~ MLXLbyte3 &&  MLXLbyte2 == (unsigned char)~ MLXLbyte4) {
          if((MLXLbyte2 & 3) == 2) { // error code, not angular data
              mlxLstatus=1;
              mlxLerrcode=MLXLbyte2 >> 2; 
          } else {
              MLXLdata = ((unsigned int) MLXLbyte1 << 6) | ((unsigned int) MLXLbyte2 >>  2) ;

              signed int change=0;
              if(MLXLold != -1) // If old value is known at all
                 change = (signed int)MLXLdata - MLXLold ;  
              if(change < (-16384/2))
                change+=16384;
              if(change > (16384/2))
                change-=16384;
              MLXaccumulatorL+=change;              
              mlxRstatus=2;
              MLXLold = MLXLdata;
          }
      } else {
          mlxLstatus=3;
      }
      u4tb=0xFF;
      break;
  case 6:
      uDelay(25); // t4, 8+uS on scope, 6.9 required
      CS4=1;
      ta3  = 50*15; // Set timer 1500us or 1.5ms
      ta3os = 1; // start timer
      break;
  default:   
      if(mlx1whoamistatus==1) // no need for delay
          u4tb=0xFF;
      else {
          ta3  = 25; // Set timer 50us 
          ta3os = 1; // start timer
      }
  } 
  mlx1whoamistatus++;
  ir_s4ric = 0;
    ERRORLED=0;
  
}


void
SPI7_Init(void) { // Left Melexis 90316
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

    clk0_u7c0 = 0;                                         // Clock source f1 for u4brg
    clk1_u7c0 = 0;                                         // 
    txept_u7c0 = 0;                                        // Transmit register empty flag 
    crd_u7c0 = 1;                                          // CTS disabled when 1
    ckpol_u7c0 = 1;                                        // CLK Polarity 
    uform_u7c0 = 1;                                        // 1=MSB first

    te_u7c1 = 1;                                           // 1=Transmission Enable
    ti_u7c1 = 0;                                           // Must be 0 to send or receive
    re_u7c1 = 1;                                           // Reception Enable when 1
    ri_u7c1 = 0;                                           // Receive complete flag - U4RB is empty.
    u7irs = 1;                                        // Interrupt  when transmission  is completed, U4TB is empty. 
    u7rrm = 0;                                        // Continuous receive mode off
    u7brg = 0x60;                                             // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);

    DISABLE_IRQ
    /* 
     * Lowest interrupt priority
     * we do not care about speed
     */
    ilvl_s7ric =1;   
    ir_s7ric   =0;            
    ENABLE_IRQ 

}

signed int  MLXaccumulatorL=0LL;
signed int  MLXaccumulatorR=0LL;

#pragma vector = UART7_RX
__interrupt void _uart7_receive(void) {
    ERRORLED=1;


  /* Used to reference a specific location in the array while string the
  received data.   */
  unsigned char b=u7rb; 
  /*
  This chip protocol:
  2 bytes sent to chip - AA, FF
  2 bytes responded with data
  2 bytes of same data inverted responded
  4 bytes of 0xFF
  10 in total.
  From state machine perspective 0 and 1 are AA,FF
  The 2 and 3 contain some data, rest can be jsut ignored
  But we read also 4...5 and implement error checking on it
  */
  switch(mlx2whoamistatus) {
  case 2:
      MLXRbyte1=(int)b;
      ta0  = 25; // Set timer 50us 
      ta0os = 1; // start timer
      break;
  case 3:
      MLXRbyte2=(int)b;
      ta0  = 25; // Set timer 50us 
      ta0os = 1; // start timer
      break;
  case 4:
      MLXRbyte3=(int)b;
      ta0  = 25; // Set timer 50us 
      ta0os = 1; // start timer
      break;
  case 5:
      MLXRbyte4=(int)b;
      uDelay(200); 
      if( MLXRbyte1 == (unsigned char)~ MLXRbyte3 &&  MLXRbyte2 == (unsigned char)~ MLXRbyte4) {
          if((MLXRbyte2 & 3) == 2) { // error code, not angular data
              mlxRstatus=1;
              mlxRerrcode=MLXRbyte2 >> 2; 
          } else {
              MLXRdata = ((unsigned int) MLXRbyte1 << 6) | ((unsigned int) MLXRbyte2 >>  2) ;
              signed int change=0;
              if(MLXRold != -1) // If old value is known at all
                 change = (signed int)MLXRdata - MLXRold ;  
              if(change < (-16384/2))
                change+=16384;
              if(change > (16384/2))
                change-=16384;
              MLXaccumulatorR+=change;    
              mlxRstatus=2;
              MLXRold = MLXRdata;
          }
      } else {
          mlxRstatus=3;
      }
      u7tb=0xFF;
      break;
  case 6:
      uDelay(25); // t4, 8+uS on scope, 6.9 required
      CS7=1;
      ta0  = 50*15; // Set timer 1500us or 1.5ms
      ta0os = 1; // start timer
      break;
  default:   
      if(mlx2whoamistatus==1) // no need for delay
          u7tb=0xFF;
      else {
          ta0  = 25; // Set timer 50us 
          ta0os = 1; // start timer
      }
  }
  mlx2whoamistatus++;

  /* Clear the 'reception complete' flag. */
  ir_s7ric = 0;
    ERRORLED=0;
  
}
