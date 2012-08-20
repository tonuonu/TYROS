/*
 *  Copyright (c) 2011, Tonu Samuel
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

unsigned int mlx1data=0;
unsigned int mlx2data=0;
unsigned char tmpmlx1data1;
unsigned char tmpmlx1data2;
unsigned char tmpmlx1data3;
unsigned char tmpmlx1data4;

unsigned char tmpmlx2data1;
unsigned char tmpmlx2data2;
unsigned char tmpmlx2data3;
unsigned char tmpmlx2data4;

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
    ilvl_s4ric =0x06;   
    ir_s4ric   =0;            
    ENABLE_IRQ

}

#pragma vector = UART4_RX
__interrupt void _uart4_receive(void) {
  ERRORLED=1;
    int j;
  /* Used to reference a specific location in the array while string the
  received data.   */
  unsigned char b=u4rb; 
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
  switch(mlx1whoamistatus) {
  case 2:
      tmpmlx1data1=(int)b;
      uDelay(200); 
      u4tb=0xFF;
      break;
  case 3:
      tmpmlx1data2=(int)b;
      uDelay(200); 
      u4tb=0xFF;
      break;
  case 4:
      tmpmlx1data3=(int)b;
      uDelay(200); 
      u4tb=0xFF;
      break;
  case 5:
      tmpmlx1data4=(int)b;
      uDelay(200); 
      if( tmpmlx1data1 == (unsigned char)~ tmpmlx1data3 &&  tmpmlx1data2 == (unsigned char)~ tmpmlx1data4) {
          if((tmpmlx1data2 & 3) == 2) { // error code, not angular data
          //    mlx1data = 999;
          } else {
              mlx1data = ((unsigned int) tmpmlx1data1 << 6) | ((unsigned int) tmpmlx1data2 >>  2) ;
          }
      }
      u4tb=0xFF;
      break;
  case 9:
      uDelay(25); // t4, 8+uS on scope, 6.9 required
      CS4=1;
      for(j=0;j<25;j++)            
          uDelay(255); // t5, 300/1500uS required, 1600 on scope
      CS4=0;
      uDelay(6); // t6, 10+uS on scope, 6.9 required
      u4tb=0xAA;
      mlx1whoamistatus=-1;
      break;
  default:        
      uDelay(200); // t2 and t7, 15uS/45uS required,  
      u4tb=0xFF;
  } 
  mlx1whoamistatus++;

  /* Clear the 'reception complete' flag. */
  ir_s4ric = 0;
  
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
    u7irs = 1;                                        // Interrupt  when transmission  is completed, U4TB is empty. 
    u7rrm = 0;                                        // Continuous receive mode off
#if 0
    // All "undefined" errors here
    u7lch = 0;                                        // Logical inversion off 

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
    u7brg = 0x60;                                             // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);

    DISABLE_IRQ
    ilvl_s7ric =0x07;   
    ir_s7ric   =0;            
    ENABLE_IRQ
    
    

}


#pragma vector = UART7_RX
__interrupt void _uart7_receive(void) {
  ERRORLED=1;
    int j;
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
      tmpmlx2data1=(int)b;
      uDelay(200); 
      u7tb=0xFF;
      break;
  case 3:
      tmpmlx2data2=(int)b;
      uDelay(200); 
      u7tb=0xFF;
      break;
  case 4:
      tmpmlx2data3=(int)b;
      uDelay(200); 
      u7tb=0xFF;
      break;
  case 5:
      tmpmlx2data4=(int)b;
      uDelay(200); 
      if( tmpmlx2data1 == (unsigned char)~ tmpmlx2data3 &&  tmpmlx2data2 == (unsigned char)~ tmpmlx2data4) {
          if((tmpmlx2data2 & 3) == 2) { // error code, not angular data
          //    mlx1data = 999;
          } else {
              mlx2data = ((unsigned int) tmpmlx2data1 << 6) | ((unsigned int) tmpmlx2data2 >>  2) ;
          }
      }
      u7tb=0xFF;
      break;
  case 9:
      uDelay(25); // t4, 8+uS on scope, 6.9 required
      CS7=1;
      for(j=0;j<25;j++)            
          uDelay(255); // t5, 300/1500uS required, 1600 on scope
      CS7=0;
      uDelay(6); // t6, 10+uS on scope, 6.9 required
      u7tb=0xAA;
      mlx2whoamistatus=-1;
      break;
  default:        
      uDelay(200); // t2 and t7, 15uS/45uS required,  
      u7tb=0xFF;
  } 
  mlx2whoamistatus++;

  /* Clear the 'reception complete' flag. */
  ir_s7ric = 0;
  
}
