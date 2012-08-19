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
#include "mma7455l.h"
int mlx1whoamistatus=0;
char mlx1_RecBuff[8];

char mlx1data1=0;
char mlx1data2=0;


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
    ckpol_u4c0 = 1;                                        // CLK Polarity 0 rising edge, 1 falling edge
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

    u4brg = 0x55;                                          // (unsigned char)(((f1_CLK_SPEED)/(2*BIT_RATE))-1);    
    pu27=1;
    DISABLE_IRQ
    ilvl_s4ric =0x06;   
    ir_s4ric   =0;            
    ENABLE_IRQ

}

#pragma vector = UART4_RX
__interrupt void _uart4_receive(void) {
//      ERRORLED=1;
    int j;
  /* Used to reference a specific location in the array while string the
  received data.   */
  static unsigned char uc_cnt=0;
  /* Copy the received data to the global variable 'mlx1_RecBuff'	*/
  mlx1_RecBuff[uc_cnt] = (char) u4rb ;
 
  switch(mlx1whoamistatus) {
  case 0: // Request 0xAA sent 
      // 12uS needed. On 48Mhz each cycle is ~21nS, so
      // 2300nS/12=~220
      for(j=0;j<2;j++)            
          uDelay(255); 
      u4tb=0xFF;
  case 1: // Request 0xFF sent, sending dummy byte to get the answer
      for(j=0;j<2;j++)            
          uDelay(255); 
      u4tb=0xFF;
  case 2:
      mlx1data1=(int)mlx1_RecBuff[uc_cnt];
      for(j=0;j<2;j++)            
          uDelay(255); 
      u4tb=0xFF;
  case 3:
      mlx1data2=(int)mlx1_RecBuff[uc_cnt];
      CS4=0;
      for(j=0;j<2;j++)            
          uDelay(255); 
      CS4=1;
      for(j=0;j<2;j++)            
          uDelay(255); 
      u4tb=0xAA;
      mlx1whoamistatus=-1;
      break;
  } 
  mlx1whoamistatus++;

  /* Check if the buffer size is exceed. If it is then reset the 'uc_cnt'
  variable.    */
  if(uc_cnt++ >= sizeof(mlx1_RecBuff)) {
    /* Reinitialize the buffer reference.	*/
    uc_cnt = 0;
  }

  /* Clear the 'reception complete' flag.	*/
  ir_s4ric = 0;
  //ERRORLED=0;
  
}

#if 0
        int j;
        // 300uS needed. On 48Mhz each cycle is ~21nS, so
        // 300 000nS/21=~1200
        for(j=0;j<7;j++)
            uDelay(255); 

        CS4=0; // enable left Melexis
        // 300uS needed. On 48Mhz each cycle is ~21nS, so
        // 300 000nS/21=~1200
        for(j=0;j<2;j++) {
            uDelay(255); 
        }
        SPI4_send(0xAA);
        // 12uS needed. On 48Mhz each cycle is ~21nS, so
        // 2300nS/12=~220
        for(j=0;j<2;j++)            
            uDelay(255); 
      
        SPI4_send(0xFF);
        for(j=0;j<2;j++)
            uDelay(255); 

        int i;
        for(i=0;i<4;i++) {
            unsigned short c; /* 16 bit value */
            pd9_6=0;
            c=SPI4_receive();
            sprintf(buf,"SPI4 %x",c);
            write(buf);
            pd9_6=1;
            for(j=0;j<2;j++) {
                uDelay(255); 
            }
        }
        CS4=1; // disable melexis   
#endif   