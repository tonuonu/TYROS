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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "uart.h"
#include "hwsetup.h"
#include "gyro.h"
#include "SPI.h"
#include "mma7455l.h"

int accok=0;
int accwhoami=0;
signed char accx=0,accy=0,accz=0;
int accwhoamistatus=0;

/* Buffer to store the received data	*/
char acc_RecBuff[8];


void
SPI2_Init(void) { // Accel sensor
    pu22=1; // pull up for CLK2 or p7_2
#define f1_CLK_SPEED 24000000
//    u2brg =  (unsigned char)(((f1_CLK_SPEED)/(2*100000))-1);
    // 8MHz max
    u2brg =  (unsigned char)(((f1_CLK_SPEED)/(2*400000))-1);

    CS2d = PD_OUTPUT;
    CS2=1;
    CLOCK2d = PD_OUTPUT;
    CLOCK2s = PF_UART;
    TX2d = PD_OUTPUT;
    TX2s = PF_UART;
    RX2s = PF_UART;

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
    nch_u2c0 = 0;                                          // 0=Output mode "open drain" for TXD and CLOCK pin 
    ckpol_u2c0 = 0;                                        // CLK Polarity 0 rising edge, 1 falling edge
    uform_u2c0 = 1;                                        // 1=MSB first

    te_u2c1 = 1;                                           // 1=Transmission Enable
    ti_u2c1 = 0;                                           // Must be 0 to send or receive
    re_u2c1 = 1;                                           // Reception Enable when 1
    ri_u2c1 = 0;                                           // Receive complete flag - U4RB is empty.
    u2irs_u2c1 = 1;                                        // Interrupt  when transmission is completed. 
    u2rrm_u2c1 = 0;                                        // Continuous receive mode off
    u2lch_u2c1 = 0;                                        // Logical inversion off 

    u2smr = 0x00;
    u2smr2 = 0x00;

    sse_u2smr3 = 0;                                        // SS is disabled when 0
    ckph_u2smr3 = 0;                                       // Non clock delayed 
    dinc_u2smr3 = 0;                                       // Master mode when 0
    nodc_u2smr3 = 0;                                       // Select a clock output  mode "push-pull" when 0 
    err_u2smr3 = 0;                                        // Error flag, no error when 0 
    dl0_u2smr3 = 0;                                        // Set 0 for no  delay 
    dl1_u2smr3 = 0;                                        // Set 0 for no  delay 
    dl2_u2smr3 = 0;                                        // Set 0 for no  delay 

    u2smr4 = 0x00;

    DISABLE_IRQ
    ilvl_s2ric =0x05;       
    ir_s2ric   =0;            
    ENABLE_IRQ
 
}


#pragma vector = UART2_RX
__interrupt void _uart2_receive(void) {
 

  /* Used to reference a specific location in the array while string the
  received data.   */
  static unsigned char uc_cnt=0;
  /* Copy the received data to the global variable 'acc_RecBuff'	*/
  acc_RecBuff[uc_cnt] = (char) u2rb ;
 
  switch(accwhoamistatus) {
  case 0: // Request sent, sending dummy byte to get the answer
  case 4:
  case 6:
  case 8:
      u2tb=0xFF;
      break;
  case 1: // WHOAMI answer received. Sending request to write REG_MCTL
      accwhoami=(int)acc_RecBuff[uc_cnt];
      CS2=1;
      uDelay(5);
      CS2=0;
      u2tb=(MMA7455L_REG_MCTL << 1) | WRITE_BIT; 
      break;
  case 2: // REG_MCTL written, writing _MODE_MEASUREMENT into it
      u2tb=((MMA7455L_GSELECT_2 | MMA7455L_MODE_MEASUREMENT) << 1) | WRITE_BIT; 
      break;
  case 3: // _MODE_MEASUREMENT written. Trying to get XOUTL
      CS2=1;
      uDelay(5);
      CS2=0;
      u2tb=MMA7455L_REG_XOUT8 << 1;
      break;
  case 5: // XOUTL sent, trying to read answer
      accx=(int)acc_RecBuff[uc_cnt];
      CS2=1;
      uDelay(5);
      CS2=0;
      u2tb=MMA7455L_REG_YOUT8 << 1;
      break;
  case 7: // YOUTL sent, trying to read answer
      accy=(int)acc_RecBuff[uc_cnt];
      CS2=1;
      uDelay(5);
      CS2=0;
      u2tb=MMA7455L_REG_ZOUT8 << 1;
      break;
  case 9: // ZOUTL sent, trying to read answer
      accz=(int)acc_RecBuff[uc_cnt];
      CS2=1;
      uDelay(5);
      CS2=0;
      u2tb=MMA7455L_REG_WHOAMI << 1;
      accwhoamistatus=-1;
      break;
  } 
  accwhoamistatus++;

  /* Check if the buffer size is exceed. If it is then reset the 'uc_cnt'
  variable.    */
  if(uc_cnt++ >= sizeof(acc_RecBuff)) {
    /* Reinitialize the buffer reference.	*/
    uc_cnt = 0;
  }

  /* Clear the 'reception complete' flag.	*/
  ir_s2ric = 0;
  
}

