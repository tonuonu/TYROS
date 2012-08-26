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



void
SPI3_Init(void) // OLED
{
    OLED_DATACOMMANDd = PD_OUTPUT;
    OLED_RESETd = PD_OUTPUT;
  
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
    ckpol_u3c0 = 0;                                        // CLK Polarity 0 rising edge, 1 falling edge (0 ok)
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

void
SPI0_send_data(unsigned char c) {
    while (ti_u0c1 == 0) {
        NOP();
    }
    uDelay(200);
    u0tb = c;
    uDelay(200);
}

unsigned short 
SPI0_receive(void) {
    unsigned short r;
  //  SPI0_send_data(0xFF);
  //  uDelay(200);
    uDelay(200);
    while (ri_u0c1 == 0) {
        NOP();
    }
    r=u0rb;    
    ri_u0c1=0;
    uDelay(200);

    return r;
}

/*
unsigned short 
accelerometer_receive(void) {
    unsigned short r=0;
    SPI2_send_data(0xFF);
    while (ri_u2c1 == 0) {
        NOP();
    }
    r=u2rb;    
    ri_u2c1=0;
    return r;
}
*/
void
SPI3_send_data(unsigned char c) {
    while (ti_u3c1 == 0)
        NOP();
    uDelay(SPI_DELAY);
    OLED_DATACOMMAND = 1;
    uDelay(SPI_DELAY);
    u3tb = c;
}

void
SPI3_send_cmd(unsigned char c) {
    while (ti_u3c1 == 0)
        NOP();
    uDelay(SPI_DELAY);
    OLED_DATACOMMAND = 0;
    uDelay(SPI_DELAY);
    u3tb = c;
}


void
SPI6_send(unsigned short c) {
  while (ti_u6c1 == 0) {
        NOP();
  }
  uDelay(SPI_DELAY);
  ti_u6c1=0;
  u6tb = c;

}

short unsigned
SPI6_receive(void) {
  short unsigned r;
  SPI6_send(0xFF);
  uDelay(SPI_DELAY);  
  uDelay(SPI_DELAY);  
  while (ri_u6c1 == 0) {
        NOP();
  }  
  r=u6rb;
  ri_u6c1=0;
  return r;
}

void
SPI7_send(unsigned short c) {
  while (ti_u7c1 == 0)
        NOP();
  uDelay(SPI_DELAY);
  ti_u7c1=0;
  u7tb = c;

}
