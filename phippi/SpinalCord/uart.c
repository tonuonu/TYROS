/*
 *  Copyright (c) 2011, T�nu Samuel
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
#include "uart.h"
#include "hwsetup.h"
#include <stdio.h>
#include <string.h>
#include <iso646.h>
#include "main.h"
#include "mma7455l.h"

#define base_freq (24000000)
/* Used to store the received data */
volatile unsigned char U5_in;
volatile unsigned char U4_in;

char tx5_buff[TX_BUFF_SIZE];
char command[TX_BUFF_SIZE]="";
unsigned short tx5_ptr = 0;
volatile unsigned short tx5_ptrr = 0;

char rx8_buff[RX_BUFF_SIZE];
unsigned short rx8_ptr = 0;
char rx5_buff[RX_BUFF_SIZE];
unsigned short rx5_ptr = 0;

#define	f1_CLK_SPEED	base_freq
/* Buffer to store the received data	*/
char c_RecBuff[8];


#pragma vector = UART2_RX
__interrupt void _uart2_receive(void) {
  /* Used to reference a specific location in the array while string the
  received data.   */
  static unsigned char uc_cnt=0;
  /* Copy the received data to the global variable 'c_RecBuff'	*/
  c_RecBuff[uc_cnt] = (char) u2rb ;
 
  switch(accwhoamistatus) {
  case 0: // Request sent, sending dummy byte to get the answer
  case 4:
  case 6:
  case 8:
      u2tb=0xFF;
      break;
  case 1: // WHOAMI answer received. Sending request to write REG_MCTL
      accwhoami=(int)c_RecBuff[uc_cnt];
      CS2=1;
//      uDelay(50);
      CS2=0;
//      uDelay(50);
      u2tb=(MMA7455L_REG_MCTL << 1) | WRITE_BIT; 
      break;
  case 2: // REG_MCTL written, writing _MODE_MEASUREMENT into it
      u2tb=((MMA7455L_GSELECT_2 | MMA7455L_MODE_MEASUREMENT) << 1) /*| WRITE_BIT*/; 
      break;
  case 3: // _MODE_MEASUREMENT written. Trying to get XOUTL
      CS2=1;
//      uDelay(50);
      CS2=0;
//      uDelay(50);
      u2tb=MMA7455L_REG_XOUT8 << 1;
      break;
  case 5: // XOUTL sent, trying to read answer
      accx=(int)c_RecBuff[uc_cnt];
      CS2=1;
//      uDelay(50);
      CS2=0;
//      uDelay(50);
      u2tb=MMA7455L_REG_YOUT8 << 1;
      break;
  case 7: // YOUTL sent, trying to read answer
      accy=(int)c_RecBuff[uc_cnt];
      CS2=1;
//      uDelay(50);
      CS2=0;
//      uDelay(50);
      u2tb=MMA7455L_REG_ZOUT8 << 1;
      break;
  case 9: // ZOUTL sent, trying to read answer
      accz=(int)c_RecBuff[uc_cnt];
      CS2=1;
//      uDelay(50);
      CS2=0;
//      uDelay(50);
      u2tb=MMA7455L_REG_WHOAMI << 1;
      ERRORLED=1;
      accwhoamistatus=-1;
      break;
  } 
  accwhoamistatus++;

  
  /* Dummy write to the receive register to reinitiate a receive operation.	*/
//  u2rb = 0x00;
  /* Check if the buffer size is exceed. If it is then reset the 'uc_cnt'
  variable.    */
  if(uc_cnt++ >= sizeof(c_RecBuff)) {
    /* Reinitialize the buffer reference.	*/
    uc_cnt = 0;
  }

  /* Clear the 'reception complete' flag.	*/
  ir_s2ric = 0;
//  CS2=1;
  
}


int putchar (int i ) {
    if( tx5_ptr == tx5_ptrr) { 
        while(ti_u5c1 == 0) {
            NOP();
        }
        u5tb = (short) i;
    } else {
        tx5_buff[tx5_ptr++ ]= (unsigned char)i;
        if (tx5_ptr >= TX_BUFF_SIZE) {
            tx5_ptr = 0;
        }
    }
    return i;
}

#pragma vector = UART5_TX
__interrupt void _uart5_transmit(void)
{
    if( tx5_ptr != tx5_ptrr) {
        u5tb = (short) tx5_buff[tx5_ptrr++];
        if ( tx5_ptrr >= TX_BUFF_SIZE ) 
            tx5_ptrr = 0;
    }
}

void 
uart5_init(void) {
    u5brg = (f1_CLK_SPEED / 16 / 115200) - 1;	
//   u5brg = (f1_CLK_SPEED / 16 / 9600) - 1;	

    smd0_u5mr  = 1;  // 8 bit character lenght
    smd1_u5mr  = 0;  // 8 bit character lenght
    smd2_u5mr  = 1;  // 8 bit character lenght
    ckdir_u5mr = 0; // internal clock
    stps_u5mr  = 0; // Stop bit length 0
    prye_u5mr  = 0; // parity not enabled
    pry_u5mr   = 0; // parity not enabled
    iopol_u5mr = 0; // IO polarity is normal

    clk0_u5c0  = 0; // clock source f1
    clk1_u5c0  = 0; // clock source f1
    txept_u5c0 = 0; // Transmit register empty flag
    crd_u5c0   = 1; // CTS disabled
    nch_u5c0   = 0; // Select an output mode of the TXDi pin??
    ckpol_u5c0 = 0; // Select a transmit/receive clock polarity
    uform_u5c0 = 0; // Select either LSB first or MSB first

    te_u5c1    = 1; // Set the bit to 1 to enable data transmission/reception
    ti_u5c1    = 0; // Transmit buffer empty flag
    re_u5c1    = 1; // Set the bit to 1 to enable data reception
    ri_u5c1    = 0; // Receive complete flag
    u5irs_u5c1 = 0; // Select a source for the UARTi transmit interrupt
    u5rrm_u5c1 = 0; // Set the bit to 1 to use continuous receive mode
    u5lch_u5c1 = 0; // Set the bit to 1 to use logic inversion
    
    u5tb = u5rb;	
    u5tb = 0;			
        
    s5ric = 0x02;   // Receive interrupt
    s5tic = 0x01;   // Send interrupt

    TX5s = PF_UART;
    TX5d = PD_OUTPUT;
    RX5s = PF_UART;
}

void 
uart8_init(void) {
        u8brg = (f1_CLK_SPEED / 16 / 9600) - 1;
	u8mr = 0x05;
  	u8c0 = 0x10;
	u8tb = u8rb;
  	u8tb = 0;
	s8ric = 0x03;
	p7_3s = 0x07;
	pd7_3 = 1;
	pd7_5 = 0;
        pu23 = 1;  // RX8 pullup
	u8c1 = 0x05;
}

#if 0
void 
uart7_init(void) {
        u7brg = (f1_CLK_SPEED / 16 / 9600) - 1;
	u7mr = 0x05;
  	u7c0 = 0x10;
	u7tb = u7rb;
  	u7tb = 0;
	s7ric = 0x03;
	p5_4s = 0x07;
	pd5_4 = 1; //tx
	pd5_6 = 0; //rx
        pu13 = 1;  // RX7 pullup
	u7c1 = 0x05;
}
#endif

void 
putchar5hexnr (unsigned char d) {
    if (d> 9) {
        d+=7;
    }
    putchar(d+'0');
}

void 
puthex ( unsigned char d) {
    putchar5hexnr( d>>4 );
    putchar5hexnr(d&0x0F);
}

void write(char *c) {
   char *ptr=c;
   while(*ptr)
       putchar((int)*ptr++);
}

void writeln(char *c) {
   write(c);
   putchar(0x0a);
   putchar(0x0d);
}    


#pragma vector = UART5_RX
__interrupt void _uart5_receive(void) {
    if (rx5_ptr >= RX_BUFF_SIZE) {
        rx5_ptr = RX_BUFF_SIZE-1;
    }
    while(ri_u5c1 == 0) {
        /* Make sure that the receive is complete */
    }

    /* Read the received data */
    U5_in = (unsigned char)u5rb;
    switch(U5_in) {
    case 13:
        rx5_buff[rx5_ptr]=0;
        strcpy(command,rx5_buff);
        rx5_ptr=0;
        write(VT100CURSORRESTORE);
        putchar(0x0a);
        putchar(0x0d);
        write(VT100CURSORSAVE);

        break;
    case 127:
        rx5_ptr--;
        rx5_buff[rx5_ptr]= 0;
        break;
    default:
       rx5_buff[rx5_ptr]= U5_in;
       rx5_ptr++;
       write(VT100CURSORRESTORE);
       putchar(U5_in);
       write(VT100CURSORSAVE);

       break;
    }
}

void 
uart8_send ( unsigned char d) {
    while(ti_u8c1 == 0) {
        NOP();
    }
    u8tb = (short)d;
}

void 
uart7_send(unsigned char d) {
    while(ti_u7c1 == 0) {
        NOP();
    }
    u7tb = (short)d;
}

#if 1
#pragma vector = UART4_RX
__interrupt void _uart4_receive(void) {
    if (rx5_ptr >= RX_BUFF_SIZE) {
        rx5_ptr = RX_BUFF_SIZE-1;
    } 
    while(ri_u4c1 == 0) {
        /* Make sure that the receive is complete */
    }

    /* Read the received data */
    U4_in = (unsigned char)u4rb;
    switch(U4_in) {
    case 13:
        rx5_buff[rx5_ptr]=0;
        strcpy(command,rx5_buff);
        rx5_ptr=0;
        putchar(0x0a);
        putchar(0x0d);
        break;
    case 127:
        rx5_ptr--;
        rx5_buff[rx5_ptr]= 0;
        break;
    default:
       rx5_buff[rx5_ptr]= U5_in;
       rx5_ptr++;
       putchar(U4_in);
       break;
    }
}
#endif
