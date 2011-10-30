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
#include "uart.h"
#include "hwsetup.h"
#include <stdio.h>
#include <string.h>
#include <iso646.h>
#include "main.h"

#define base_freq (24000000)
/* Used to store the received data */
volatile unsigned char U5_in;

char tx5_buff[TX_BUFF_SIZE];
char command[TX_BUFF_SIZE]="";
unsigned short tx5_ptr = 0;
volatile unsigned short tx5_ptrr = 0;

char rx8_buff[RX_BUFF_SIZE];
unsigned short rx8_ptr = 0;
char rx5_buff[RX_BUFF_SIZE];
unsigned short rx5_ptr = 0;

#define	f1_CLK_SPEED	base_freq

int putchar (int i ) {
    if( tx5_ptr == tx5_ptrr) { 
        while(ti_u5c1 == 0)
            NOP();
        u5tb = (short) i;
    } else {
       tx5_buff[tx5_ptr++ ]= (unsigned char)i;
       if (tx5_ptr >= TX_BUFF_SIZE) 
           tx5_ptr = 0;
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


void uart5_init(void)
{
	u5brg = (f1_CLK_SPEED / 16 / 115200) - 1;	
	u5mr = 0x05;		
  	u5c0 = 0x10; 		
	u5tb = u5rb;	
  	u5tb = 0;			
	s5ric = 0x02;	
   	s5tic = 0x01;	
	p7_6s = 0x03;
	pd7_6 = 1;
	pd8_0 = 0;
	u5c1 = 0x05; 
}

void uart8_init(void) {
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

void uart7_init(void) {
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

void putchar5hexnr (unsigned char d) {
    if (d> 9) 
        d+=7;
    putchar(d+'0');
}

void puthex ( unsigned char d) {
    putchar5hexnr( d>>4 );
    putchar5hexnr(d&0x0F);
}


#pragma vector = UART5_RX
__interrupt void _uart5_receive(void)
{
    if (rx5_ptr >= RX_BUFF_SIZE) 
        rx5_ptr = RX_BUFF_SIZE-1;

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
       putchar(U5_in);
       break;
    }
}

void uart8_send ( unsigned char d) {
    while(ti_u8c1 == 0) 
        NOP();
    u8tb = (short)d;
}

void  uart7_send(unsigned char d) {
    while(ti_u7c1 == 0) 
        NOP();
    u7tb = (short)d;
}

