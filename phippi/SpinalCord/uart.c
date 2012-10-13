/*
 *  Copyright (c) 2011, 2012 Tonu Samuel
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iso646.h>
#include "main.h"
#include "locking.h"
#include "mma7455l.h"

#define base_freq (24000000)
/* Used to store the received data */
volatile unsigned char U0_in;
volatile unsigned char U4_in;

char tx0_buff[TX_BUFF_SIZE];
char command[TX_BUFF_SIZE]="";
unsigned short tx0_ptr = 0;
volatile unsigned short tx0_ptrr = 0;

char rx0_buff[RX_BUFF_SIZE];
unsigned short rx0_ptr = 0;

#define	f1_CLK_SPEED	base_freq

void parsecmd( char *command) {
    char buf[256];
    if(command[0]!=0) {
        char *tok;
        if(strncmp(command,"twist ",6)==0) {
            int tmp;
            for(tmp=0,tok = strtok(command," "); tok && tmp<=6 ; tok=strtok(0," "),tmp++) {
                if(tmp>0) {
                    twist[tmp-1]=strtod(tok,NULL);
                }
            }                
            twistcmdage=0;
//            sprintf(buf,"new twist x=%f(m/s), y=%f(m/s), yaw=%f(deg)",twist[0],twist[1],twist[5]);
//            write(buf);
        } else if(strncmp(command,"pwm ",4)==0) {
            int tmp;            
            for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                if(tmp >= 1) {
                    pwmtarget[tmp-1]=(int)__ROUND(strtod(tok,NULL)); 
                }
            }
            if(pwmtarget[0]> 100) pwmtarget[0]= 100;
            if(pwmtarget[1]> 100) pwmtarget[1]= 100;
            if(pwmtarget[0]<-100) pwmtarget[0]=-100;
            if(pwmtarget[1]<-100) pwmtarget[1]=-100;
            sprintf(buf,"manual pwm left=%d%%, right=%d%%",pwmtarget[0],pwmtarget[1]);
//            write(buf);
        } else if(strncmp(command,"reset",5)==0) {
            sprintf(buf,"RESETTING BOARD");
//            write(buf);              
            asm("jmp 0xfcffffff");       
        } else if(strncmp(command,"panda ",6)==0) {
            int tmp;
            for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                if(tmp == 1) {
                    PANDA=(int)__ROUND(strtod(tok,NULL)); 
                }
            }
            sprintf(buf,"panda %s",PANDA ? "on":"off");
 //           write(buf);              
        } else if(strncmp(command,"buzzer ",7)==0) {
            int tmp;
            for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                if(tmp == 1) {
                    buzzer=(int)__ROUND(strtod(tok,NULL)); 
                }
            }
            sprintf(buf,"buzzer %s",buzzer ? "on":"off");
 //           write(buf);              
        } else if(strncmp(command,"charge ",7)==0) {
            int tmp;
            for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                if(tmp == 1) {
                    CHARGE=(int)__ROUND(strtod(tok,NULL)); 
                }
            }
            sprintf(buf,"charge %s",CHARGE ? "on":"off");
//            write(buf);              
        } else if(strncmp(command,"kick ",5)==0) {
            int tmp;
            for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                if(tmp == 1) {
                    KICK=(int)__ROUND(strtod(tok,NULL));                    
                }
            }
            sprintf(buf,"kick %s",KICK ? "on":"off");
//            write(buf);              
        } else if(strncmp(command,"joy",3)==0) {
            sprintf(buf,"joy %s%s%s%s%s",
                    JOY_UP ? ""    :"up ",
                    JOY_DOWN ? ""  :"down ",
                    JOY_LEFT ? ""  :"left ",
                    JOY_RIGHT ? "" :"right ",
                    JOY_CENTER ? "":"center" );
//            write(buf);                            
        } else {      
//            sprintf(buf,"Unknown command:'%s'",command);
//            writeln(buf);                            
        }
        command[0]=0;
    }        
}


#pragma vector = UART0_TX
__interrupt void _uart0_transmit(void) {
//    while(!get_lock2()); 
    if(!text[lineno][textpos]){
        textpos=0;
        lineno++;
        if(lineno>7) {
            lineno=0;    
            redraw_infoscreen=1;
        }
    }
    u0tb=text[lineno][textpos];
//    release_lock2();
    textpos++;
    if(textpos>(100-1)) {
        lineno++;
        if(lineno>7) {
            lineno=0;
        }
        textpos=0;
    }
    ir_s0tic = 0;
}

#pragma vector = UART0_RX
__interrupt void _uart0_receive(void) {
    /* Read the received data */
    U0_in = (unsigned char)u0rb;
    switch(U0_in) {
    case 13:
        rx0_buff[rx0_ptr]=0;
        /* No null-character is implicitly appended to the end of destination, so destination will only be null-terminated if the length of the C string in source is less than num. */
        strncpy(command,rx0_buff,RX_BUFF_SIZE-1);
        rx0_ptr=0;
        rx0_buff[rx0_ptr]=0;
        parsecmd(command);
        break;
    case 127:
        rx0_ptr--;
        rx0_buff[rx0_ptr]= 0;
        break;
    default:
       rx0_buff[rx0_ptr]= U0_in;
       rx0_ptr++;
       rx0_buff[rx0_ptr]= 0;
       break;
    }
    if (rx0_ptr >= RX_BUFF_SIZE) {
        rx0_ptr = RX_BUFF_SIZE-1;
    }

    ir_s0ric = 0;
}

void 
uart0_init(void) {
//    u0brg = (f1_CLK_SPEED / 16 / 115200) - 1;	
    u0brg = (f1_CLK_SPEED / 16 / 57600) - 1;	

    smd0_u0mr  = 1;  // 8 bit character lenght
    smd1_u0mr  = 0;  // 8 bit character lenght
    smd2_u0mr  = 1;  // 8 bit character lenght
    ckdir_u0mr = 0; // internal clock
    stps_u0mr  = 0; // Stop bit length 0
    prye_u0mr  = 0; // parity not enabled
    pry_u0mr   = 0; // parity not enabled
    iopol_u0mr = 0; // IO polarity is normal

    clk0_u0c0  = 0; // clock source f1
    clk1_u0c0  = 0; // clock source f1
    txept_u0c0 = 0; // Transmit register empty flag
    crd_u0c0   = 1; // CTS disabled
    nch_u0c0   = 0; // Select an output mode of the TXDi pin??
    ckpol_u0c0 = 0; // Select a transmit/receive clock polarity
    uform_u0c0 = 0; // Select either LSB first or MSB first

    te_u0c1    = 1; // Set the bit to 1 to enable data transmission/reception
    ti_u0c1    = 0; // Transmit buffer empty flag
    re_u0c1    = 1; // Set the bit to 1 to enable data reception
    ri_u0c1    = 0; // Receive complete flag
    u0irs_u0c1 = 0; // Interrupt  when transmission is not yet completed. 
    u0rrm_u0c1 = 0; // Set the bit to 1 to use continuous receive mode
    u0lch_u0c1 = 0; // Set the bit to 1 to use logic inversion
    
    u0tb = u0rb;	
    u0tb = 0;			
    DISABLE_IRQ;
    /*
     * Serial receive interrupt has higher priority because we may
     * miss the byte easily otherwise. We have no control over them
     * coming in.
     */
    ilvl_s0ric =2; 
    ir_s0ric   =0; 
    /* Asynchronous link looses sync if we do not send bytes on time */
    ilvl_s0tic =1; 
    ir_s0tic   =0; 
    ENABLE_IRQ;
    TX0s = PF_UART;
    TX0d = PD_OUTPUT;
    RX0s = PF_UART;
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

