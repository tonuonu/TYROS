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

#pragma vector = UART0_TX
__interrupt void _uart0_transmit(void) {
    ERRORLED=1;
  if(!text[lineno][textpos]){
      textpos=0;
      lineno++;
      if(lineno>7) {
        lineno=0;
      }
  }
  u0tb=text[lineno][textpos];
  textpos++;
  if(textpos>(100-1)) {
      lineno++;
      if(lineno>7) {
        lineno=0;
      }
      textpos=0;
  }
  ir_s0tic = 0;
    ERRORLED=0;

}

#pragma vector = UART0_RX
__interrupt void _uart0_receive(void) {
    ERRORLED=1;
    
    /* Read the received data */
    U0_in = (unsigned char)u0rb;
    switch(U0_in) {
    case 13:
        rx0_buff[rx0_ptr]=0;
        strcpy(command,rx0_buff);
        rx0_ptr=0;
    //    putchar(0x0a);
     //   putchar(0x0d);
        break;
    case 127:
        rx0_ptr--;
        rx0_buff[rx0_ptr]= 0;
        break;
    default:
       rx0_buff[rx0_ptr]= U0_in;
       rx0_ptr++;
     //  putchar(U0_in);
       break;
    }
    if (rx0_ptr >= RX_BUFF_SIZE) {
        rx0_ptr = RX_BUFF_SIZE-1;
    }
#if 0
    if(0) {
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
                // Yaw
                if(twist[5]>0.01) {
                    pwmtarget[0]= 100;
                    pwmtarget[0]= -100;
                } else if(twist[5]<0.01) {
                    pwmtarget[0]= -100;
                    pwmtarget[0]= +100;
                } else {
                    // FIXME: clearly bit wrong algorithm here                  
                    // X axle speed
                    if(twist[0]>0.01) {
                        pwmtarget[0]= +100;
                        pwmtarget[0]= +100;
                    } else if(twist[0]<0.01) {
                        pwmtarget[0]= -100;
                        pwmtarget[0]= -100;
                    }
                                       
                    // Y axle speed
                    if(twist[1]>0.01) {
                        pwmtarget[0]= +100;
                        pwmtarget[0]= +100;
                    } else if(twist[1]<0.01) {
                        pwmtarget[0]= -100;
                        pwmtarget[0]= -100;
                    }
                }                  
                sprintf(buf,"new twist x=%f(m/s), y=%f(m/s), yaw=%f(deg)",twist[0],twist[1],twist[5]);
                write(buf);
            } else if(strncmp(command,"pwm ",4)==0) {
                int tmp;            
                for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                    if(tmp >= 1) {
                        pwmtarget[tmp-1]=(int)strtod(tok,NULL); 
                    }
                }
                if(pwmtarget[0]> 100) pwmtarget[0]= 100;
                if(pwmtarget[1]> 100) pwmtarget[1]= 100;
                if(pwmtarget[0]<-100) pwmtarget[0]=-100;
                if(pwmtarget[1]<-100) pwmtarget[1]=-100;
                sprintf(buf,"manual pwm left=%d%%, right=%d%%",pwmtarget[0],pwmtarget[1]);               
                write(buf);
            } else if(strncmp(command,"reset",5)==0) {
                sprintf(buf,"RESETTING BOARD");
                write(buf);              
//                asm("jmp 0xfffffffc");       
                asm("jmp 0xfcffffff");       
//                asm("jmp 0xffff801c");
//                asm("jmp 0x1c80ffff");
                // 1c 80 ff ff
            } else if(strncmp(command,"panda ",6)==0) {
                int tmp;
                for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                    if(tmp == 1) {
                        PANDA=(int)strtod(tok,NULL); 
                    }
                }
                sprintf(buf,"panda %s",PANDA ? "on":"off");
                write(buf);              
            } else if(strncmp(command,"buzzer ",7)==0) {
                int tmp;
                for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                    if(tmp == 1) {
                        buzzer=(int)strtod(tok,NULL); 
                    }
                }
                sprintf(buf,"buzzer %s",buzzer ? "on":"off");
                write(buf);              
            } else if(strncmp(command,"charge ",7)==0) {
                int tmp;
                for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                    if(tmp == 1) {
                        CHARGE=(int)strtod(tok,NULL); 
                    }
                }
                sprintf(buf,"charge %s",CHARGE ? "on":"off");
                write(buf);              
            } else if(strncmp(command,"kick ",5)==0) {
                int tmp;
                for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                    if(tmp == 1) {
                        KICK=(int)strtod(tok,NULL);                    
                    }
                }
                sprintf(buf,"kick %s",KICK ? "on":"off");
                write(buf);              
            } else if(strncmp(command,"joy",3)==0) {
                sprintf(buf,"joy %s%s%s%s%s",
                        JOY_UP ? ""    :"up ",
                        JOY_DOWN ? ""  :"down ",
                        JOY_LEFT ? ""  :"left ",
                        JOY_RIGHT ? "" :"right ",
                        JOY_CENTER ? "":"center" );
                write(buf);                            
            } else {      
                sprintf(buf,"Unknown command:'%s'",command);
                writeln(buf);                            
            }
            putchar('>');    
            putchar(' ');
            write(VT100CURSORSAVE);
            command[0]=0;
        }        
#if 0        
        write(VT100CURSORMELEXISR);
        write("Melexis R:");
        sprintf(buf,"%1d ",mlxrightstatus);
        write(buf);
        sprintf(buf,"%6d ",mlxleftdata);
        write(buf);


      /* Gyroscope */
        write(VT100CURSORGYRO);
        write("Gyroscope:");
//        sprintf(buf,"(%2d) ",gyrowhoamistatus);
//        write(buf);
// From 1/(0xFFFF/250dps)
        // http://www.wolframalpha.com/input/?i=1%2F%280xFFFF%2F250%29+degrees+in+radians
#define GYRORATE (6.658e-5)
//        if(accok && accwhoami==85) {
            sprintf(buf," x:%8.5f y:%8.5f z:%8.5f",gyrox*GYRORATE,gyroy*GYRORATE,gyroz*GYRORATE);
            write(buf);
//        } else {
//            write("ERROR");
//        } 
/* Acceleration sensor */
        write(VT100CURSORACC);
        write("Acceleration: ");
 //       sprintf(buf,"(%2d) ",accstatus);
 //       write(buf);

//        if(accok && accwhoami==85) {
        if(acccalcnt<CALIBRATIONSAMPLES) {
 //           sprintf(buf,"whoami %2d CALIBRATING",accwhoami);
        } else {
          // http://www.wolframalpha.com/input/?i=G+to+m%2Fs2
          // Whole 8bit range is +-2G, so each unit is 4/256 G-s
          // Each G is 9.809 m/s2
          sprintf(buf,"x:%8.4f y:%8.4f z:%8.4f",4.0*9.807/256.0*accx,4.0*9.807/256.0*accy,4.0*9.807/256.0*accz);
        }
            write(buf);
//        } else {
//            write("ERROR");
//        } 
        int ad[4];
        ad[0]=AD00 & 0x3FF;
        ad[1]=AD01 & 0x3FF;
        ad[2]=AD02 & 0x3FF;
        ad[3]=AD03 & 0x3FF;
        bat=(float)ad[3]*(13.64/0x3FF);
        float capacitor=(float)ad[2]/2.27333333 ; // 0x3FF/450V
        write(VT100CURSORCAPACITOR);
        sprintf(buf,"Capacitor: %3.0f V", capacitor);
        write(buf);
        write(VT100CURSORBATTERY);
        sprintf(buf,"Battery: %3.1f V", bat );
        write(buf);
        write(VT100CURSORLEFTMOTOR);
        sprintf(buf,"Left motor: %4.1fA ", (float)ad[0]/50.0 );
        write(buf);
/*        
        sprintf(buf,"(%2d) (%3u %3u %3u %3u) %f ",
                (unsigned char)mlx2whoamistatus, 
                (unsigned char)tmpmlxrightdata1,(unsigned char)tmpmlxrightdata2,
                (unsigned char)~tmpmlxrightdata3,(unsigned char)~tmpmlxrightdata4,revolutions);
        */
        sprintf(buf,"%6.1f m",distanceleft);
        write(buf);

/* switch(mlxleftstatus) {
        case 1:
            write("sensor err ");
            break;          
        case 2:
            write("sensor ok  ");
            break;          
        case 3:
            write("no sensor ");
            break;          
        }
        sprintf(buf,"%6d ",mlxrightdata);
        write(buf);
*/
        
        write(VT100CURSORRIGHTMOTOR);
        sprintf(buf,"Right motor: %4.1fA ", (float)ad[1]/50.0 );
        write(buf);
        sprintf(buf,"%6.1f m",distanceright );
        write(buf);
        
        write(VT100CURSORODOMETRY);
        sprintf(buf,"Odometry: dx:%f dy:%f yaw:%f",dx,dy,yaw );
        write(buf);
        
        if(bat<6.8) 
            errorflag=1;

        write(VT100CURSORMELEXISE);
        write("5V LDO:");
        write(MELEXIS_EN ? "OFF":"ON "); // Low is Enable
   

        write(VT100CURSORBALL);
        write("Ball:");
        write(BALL_DETECT ? "No ":"Yes");
        write(VT100CURSORPANDA);
        write("Panda:");
        write(PANDA ? "ON ":"OFF");
        write(VT100CURSORCHARGER);
        write("Charger:");
        if(autocharge) 
            write("AUTO");
        else
            write(CHARGE ? "ON ":"OFF");
        if(autocharge && capacitor<400.0)
          CHARGE=1;
        if(CHARGE_DONE==0) { // Charging is DONE when signal is low
          CHARGE=0; // Turn off CHARGE pin
        }
        if(errorflag) {
    //      ERRORLED=1;
        } else {
    //      ERRORLED=0; 
        } 
#endif
    }
        
#endif 
    ir_s0ric = 0;
    ERRORLED=0;
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

void 
uart7_send(unsigned char d) {
    while(ti_u7c1 == 0) {
        NOP();
    }
    u7tb = (short)d;
}

