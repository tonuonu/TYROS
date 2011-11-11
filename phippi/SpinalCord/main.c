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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "uart.h"
#include "hwsetup.h"
extern int alarm;

volatile unsigned short ticks;

#define TIMERB2COUNT	10
#define SPI_DELAY (50)
void
SPI_send_data(unsigned char c) {
    while (ti_u3c1 == 0)
        NOP();
    uDelay(SPI_DELAY);
    p4_0 = 1;
    uDelay(SPI_DELAY);
    u3tb = c;
}

void
SPI_send_cmd(unsigned char c) {
    while (ti_u3c1 == 0)
        NOP();
    uDelay(SPI_DELAY);
    p4_0 = 0;
    uDelay(SPI_DELAY);
    u3tb = c;
}

void
SPI4_send(unsigned short c) {
  while (ti_u4c1 == 0)
        NOP();
  uDelay(SPI_DELAY);
  ti_u4c1=0;
  u4tb = c;

}

short unsigned
SPI4_receive(void) {
  short unsigned r;
  SPI4_send(0xFF);
  while (ri_u4c1 == 0)
        NOP();
  r=u4rb;
  ri_u4c1=0;
  return r;
}

void write(char *c) {
   char *ptr=c;
   while(*ptr)
       putchar((int)*ptr++);
   putchar(0x0a);
   putchar(0x0d);
}

extern char command[TX_BUFF_SIZE];

double twist[6]={0,0,0,0,0,0};
int pwm[2]={0,0};

void tmr1_init(void) {
	/* Configure the port pin to provide pulse output	*/
	p3_2s = 1;
	pd3_2 = 1;

	/*setting the timer value for 40kHz frequency*/
	ta1 = (unsigned short) (((8000000/16)/40000)-1);
	
	/*  ta1 mode register
	  b1:b0	- TMOD0,TMOD1 - 00 (Timer mode selected)
   	  b2 	- MR0		  - 0 (Set to 0)
  	  b4:b3 - MR2:MR1	  - 0 (Gate function note selected)
   	  b5	- MR3		  - 0 (Set to 0 in timer mode)
   	  b7:b6	- TCK1,TCK0   - f/8 clock source selected
  	  b7	- UFORM		 - 0 (LSB first selected) */

   	ta1mr = 0x80;

        /*start timer */ 
	ta1s = 1;

}

void tmr2_init(void) {
	/* Configure the port pin to provide pulse output	*/
	p3_4s = 1;
	pd3_4 = 1;

	/*setting the timer value for 40kHz frequency*/
	ta2 = (unsigned short) (((8000000/16)/40000)-1);
	
	/*  ta1 mode register
	  b1:b0	- TMOD0,TMOD1 - 00 (Timer mode selected)
   	  b2 	- MR0		  - 0 (Set to 0)
  	  b4:b3 - MR2:MR1	  - 0 (Gate function note selected)
   	  b5	- MR3		  - 0 (Set to 0 in timer mode)
   	  b7:b6	- TCK1,TCK0   - f/8 clock source selected
  	  b7	- UFORM		 - 0 (LSB first selected) */

   	ta2mr = 0x80;

        /*start timer */ 
	ta2s = 1;
}

void tmr3_init(void) {
    // Init_TMRA0 1 mS timer
    ta3mr = 0x80;                                          // timer mode,fc/8 = 1,0 MHz
    ta3 = 24;                                              // 1MHz/25 - 1; 48 oli Fi = 40kHz
    ta3ud = 0;                                             // down count
    ta3ic = 2;                                             // level 2 interrupt
    ta3s = 1;
    ticks = 0;
}
void
main(void) {
    int j;
    HardwareSetup();
    LED1=0;
    LED2=1;
#if 1
    OLED_Set_Display_Mode(0x02);                           // Entire Display On
    OLED_Set_Display_On_Off(0x01);                         // Display On
    OLED_Set_Display_Mode(0x00);                           // Entire Display Off
    OLED_Show_Logo();
    OLED_Set_Display_Mode(0x02);                           // Entire Display Off
    LED2=0;
    Delay(2);
    OLED_Fade_Out();
    OLED_Fill_RAM(0x00);
    OLED_Fade_In();
    write("");
    write("Robot!");
    write("http://phippi.jes.ee/");
    putchar('>');    
    putchar(' ');
#endif
//    tmr1_init();
//    tmr2_init();
//    tmr3_init();
    while (1) {      
        char buf[256];
        if (status.sek_flag==1) {
            status.sek_flag=0;
            LED1 ^= 1;
        }
        if(command[0]!=0) {
            char *tok;
            if(strncmp(command,"twist ",6)==0) {
                int tmp;
                for(tmp=0,tok = strtok(command," "); tok && tmp<=6 ; tok=strtok(0," "),tmp++) {
                    if(tmp>0) {
                        twist[tmp-1]=strtod(tok,NULL); 
                    }
                }
                sprintf(buf,"new twist x=%f(m/s), y=%f(m/s), yaw=%f(deg)",twist[0],twist[1],twist[5]);
                write(buf);
            } else if(strncmp(command,"pwm ",4)==0) {
                int tmp;
                for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                    if(tmp >= 1) {
                        pwm[tmp-1]=(int)strtod(tok,NULL); 
                    }
                }
                if(pwm[0]> 100) pwm[0]= 100;
                if(pwm[1]> 100) pwm[1]= 100;
                if(pwm[0]<-100) pwm[0]=-100;
                if(pwm[1]<-100) pwm[1]=-100;
                sprintf(buf,"manual pwm left=%d%%, right=%d%%",pwm[0],pwm[1]);
                ta1=(int)abs(pwm[0]*TIMERB2COUNT);
                ta2=(int)abs(pwm[1]*TIMERB2COUNT);
                if(pwm[0] > 0) {
                    p2_0=1;
                    p2_1=0;
                } else {
                    p2_0=0;
                    p2_1=1;
                }
                if(pwm[1] > 0) {
                    p2_2=1;
                    p2_3=0;
                } else {
                    p2_2=0;
                    p2_3=1;
                }
                p2_4=1;
                p2_5=1;
                p2_6=1;
                p2_7=1;
               
                write(buf);              
            } else if(strncmp(command,"panda ",6)==0) {
                int tmp;
                for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                    if(tmp == 1) {
                        p3_5=(int)strtod(tok,NULL); 
                    }
                }
                sprintf(buf,"panda %s",p3_5 ? "on":"off");
                write(buf);              
            } else if(strncmp(command,"charge ",7)==0) {
                int tmp;
                for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                    if(tmp == 1) {
                        p1_2=(int)strtod(tok,NULL); 
                    }
                }
                sprintf(buf,"charge %s",p1_2 ? "on":"off");
                write(buf);              
            } else if(strncmp(command,"joy",3)==0) {
                sprintf(buf,"joy %s%s%s%s%s",
                        p1_3 ? ""    :"up ",
                        p1_4 ? ""  :"down ",
                        p1_5 ? ""  :"left ",
                        p1_6 ? "" :"right ",
                        p1_7 ? "":"center" );
                write(buf);                            
            } else       
              write("Unknown command:'");
            putchar('>');    
            putchar(' ');
            command[0]=0;
        }        
        // 300uS needed. On 48Mhz each cycle is ~21nS, so
        // 300 000nS/21=~1200
        for(j=0;j<7;j++)
            uDelay(255); 

        p9_4=0;
        // 300uS needed. On 48Mhz each cycle is ~21nS, so
        // 300 000nS/21=~1200
        for(j=0;j<2;j++)
            uDelay(255); 

        SPI4_send(0xAA);
        // 12uS needed. On 48Mhz each cycle is ~21nS, so
        // 2300nS/12=~220
        for(j=0;j<2;j++)            
            uDelay(255); 
      
        SPI4_send(0xFF);
        for(j=0;j<2;j++)
            uDelay(255); 

        int x;
        for(x=0;x<4;x++) {
            unsigned short c; /* 16 bit value */
            pd9_6=0;
    dinc_u4smr3 = 1;                                       // Master mode when 0
            c=SPI4_receive();
            pd9_6=1;
    dinc_u4smr3 = 0;                                       // slave mode when 1

#if 1
            sprintf( /*(char *)*/ buf, "%s%s%s%s%s SPI4 %04x ",
                (c & (1 << 11)) ? "Arbitr " : "",
                (c & (1 << 12)) ? "Overr " : "",
                (c & (1 << 13)) ? "Fram " : "",
                (c & (1 << 14)) ? "Pari " : "",
                (c & (1 << 15)) ? "Sum " : "", c & 0xFF);
            OLED_Show_String(1, buf, 0 /* left */ , x*8 /* top */ );
#endif
            for(j=0;j<2;j++)            
                uDelay(255); 
        }
              
        p9_4=1;
    }
}
