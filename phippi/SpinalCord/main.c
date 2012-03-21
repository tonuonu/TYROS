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
gyro_send_data(unsigned char c) {
    u6tb = c;
    while (ti_u6c1 == 0) {
        // wait till transmission is over 
        // NOP();
    }
    uDelay(100);
}

unsigned short 
gyro_receive(void) {
    unsigned short r;
    gyro_send_data(0xFF);
    while (ri_u6c1 == 0) {
        //NOP();
    }
    r=u6rb;    
    ri_u6c1=0;
    return r;
}



void
SPI3_send_data(unsigned char c) {
    while (ti_u3c1 == 0)
        NOP();
    uDelay(SPI_DELAY);
    p4_0 = 1;
    uDelay(SPI_DELAY);
    u3tb = c;
}

void
SPI3_send_cmd(unsigned char c) {
    while (ti_u3c1 == 0)
        NOP();
    uDelay(SPI_DELAY);
    p4_0 = 0;
    uDelay(SPI_DELAY);
    u3tb = c;
}

void
SPI7_send(unsigned short c) {
  while (ti_u7c1 == 0)
        NOP();
  uDelay(SPI_DELAY);
  ti_u7c1=0;
  u7tb = c;

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

void
SPI6_send(unsigned short c) {
  while (ti_u6c1 == 0)
        NOP();
  uDelay(SPI_DELAY);
  ti_u6c1=0;
  u6tb = c;

}

short unsigned
SPI6_receive(void) {
  short unsigned r;
  SPI6_send(0xFF);
  while (ri_u6c1 == 0)
        NOP();
  r=u6rb;
  ri_u6c1=0;
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
int pwmtarget[2]={0,0};

void
main(void) {
    HardwareSetup();    
    LED1=1;
#if 1
    OLED_Set_Display_Mode(0x02);                           // Entire Display On
    OLED_Set_Display_On_Off(0x01);                         // Display On
    OLED_Set_Display_Mode(0x00);                           // Entire Display Off
    OLED_Show_Logo();
    OLED_Set_Display_Mode(0x02);                           // Entire Display Off
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
    while (1) {
        char buf[256];
        if (status.sek_flag==1) {
            status.sek_flag=0;
            LED1 ^= 1;
        }
                
        if(command[0]!=0) {
            char *tok;
            if(strncmp(command,"gyro",4)==0) {

            } else if(strncmp(command,"twist ",6)==0) {
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
            } else if(strncmp(command,"panda ",6)==0) {
                int tmp;
                for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                    if(tmp == 1) {
                        PANDA=(int)strtod(tok,NULL); 
                    }
                }
                sprintf(buf,"panda %s",PANDA ? "on":"off");
                write(buf);              
            } else if(strncmp(command,"charge ",7)==0) {
                int tmp;
                for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                    if(tmp == 1) {
                        p1_2=(int)strtod(tok,NULL); 
                    }
                }
                sprintf(buf,"charge %s",CHARGE ? "on":"off");
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
              write("Unknown command:'");
            }
            putchar('>');    
            putchar(' ');
            command[0]=0;
        }        
        
#if 0        
        
        // 300uS needed. On 48Mhz each cycle is ~21nS, so
        // 300 000nS/21=~1200
        for(j=0;j<7;j++)
            uDelay(255); 

        CS4=0; // enable left Melexis
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
            c=SPI4_receive();
            sprintf(buf,"SPI4 %x",c);
            write(buf);
            pd9_6=1;
            for(j=0;j<2;j++)
                uDelay(255); 
        }
        CS4=1; // disable melexis
        
#endif        

unsigned short c1,c2,c3,c4; /* 16 bit value */
#if 0
        CS6=0; // enable gyro
        uDelay(100);
        /*
         * 0x80 is most significant bit=1 and indicates we 
         * want to read register, not write. 0x0F is "whoami"
         * and should be responded by 1101 0011 or 211 in dec or 0xd3 in hex 
         */
        gyro_send_data(0x0F | 0x80); 
        uDelay(100);
        c=gyro_receive();
        CS6=1; // disable gyro
        sprintf(buf,"gyro %x",c);
        write(buf);
#endif
        uDelay(50);
#if 1
        CS6=0; // enable gyro
        uDelay(100);
        /*
         * 0x80 is most significant bit=1 and indicates we 
         * want to read register, not write. 0x0F is "whoami"
         * and should be responded by 1101 0011 or 211 in dec or 0xd3 in hex 
         */
        gyro_send_data(0x20 | 0x00); 
        uDelay(100);
        gyro_send_data(1|2|4|8); 
        CS6=1; // disable gyro
#endif
        uDelay(100);
#if 1
        CS6=0; // enable gyro
        uDelay(100);
        /*
         * 0x80 is most significant bit=1 and indicates we 
         * want to read register, not write. 0x0F is "whoami"
         * and should be responded by 1101 0011 or 211 in dec or 0xd3 in hex 
         */
        gyro_send_data(0x26 | 0x80); 
        uDelay(100);
        c1=gyro_receive();
        CS6=1; // disable gyro
#endif
        uDelay(100);
#if 1
        CS6=0; // enable gyro
        uDelay(100);
        /*
         * 0x80 is most significant bit=1 and indicates we 
         * want to read register, not write. 0x0F is "whoami"
         * and should be responded by 1101 0011 or 211 in dec or 0xd3 in hex 
         */
        gyro_send_data(0x29 | 0x80); 
        uDelay(100);
        c2=gyro_receive();
        CS6=1; // disable gyro
#endif
        uDelay(100);
#if 1
        CS6=0; // enable gyro
        uDelay(100);
        /*
         * 0x80 is most significant bit=1 and indicates we 
         * want to read register, not write. 0x0F is "whoami"
         * and should be responded by 1101 0011 or 211 in dec or 0xd3 in hex 
         */
        gyro_send_data(0x2b | 0x80); 
        uDelay(100);
        c3=gyro_receive();
        CS6=1; // disable gyro
#endif
        uDelay(100);        
#if 1
        CS6=0; // enable gyro
        uDelay(100);
        /*
         * 0x80 is most significant bit=1 and indicates we 
         * want to read register, not write. 0x0F is "whoami"
         * and should be responded by 1101 0011 or 211 in dec or 0xd3 in hex 
         */
        gyro_send_data(0x2d | 0x80); 
        uDelay(100);
        c4=gyro_receive();
        CS6=1; // disable gyro

        sprintf(buf,"gyro temp %d x_h %d y_h %d z_h %d",(signed char)(c1 &0xff),(signed char)(c2&0xff),(signed char)(c3&0xff),(signed char)(c4&0xff));
        write(buf);
#if 0
        sprintf(buf,"%s %s %s %s %s"
                ,(c2 & (1 << 12)) ? "abt ":""  
                ,(c2 & (1 << 13)) ? "oer ":""
                ,(c2 & (1 << 14)) ? "fer ":""
                ,(c2 & (1 << 15)) ? "per ":""
                ,(c2 & (1 << 16)) ? "sum ":""
                  );
        write(buf);

        sprintf(buf,"%s %s %s %s %s"
                ,(c3 & (1 << 12)) ? "abt ":""  
                ,(c3 & (1 << 13)) ? "oer ":""
                ,(c3 & (1 << 14)) ? "fer ":""
                ,(c3 & (1 << 15)) ? "per ":""
                ,(c3 & (1 << 16)) ? "sum ":""
                  );
        write(buf);
#endif
#endif
    }
}
