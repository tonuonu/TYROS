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

extern int alarm;
volatile unsigned short ticks;
volatile unsigned short ticks2;

extern char command[TX_BUFF_SIZE];

double twist[6]={0,0,0,0,0,0};
int pwm[2]={0,0};
int pwmtarget[2]={0,0};
int buzzer=1; // If audible heartbeat is on
float bat=0.0f;
float capacitor=0.0f;
float leftmotorcurrent =0.0f ;
float rightmotorcurrent=0.0f ;
        
void updateOLED1 () {
    OLED_Show_String(  1, "Panda is", 0, 0*8);
    OLED_Show_String(  1, "Charger is", 0, 1*8);
    OLED_Show_String(  1, "Left PWM is      %", 0, 2*8);
    OLED_Show_String(  1, "Right PWM is     %", 0, 3*8);
    OLED_Show_String(  1, "ROS seen    sec ago", 0, 4*8);
}

void updateOLED () {
    char buf[8];
    if(PANDA) {
        OLED_Show_String(  1, "on ", 17, 0*8);
    } else {
        OLED_Show_String(  1, "off", 17, 0*8);
    }
    if(CHARGE) {
        OLED_Show_String(  1, "on ", 21, 1*8);
    } else {
        OLED_Show_String(  1, "off", 21, 1*8);      
    }
    sprintf(buf,"%4d", pwm[0]);
    OLED_Show_String(  1, buf, 26, 2*8);
    sprintf(buf,"%4d", pwm[1]);
    OLED_Show_String(  1, buf, 26, 3*8); 

}

void
main(void) {
    int j;
    int autocharge=0;
    HardwareSetup();    
    LED1=1;
    OLED_Set_Display_On_Off(0x01);                         // Display On
    OLED_Set_Display_Mode(0x00);                           // Entire Display Off
    OLED_Show_Logo();
    OLED_Set_Display_Mode(0x02);                           // Entire Display On
#if 0
    write(VT100RESET);
    write(VT100RESETATTR);
    write(VT100CURSORNULL);
    write(VT100ERASESCREEN);
    write("+------------------------------------------------------------------------------+");
    write(VT100CURSORPROMPT);
    write("+------------------------------------------------------------------------------+");
    write(VT100SCROLLSCREEN);
    write(VT100CURSORHOME);
    writeln("Robot!");
    writeln("http://phippi.jes.ee/");
    putchar('>'); 
    putchar(' ');
    write(VT100CURSORSAVE);
#endif
    u0tb=0;
    Delay(2);
    OLED_Fade_Out();
    OLED_Fill_RAM(0x00);
    OLED_Fade_In();
    updateOLED1();    
    PANDA=1;
    CS4=0;
    CS7=0;
    // 300uS needed. On 48Mhz each cycle is ~21nS, so
    // 300 000nS/21=~1200
    for(j=0;j<2;j++) {
        uDelay(255); 
    }
    u4tb=0xAA;
    u7tb=0xAA;
    accelerometer_write_reg( MMA7455L_REG_I2CAD); 
    CS6=0;
    u6tb=L3G4200D_WHOAMI | 0x80;
    while (1) {
        char buf[256];
    
          

 
#if 1
        //if(ticks % 100 == 0) {
            updateOLED();    
        //}
#endif
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
            } else if(strncmp(command,"autocharge ",11)==0) {
                int tmp;
                for(tmp=0,tok = strtok(command," "); tok && tmp<=2 ; tok=strtok(0," "),tmp++) {
                    if(tmp == 1) {
                        autocharge=(int)strtod(tok,NULL); 
                    }
                }
                sprintf(buf,"autocharge %s",autocharge ? "on":"off");
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
 //       sprintf(buf,"(%2d) ",accwhoamistatus);
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
}
