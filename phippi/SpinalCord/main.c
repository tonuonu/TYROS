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

extern char command[TX_BUFF_SIZE];

double twist[6]={0,0,0,0,0,0};
int pwm[2]={0,0};
int pwmtarget[2]={0,0};
int lastpwm=0;

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
    sprintf(buf,"%2d", lastpwm);
    OLED_Show_String(  1, buf, 19, 4*8); 
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
    Delay(2);
    OLED_Fade_Out();
    OLED_Fill_RAM(0x00);
    OLED_Fade_In();
    updateOLED1();    
    PANDA=1;
    MELEXIS_EN=0; // Low is ON
    // 300uS needed. On 48Mhz each cycle is ~21nS, so
    // 300 000nS/21=~1200
    for(j=0;j<2;j++) {
        uDelay(255); 
    }
    u4tb=0xAA;
    CS2=0;
    accelerometer_read_reg( MMA7455L_REG_WHOAMI ); 
    CS6=0;
    u6tb=L3G4200D_WHOAMI | 0x80;
//    wdc7=0; // Wait ~20ms before reset
//    cm06=1; // Make Watchdog RESET, not just call inteerupt
//    wdts=1; // Start Watchdog
    while (1) {
        int errorflag=0;
        char buf[256];
        if (status.sek_flag==1) {
            status.sek_flag=0;
            LED1 ^= 1;
            if(lastpwm < 60) {
                lastpwm++;
            }
        }
        if(ticks % 10 == 0) {
            updateOLED();    
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
                lastpwm=0;
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
                asm("jmp 0xfffffffc");            
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
        
#if 1
        write(VT100CURSORMELEXISL);
        write("Melexis L :");
        sprintf(buf,"(%2d) ",mlx1whoamistatus);
        write(buf);
        sprintf(buf,"%d %d",mlx1data1,mlx1data2);
        write(buf);
//        write(VT100CURSORMELEXISR);
//        write("Melexis R :");

#endif        

      /* Gyroscope */
#if 1
        write(VT100CURSORGYRO);
        write("Gyro: ");
        sprintf(buf,"(%2d) ",gyrowhoamistatus);
        write(buf);

//        if(accok && accwhoami==85) {
            sprintf(buf,"whoami %3u temp %4d x:%4d y:%4d z:%4d",gyrowhoami,35-gyrotemp,gyrox,gyroy,gyroz);
            write(buf);
//        } else {
//            write("ERROR");
//        } 
#endif
/* Acceleration sensor */
#if 1
        write(VT100CURSORACC);
        write("Acceleration: ");
        sprintf(buf,"(%2d) ",accwhoamistatus);
        write(buf);

//        if(accok && accwhoami==85) {
        if(acccalcnt<100) {
            sprintf(buf,"whoami %2d CALIBRATING",accwhoami);
        } else {
            sprintf(buf,"whoami %2d x:%4d y:%4d z:%4d temp:%4d",accwhoami,accx,accy,accz,acctout);
        }
            write(buf);
//        } else {
//            write("ERROR");
//        } 
#endif
#if 1
        int ad[4];
        ad[0]=AD00 & 0x3FF;
        ad[1]=AD01 & 0x3FF;
        ad[2]=AD02 & 0x3FF;
        ad[3]=AD03 & 0x3FF;
        float bat=(float)ad[3]*(13.64/0x3FF);
        float capacitor=(float)ad[2]/2.27333333 ; // 0x3FF/450V
        write(VT100CURSORAD);
        sprintf(buf,"(L motor %.1fA) ", (float)ad[0]/50.0 );
        write(buf);
        sprintf(buf,"(R motor %.1fA) ", (float)ad[1]/50.0 );
        write(buf);
        sprintf(buf,"(Capacitor %.0fV) ", capacitor);
        write(buf);
        sprintf(buf,"(battery %.1fV) ", bat );
        write(buf);
        if(bat<7.0) 
          errorflag=1;
#endif

        write(VT100CURSORMELEXISE);
        write("5V LDO :");
        write(MELEXIS_EN ? "OFF":"ON "); // Low is Enable
   

        write(VT100CURSORBALL);
        write("Ball :");
        write(BALL_DETECT ? "No ":"Yes");
        write(VT100CURSORPANDA);
        write("Panda :");
        write(PANDA ? "ON ":"OFF");
        write(VT100CURSORCHARGER);
        write("Charger :");
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
        //  ERRORLED=1; 
        } else {
        //  ERRORLED=0; 
        } 
        
    }
}
