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
#include <stdlib.h>
#include <string.h>
#include "uart.h"
#include "main.h"
#include "hwsetup.h"
#include "gyro.h"
#include "SPI.h"
#include "mma7455l.h"
#include "uart.h"


extern int alarm;
volatile unsigned short ticks;
volatile unsigned short ticks2;

extern char command[TX_BUFF_SIZE];

double twist[6]={0,1,0,0,0,0};
int twistflag=0;
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

int
main(void) {
    int j;
    //int autocharge=0;
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
    /* 
     * Position sensors interrupt-driven state machine start  
     */
    CS4=0;
    CS7=0;
    // 300uS needed. On 48Mhz each cycle is ~21nS, so
    // 300 000nS/21=~1200
    for(j=0;j<2;j++) {
        uDelay(255); 
    }
    u4tb=0xAA;
    u7tb=0xAA;
    /* 
     * Acceleration sensor interrupt-driven state machine start  
     */
    CS2=0;
    uDelay(16);
    accelerometer_write_reg(MMA7455L_REG_MCTL); 
    /* 
     * Gyroscopic sensor interrupt-driven state machine start  
     */
    CS6=0;
    u6tb=L3G4200D_WHOAMI | 0x80;
    while(1) {
        unsigned long r;
        char buf[256];
        char command[256];
        __wait_for_interrupt();
#if 0
        if((r=__read(0,command,sizeof(command)))>0) {
            unsigned char in = (unsigned char)command[0];
            switch(in) {
            case 13:
                rx0_buff[rx0_ptr]=0;
                /* No null-character is implicitly appended to the end of destination, so destination will only be null-terminated if the length of the C string in source is less than num. */
                strncpy(command,rx0_buff,RX_BUFF_SIZE-1);
                rx0_ptr=0;
                rx0_buff[rx0_ptr]=0;
                parsecmd(command);
                snprintf(buf,sizeof(buf),"parse '%s' %lu\n",command,r);
                __write(1,buf,strlen(buf));
                break;
            case 127:
                rx0_ptr--;
                rx0_buff[rx0_ptr]= 0;
                break;
            default:
               rx0_buff[rx0_ptr]= in;
               rx0_ptr++;
               rx0_buff[rx0_ptr]= 0;
               break;
            }
            if (rx0_ptr >= RX_BUFF_SIZE) {
                rx0_ptr = RX_BUFF_SIZE-1;
            }
            udelay(100000);
        } else {
            udelay(100000);
        }
#endif
    }
}
