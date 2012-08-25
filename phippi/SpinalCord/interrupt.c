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
#include "SPI.h"
#include "mma7455l.h"

int todocase=0;

#pragma vector=TIMER_A0
__interrupt void
oneshot1(void) {
  switch(mlx2whoamistatus) {
  case 6+1:
    CS7=0;
    uDelay(6); // t6, 10+uS on scope, 6.9 required
    u7tb=0xAA;
    mlx2whoamistatus=0;
    break;
  default:
    u7tb=0xFF;
  }   
    ir_ta0ic = 0;
}

#pragma vector=TIMER_A3
__interrupt void
oneshot2(void) {
  
    CS4=0;
    uDelay(6); // t6, 10+uS on scope, 6.9 required
    u4tb=0xAA;
    mlx1whoamistatus=0;
  ir_ta3ic = 0;
}

#pragma vector=TIMER_B5
__interrupt void
s_int(void) {
    // This interrupt gets called 48 times per second
    // We may want to do something once per second in main loop
    // so we set flag to indicate when to do.
    ticks++;
    switch(ticks % 100) {
    case 0:
        status.sek_flag=1;
        if(buzzer || bat < 6.6) {
            // This turns on PWM on buzzer
            ta4=1;
        }
        break;
    case 1:  
        if(bat>6.3 || bat < 4.0) { // If battery is not dead yet 
            ta4=0; // Turn off buzzer
        }
        break;
    case 50:  
        if(bat>6.1 ) { // If battery is not dead yet 
            ta4=0; // Turn off buzzer
        }
        break;
    }
    
    if(JOY_RIGHT == 0) {
//      OLED_Set_Display_Mode(0x03);                           // Inverse display
      PANDA = 0;
    } else    
    if(JOY_LEFT == 0) {
//        OLED_Set_Display_Mode(0x02);                           // Normal display
        PANDA = 1;
    } 

    if(JOY_UP == 0) {
        pwmtarget[0]+=2;
        pwmtarget[1]+=2;
    } else    
    if(JOY_DOWN == 0) {
        pwmtarget[0]-=2;
        pwmtarget[1]-=2;
    } 
    
    if(pwmtarget[0]> 100) pwmtarget[0]= 100;
    if(pwmtarget[1]> 100) pwmtarget[1]= 100;
    if(pwmtarget[0]<-100) pwmtarget[0]=-100;
    if(pwmtarget[1]<-100) pwmtarget[1]=-100;    
#if 1
    // Make sure pwm-s get closer to targets but not too fast. 
    if(pwmtarget[0] < pwm[0]) {
        pwm[0]--;
    } else if(pwmtarget[0] > pwm[0]) { 
        pwm[0]++;
    }
    
    if(pwmtarget[1] < pwm[1]) {
        pwm[1]--;
    } else if(pwmtarget[1] > pwm[1]) {
        pwm[1]++;
    }
#else
    pwm[0]=pwmtarget[0];
    pwm[1]=pwmtarget[1];
#endif    
    // Update MCU PWM timers for new values
    ta1=(int)abs(pwm[0]*TIMERB2COUNT/100);
    ta2=(int)abs(pwm[1]*TIMERB2COUNT/100);
    // Make sure proper bits set on motor drivers to go forward or backward
    if(pwm[0] == 0) {
        RIGHT_INA=0; // right in a
        RIGHT_INB=0; // right in b      
        RIGHT_DIAGA=0; // right diag a (disable)
        RIGHT_DIAGB=0; // right diag b (disable)
    } else if(pwm[0] > 0) {
        RIGHT_INA=1; // right in a
        RIGHT_INB=0; // right in b
        RIGHT_DIAGA=1; // right diag a (enable)
        RIGHT_DIAGB=1; // right diag b (enable)
    } else {
        RIGHT_INA=0; // right in a
        RIGHT_INB=1; // right in b
        RIGHT_DIAGA=1; // right diag a (enable)
        RIGHT_DIAGB=1; // right diag b (enable)
    }
    
    if(pwm[1] == 0) {
        LEFT_INA=0; // right in a
        LEFT_INB=0; // right in b      
        LEFT_DIAGA=0; // left diag a (disable)
        LEFT_DIAGB=0; // left diag b (disable)
    } else if(pwm[1] > 0) {
        LEFT_INA=1; // left in a
        LEFT_INB=0; // left in b
        LEFT_DIAGA=1; // left diag a (enable)
        LEFT_DIAGB=1; // left diag b (enable)
    } else {
        LEFT_INA=0; // left in a
        LEFT_INB=1; // left in b
        LEFT_DIAGA=1; // left diag a (enable)
        LEFT_DIAGB=1; // left diag b (enable)
    }
        
    // Reduce PWM targets for next turn. This makes motors slow down in 
    // ~2 seconds if no new commands are received.
    
#if 1
    if(pwmtarget[0] > 0) {
        pwmtarget[0]--;
    } else if(pwmtarget[0] < 0) {
        pwmtarget[0]++;
    }

    if(pwmtarget[1] > 0) {
        pwmtarget[1]--;
    } else if(pwmtarget[1] < 0) { 
        pwmtarget[1]++;
    }
#endif    
       
/*     if(ticks % 48 == 1  ) {
         ta4=0;
    }*/    
}

#if 0
// 1000 Hz interrupt
#pragma vector=TIMER_A3
__interrupt void
ms_int(void) {
 
}
#endif