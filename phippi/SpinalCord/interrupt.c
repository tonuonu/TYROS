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
#include <math.h>

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
  switch(mlx1whoamistatus) {
  case 6+1:
    CS4=0;
    uDelay(6); // t6, 10+uS on scope, 6.9 required
    u4tb=0xAA;
    mlx1whoamistatus=0;
    break;
  default:
    u4tb=0xFF;
  }   
  ir_ta3ic = 0;
}

#pragma vector=TIMER_B5
__interrupt void
s_int(void) {
    /* 
     * This interrupt gets called 48 times per second
     * We may want to do something once per second in main loop
     * so we set flag to indicate when to do.
     */
    ticks++;
    if(JOY_UP == 0) {
             pwmtarget[0]+=2;
             pwmtarget[1]+=2;
    } else    
    if(JOY_DOWN == 0) {
            pwmtarget[0]-=2;
            pwmtarget[1]-=2;
    } 

    if(ticks %20==0){
        if(JOY_RIGHT == 0) {
            if(mode<4)
                mode++;
        }
        if(JOY_LEFT == 0) {
            if(mode>0)
                mode--;
        } 

    }
    if(CHARGE_DONE==0) {
        CHARGE=0;
    }

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
    case 70:
      {
        // Read analog values and compute data
        int ad[4];
        ad[0]=AD00 & 0x3FF;
        ad[1]=AD01 & 0x3FF;
        ad[2]=AD02 & 0x3FF;
        ad[3]=AD03 & 0x3FF;
        bat=(float)ad[3]*(13.64/0x3FF);
        capacitor=(float)ad[2]/2.27333333 ; // 0x3FF/450V
        leftmotorcurrent =(float)ad[0]/50.0 ; // Tambov!
        rightmotorcurrent=(float)ad[1]/50.0 ;
      }
      break;

     case 77: // Just at some random time but 10 times per second
        redraw_infoscreen_buffers();
    
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
    /* 
    sammu pikkuseks saan umbes 54.5 mm
    Kalle-Gustav Kruus: 4,306 tais mootori pooret sammu kohta
    Kalle-Gustav Kruus: nagu 0.1mm põõrde kohta umbes
    Tonu Samuel: 0.07900917431  mm siis poorde kohta
    /1000.0 on, et meetriteks teisendamine
    */
#define robotwidth 0.175     
#define sign(x) ((x>0.0f) - (x<0.0f))
    distanceleft =revolutions1*0.07900917431f/1000.0f;
    distanceright=revolutions2*0.07900917431f/1000.0f;
    revolutions1=0.0f;
    revolutions2=0.0f;
    if(distanceleft == distanceright) {
        dx=0.0f;
        dy=distanceleft;
    } else {
        float smallradius=robotwidth/((distanceleft<distanceright ? distanceright/distanceleft:distanceleft/distanceright)-1);
        float angletraveled=(distanceleft<distanceright ? distanceleft:distanceright)/smallradius;
        float centerradius=smallradius+(robotwidth/2.0f);
        dy=sin(angletraveled)*centerradius;
        dx=cos(angletraveled)*centerradius * (distanceleft < distanceright ? 1.0f : -1.0f)
          - (sign(distanceleft)+sign(distanceright)!=1  ? centerradius : 0);
    }
    
    
/*     if(ticks % 48 == 1  ) {
         ta4=0;
    }*/    
}


#pragma vector=TIMER_B4
__interrupt void
b4_int(void) {
    /* 
     * This interrupt gets called 48 times per second
     * We may want to do something once per second in main loop
     * so we set flag to indicate when to do.
     */
    ticks2++;
    switch(ticks2 % 2) {
    case 0:
      break;
    case 1: 
      break;
    }    
        
}


#if 0
// 1000 Hz interrupt
#pragma vector=TIMER_A3
__interrupt void
ms_int(void) {
 
}
#endif