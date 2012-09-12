/*
 *  Copyright (c) 2011,2012 Tonu Samuel
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

/*
 * Timer A0 is used in "one shot" mode
 * to provide long delay waiting service for
 * SPI7 with rotational position sensor attached.
 */
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
        break;
    }   
    ir_ta0ic = 0;
}

/*
 * Timer A3 is used in "one shot" mode
 * to provide long delay waiting service for
 * SPI4 with rotational position sensor attached.
 */
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
            break;
    }   
    ir_ta3ic = 0;
}

#pragma vector=TIMER_B5
__interrupt void
s_int(void) {
    /* 
     * This interrupt gets called 100 times per second
     */
    ticks++;
    /*
     * Depending on Joystick we may want to accelerate or decelerate
     */
    if(JOY_UP == 0) {
        pwmtarget[0]+=2;
        pwmtarget[1]+=2;
        if(pwmtarget[0]> 100) {
            pwmtarget[0]= 100;
        }
        if(pwmtarget[1]> 100) {
            pwmtarget[1]= 100;
        }
    } else    
    if(JOY_DOWN == 0) {
        pwmtarget[0]-=2;
        pwmtarget[1]-=2;
        if(pwmtarget[0]<-100) {
            pwmtarget[0]=-100;
        }
        if(pwmtarget[1]<-100) {
            pwmtarget[1]=-100;    
        }
    } 
    /*
     * This is poor mans "debounce" function for joystick. We make sure\
     * it gets just maximum 1 click during 1/5 sec
     */
    if(ticks%20==0){
        if(JOY_RIGHT == 0 && mode<4) {
            mode++;
        }
        if(JOY_LEFT == 0 && mode>0) {
            mode--;
        } 
        
    }
    /*
     * When capacitor is full, LT3750 chip drives CHARGE_DONE low.
     * Problem is, that capacitor will be never charged again if CHARGE
     * pin does not go down before getting up again.
     */
    if(CHARGE_DONE==0) {
        CHARGE=0;
    }
    {
        // Read analog values and compute data
        int ad[4];
        ad[0]=AD00 & 0x3FF;
        ad[1]=AD01 & 0x3FF;
        ad[2]=AD02 & 0x3FF;
        ad[3]=AD03 & 0x3FF;
        bat=bat*0.9f+(float)ad[3]*(13.64/0x3FF)*0.1;
        capacitor=capacitor*0.99+(float)ad[2]/2.27333333*0.01 ; // 0x3FF/450V
        leftmotorcurrent =leftmotorcurrent *0.9+(float)ad[0]/50.0*0.1; // 50 is Tambov!
        rightmotorcurrent=rightmotorcurrent*0.9+(float)ad[1]/50.0*0.1 ;
        /*
         * If current is over 10A on motors, then something is badly wrong
         * and better to make emergency stop. We may get fire otherwise.
         */
        if(leftmotorcurrent>10.0f || rightmotorcurrent>10.0f) {
            ERRORLED=1;
            pwmtarget[0]=0;
            pwmtarget[1]=0;
            pwm[0]=0;
            pwm[1]=0;
            ta1=0;
            ta2=0;
        } else {
            ERRORLED=0;
        }
    }
    /*
     * Here are lot of activities we want to do once per second
     * We try to keep interrupt run time lower by distributing
     * these activites to different 1/100th-s of second.
     */
    switch(ticks % 100) {
        case 0:
            LED1 ^= 1;      
            if(buzzer || bat < 6.6f) {
                /*
                 * This turns on PWM on buzzer to indicate hearbeat. Buzzer
                 * can be commanded off but we override it if batter is low
                 */
                ta4=1;
            }
            break;
        case 1:  
            if(bat>6.3f || bat < 4.0f) { 
                /*
                 * If battery is not dead yet, turn off buzzer.
                 * Avoid false "low battery" alarm when we are not connected to 
                 * any at all. This may happane when we are just USB bus or 
                 * E8a programmer powered
                 */
                ta4=0; 
            }
            break;
        case 50:
            /*
             * When battery is low, make looong beeps.
             */
            if(bat>6.1f ) { 
                ta4=0; 
            }
            break;
        case 70:
           break;

        case 7: 
        case 27: 
        case 47: 
        case 67: 
        case 87:
           /*
            * Just at some random time but 5 times per second
            * with equal interval we update screen data. 
            * at 57.6kb/s we output 7.2kbytes per second.
            * This means we can output lines of 150 bytes data
            * (this is our hardcoded maximum) 48 times per second. 
            * 48/8 lines = 6 times per second. We update those numbers 
            * 5 times per second to stay on safe side. 
            */
           redraw_infoscreen_buffers();
           break;
    }
   

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
}


