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



#pragma vector=TIMER_B5
__interrupt void
s_int(void) {
    // This interrupt gets called 48 times per second
  
    // We may want to do something once per second in main loop
    // so we set flag to indicate when to do.
    if(++ticks % 48 == 0) {
      status.sek_flag=1;
      // This turns on PWM on buzzer
        ta4=(int)abs(64 -  p1_3*2 + p1_4*2 + p1_5*4 + p1_6*8 + p1_3*16 *TIMER4COUNT);
    } else if(ticks % 48 == 1  ) {
        // Turn off buzzer
        ta4=0;
    }
    
    // Make sure pwm-s get closer to targets but not too fast. 
    if(pwmtarget[0] < pwm[0]) {
        pwm[0]--;
    } else if(pwmtarget[0] > pwm[0]) { 
        pwm[0]++;
    }
    
    if(pwmtarget[1] < pwm[1]) {
        pwm[0]--;
    } else if(pwmtarget[1] > pwm[1]) {
        pwm[1]++;
    }

    // Update MCU PWM timers for new values
    ta1=(int)abs(pwm[0]*TIMERB2COUNT);
    ta2=(int)abs(pwm[1]*TIMERB2COUNT);


    // Make sure proper bits set on motor drivers to go forward or backward
    if(pwm[0] > 0) {
        RIGHT_INA=1; // right in a
        RIGHT_INB=0; // right in b
    } else {
        RIGHT_INA=0; // right in a
        RIGHT_INB=1; // right in b
    }
    
    if(pwm[1] > 0) {
        LEFT_INA=1; // left in a
        LEFT_INB=0; // left in b
    } else {
        LEFT_INA=0; // left in a
        LEFT_INB=1; // left in b
    }

    // Enable motors
    RIGHT_DIAGA=1; // right diag a (enable)
    RIGHT_DIAGB=1; // right diag b (enable)
    LEFT_DIAGA=1; // left diag a (enable)
    LEFT_DIAGB=1; // left diag b (enable)
        
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
    
#if 0
      p1_3 ? ""    :"up ",
      p1_4 ? ""  :"down ",
      p1_5 ? ""  :"left ",
      p1_6 ? "" :"right ",
      p1_7 ? "":"center" );
#endif   
   
     if(ticks % 48 == 1  ) {
         ta4=0;
    }    
}

// 1000 Hz interrupt
#pragma vector=TIMER_A3
__interrupt void
ms_int(void) {

}
