/*
 *  Copyright (c) 2012 Tonu Samuel and Kalle-Gustav Kruus
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
#include "hwsetup.h"
#include "main.h"
#include "locking.h"
#include "mma7455l.h"
#include "intrinsics.h"
#include <math.h>

char text[8][150]; // 64 is not enough as we insert VT100 escape sequences
int lineno=0;
int textpos=0;
int mode=MODE_MANUAL;

#define VT100NORMAL     "\x1b" "[0m"
#define VT100BOLD       "\x1b" "[1m"
#define VT100BLINK      "\x1b" "[5m"
#define VT100REVERSE    "\x1b" "[7m"
#define VT100ERASETOEND "\x1b" "[0K"

const char *
melexiscode(int errcode) {
    const  char *err;
    switch(errcode){
        case 1:
            err="ADC Failure";
            break;
        case 2:
            err="ADC Saturation";
            break;
        case 4:
            err="Analog gain below trimmed threshold";
            break;
        case 8:
            err="Magnetic field too weak";
            break;
        case 16:
            err="Magnetic field too strong";
            break;
        case 32:
            err="Analog gain above trimmed threshold";
            break;
        case 64:
            err="Never happens";
            break;
        case 128:
            err="Analog chain rough offset compensation";
            break;
        case 256:
            err="Supply is over 7V";
            break;
        default:
            err="Should not happen";
            break;
    }
    return err;
}

float yaw=0.0f;


void 
redraw_infoscreen_buffers(void) {

#define ROBOTWIDTH    (0.175)   // 17.5 cm is distance from left to right legs
#define TURNSPERSTEP  (4.306)   // Motor shaft makes 4.306 turns for one step 
#define METERSPERSTEP (0.0545) // One step is 54.5mm but we use meters...
#define UNITSPERTURN  (16384.0) // Motor shaft position sensor has 14 bits resolution
#define sign(x) ((x>0.0) - (x<0.0))
    /*
     * To have consistent gyroscope reading, we make local copy of them while 
     * keeping interrupts disabled
     */
    signed int mygyrox,mygyroy,mygyroz;
  
    /*
     * When both side legs have walked some distance,
     * robot has moved on imaginary curve. We compute
     * radius of this curve in meters. "smallradius"
     * is distance from center of this curve to nearest
     * robot legs. "centerradius" is from center of
     * curve to center of robot.
     */
    float centerradius;
    float smallradius;
    float revolutionsL, revolutionsR;
    float distanceL, distanceR;
    /* 
     * Result of computation will be three numbers:
     * dx is position change on X axis in meters 
     * dy is position change on Y axis in meters 
     * yaw is angle change of robot in radians
     */
    float dx, dy;
    char turnL;
    /*
     * Rotational angle sensors may return error codes. We
     * put textual explanation here if needed.
     */
    char *tmpLerror="", *tmpRerror="";
    /*
     * Compute percentage of battery left. 2cell LiPo batteries are known
     * to vary between 6..8.4V, so we scale this range from 0..100%.
     */
    float batpercent=(bat-6.0f)/2.4f*100.0f;
    float UPDATESPERSEC=100.0f/redraw_infoscreen_ticks_passed;
    redraw_infoscreen_ticks_passed=0;

    if(batpercent<0.0f) {
      batpercent=0.0f;
    } else if(batpercent>100.0f){
      batpercent=100.0f;
    }
    /* 
     * Make sure other interrupts does not update MLXaccumulatorL
     * and MLXaccumulatorR while we work with them.
     */
    while(!get_lock1()); 
    revolutionsL= (float)MLXaccumulatorL/UNITSPERTURN;
    revolutionsR= (float)MLXaccumulatorR/UNITSPERTURN;
#if 1
    MLXaccumulatorL=0;
    MLXaccumulatorR=0;
#endif
    /* Make also consistent local copy of gyro readings */
    mygyrox=gyrox;
    mygyroy=gyroy;
    mygyroz=gyroz;
    release_lock1(); 

    static int stallL=0, stallR=0;
    if(fabs(revolutionsL) < 0.05f)      {
        if(stallL < pwm[0]) {
            stallL = pwm[0];
        }
    }
    if(fabs(revolutionsR) < 0.05f)      {
        if(stallR < pwm[1]) {
            stallR = pwm[1];
        }
    }
      
    /*
     * We do everything in SI units, so distance 
     * traveled with left and right legs computed in meters
     */
    distanceL=revolutionsL/TURNSPERSTEP*METERSPERSTEP;
    distanceR=revolutionsR/TURNSPERSTEP*METERSPERSTEP;
    
    if(distanceL == distanceR) {
        /*
         * When both legs have traveled equal distance, we have no curve
         * and our relative location has changed by 0 in X axis (no left or 
         * right turn), Y has incremented by distance traveled and angle
         * change is 0
         */
        yaw=0.0f;
        smallradius =INFINITY;
        centerradius=ROBOTWIDTH/2.0;
        dx=0.0f;
        dy=distanceL;
        turnL=0;
    } else if(distanceL==0 || distanceR==0) {
        /* 
         * When only one side legs have moved then robot rotates over imaginary 
         * center under other legs
         */
        smallradius=0.0f;
        centerradius=ROBOTWIDTH/2.0;
        if(distanceL==0) {
            yaw=distanceR/ROBOTWIDTH;
            turnL=1;
        } else {
            yaw=distanceL/ROBOTWIDTH;
            turnL=0;
        }
    } else {
        /*
         * If we end up here, then all exceptional cases are handled and
         * both sides have walked some unequal distance. Center of circle
         * is somewhere left (when distanceL<distanceR) or right (when 
         * distanceL>distanceR) of us. Exception is, when direction of
         * left and right side were different. Then center is somewhere 
         * under robot body.
         */
        turnL = distanceL < distanceR ? 1 : 0;
        if(turnL) {
            /* 
             * We are turning left. Imaginary center is left of us
             */
            smallradius=ROBOTWIDTH/(distanceR/distanceL-1.0f);
            yaw=-distanceL/smallradius;
        } else {
            /* 
             * We are turning right. Imaginary center is right of us
             */
            smallradius=ROBOTWIDTH/(distanceL/distanceR-1.0f);
            yaw=distanceR/smallradius;
        }
        centerradius=smallradius+(ROBOTWIDTH/2.0f);
        /*
         * Here is nonmature code. If one side goes forward another backward,
         * where is our center?
         */
        if(sign(distanceL) != sign(distanceR)) {
            yaw/=2.0f;
        }
    }
    /* Compute our relative location now */
    dy=sin(yaw)*centerradius;
    dx=cos(yaw)*centerradius-centerradius;
    /* If we turn left, X coordinate should get negative */
    if(!turnL) {
        dx=dx*-1.0f;
    }
    /* 
     * Next constant is from 
     * http://www.wolframalpha.com/input/?i=1%2F%280xFFFF%2F250%29+degrees+in+radians
     */
#define GYRORATE (0.00006658)
    switch(mode) {
        case MODE_MANUAL:
            if(bat<6.0) {  
                tmpLerror=VT100REVERSE "Position sensors LDO off due to low battery " VT100NORMAL " ";
            } else {
                if(mlxRstatus==3) {
                    tmpRerror=VT100REVERSE "Right position sensor missing" VT100NORMAL " ";
                } else if(mlxRstatus==1) {
                    tmpRerror=VT100REVERSE "Right position sensor returns error" VT100NORMAL " ";
                }
                if(mlxLstatus==3) {
                    tmpLerror=VT100REVERSE "Left position sensor missing" VT100NORMAL " ";
                } else if(mlxLstatus==1) {
                    tmpLerror=VT100REVERSE "Left position sensor returns error" VT100NORMAL " ";
                }
            }
            snprintf(text[0],sizeof(text[0]),"\x1b" "[1;1H" "Status: %s%s%s" VT100NORMAL VT100ERASETOEND,gyrowhoami!=211? "Gyroscope error":mlxRstatus==2 && mlxLstatus==2?"OK":"",tmpRerror,tmpLerror);
            snprintf(text[1],sizeof(text[1]),"\x1b" "[2;1H" "Battery: %s %4.1fV %3.0f%% Panda %s" VT100ERASETOEND,bat>6.9 ?"normal      ":bat>6.3 ?"LOW         ":bat>5.0 ?"CRITICAL    ":"disconnected",bat,batpercent,PANDA ? "on ":"off");
            snprintf(text[2],sizeof(text[2]),"\x1b" "[3;1H" "Left drive: %s %5.2fm/s %5.0frpm pwm:%3u%% %4.1fA diff:%8.2f stall:%3d" VT100ERASETOEND,pwm[0]<0 ?"backward":pwm[0]>0 ?"forward ":"brake   ",distanceL*10.0f,revolutionsL*UPDATESPERSEC*60.0,(int)__ROUND(abs(pwm[0])),leftmotorcurrent,revolutionsL,stallL);
            snprintf(text[3],sizeof(text[3]),"\x1b" "[4;1H" "Right drive:%s %5.2fm/s %5.0frpm pwm:%3u%% %4.1fA diff:%8.2f stall:%3d" VT100ERASETOEND,pwm[1]<0 ?"backward":pwm[0]>0 ?"forward ":"brake   ",distanceR*10.0f,revolutionsR*UPDATESPERSEC*60.0,(int)__ROUND(abs(pwm[1])),rightmotorcurrent,revolutionsR,stallR);
            snprintf(text[4],sizeof(text[4]),"\x1b" "[5;1H" "Coilgun: %3.0fV, %s, %s" VT100ERASETOEND,capacitor,CHARGE ? "charging":"waiting ",BALL_DETECT ? "no ball":"Ball! ");
            snprintf(text[5],sizeof(text[5]),"\x1b" "[6;1H" "Odometry: x:%8.5fm y:%8.5fm yaw:%7.4frad" VT100ERASETOEND,dx,dy,yaw);
            snprintf(text[6],sizeof(text[6]),"\x1b" "[7;1H" "Gyro: temp:%3d x:%8.5frad/s y:%8.5frad/s z:%8.5frad/s" VT100ERASETOEND,36-gyrotemp,GYRORATE*mygyrox,GYRORATE*mygyroy,GYRORATE*mygyroz);
            snprintf(text[7],sizeof(text[7]),"\x1b" "[8;1H" VT100BOLD VT100REVERSE ">Normal<" VT100NORMAL "Competition Debug acc sensor Debug pos sensors Debug gyro " VT100NORMAL VT100ERASETOEND);

            break;
        case MODE_COMPETITION:
            if(capacitor<395.0f) {
                CHARGE=1;
            }
            snprintf(text[0],sizeof(text[0]),"\x1b" "[1;1H" "'%s'" VT100ERASETOEND,rx0_buff);
            snprintf(text[1],sizeof(text[1]),"\x1b" "[2;1H" "Battery: %s %4.1fV %3.0f%% Panda %s" VT100ERASETOEND,bat>6.9 ?"normal      ":bat>6.3 ?"LOW         ":bat>5.0 ?"CRITICAL    ":"disconnected",bat,batpercent,PANDA ? "on ":"off");
            snprintf(text[2],sizeof(text[2]),"\x1b" "[3;1H %lf %lf %lf %lf %lf %lf" VT100ERASETOEND,
                     twist[0],
                     twist[1],
                     twist[2],
                     twist[3],
                     twist[4],
                     twist[5]
                     );
            snprintf(text[3],sizeof(text[3]),"\x1b" "[4;1H" VT100ERASETOEND);
            snprintf(text[4],sizeof(text[4]),"\x1b" "[5;1H" "Coilgun: %3.0fV, %s, %s" VT100ERASETOEND,capacitor,CHARGE ? "charging":"waiting ",BALL_DETECT? "Ball! ":"no ball");
//   while(!get_lock2()); 
            snprintf(text[5],sizeof(text[5]),"\x1b" "[6;1H" "Odometry: x:%10.7fm y:%10.7fm yaw:%10.7frad" VT100ERASETOEND,dx,dy,yaw);
            snprintf(text[6],sizeof(text[6]),"\x1b" "[7;1H" "Gyro: temp:%3d x:%10.7frad/s y:%10.7frad/s z:%10.7frad/s" VT100ERASETOEND,36-gyrotemp,GYRORATE*mygyrox,GYRORATE*mygyroy,GYRORATE*mygyroz);
//  release_lock2();
            snprintf(text[7],sizeof(text[7]),"\x1b" "[8;1H" " Normal" VT100BOLD VT100REVERSE ">Competition<" VT100NORMAL "Debug acc sensor Debug pos sensors Debug gyro" VT100ERASETOEND);
            break;
        case MODE_DEBUG_ACCSENSOR:
            snprintf(text[0],sizeof(text[0]),"\x1b" "[1;1H" "accok %d accwhoami %d accx %d accy %d accz %d acctout %d accstatus %d" VT100ERASETOEND,accok , accwhoami , accx , accy , accz , acctout , accstatus );
            snprintf(text[1],sizeof(text[1]),"\x1b" "[2;1H" " accx %10.7fG    accy %10.7fG    accz %10.7fG " VT100ERASETOEND, (float)accx/64.0 , (float)accy/64.0 , (float)accz/64.0 );
            snprintf(text[2],sizeof(text[2]),"\x1b" "[3;1H" " accx %10.7fm/s2 accy %10.7fm/s2 accz %10.7fm/s2 " VT100ERASETOEND, (float)accx/64.0*9.807 , (float)accy/64.0*9.807 , (float)accz/64.0*9.807 );
            snprintf(text[3],sizeof(text[3]),"\x1b" "[4;1H" "" VT100ERASETOEND);
            snprintf(text[4],sizeof(text[4]),"\x1b" "[5;1H" "" VT100ERASETOEND);
            snprintf(text[5],sizeof(text[5]),"\x1b" "[6;1H" "" VT100ERASETOEND);
            snprintf(text[6],sizeof(text[6]),"\x1b" "[7;1H" "" VT100ERASETOEND);
            snprintf(text[7],sizeof(text[7]),"\x1b" "[8;1H" " Normal Competition"VT100BOLD VT100REVERSE ">Debug acc sensor<" VT100NORMAL "Debug pos sensors Debug gyro" VT100ERASETOEND);
            break;
        case MODE_DEBUG_POSSENSOR:
//    distanceL=revolutionsL/TURNSPERSTEP*METERSPERSTEP;
//    distanceR=revolutionsR/TURNSPERSTEP*METERSPERSTEP;
    
            snprintf(text[0],sizeof(text[0]),"\x1b" "[1;1H" "" VT100ERASETOEND);
            snprintf(text[1],sizeof(text[1]),"\x1b" "[2;1H" "" VT100ERASETOEND);
            snprintf(text[2],sizeof(text[2]),"\x1b" "[3;1H" "twist: %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f age: %d " VT100ERASETOEND,twist[0],twist[1],twist[2],twist[3],twist[4],twist[5],twistcmdage);
            snprintf(text[3],sizeof(text[3]),"\x1b" "[4;1H" "revs %7.3f %7.3f distance %7.4fm %7.4fm" VT100ERASETOEND,revolutionsL,revolutionsR,distanceL,distanceR);
            snprintf(text[4],sizeof(text[4]),"\x1b" "[5;1H" "dx:%9.6f dy:%9.6f angle:%7.4frad" VT100ERASETOEND,dx,dy,yaw);
            snprintf(text[5],sizeof(text[5]),"\x1b" "[6;1H" "left  %5.1frps, %4.0frpm %5.1f steps/s %6.1f steps/min %7.4fm/s" VT100ERASETOEND
                ,revolutionsL*UPDATESPERSEC,revolutionsL*UPDATESPERSEC*60.0,revolutionsL/TURNSPERSTEP*UPDATESPERSEC,revolutionsL/TURNSPERSTEP*UPDATESPERSEC*60.0,distanceL*UPDATESPERSEC);
            snprintf(text[6],sizeof(text[6]),"\x1b" "[7;1H" "right %5.1frps, %4.0frpm %5.1f steps/s %6.1f steps/min %7.4fm/s" VT100ERASETOEND
                ,revolutionsR*UPDATESPERSEC,revolutionsR*UPDATESPERSEC*60.0,revolutionsR/TURNSPERSTEP*UPDATESPERSEC,revolutionsR/TURNSPERSTEP*UPDATESPERSEC*60.0,distanceR*UPDATESPERSEC);
            snprintf(text[7],sizeof(text[7]),"\x1b" "[8;1H" " Normal Competition Debug acc sensor"VT100BOLD VT100REVERSE ">Debug pos sensors<" VT100NORMAL "Debug gyro " VT100ERASETOEND); 
            break;
        case MODE_DEBUG_GYRO:
            snprintf(text[0],sizeof(text[0]),"\x1b" "[1;1H" "Gyro raw: whoami:%3d temp:%3d x:%6d y:%6d z:%6d" VT100ERASETOEND,gyrowhoami,gyrotemp,gyrorawx,gyrorawy,gyrorawz);
            snprintf(text[1],sizeof(text[1]),"\x1b" "[2;1H" "Gyro calibrationmin min:      x:%6d y:%6d z:%6d" VT100ERASETOEND,gyrominx,gyrominy,gyrominz);
            snprintf(text[2],sizeof(text[2]),"\x1b" "[3;1H" "Gyro calibrationmin max:      x:%6d y:%6d z:%6d" VT100ERASETOEND,gyromaxx,gyromaxy,gyromaxz);
            snprintf(text[3],sizeof(text[3]),"\x1b" "[4;1H" "Gyro filtered:                x:%6d y:%6d z:%6d" VT100ERASETOEND,mygyrox,mygyroy,mygyroz);
            snprintf(text[4],sizeof(text[4]),"\x1b" "[5;1H" "Gyro computed: temp:%3d x:%10.7frad/s y:%10.7frad/s z:%10.7frad/s" VT100ERASETOEND,36-gyrotemp,GYRORATE*mygyrox,GYRORATE*mygyroy,GYRORATE*mygyroz);
            snprintf(text[5],sizeof(text[5]),"\x1b" "[6;1H" "" VT100ERASETOEND);
            snprintf(text[6],sizeof(text[6]),"\x1b" "[7;1H" "" VT100ERASETOEND);
            snprintf(text[7],sizeof(text[7]),"\x1b" "[8;1H" " Normal Competition Debug acc sensor Debug pos sensors" VT100BOLD VT100REVERSE ">Debug gyro<" VT100NORMAL VT100ERASETOEND);
            break;
        default:
            break;
    }
}