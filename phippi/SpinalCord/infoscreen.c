/*
 *  Copyright (c) 2012, Tonu Samuel and Kalle-Gustav Kruus
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
#include <math.h>
/*
Competition:
  0         1         2         3         4         5         6
  0123456789012345678901234567890123456789012345678901234567890123
1 Status: OK                                                      1      
2 Battery: CRITICAL x.xxV xxx% Panda: off                         2 Good, LOW
3 Left drive:  forward  x.xm/s -xxxxrpm pwm:xxx% xx.xA            3
4 Right drive: backward x.xm/s -xxxxrpm pwm:xxx% xx.xA            4
5 Coilgun: xxxV Charging, no ball                                 5
6 Odometry: x:-0.xxxxxm y:-0.xxxxxm yaw:-0.xxxxxrad/s             6
7  Competition mode! Pull joystick down to start or up to cancel  7 *
8  Normal Competition Demo Debug drivetrain Debug sensors         8            
  0123456789012345678901234567890123456789012345678901234567890123
  0         1         2         3         4         5         6 
*/
char linedata[8][150]; // 64 is not enough as we insert VT100 escape sequences
int lineno=0;
int linepos=0;
int mode=MODE_MANUAL;

#define VT100NORMAL  "\x1b" "[0m"
#define VT100BOLD    "\x1b" "[1m"
#define VT100BLINK   "\x1b" "[5m"
#define VT100REVERSE "\x1b" "[7m"
#define VT100ERASETOEND "\x1b" "[0K"
const char *melexiscode(int errcode) {
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
void 
redraw_infoscreen_buffers(void) {
    float wtfangle=0.0f;
    float smallradius=0.0f;

    char *tmpmlxrighterror="";
    char *tmpmlxlefterror="";
#define ROBOTWIDTH    (0.175)    // 17.5 cm is distance from left to right legs
#define TURNSPERSTEP  (4.306)   // Motor shaft makes 4.306 turns for one step 
#define METERSPERSTEP (0.00545) // One step is 54.5mm
#define UNITSPERTURN  (16384.0) // Rotation position sensor have 14 bits resolution
#define sign(x) ((x>0.0) - (x<0.0))
    distanceleft =(float)MLXaccumulatorL/UNITSPERTURN/TURNSPERSTEP*METERSPERSTEP;
    distanceright=(float)MLXaccumulatorR/UNITSPERTURN/TURNSPERSTEP*METERSPERSTEP;
    float centerradius=ROBOTWIDTH/2.0;
    if(distanceleft == distanceright) {
        smallradius= INFINITY;
        dx=0.0f;
        dy=distanceleft;
    } else if(distanceleft==0) {
        wtfangle=distanceright/ROBOTWIDTH;
    } else if(distanceright==0) {
        wtfangle=distanceleft/ROBOTWIDTH;
    } else {
        smallradius=ROBOTWIDTH/((distanceleft<distanceright ? distanceright/distanceleft:distanceleft/distanceright)-1.0f);
        centerradius=smallradius+(ROBOTWIDTH/2.0f);
        wtfangle=(distanceleft<distanceright ? distanceleft:distanceright)/smallradius;
        if(sign(distanceleft) != sign(distanceright)) {
            wtfangle/=2;
        }
    }
    dy=sin(wtfangle)*centerradius;
    dx=cos(wtfangle)*centerradius-centerradius;
    if(distanceleft>distanceright && distanceleft != 0 && distanceright != 0)
        dx=dx*-1.0;
  switch(mode) {
  case MODE_MANUAL:
    if(mlxrightstatus==3) {
      tmpmlxrighterror=VT100REVERSE "Right position sensor missing" VT100NORMAL " ";
    } else if(mlxrightstatus==1) {
      tmpmlxrighterror=VT100REVERSE "Right position sensor returns error" VT100NORMAL " ";
    }
    if(mlxleftstatus==3) {
      tmpmlxlefterror=VT100REVERSE "Left position sensor missing" VT100NORMAL " ";
    } else if(mlxleftstatus==1) {
      tmpmlxlefterror=VT100REVERSE "Left position sensor returns error" VT100NORMAL " ";
    }
    snprintf(linedata[0],sizeof(linedata[0]),"\x1b" "[1;1H" "Status: %s%s%s" VT100NORMAL VT100ERASETOEND,gyrowhoami!=211? "Gyroscope error":mlxrightstatus==2 && mlxleftstatus==2?"OK":"",tmpmlxrighterror,tmpmlxlefterror);
    snprintf(linedata[1],sizeof(linedata[1]),"\x1b" "[2;1H" "Battery: %s %4.1fV %3.0f%% Panda %s" VT100ERASETOEND,bat>6.9 ?"normal      ":bat>6.3 ?"LOW         ":bat>5.0 ?"CRITICAL    ":"disconnected",bat,(bat-6.0f)/2.4f*100.0f,PANDA ? "on ":"off");
    snprintf(linedata[2],sizeof(linedata[2]),"\x1b" "[3;1H" "Left drive: %s %4.2fm/s %.0frpm pwm:%3u%% %4.1fA" VT100ERASETOEND,pwm[0]<0 ?"backward":pwm[0]>0 ?"forward ":"brake   ",distanceleft *10.0f,(float)MLXaccumulatorL/UNITSPERTURN*2.0*60.0,(int)abs(pwm[0]),leftmotorcurrent);
    snprintf(linedata[3],sizeof(linedata[3]),"\x1b" "[4;1H" "Right drive:%s %4.2fm/s %.0frpm pwm:%3u%% %4.1fA" VT100ERASETOEND,pwm[1]<0 ?"backward":pwm[0]>0 ?"forward ":"brake   ",distanceright*10.0f,(float)MLXaccumulatorR/UNITSPERTURN*2.0*60.0,(int)abs(pwm[1]),rightmotorcurrent);
    snprintf(linedata[4],sizeof(linedata[4]),"\x1b" "[5;1H" "Coilgun: %3.0fV, %s, %s" VT100ERASETOEND,capacitor,CHARGE ? "charging":"waiting ",BALL_DETECT ? "no ball":"Ball! ");
    snprintf(linedata[5],sizeof(linedata[5]),"\x1b" "[6;1H" "Odometry: x:%fm y:%fm yaw:%frad" VT100ERASETOEND,dx,dy,wtfangle);
    snprintf(linedata[6],sizeof(linedata[6]),"\x1b" "[7;1H" "" VT100ERASETOEND);
    snprintf(linedata[7],sizeof(linedata[7]),"\x1b" "[8;1H" VT100BOLD VT100REVERSE ">Normal<" VT100NORMAL "Competition Demo Debug drivetrain Debug sensors " VT100NORMAL VT100ERASETOEND);
    break;
  case MODE_COMPETITION:
    if(capacitor<395.0f) {
        CHARGE=1;
    }
    snprintf(linedata[0],sizeof(linedata[0]),"\x1b" "[1;1H" VT100ERASETOEND);
    snprintf(linedata[1],sizeof(linedata[1]),"\x1b" "[2;1H" "Battery: %s %4.1fV %3.0f%% Panda %s" VT100ERASETOEND,bat>6.9 ?"normal      ":bat>6.3 ?"LOW         ":bat>5.0 ?"CRITICAL    ":"disconnected",bat,(bat-6.0f)/2.4f*100.0f,PANDA ? "on ":"off");
    snprintf(linedata[2],sizeof(linedata[2]),"\x1b" "[3;1H" VT100ERASETOEND);
    snprintf(linedata[3],sizeof(linedata[3]),"\x1b" "[4;1H" VT100ERASETOEND);
    snprintf(linedata[4],sizeof(linedata[4]),"\x1b" "[5;1H" "Coilgun: %3.0fV, %s, %s" VT100ERASETOEND,capacitor,CHARGE ? "charging":"waiting ",BALL_DETECT? "Ball! ":"no ball");
    snprintf(linedata[5],sizeof(linedata[5]),"\x1b" "[6;1H" "Odometry: x:%10.7fm y:%10.7fm yaw:%10.7frad" VT100ERASETOEND,dx,dy,yaw);
    snprintf(linedata[6],sizeof(linedata[6]),"\x1b" "[7;1H" "             " VT100BOLD VT100REVERSE VT100BLINK "PUSH JOYSTICK FORWARD TO GO" VT100NORMAL VT100ERASETOEND );
    snprintf(linedata[7],sizeof(linedata[7]),"\x1b" "[8;1H" " Normal" VT100BOLD VT100REVERSE ">Competition<" VT100NORMAL "Demo Debug drivetrain Debug sensors" VT100ERASETOEND);
    break;
  case MODE_DEMO:
    snprintf(linedata[0],sizeof(linedata[0]),"\x1b" "[1;1H" "" VT100ERASETOEND);
    snprintf(linedata[1],sizeof(linedata[1]),"\x1b" "[2;1H" "" VT100ERASETOEND);
    snprintf(linedata[2],sizeof(linedata[2]),"\x1b" "[3;1H" "" VT100ERASETOEND);
    snprintf(linedata[3],sizeof(linedata[3]),"\x1b" "[4;1H" "" VT100ERASETOEND);
    snprintf(linedata[4],sizeof(linedata[4]),"\x1b" "[5;1H" "" VT100ERASETOEND);
    snprintf(linedata[5],sizeof(linedata[5]),"\x1b" "[6;1H" "" VT100ERASETOEND);
    snprintf(linedata[6],sizeof(linedata[6]),"\x1b" "[7;1H" "" VT100ERASETOEND);
    snprintf(linedata[7],sizeof(linedata[7]),"\x1b" "[8;1H" " Normal Competition"VT100BOLD VT100REVERSE ">Demo<" VT100NORMAL "Debug drivetrain Debug sensors" VT100ERASETOEND);
    break;
  case MODE_DEBUG_DRIVETRAIN:
    snprintf(linedata[0],sizeof(linedata[0]),"\x1b" "[1;1H" "" VT100ERASETOEND);
    snprintf(linedata[1],sizeof(linedata[1]),"\x1b" "[2;1H" "" VT100ERASETOEND);
    snprintf(linedata[2],sizeof(linedata[2]),"\x1b" "[3;1H" "" VT100ERASETOEND);
    snprintf(linedata[3],sizeof(linedata[3]),"\x1b" "[4;1H" "" VT100ERASETOEND);
    snprintf(linedata[4],sizeof(linedata[4]),"\x1b" "[5;1H" "dx:%f dy:%f dl:%fm dr:%fm angle:%f" VT100ERASETOEND,dx,dy,distanceleft,distanceright,wtfangle);
    snprintf(linedata[5],sizeof(linedata[5]),"\x1b" "[6;1H" "left  %.1frps, %.1frpm %.1f steps/s %.1f steps/min %.4fm/s" VT100ERASETOEND
             ,(float)MLXaccumulatorL/UNITSPERTURN*2.0,(float)MLXaccumulatorL/UNITSPERTURN*2.0*60.0,(float)MLXaccumulatorL/UNITSPERTURN/TURNSPERSTEP*2.0,(float)MLXaccumulatorL/UNITSPERTURN/TURNSPERSTEP*2.0*60.0,distanceleft*2.0f);
    snprintf(linedata[6],sizeof(linedata[6]),"\x1b" "[7;1H" "right %.1frps, %.1frpm %.1f steps/s %.1f steps/min %.4fm/s" VT100ERASETOEND
             ,(float)MLXaccumulatorR/UNITSPERTURN*2.0,(float)MLXaccumulatorR/UNITSPERTURN*2.0*60.0,(float)MLXaccumulatorR/UNITSPERTURN/TURNSPERSTEP*2.0,(float)MLXaccumulatorR/UNITSPERTURN/TURNSPERSTEP*2.0*60.0,distanceright*2.0f);
    snprintf(linedata[7],sizeof(linedata[7]),"\x1b" "[8;1H" " Normal Competition Demo"VT100BOLD VT100REVERSE ">Debug drivetrain<" VT100NORMAL "Debug sensors " VT100ERASETOEND); 
    break;
  case MODE_DEBUG_SENSORS:
    {
        const char *errright="";
        const char *errleft="";
        if(mlxrightstatus==1)
            errright=melexiscode(mlxrighterrorcode);
        if(mlxleftstatus ==1)
            errleft =melexiscode(mlxlefterrorcode);
        snprintf(linedata[0],sizeof(linedata[0]),"\x1b" "[1;1H" "Gyro raw: whoami:%3d temp:%3d x:%6d y:%6d z:%6d"VT100ERASETOEND,gyrowhoami,gyrotemp,gyrorawx,gyrorawy,gyrorawz);
        snprintf(linedata[1],sizeof(linedata[1]),"\x1b" "[2;1H" "Gyro filtered:       temp:%3d x:%6d y:%6d z:%6d"VT100ERASETOEND,36-gyrotemp,gyrox,gyroy,gyroz);
        snprintf(linedata[2],sizeof(linedata[2]),"\x1b" "[3;1H" "Gyro calibrationmin min:      x:%6d y:%6d z:%6d"VT100ERASETOEND,gyrominx,gyrominy,gyrominz);
        snprintf(linedata[3],sizeof(linedata[3]),"\x1b" "[4;1H" "Gyro calibrationmin max:      x:%6d y:%6d z:%6d"VT100ERASETOEND,gyromaxx,gyromaxy,gyromaxz);
        snprintf(linedata[4],sizeof(linedata[4]),"\x1b" "[5;1H" "Left drive:  mlx status:%3d (%s %s) mlx raw: %5d"VT100ERASETOEND,mlxleftstatus ,mlxleftstatus ==3 ? "no sensor ":(mlxleftstatus !=2 ? "Err:":"OK        "),errleft,MLXLdata );     
        snprintf(linedata[5],sizeof(linedata[5]),"\x1b" "[6;1H" "Right drive: mlx status:%3d (%s %s) mlx raw: %5d"VT100ERASETOEND,mlxrightstatus,mlxrightstatus==3 ? "no sensor ":(mlxrightstatus!=2 ? "Err:":"OK        "),errright,MLXRdata);     
        snprintf(linedata[7],sizeof(linedata[7]),"\x1b" "[8;1H" " Normal Competition Demo Debug drivetrain"VT100BOLD VT100REVERSE ">Debug sensors<" VT100NORMAL VT100ERASETOEND);
    }
    break;
  default:
    break;
  }

    MLXaccumulatorL=0;
    MLXaccumulatorR=0;

}