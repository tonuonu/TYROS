
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
char linedata[8][100]; // 64 is not enough as we insert VT100 escape sequences
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
    float angletraveled=0.0f;
    float smallradius=0.0f;

    char *tmpmlxrighterror="";
    char *tmpmlxlefterror="";
  
  switch(mode) {
  case MODE_MANUAL:
    /*
    0         1         2         3         4         5         6
0123456789012345678901234567890123456789012345678901234567890123
Status: OK                                                      1 
Battery: CRITICAL x.xxV xxx% Panda: off                         2 Good, LOW
Left drive:  forward  x.xxm/s -xxxxrpm pwm:xxx% xx.xA           3
Right drive: backward x.xxm/s -xxxxrpm pwm:xxx% xx.xA           4
Coilgun: xxxV Charging, no ball                                 5 Charging, Charged, Low
Odometry: x:-0.xxxxxxxm y:-0.xxxxxxxm yaw:-0.xxxxxxxrad/s       6   
Gyroscope:                                                      7
 Normal Competition Demo Debug drivetrain Debug sensors         8
0123456789012345678901234567890123456789012345678901234567890123
0         1         2         3         4         5         6 */
//    #define VT100CURSORODOMETRY   "\x1b" "[02;1H"
    
    
    
    /*        
        sprintf(buf,"(%2d) (%3u %3u %3u %3u) %f ",
                (unsigned char)mlx2whoamistatus, 
                (unsigned char)tmpMLXRdata1,(unsigned char)tmpMLXRdata2,
                (unsigned char)~tmpMLXRdata3,(unsigned char)~tmpMLXRdata4,revolutions);
        */
//        sprintf(buf,"%6.1f m",distanceleft);
  
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
*/
    if(mlxrightstatus==3) {
      tmpmlxrighterror=VT100REVERSE "Right melexis sensor does not respond " VT100NORMAL;
    } else if(mlxrightstatus==1) {
      tmpmlxrighterror=VT100REVERSE "Right melexis sensor returns error " VT100NORMAL;
    }
    if(mlxleftstatus==3) {
      tmpmlxlefterror=VT100REVERSE "Left melexis sensor does not respond " VT100NORMAL;
    } else if(mlxleftstatus==1) {
      tmpmlxlefterror=VT100REVERSE "Left melexis sensor returns error " VT100NORMAL;
    }
      
    snprintf(linedata[0],sizeof(linedata[0]),"\x1b" "[1;1H" "Status: %s%s%s" VT100ERASETOEND,gyrowhoami!=211? "Gyroscope error":mlxrightstatus==2 && mlxleftstatus==2?"OK             ":"",tmpmlxrighterror,tmpmlxlefterror);
    snprintf(linedata[1],sizeof(linedata[1]),"\x1b" "[2;1H" "Battery: %s %4.1fV %3.0f%% Panda %s" VT100ERASETOEND,bat>6.9 ?"normal      ":bat>6.3 ?"LOW         ":bat>5.0 ?"CRITICAL    ":"disconnected",bat,(bat-6.0f)/2.4f*100.0f,PANDA ? "on ":"off");
    snprintf(linedata[2],sizeof(linedata[2]),"\x1b" "[3;1H" "Left drive: %s %4.2fm/s %5drpm pwm:%3u%% %4.1fA" VT100ERASETOEND,pwm[0]<0 ?"backward":pwm[0]>0 ?"forward ":"brake   ",distanceleft *10.0f,345,(int)abs(pwm[0]),leftmotorcurrent);
    snprintf(linedata[3],sizeof(linedata[3]),"\x1b" "[4;1H" "Right drive:%s %4.2fm/s %5drpm pwm:%3u%% %4.1fA" VT100ERASETOEND,pwm[1]<0 ?"backward":pwm[0]>0 ?"forward ":"brake   ",distanceright*10.0f,345,(int)abs(pwm[1]),rightmotorcurrent);
    snprintf(linedata[4],sizeof(linedata[4]),"\x1b" "[5;1H" "Coilgun: %3.0fV, %s, %s" VT100ERASETOEND,capacitor,CHARGE ? "charging":"waiting ",BALL_DETECT? "Ball! ":"no ball");
    snprintf(linedata[5],sizeof(linedata[5]),"\x1b" "[6;1H" "Odometry: x:%10.7fm y:%10.7fm yaw:%10.7frad" VT100ERASETOEND,dx,dy,yaw);
    snprintf(linedata[6],sizeof(linedata[6]),"\x1b" "[7;1H" "" VT100ERASETOEND);
    snprintf(linedata[7],sizeof(linedata[7]),"\x1b" "[8;1H" VT100BOLD VT100REVERSE ">Normal<" VT100NORMAL "Competition Demo Debug drivetrain Debug sensors " VT100NORMAL VT100ERASETOEND);
    
    break;
  case MODE_COMPETITION:
    if(capacitor<390.0f) {
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
    
    
    /* 
    sammu pikkuseks saan umbes 54.5 mm
    Kalle-Gustav Kruus: 4,306 tais mootori pooret sammu kohta
    Kalle-Gustav Kruus: nagu 0.1mm põõrde kohta umbes
    Tonu Samuel: 0.07900917431  mm siis poorde kohta
    /1000.0 on, et meetriteks teisendamine
    */
#define robotwidth 0.175     
#define sign(x) ((x>0.0f) - (x<0.0f))
    distanceleft =(float)MLXaccumulatorL/16384.0/4.306*0.00545;
    distanceright=(float)MLXaccumulatorR/16384.0/4.306*0.00545;
    if(distanceleft == distanceright) {
        dx=0.0f;
        dy=distanceleft;
    } else {
        if(distanceleft==0 ) {
          smallradius=robotwidth;
          angletraveled=distanceright/smallradius;
        } else if(distanceright==0 ) {
          smallradius=robotwidth;
          angletraveled=distanceleft/smallradius;
        }else {
          smallradius=robotwidth/((distanceleft<distanceright ? distanceright/distanceleft:distanceleft/distanceright)-1);
          angletraveled=(distanceleft<distanceright ? distanceleft:distanceright)/smallradius;
        }
        float centerradius=smallradius+(robotwidth/2.0f);
        dy=sin(angletraveled)*centerradius;
        dx=cos(angletraveled)*centerradius * (distanceleft < distanceright ? 1.0f : -1.0f)
          - (sign(distanceleft)+sign(distanceright)!=1  ? centerradius : 0);
    }
    
    snprintf(linedata[0],sizeof(linedata[0]),"\x1b" "[1;1H" "" VT100ERASETOEND);
    snprintf(linedata[1],sizeof(linedata[1]),"\x1b" "[2;1H" "" VT100ERASETOEND);
    snprintf(linedata[2],sizeof(linedata[2]),"\x1b" "[3;1H" "" VT100ERASETOEND);
    snprintf(linedata[3],sizeof(linedata[3]),"\x1b" "[4;1H" "" VT100ERASETOEND);
    snprintf(linedata[4],sizeof(linedata[4]),"\x1b" "[5;1H" "dx:%f dy:%f dl:%f dr:%f angle:%f" VT100ERASETOEND,dx,dy,distanceleft,distanceright,angletraveled);
    snprintf(linedata[5],sizeof(linedata[5]),"\x1b" "[6;1H" "left  %.1frps, %.1frpm %.1f steps/s %.1f steps/min %.4fm/s" VT100ERASETOEND
             ,(float)MLXaccumulatorL/16384.0*2.0,(float)MLXaccumulatorL/16384.0*2.0*60.0,(float)MLXaccumulatorL/16384.0/4.306*2.0,(float)MLXaccumulatorL/16384/4.306*2.0*60.0,distanceleft*2.0f);
    snprintf(linedata[6],sizeof(linedata[6]),"\x1b" "[7;1H" "right %.1frps, %.1frpm %.1f steps/s %.1f steps/min %.4fm/s" VT100ERASETOEND
             ,(float)MLXaccumulatorR/16384.0*2.0,(float)MLXaccumulatorR/16384.0*2.0*60.0,(float)MLXaccumulatorR/16384.0/4.306*2.0,(float)MLXaccumulatorR/16384/4.306*2.0*60.0,distanceright*2.0f);
    MLXaccumulatorL=0;
    MLXaccumulatorR=0;
     
    snprintf(linedata[7],sizeof(linedata[7]),"\x1b" "[8;1H" " Normal Competition Demo"VT100BOLD VT100REVERSE ">Debug drivetrain<" VT100NORMAL "Debug sensors " VT100ERASETOEND); 
    break;

  case MODE_DEBUG_SENSORS:
    {
    const char *errright="";
    const char *errleft="";
    if(mlxrightstatus==1)
      errright=melexiscode(mlxrighterrorcode);
    if(mlxleftstatus==1)
      errleft=melexiscode(mlxlefterrorcode);
    snprintf(linedata[0],sizeof(linedata[0]),"\x1b" "[1;1H" "Gyro raw: whoami:%3d temp:%3d x:%6d y:%6d z:%6d"VT100ERASETOEND,gyrowhoami,gyrotemp,gyrorawx,gyrorawy,gyrorawz);
    snprintf(linedata[1],sizeof(linedata[1]),"\x1b" "[2;1H" "Gyro filtered:       temp:%3d x:%6d y:%6d z:%6d"VT100ERASETOEND,36-gyrotemp,gyrox,gyroy,gyroz);
    snprintf(linedata[2],sizeof(linedata[2]),"\x1b" "[3;1H" "Gyro calibrationmin min:      x:%6d y:%6d z:%6d"VT100ERASETOEND,gyrominx,gyrominy,gyrominz);
    snprintf(linedata[3],sizeof(linedata[3]),"\x1b" "[4;1H" "Gyro calibrationmin max:      x:%6d y:%6d z:%6d"VT100ERASETOEND,gyromaxx,gyromaxy,gyromaxz);
    snprintf(linedata[4],sizeof(linedata[4]),"\x1b" "[5;1H" "Left drive:  mlx status:%3d (%s %s) mlx raw: %5d"VT100ERASETOEND,mlxleftstatus ,mlxleftstatus ==3 ? "no sensor ":(mlxleftstatus !=2 ? "Err:":"OK        "),errleft,MLXLdata );     
    snprintf(linedata[5],sizeof(linedata[5]),"\x1b" "[6;1H" "Right drive: mlx status:%3d (%s %s) mlx raw: %5d"VT100ERASETOEND,mlxrightstatus,mlxrightstatus==3 ? "no sensor ":(mlxrightstatus!=2 ? "Err:":"OK        "),errright,MLXRdata);     

/*Debug sensors:
  0         1         2         3         4         5         6
  0123456789012345678901234567890123456789012345678901234567890123
1 Gyro raw: whoami:xx temp:xx x: xxxxxxxx y: xxxxxxxx z: xxxxxxxx 1
2 Gyro filtered:      temp:xx x: xxxxxxxx y: xxxxxxxx z: xxxxxxxx 2
3 Left drive:  mlx raw: xxxxx pwm:xxx% current raw: xxx xx.xA     3      
4 Right drive: mlx raw: xxxxx pwm:xxx% current raw: xxx xx.xA     4      
5 Coilgun: raw:xxx xxxV Charging, no ball                         5       
6 Odometry: x:-0.xxxxxm y:-0.xxxxxm yaw:-0.xxxxxrad/s             6
8                                                      8
  0123456789012345678901234567890123456789012345678901234567890123
  0         1         2         3         4         5         6 
*/
    snprintf(linedata[7],sizeof(linedata[7]),"\x1b" "[8;1H" " Normal Competition Demo Debug drivetrain"VT100BOLD VT100REVERSE ">Debug sensors<" VT100NORMAL VT100ERASETOEND);
    }
    break;
  default:
 //   mode=0;
    break;
  }


}