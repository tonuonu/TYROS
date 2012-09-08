
#include "ior32c111.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "hwsetup.h"
#include "main.h"
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
    char *tmpmlx1error="";
    char *tmpmlx2error="";
  
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
                (unsigned char)tmpmlx2data1,(unsigned char)tmpmlx2data2,
                (unsigned char)~tmpmlx2data3,(unsigned char)~tmpmlx2data4,revolutions);
        */
//        sprintf(buf,"%6.1f m",distanceleft);
  
/* switch(mlx2status) {
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
    if(mlx1status==3) {
      tmpmlx1error="Left melexis sensor does not respond ";
    } else if(mlx1status==1) {
      tmpmlx1error="Left melexis sensor returns error ";
    }
    if(mlx2status==3) {
      tmpmlx2error="Right melexis sensor does not respond ";
    } else if(mlx2status==1) {
      tmpmlx2error="Right melexis sensor returns error ";
    }
      
    snprintf(linedata[0],sizeof(linedata[0]),"\x1b" "[1;1H" "Status: %s%s%s" VT100ERASETOEND,gyrowhoami!=211? "Gyroscope error":mlx1status==2 && mlx2status==2?"OK             ":"",tmpmlx1error,tmpmlx2error);
    snprintf(linedata[1],sizeof(linedata[1]),"\x1b" "[2;1H" "Battery: %s %4.2fV %3.0f%% Panda %s" VT100ERASETOEND,bat>6.9 ?"normal      ":bat>6.3 ?"LOW         ":bat>5.0 ?"CRITICAL    ":"disconnected",bat,(bat-6.0f)/2.4f*100.0f,PANDA ? "on ":"off");
    snprintf(linedata[2],sizeof(linedata[2]),"\x1b" "[3;1H" "Left drive: %s %4.2fm/s %5drpm pwm:%3u%% %4.1fA" VT100ERASETOEND,pwm[0]<0 ?"backward":pwm[0]>0 ?"forward ":"brake   ",distanceleft *10.0f,345,(int)abs(pwm[0]),leftmotorcurrent);
    snprintf(linedata[3],sizeof(linedata[3]),"\x1b" "[4;1H" "Right drive:%s %4.2fm/s %5drpm pwm:%3u%% %4.1fA" VT100ERASETOEND,pwm[1]<0 ?"backward":pwm[0]>0 ?"forward ":"brake   ",distanceright*10.0f,345,(int)abs(pwm[1]),rightmotorcurrent);
    snprintf(linedata[4],sizeof(linedata[4]),"\x1b" "[5;1H" "Coilgun: %3.0fV, %s, %s" VT100ERASETOEND,capacitor,CHARGE ? "charging":"waiting ",BALL_DETECT? "Ball! ":"no ball");
    snprintf(linedata[5],sizeof(linedata[5]),"\x1b" "[6;1H" "Odometry: x:%10.7fm y:%10.7fm yaw:%10.7frad" VT100ERASETOEND,dx,dy,yaw);
    snprintf(linedata[6],sizeof(linedata[6]),"\x1b" "[7;1H" "" VT100ERASETOEND);
    snprintf(linedata[7],sizeof(linedata[7]),"\x1b" "[8;1H" VT100BOLD VT100REVERSE ">Normal<" VT100NORMAL "Competition Demo Debug drivetrain Debug sensors " VT100NORMAL VT100ERASETOEND);
    
    break;
  case MODE_COMPETITION:
    if(capacitor<350.0f) {
        CHARGE=1;
    }
    
    snprintf(linedata[0],sizeof(linedata[0]),"\x1b" "[1;1H" VT100ERASETOEND);
    snprintf(linedata[1],sizeof(linedata[1]),"\x1b" "[2;1H" VT100ERASETOEND);
    snprintf(linedata[2],sizeof(linedata[2]),"\x1b" "[3;1H" VT100ERASETOEND);
    snprintf(linedata[3],sizeof(linedata[3]),"\x1b" "[4;1H" VT100BOLD VT100REVERSE VT100BLINK "PRESS JOYSTICK TO GO" VT100NORMAL VT100ERASETOEND );
    snprintf(linedata[4],sizeof(linedata[4]),"\x1b" "[5;1H" "Coilgun: %3.0fV, %s, %s" VT100ERASETOEND,capacitor,CHARGE ? "charging":"waiting ",BALL_DETECT? "Ball! ":"no ball");
    snprintf(linedata[5],sizeof(linedata[5]),"\x1b" "[6;1H" "Odometry: x:%10.7fm y:%10.7fm yaw:%10.7frad" VT100ERASETOEND,dx,dy,yaw);
    snprintf(linedata[6],sizeof(linedata[6]),"\x1b" "[7;1H" VT100ERASETOEND);
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
  case MODE_DEBUG_SENSORS:
    {
    const char *err1="";
    const char *err2="";
    if(mlx1status==1)
      err1=melexiscode(mlx1errorcode);
    if(mlx2status==1)
      err2=melexiscode(mlx2errorcode);
    snprintf(linedata[0],sizeof(linedata[0]),"\x1b" "[1;1H" "Gyro raw: whoami:%3d temp:%3d x:%6d y:%6d z:%6d%s",gyrowhoami,gyrotemp,gyrorawx,gyrorawy,gyrorawz,"\x1b" "[0K");
    snprintf(linedata[1],sizeof(linedata[1]),"\x1b" "[2;1H" "Gyro filtered:       temp:%3d x:%6d y:%6d z:%6d%s",36-gyrotemp,gyrox,gyroy,gyroz,"\x1b" "[0K");
    snprintf(linedata[2],sizeof(linedata[2]),"\x1b" "[3;1H" "Gyro calibrationmin min:      x:%6d y:%6d z:%6d%s",gyrominx,gyrominy,gyrominz,"\x1b" "[0K");
    snprintf(linedata[3],sizeof(linedata[3]),"\x1b" "[4;1H" "Gyro calibrationmin max:      x:%6d y:%6d z:%6d%s",gyromaxx,gyromaxy,gyromaxz,"\x1b" "[0K");
    snprintf(linedata[4],sizeof(linedata[4]),"\x1b" "[5;1H" "Left drive:  mlx status:%3d (%s %s) mlx raw: %5d%s",mlx1status,mlx1status==3 ? "no sensor ":(mlx1status!=2 ? "Err:":"OK        "),err1,mlx1data,"\x1b" "[0K");     
    snprintf(linedata[5],sizeof(linedata[5]),"\x1b" "[6;1H" "Right drive: mlx status:%3d (%s %s) mlx raw: %5d%s",mlx2status,mlx2status==3 ? "no sensor ":(mlx2status!=2 ? "Err:":"OK        "),err2,mlx2data,"\x1b" "[0K");     
    snprintf(linedata[6],sizeof(linedata[6]),"\x1b" "[7;1H" "%s","\x1b" "[0K");
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
  case MODE_DEBUG_DRIVETRAIN:
    snprintf(linedata[0],sizeof(linedata[0]),"\x1b" "[1;1H" "%s","\x1b" "[0K");
    snprintf(linedata[1],sizeof(linedata[1]),"\x1b" "[2;1H" "%s","\x1b" "[0K");
    snprintf(linedata[2],sizeof(linedata[2]),"\x1b" "[3;1H" "%s","\x1b" "[0K");
    snprintf(linedata[3],sizeof(linedata[3]),"\x1b" "[4;1H" "%s","\x1b" "[0K");
    snprintf(linedata[4],sizeof(linedata[4]),"\x1b" "[5;1H" "%s","\x1b" "[0K");
    snprintf(linedata[5],sizeof(linedata[5]),"\x1b" "[6;1H" "%s","\x1b" "[0K");
    snprintf(linedata[6],sizeof(linedata[6]),"\x1b" "[7;1H" "%s","\x1b" "[0K");
    snprintf(linedata[7],sizeof(linedata[7]),"\x1b" "[8;1H" " Normal Competition Demo"VT100BOLD VT100REVERSE ">Debug drivetrain<" VT100NORMAL "Debug sensors " VT100ERASETOEND); 
    break;
  default:
 //   mode=0;
    break;
  }


}