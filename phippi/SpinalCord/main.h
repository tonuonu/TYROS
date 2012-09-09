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

void main(void);

extern volatile unsigned short ticks;
extern volatile unsigned short ticks2;
void uart0_init(void);
void SPI3_send_cmd(unsigned char c);
void SPI3_send_data(unsigned char c);
void OLED_Init(void);
void OLED_Fade_In(void);
void OLED_Fade_Out(void);
void OLED_Checkerboard(void);
void OLED_Grayscale(void);
void OLED_Set_Start_Line(unsigned char d);
void OLED_Set_Display_Offset(unsigned char d);
void OLED_Set_Display_Mode(unsigned char d);
void OLED_Fill_RAM(unsigned char Data);
void OLED_Fade_Scroll(unsigned char a, unsigned char b, unsigned char c);
void OLED_Vertical_Scroll(unsigned char a, unsigned char b, unsigned char c);
void OLED_Show_Logo(void);
void Delay(unsigned char n);
void OLED_Test(void);
void OLED_Show_String(unsigned char a, char *Data_Pointer,
    unsigned char b, unsigned char c);
void OLED_Draw_Rectangle(unsigned char Data, unsigned char a,
    unsigned char b, unsigned char c, unsigned char d, unsigned char e);
void OLED_Set_Display_On_Off(unsigned char);
short unsigned SPI6_receive(void) ;
extern double twist[6];
extern int pwm[2];
extern int pwmtarget[2];
extern int accok, accwhoami; 
extern int accwhoamistatus;

extern unsigned char gyrowhoami;
extern signed int gyrox,gyroy,gyroz;
extern signed int gyrorawx,gyrorawy,gyrorawz;
extern int gyromaxx,gyromaxy,gyromaxz;
extern int gyrominx,gyrominy,gyrominz;

extern signed char gyrotemp;
extern int gyrowhoamistatus;
extern unsigned int MLXLdata,MLXRdata;
extern int mlxrighterrorcode;
extern int mlxlefterrorcode;
extern int mlx1whoamistatus;
extern int mlx2whoamistatus;
extern int buzzer;
extern float bat, capacitor, leftmotorcurrent , rightmotorcurrent ;

extern char mlxrightstatus,mlxleftstatus;

extern unsigned char tmpMLXRdata1;
extern unsigned char tmpMLXRdata2;
extern unsigned char tmpMLXRdata3;
extern unsigned char tmpMLXRdata4;

extern signed int  MLXaccumulatorL;
extern signed int  MLXaccumulatorR;
extern float distanceleft;
extern float distanceright;
extern float dx;
extern float dy;
extern float yaw;

extern char linedata[8][100];
extern int lineno;
extern int linepos;
extern int mode;
void redraw_infoscreen_buffers(void);
enum {
  MODE_MANUAL,
  MODE_COMPETITION,
  MODE_DEMO,
  MODE_DEBUG_DRIVETRAIN,
  MODE_DEBUG_SENSORS,
};
/*
 * IAR-HEW compatibility 
 */
#define _asm asm

/*
 * 1 cycle delay 
 */
#define	NOP()			{_asm("nop");}

static inline void
uDelay(unsigned char l)
{
    while (l--)
        NOP();
}

#define ENABLE_IRQ   	{_asm("FSET I");}
#define DISABLE_IRQ	{_asm("FCLR I");}

void
Delay(unsigned char n);
