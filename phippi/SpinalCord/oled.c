#include "types.h"
#include "ior32c111.h"
#include "main.h"
#include "tnroman.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Pin Definition
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define LCD_RESET p4_2

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Global Variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define	OLED_Shift	0x1C
#define OLED_Max_Column	0x3F				   // 256/4-1
#define OLED_Max_Row	0x3F				   // 64-1
#define	OLED_Brightness	0x0F

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Delay Time
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
Delay(unsigned char n)
{
    unsigned char i, j, k;
    for (k = 0; k < n; k++)
	for (i = 0; i < 131; i++)
	    for (j = 0; j < 15; j++)
		uDelay(203);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Instruction Setting
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
inline void
OLED_Set_Column_Address(unsigned char a, unsigned char b) {
    SPI_send_cmd(0x15);				   // Set Column Address
    SPI_send_data(a);					   //   Default => 0x00
    SPI_send_data(b);					   //   Default => 0x77
}

inline void
OLED_Set_Row_Address(unsigned char a, unsigned char b) {
    SPI_send_cmd(0x75);				   // Set Row Address
    SPI_send_data(a);					   //   Default => 0x00
    SPI_send_data(b);					   //   Default => 0x7F
}

inline void
OLED_Set_Write_RAM() {
    SPI_send_cmd(0x5C);				   // Enable MCU to Write into RAM
}

inline void
OLED_Set_Read_RAM() {
    SPI_send_cmd(0x5D);				   // Enable MCU to Read from RAM
}

void
OLED_Set_Remap_Format(unsigned char d) {
    SPI_send_cmd(0xA0);				   // Set Re-Map / Dual COM Line Mode
    SPI_send_data(d);					   //   Default => 0x40
    //     Horizontal Address Increment
    //     Column Address 0 Mapped to SEG0
    //     Disable Nibble Remap
    //     Scan from COM0 to COM[N-1]
    //     Disable COM Split Odd Even
    SPI_send_data(0x11);				   //   Default => 0x01 (Disable Dual COM Mode)
}
void

OLED_Set_Start_Line(unsigned char d) {
    SPI_send_cmd(0xA1);				   // Set Vertical Scroll by RAM
    SPI_send_data(d);					   //   Default => 0x00
}

void
OLED_Set_Display_Offset(unsigned char d) {
    SPI_send_cmd(0xA2);				   // Set Vertical Scroll by Row
    SPI_send_data(d);					   //   Default => 0x00
}

void
OLED_Set_Display_Mode(unsigned char d) {
    SPI_send_cmd(0xA4 | d);				   // Set Display Mode
    //   Default => 0xA4
    //     0xA4 (0x00) => Entire Display Off, All Pixels Turn Off
    //     0xA5 (0x01) => Entire Display On, All Pixels Turn On at GS Level 15
    //     0xA6 (0x02) => Normal Display
    //     0xA7 (0x03) => Inverse Display
}

void
OLED_Set_Partial_Display(unsigned char a, unsigned char b, unsigned char c) {
    SPI_send_cmd(0xA8 | a);				   // Default => 0x8F
    //   Select Internal Booster at Display On
    if (a == 0x00) {
	SPI_send_data(b);
	SPI_send_data(c);
    }
}

void
OLED_Set_Function_Selection(unsigned char d) {
    SPI_send_cmd(0xAB);				   // Function Selection
    SPI_send_data(d);					   //   Default => 0x01
    //     Enable Internal VDD Regulator
}

void
OLED_Set_Display_On_Off(unsigned char d) {
    SPI_send_cmd(0xAE | d);				   // Set Display On/Off
    //   Default => 0xAE
    //     0xAE (0x00) => Display Off (Sleep Mode On)
    //     0xAF (0x01) => Display On (Sleep Mode Off)
}

void
OLED_Set_Phase_Length(unsigned char d) {
    SPI_send_cmd(0xB1);				   // Phase 1 (Reset) & Phase 2 (Pre-Charge) Period Adjustment
    SPI_send_data(d);					   //   Default => 0x74 (7 Display Clocks [Phase 2] / 9 Display Clocks [Phase 1])
    //     D[3:0] => Phase 1 Period in 5~31 Display Clocks
    //     D[7:4] => Phase 2 Period in 3~15 Display Clocks
}

void
OLED_Set_Display_Clock(unsigned char d) {
    SPI_send_cmd(0xB3);				   // Set Display Clock Divider / Oscillator Frequency
    SPI_send_data(d);					   //   Default => 0xD0
    //     A[3:0] => Display Clock Divider
    //     A[7:4] => Oscillator Frequency
}

void
OLED_Set_Display_Enhancement_A(unsigned char a, unsigned char b) {
    SPI_send_cmd(0xB4);				   // Display Enhancement
    SPI_send_data(0xA0 | a);				   //   Default => 0xA2
    //     0xA0 (0x00) => Enable External VSL
    //     0xA2 (0x02) => Enable Internal VSL (Kept VSL Pin N.C.)
    SPI_send_data(0x05 | b);				   //   Default => 0xB5
    //     0xB5 (0xB0) => Normal
    //     0xFD (0xF8) => Enhance Low Gray Scale Display Quality
}

void
OLED_Set_GPIO(unsigned char d) {
    SPI_send_cmd(0xB5);				   // General Purpose IO
    SPI_send_data(d);					   //   Default => 0x0A (GPIO Pins output Low Level.)
}

void
OLED_Set_Precharge_Period(unsigned char d) {
    SPI_send_cmd(0xB6);				   // Set Second Pre-Charge Period
    SPI_send_data(d);					   //   Default => 0x08 (8 Display Clocks)
}

void
OLED_Set_Precharge_Voltage(unsigned char d) {
    SPI_send_cmd(0xBB);				   // Set Pre-Charge Voltage Level
    SPI_send_data(d);					   //   Default => 0x17 (0.50*VCC)
}

void
OLED_Set_VCOMH(unsigned char d) {
    SPI_send_cmd(0xBE);				   // Set COM Deselect Voltage Level
    SPI_send_data(d);					   //   Default => 0x04 (0.80*VCC)
}

void
OLED_Set_Contrast_Current(unsigned char d) {
    SPI_send_cmd(0xC1);				   // Set Contrast Current
    SPI_send_data(d);					   //   Default => 0x7F
}

void
OLED_Set_Master_Current(unsigned char d) {
    SPI_send_cmd(0xC7);				   // Master Contrast Current Control
    SPI_send_data(d);					   //   Default => 0x0f (Maximum)
}

void
OLED_Set_Multiplex_Ratio(unsigned char d) {
    SPI_send_cmd(0xCA);				   // Set Multiplex Ratio
    SPI_send_data(d);					   //   Default => 0x7F (1/128 Duty)
}

void
OLED_Set_Display_Enhancement_B(unsigned char d) {
    SPI_send_cmd(0xD1);				   // Display Enhancement
    SPI_send_data(0x82 | d);				   //   Default => 0xA2
    //     0x82 (0x00) => Reserved
    //     0xA2 (0x20) => Normal
    SPI_send_data(0x20);
}

void
OLED_Set_Command_Lock(unsigned char d) {
    SPI_send_cmd(0xFD);				   // Set Command Lock
    SPI_send_data(0x12 | d);				   //   Default => 0x12
    //     0x12 => Driver IC interface is unlocked from entering command.
    //     0x16 => All Commands are locked except 0xFD.
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Graphic Acceleration (Partial or Full Screen)
//
//    a: Line Width
//    b: Column Address of Start
//    c: Column Address of End
//    d: Row Address of Start
//    e: Row Address of End
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Draw_Rectangle(unsigned char Data, unsigned char a, unsigned char b,
    unsigned char c, unsigned char d, unsigned char e) {
    unsigned char i, j, k, l;
    k = a % 4;
    if (k == 0) {
	l = (a / 4) - 1;
    } else {
	l = a / 4;
    }
    OLED_Set_Column_Address(OLED_Shift + b, OLED_Shift + c);
    OLED_Set_Row_Address(d, (d + a - 1));
    OLED_Set_Write_RAM();
    for (i = 0; i < (c - b + 1); i++)
	for (j = 0; j < a; j++) {
	    SPI_send_data(Data);
	    SPI_send_data(Data);
	}
    OLED_Set_Column_Address(OLED_Shift + (c - l), OLED_Shift + c);
    OLED_Set_Row_Address(d + a, e - a);
    OLED_Set_Write_RAM();
    for (i = 0; i < (e - d + 1); i++) {
	for (j = 0; j < (l + 1); j++) {
	    if (j == 0) {
		switch (k) {
		case 0:
		    SPI_send_data(Data);
		    SPI_send_data(Data);
		    break;
		case 1:
		    SPI_send_data(0x00);
		    SPI_send_data(Data & 0x0F);
		    break;
		case 2:
		    SPI_send_data(0x00);
		    SPI_send_data(Data);
		    break;
		case 3:
		    SPI_send_data(Data & 0x0F);
		    SPI_send_data(Data);
		    break;
		}
	    } else {
		SPI_send_data(Data);
		SPI_send_data(Data);
	    }
	}
    }
    OLED_Set_Column_Address(OLED_Shift + b, OLED_Shift + c);
    OLED_Set_Row_Address((e - a + 1), e);
    OLED_Set_Write_RAM();
    for (i = 0; i < (c - b + 1); i++)
	for (j = 0; j < a; j++) {
	    SPI_send_data(Data);
	    SPI_send_data(Data);
	}
    OLED_Set_Column_Address(OLED_Shift + b, OLED_Shift + (b + l));
    OLED_Set_Row_Address(d + a, e - a);
    OLED_Set_Write_RAM();
    for (i = 0; i < (e - d + 1); i++) {
	for (j = 0; j < (l + 1); j++) {
	    if (j == l) {
		switch (k) {
		case 0:
		    SPI_send_data(Data);
		    SPI_send_data(Data);
		    break;
		case 1:
		    SPI_send_data(Data & 0xF0);
		    SPI_send_data(0x00);
		    break;
		case 2:
		    SPI_send_data(Data);
		    SPI_send_data(0x00);
		    break;
		case 3:
		    SPI_send_data(Data);
		    SPI_send_data(Data & 0xF0);
		    break;
		}
	    } else {
		SPI_send_data(Data);
		SPI_send_data(Data);
	    }
	}
    }
}

#include "phippi.h"

void
OLED_Show_Logo(void) {        
    unsigned char a=0;
    unsigned char b=64-1;
    unsigned char i, j;
    unsigned int k=0;
    OLED_Set_Column_Address(OLED_Shift + a, OLED_Shift + b );
    OLED_Set_Row_Address(0,OLED_Max_Row );
    OLED_Set_Write_RAM();
    for (i = 0; i <OLED_Max_Row+1 ; i++) { /* row */
	for (j = 0; j < 11; j++) {
	    SPI_send_data(0x00);
	}
        for (j = 0; j < 106; j++,k++) { /* col */
            if(phippi[k]==' ')
              SPI_send_data(0x00);
            else if(phippi[k]=='.')
              SPI_send_data(0xFF);
            else 
              SPI_send_data(phippi[k]);
	}
        k--;
	for (j = 0; j < 11; j++) {
	    SPI_send_data(0x00);
	}
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Regular Pattern (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Fill_RAM(unsigned char Data) {
    unsigned char i, j;
    OLED_Set_Column_Address(0x00, 0x77);
    OLED_Set_Row_Address(0x00, 0x7F);
    OLED_Set_Write_RAM();
    for (i = 0; i < 128; i++)
	for (j = 0; j < 120; j++) {
	    SPI_send_data(Data);
	    SPI_send_data(Data);
	}
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Regular Pattern (Partial or Full Screen)
//
//    a: Column Address of Start
//    b: Column Address of End (Total Columns Devided by 4)
//    c: Row Address of Start
//    d: Row Address of End
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Fill_Block(unsigned char Data, unsigned char a, unsigned char b,
    unsigned char c, unsigned char d) {
    unsigned char i, j;
    OLED_Set_Column_Address(OLED_Shift + a, OLED_Shift + b);
    OLED_Set_Row_Address(c, d);
    OLED_Set_Write_RAM();
    for (i = 0; i < (d - c + 1); i++) {
	for (j = 0; j < (b - a + 1); j++) {
	    SPI_send_data(Data);
	    SPI_send_data(Data);
	}
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Checkboard (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Checkerboard() {
    unsigned char i, j;
    OLED_Set_Column_Address(0x00, 0x77);
    OLED_Set_Row_Address(0x00, 0x7F);
    OLED_Set_Write_RAM();
    for (i = 0; i < 64; i++) {
	for (j = 0; j < 120; j++) {
	    SPI_send_data(0xF0);
	    SPI_send_data(0xF0);
	}
	for (j = 0; j < 120; j++) {
	    SPI_send_data(0x0F);
	    SPI_send_data(0x0F);
	}
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Gray Scale Bar (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Grayscale() {

	// Level 16 => Column 1~16
	OLED_Fill_Block(0xFF, 0x00, 0x03, 0x00, OLED_Max_Row);

	// Level 15 => Column 17~32
	OLED_Fill_Block(0xEE, 0x04, 0x07, 0x00, OLED_Max_Row);

	// Level 14 => Column 33~48
	OLED_Fill_Block(0xDD, 0x08, 0x0B, 0x00, OLED_Max_Row);

	// Level 13 => Column 49~64
	OLED_Fill_Block(0xCC, 0x0C, 0x0F, 0x00, OLED_Max_Row);

	// Level 12 => Column 65~80
	OLED_Fill_Block(0xBB, 0x10, 0x13, 0x00, OLED_Max_Row);

	// Level 11 => Column 81~96
	OLED_Fill_Block(0xAA, 0x14, 0x17, 0x00, OLED_Max_Row);

	// Level 10 => Column 97~112
	OLED_Fill_Block(0x99, 0x18, 0x1B, 0x00, OLED_Max_Row);

	// Level 9 => Column 113~128
	OLED_Fill_Block(0x88, 0x1C, 0x1F, 0x00, OLED_Max_Row);

	// Level 8 => Column 129~144
	OLED_Fill_Block(0x77, 0x20, 0x23, 0x00, OLED_Max_Row);

	// Level 7 => Column 145~160
	OLED_Fill_Block(0x66, 0x24, 0x27, 0x00, OLED_Max_Row);

	// Level 6 => Column 161~176
	OLED_Fill_Block(0x55, 0x28, 0x2B, 0x00, OLED_Max_Row);

	// Level 5 => Column 177~192
	OLED_Fill_Block(0x44, 0x2C, 0x2F, 0x00, OLED_Max_Row);

	// Level 4 => Column 193~208
	OLED_Fill_Block(0x33, 0x30, 0x33, 0x00, OLED_Max_Row);

	// Level 3 => Column 209~224
	OLED_Fill_Block(0x22, 0x34, 0x37, 0x00, OLED_Max_Row);

	// Level 2 => Column 225~240
	OLED_Fill_Block(0x11, 0x38, 0x3B, 0x00, OLED_Max_Row);

	// Level 1 => Column 241~256
	OLED_Fill_Block(0x00, 0x3C, OLED_Max_Column, 0x00, OLED_Max_Row);
}

#define OLED_BRIGHTNESS    15
#define OLED_BR_1B      OLED_BRIGHTNESS<<4
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Character (5x7)
//
//    a: Database
//    b: Ascii
//    c: Start X Address
//    d: Start Y Address
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Show_Font57(unsigned char a, unsigned char b, unsigned char c,
    unsigned char d) {
    const unsigned char *Src_Pointer;
    unsigned char i,p,n, Font, MSB_1, LSB_1, MSB_2, LSB_2;
    switch (a) {
    case 1:
	Src_Pointer = &Ascii_1[(b - 1)][0];
	break;
#if 0
    case 2:
	Src_Pointer = &Ascii_2[(b - 1)][0];
	break;
#endif
    }
    OLED_Set_Remap_Format(0x15);
    for (i = 0; i <= 1; i++) {      // 
	MSB_1 = *Src_Pointer;       // MSB_1 = first font databyte
	Src_Pointer++;
	if (i == 1) {
	    LSB_1 = 0x00;
	    MSB_2 = 0x00;
	    LSB_2 = 0x00;
	} else {
	    LSB_1 = *Src_Pointer;
	    Src_Pointer++;
	    MSB_2 = *Src_Pointer;
	    Src_Pointer++;
	    LSB_2 = *Src_Pointer;
	    Src_Pointer++;
	}
	OLED_Set_Column_Address(OLED_Shift + c, OLED_Shift + c);
	OLED_Set_Row_Address(d, d + 7);
	OLED_Set_Write_RAM();
        n=1;
        for (p = 0; p < 8; p++){
            if (MSB_1 & n) 
                Font = OLED_BR_1B; 
            else 
                Font = 0;
            if (LSB_1 & n) 
                Font |= OLED_BRIGHTNESS; 
            else 
                Font &= 0xF0;
	    SPI_send_data(Font);
            if (MSB_2 & n) 
                Font = OLED_BR_1B; 
            else 
                Font = 0;
            if (LSB_2 & n) 
                Font |= OLED_BRIGHTNESS; 
            else 
                Font &= 0xF0;
            SPI_send_data(Font);
            n<<=1;
        }
	c++;
    }
    OLED_Set_Remap_Format(0x14);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show String
//
//    a: Database
//    b: Start X Address
//    c: Start Y Address
//    * Must write "0" in the end...
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Show_String(unsigned char a,  char *Data_Pointer,
    unsigned char b, unsigned char c) {
     char *Src_Pointer;
    Src_Pointer = Data_Pointer;
    OLED_Show_Font57(1, 32, b, c);			   // No-Break Space
    //   Must be written first before the string start...
    do {
	OLED_Show_Font57(a, *Src_Pointer, b, c);
	Src_Pointer++;
	b += 2;
    } while (*Src_Pointer != 0);
}



//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Pattern (Partial or Full Screen)
//
//    a: Column Address of Start
//    b: Column Address of End (Total Columns Devided by 4)
//    c: Row Address of Start
//    d: Row Address of End
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Show_Pattern(unsigned char *Data_Pointer, unsigned char a,
    unsigned char b, unsigned char c, unsigned char d) {
    unsigned char *Src_Pointer;
    unsigned char i, j;
    Src_Pointer = Data_Pointer;
    OLED_Set_Column_Address(OLED_Shift + a, OLED_Shift + b);
    OLED_Set_Row_Address(c, d);
    OLED_Set_Write_RAM();
    for (i = 0; i < (d - c + 1); i++) {
	for (j = 0; j < (b - a + 1); j++) {
	    SPI_send_data(*Src_Pointer);
	    Src_Pointer++;
	    SPI_send_data(*Src_Pointer);
	    Src_Pointer++;
	}
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Vertical Scrolling (Full Screen)
//
//    a: Scrolling Direction
//       "0x00" (Upward)
//       "0x01" (Downward)
//    b: Set Numbers of Row Scroll per Step
//    c: Set Time Interval between Each Scroll Step
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Vertical_Scroll(unsigned char a, unsigned char b, unsigned char c) {
    unsigned char i, j;
    OLED_Set_Partial_Display(0x00, 0x00, OLED_Max_Row);
    switch (a) {
    case 0:
	for (i = 0; i < (OLED_Max_Row + 1); i += b) {
	    OLED_Set_Display_Offset(i + 1);
	    for (j = 0; j < c; j++) {
		uDelay(200);
	    }
	}
	break;
    case 1:
	for (i = 0; i < (OLED_Max_Row + 1); i += b) {
	    OLED_Set_Display_Offset(OLED_Max_Row - i);
	    for (j = 0; j < c; j++)
		uDelay(200);
	}
	break;
    }
    OLED_Set_Partial_Display(0x01, 0x00, 0x00);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Vertical Fade Scrolling (Full Screen)
//
//    a: Scrolling Direction
//       "0x00" (Upward - In)
//       "0x01" (Downward - In)
//       "0x02" (Upward - Out)
//       "0x03" (Downward - Out)
//    b: Set Numbers of Row Scroll per Step
//    c: Set Time Interval between Each Scroll Step
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Fade_Scroll(unsigned char a, unsigned char b, unsigned char c) {
    unsigned char i, j;
    OLED_Set_Partial_Display(0x00, 0x00, OLED_Max_Row);
    switch (a) {
    case 0:
	for (i = (OLED_Max_Row + 1); i < 128; i += b) {
	    OLED_Set_Start_Line(i);
	    for (j = 0; j < c; j++) {
		uDelay(200);
	    }
	}
	OLED_Set_Start_Line(0x00);
	for (j = 0; j < c; j++) {
	    uDelay(200);
	}
	break;
    case 1:
	for (i = 0; i < (OLED_Max_Row + 1); i += b) {
	    OLED_Set_Start_Line(OLED_Max_Row - i);
	    for (j = 0; j < c; j++) {
		uDelay(200);
	    }
	}
	break;
    case 2:
	for (i = 0; i < (OLED_Max_Row + 1); i += b) {
	    OLED_Set_Start_Line(i + 1);
	    for (j = 0; j < c; j++) {
		uDelay(200);
	    }
	}
	break;
    case 3:
	for (i = 127; i > OLED_Max_Row; i -= b) {
	    OLED_Set_Start_Line(i);
	    for (j = 0; j < c; j++) {
		uDelay(200);
	    }
	}
	break;
    }
    OLED_Set_Partial_Display(0x01, 0x00, 0x00);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Fade In (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Fade_In() {
    unsigned char i;
    OLED_Set_Display_On_Off(0x01);
    for (i = 0; i < (OLED_Brightness + 1); i++) {
	OLED_Set_Master_Current(i);
	uDelay(200);
	uDelay(200);
	uDelay(200);
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Fade Out (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Fade_Out() {
    unsigned char i;
    for (i = (OLED_Brightness + 1); i > 0; i--) {
	OLED_Set_Master_Current(i - 1);
	uDelay(200);
	uDelay(200);
	uDelay(200);
    }
    OLED_Set_Display_On_Off(0x00);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Sleep Mode
//
//    "0x01" Enter Sleep Mode
//    "0x00" Exit Sleep Mode
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Sleep(unsigned char a) {
    switch (a) {
    case 0:
	OLED_Set_Display_On_Off(0x00);
	OLED_Set_Display_Mode(0x01);
	break;
    case 1:
	OLED_Set_Display_Mode(0x02);
	OLED_Set_Display_On_Off(0x01);
	break;
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Connection Test
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Test() {
    unsigned char i;
    LCD_RESET = 0;
    for (i = 0; i < 200; i++) {
	uDelay(200);
    }
    LCD_RESET = 1;
    OLED_Set_Display_Mode(0x01);			   // Entire Display On Mode (0x00/0x01/0x02/0x03)
    while (1) {
	OLED_Set_Display_On_Off(0x01);			   // Display On (0x00/0x01)
	Delay(2);
	OLED_Set_Display_On_Off(0x00);			   // Display Off (0x00/0x01)
	Delay(2);
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Gray Scale Table Setting (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Set_Gray_Scale_Table() {
    SPI_send_cmd(0xB8);				   // Set Gray Scale Table
    SPI_send_data(0x0C);				   //   Gray Scale Level 1
    SPI_send_data(0x18);				   //   Gray Scale Level 2
    SPI_send_data(0x24);				   //   Gray Scale Level 3
    SPI_send_data(0x30);				   //   Gray Scale Level 4
    SPI_send_data(0x3C);				   //   Gray Scale Level 5
    SPI_send_data(0x48);				   //   Gray Scale Level 6
    SPI_send_data(0x54);				   //   Gray Scale Level 7
    SPI_send_data(0x60);				   //   Gray Scale Level 8
    SPI_send_data(0x6C);				   //   Gray Scale Level 9
    SPI_send_data(0x78);				   //   Gray Scale Level 10
    SPI_send_data(0x84);				   //   Gray Scale Level 11
    SPI_send_data(0x90);				   //   Gray Scale Level 12
    SPI_send_data(0x9C);				   //   Gray Scale Level 13
    SPI_send_data(0xA8);				   //   Gray Scale Level 14
    SPI_send_data(0xB4);				   //   Gray Scale Level 15
    SPI_send_cmd(0x00);					   // Enable Gray Scale Table
} 

void
OLED_Set_Linear_Gray_Scale_Table() {
    SPI_send_cmd(0xB9);				   // Set Default Linear Gray Scale Table
}


//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Initialization
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void
OLED_Init() {
    unsigned char i;
    LCD_RESET = 0;
    for (i = 0; i < 200; i++)
	uDelay(200);
    LCD_RESET = 1;
    OLED_Set_Command_Lock(0x12);			   // Unlock Basic Commands (0x12/0x16)
    OLED_Set_Display_On_Off(0x00);			   // Display Off (0x00/0x01)
    OLED_Set_Display_Clock(0xD0);			   // Set Clock as 80 Frames/Sec
    OLED_Set_Multiplex_Ratio(0x3F);			   // 1/64 Duty (0x0F~0x3F)
    OLED_Set_Display_Offset(0x00);			   // OLED_Shift Mapping RAM Counter (0x00~0x3F)
    OLED_Set_Start_Line(0x00);				   // Set Mapping RAM Display Start Line (0x00~0x7F)
    OLED_Set_Remap_Format(0x14);			   // Set Horizontal Address Increment
    //     Column Address 0 Mapped to SEG0
    //     Disable Nibble Remap
    //     Scan from COM[N-1] to COM0
    //     Disable COM Split Odd Even
    //     Enable Dual COM Line Mode
    OLED_Set_GPIO(0x00);				   // Disable GPIO Pins Input
    OLED_Set_Function_Selection(0x01);			   // Enable Internal VDD Regulator
    OLED_Set_Display_Enhancement_A(0xA0, 0xFD);		   // Enable External VSL
    // Set Low Gray Scale Enhancement
    OLED_Set_Contrast_Current(0xDF);			   // Set Segment Output Current
    OLED_Set_Master_Current(OLED_Brightness);		   // Set Scale Factor of Segment Output Current Control
    OLED_Set_Gray_Scale_Table();			   // Set Pulse Width for Gray Scale Table
    OLED_Set_Phase_Length(0xE8);			   // Set Phase 1 as 5 Clocks & Phase 2 as 14 Clocks
    OLED_Set_Display_Enhancement_B(0x20);		   // Enhance Driving Scheme Capability (0x00/0x20)
    OLED_Set_Precharge_Voltage(0x1F);			   // Set Pre-Charge Voltage Level as 0.60*VCC
    OLED_Set_Precharge_Period(0x08);			   // Set Second Pre-Charge Period as 8 Clocks
    OLED_Set_VCOMH(0x07);				   // Set Common Pins Deselect Voltage Level as 0.86*VCC
    OLED_Set_Display_Mode(0x02);			   // Normal Display Mode (0x00/0x01/0x02/0x03)
    OLED_Set_Partial_Display(0x01, 0x00, 0x00);		   // Disable Partial Display
    OLED_Fill_RAM(0x00);				   // Clear Screen
    OLED_Set_Display_On_Off(0x01);			   // Display On (0x00/0x01)
}


