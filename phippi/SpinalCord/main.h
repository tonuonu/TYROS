
void main(void);

extern volatile unsigned short ticks;
void SPI_send_cmd(unsigned char c);
void SPI_send_data(unsigned char c);
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
