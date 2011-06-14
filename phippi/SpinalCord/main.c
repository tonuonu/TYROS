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
#include "main.h"
#include "uart.h"
#include "hwsetup.h"
extern int alarm;

volatile unsigned short ticks;

#define SPI_DELAY (50)
void
SPI_send_data(unsigned char c) {
    while (ti_u3c1 == 0)
        NOP();
    uDelay(SPI_DELAY);
    p4_0 = 1;
    uDelay(SPI_DELAY);
    u3tb = c;
}

void
SPI_send_cmd(unsigned char c) {
    while (ti_u3c1 == 0)
        NOP();
    uDelay(SPI_DELAY);
    p4_0 = 0;
    uDelay(SPI_DELAY);
    u3tb = c;
}

void
SPI4_send(unsigned short c) {
    while (ti_u4c1 == 0)
        NOP();
    u4tb = c;
}


short unsigned
SPI4_receive(void) {
    u4tb=0xff;
    while (ri_u4c1 == 0)
        NOP();
    return u4rb;
}

void
main(void) {
    HardwareSetup();
    LED1=0;
    LED2=1;
    OLED_Set_Display_Mode(0x02);                           // Entire Display On
    OLED_Set_Display_On_Off(0x01);                         // Display On
    OLED_Set_Display_Mode(0x00);                           // Entire Display Off
    OLED_Show_Logo();
    OLED_Set_Display_Mode(0x02);                           // Entire Display Off
    LED2=0;
    Delay(200);
    OLED_Fade_Out();
    OLED_Fill_RAM(0x00);
    OLED_Fade_In();
    while (1) {      
        if (status.sek_flag==1) {
            char buf[256];
            status.sek_flag=0;
            uart8_send(0x55);

            LED1 ^= 1;
#if 1
            p9_4=0;
            SPI4_send(0xAA);
            SPI4_send(0xFF);
            unsigned short c; /* 16 bit value */
            c=SPI4_receive();
            sprintf( /*(char *)*/ buf, "%s%s%s%s%s SPI4 %04x ",
                    (c & (1 << 11)) ? "Arbitr " : "",
                    (c & (1 << 12)) ? "Overr " : "",
                    (c & (1 << 13)) ? "Fram " : "",
                    (c & (1 << 14)) ? "Pari " : "",
                    (c & (1 << 15)) ? "Sum " : "", c);
            OLED_Show_String(1, buf, 10 /* left */ , 0*8 /* top */ );
            c=SPI4_receive();
            sprintf((char *) buf, "SPI4 %04x ",c);
            OLED_Show_String(1, buf, 10 /* left */ , 1*8 /* top */ );           
            p9_4=1;
#endif
        }
    }
}
