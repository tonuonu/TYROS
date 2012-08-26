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

#define PD_INPUT    (0)
#define PD_OUTPUT   (1)
#define PF_TIMER    (1)
#define PF_MOTOR    (2)
#define PF_UART     (3)
#define PF_ANALOG   (0x80)

void HardwareSetup(void);

#define LED1         p3_3
#define LED1d        pd3_3

#define ERRORLEDd    pd5_2
#define ERRORLED     p5_2


#define LEFT_PWMd    pd3_2
#define LEFT_PWMs    p3_2s
#define LEFT_PWM     p3_2
#define RIGHT_PWMd   pd3_4
#define RIGHT_PWMs   p3_4s
#define RIGHT_PWM    p3_4

#define RIGHT_INA        p2_0
#define RIGHT_INB        p2_1
#define LEFT_INA         p2_2
#define LEFT_INB         p2_3
#define RIGHT_INAd       pd2_0
#define RIGHT_INBd       pd2_1
#define LEFT_INAd        pd2_2
#define LEFT_INBd        pd2_3

#define RIGHT_DIAGA      p2_4
#define RIGHT_DIAGB      p2_5
#define LEFT_DIAGA       p2_6
#define LEFT_DIAGB       p2_7
#define RIGHT_DIAGAd     pd2_4
#define RIGHT_DIAGBd     pd2_5
#define LEFT_DIAGAd      pd2_6
#define LEFT_DIAGBd      pd2_7

// Serial 0, USB interface to panda
#define CS0d         pd6_0
#define CS0          p6_0
#define CS0s         p6_0s
#define CLOCK0d      pd6_1
#define CLOCK0       p6_1
#define CLOCK0d      pd6_1
#define CLOCK0s      p6_1s
#define RX0          p6_2
#define RX0s         p6_2s
#define TX0          p6_3
#define TX0s         p6_3s
#define TX0d         pd6_3

// Serial 2, acceleration sensor
#define CS2d         pd7_7
#define CS2          p7_7
#define CS2s         p7_7s
#define TX2          p7_0
#define RX2          p7_1
#define TX2s         p7_0s
#define RX2s         p7_1s
#define TX2d         pd7_0
#define RX2d         pd7_1
#define CLOCK2       p7_2
#define CLOCK2d      pd7_2
#define CLOCK2s      p7_2s

// Serial 3, OLED
#define OLED_DATACOMMAND p4_0
#define OLED_DATACOMMANDd pd4_0

#define CLOCK3       p4_1
#define CLOCK3d      pd4_1
#define CLOCK3s      p4_1s

#define OLED_RESET   p4_2
#define OLED_RESETd  pd4_2

#define TX3          p4_3
#define TX3s         p4_3s
#define TX3d         pd4_3

#define OLED_ENABLE  p4_4
#define OLED_ENABLEd pd4_4

//#define OLED_VDD     p4_5
//#define OLED_VDDd    pd4_5

#define OLED_CS       p5_7
#define OLED_CSd      pd5_7

// Melexis LDO enable
#define MELEXIS_EN  p3_0
#define MELEXIS_ENd  pd3_0
// Serial 4, melexis rotation sensor
#define CS4         p9_4
#define CLOCK4      p9_5
#define CLOCK4s     p9_5s
#define CLOCK4d     pd9_5
#define TX4         p9_6
#define TX4s        p9_6s
#define TX4d        pd9_6

#define RX4         p9_7
#define RX4s        p9_7s

// Serial 5, old panda interface, unused
#define TX5         p7_6
#define TX5s        p7_6s
#define TX5d        pd7_6
#define RX5         p8_0
#define RX5d        pd8_0
#define RX5s        p8_0s
#define TX5         p7_6
#define TX5d        pd7_6
#define TX5s        p7_6s

// Serial 6, gyroscope
#define CLOCK6      p4_5
#define CLOCK6s     p4_5s
#define CLOCK6d     pd4_5
#define RX6         p4_6
#define RX6s        p4_6s
#define RX6d        pd4_6
#define TX6         p4_7
#define TX6s        p4_7s
#define TX6d        pd4_7
#define CS6         p5_1
#define CS6d        pd5_1
#define GYRO_INT1   p8_3
#define GYRO_INT2   p8_4

// Serial 7, melexis rotational
#define CS7         p5_3
#define CS7d        pd5_3

#define TX7         p5_4
#define TX7s        p5_4s
#define TX7d        pd5_4

#define RX7         p5_6
#define RX7s        p5_6s
#define RX7d        pd5_6

#define CLOCK7      p5_5
#define CLOCK7s     p5_5s
#define CLOCK7d     pd5_5

// Coilgun
#define BALL_DETECT p7_3
#define KICK        p7_5
#define KICKd       pd7_5

// Capacitor charger
#define CHARGE      p1_2
#define CHARGEd     pd1_2

#define CHARGE_DONE p3_7

// 5v power supply for Pandaboard
#define PANDA       p3_5
#define PANDAd      pd3_5

// Joystick
#define JOY_RIGHT   p1_3
#define JOY_LEFT    p1_4
#define JOY_DOWN    p1_5
#define JOY_UP      p1_6
#define JOY_CENTER  p1_7

#define JOY_UPd     pd1_3
#define JOY_DOWNd   pd1_4
#define JOY_LEFTd   pd1_5
#define JOY_RIGHTd  pd1_6
#define JOY_CENTERd pd1_7

// Analog
#define AN00         p10_0
#define AN01         p10_1
#define AN02         p10_2
#define AN03         p10_3
#define AN00d        pd10_0
#define AN01d        pd10_1
#define AN02d        pd10_2
#define AN03d        pd10_3
#define AN00s        p10_0s
#define AN01s        p10_1s
#define AN02s        p10_2s
#define AN03s        p10_3s

// Buzzer
#define BUZZERd     pd3_6
#define BUZZERs     p3_6s

#define TIMERB2COUNT	2000 // about 12,5kHz

#define SPI_DELAY (50)

void SPI2_Init(void);
void SPI3_Init(void);
void SPI4_Init(void);
void SPI6_Init(void);
void SPI7_Init(void);
void OLED_On(void);
void SPI6_send(unsigned short);
short unsigned SPI4_receive(void);
void SPI4_send(unsigned short c);
struct statuses {
    char sek_flag;
};
void Read_AD(void);

extern volatile struct statuses status;
