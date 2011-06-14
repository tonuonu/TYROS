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




void HardwareSetup(void);

#define LED1     p3_3
#define LED2     p3_5
#define KEY_1   p5_3
#define KEY_2   p5_2
#define KEY_3   p5_1

void SPI3_Init(void);
void SPI4_Init(void);
void uart7_Init(void);
struct statuses
{
    char sek_flag;
};

extern volatile struct statuses status;
