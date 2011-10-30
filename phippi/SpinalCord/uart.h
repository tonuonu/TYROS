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

extern void uart8_init(void);
extern void uart5_init(void);
extern void uart7_init(void);
extern void uart7_send(unsigned char);
extern void uart8_send (unsigned char);

#define TX_BUFF_SIZE  256 // 512
#define RX_BUFF_SIZE  1024

extern char tx5_buff[TX_BUFF_SIZE ];    
extern unsigned short tx5_ptr;        
extern volatile unsigned short tx5_ptrr;   
extern char rx8_buff[RX_BUFF_SIZE ];   
extern unsigned short rx8_ptr;        
extern unsigned short rx8_ptrr;      

#define OK 1
#define ERROR -1
#define FULL 0x80
#define EMPTY 0

#define rd_buff     rx8_buff
#define datat_buf   rx8_ptr


extern unsigned char get_uart5(void);